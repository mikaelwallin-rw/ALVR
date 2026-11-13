//! Phase sync (fork): steer WHEN the server produces frames so that each frame finishes decoding
//! just before the headset's render loop looks for it.
//!
//! Signal: the client's PhaseSyncReport (ReservedBuffer 'PSYN', about every 500 ms). Per picked
//! frame, margin = "render loop starts looking for a frame" minus "frame decoded" (client clock;
//! positive = the frame waited decoded, negative = the loop waited for it). The controller uses the
//! median of a report (single frames caught early or late do not pull it).
//! Actuator: the period of the server's virtual vsync (`StatisticsManager::duration_until_next_vsync`,
//! which paces SteamVR's compositor through `PostPresent`): nominal * (1 + rate / 1000), with `rate`
//! in ms of phase shift per second, |rate| <= MAX_RATE_MS_PER_S. A positive rate makes frames later.
//! Controller: PI on the margin error. The integral term converges to the clock frequency offset
//! between PC and headset (drift), so on average the server runs exactly at the headset's rate; the
//! proportional term moves the phase and is applied only while reports are fresh.
//!
//! Why the client changes its pick rules while phase sync is on (client_openxr stream.rs): a render
//! loop without a ready frame waits for one (today up to 0.8 frame). With that wait, a phase where
//! frames arrive shortly after the loop starts looking has two self-sustaining states: "the loop
//! waits a little" and "each frame waits a whole frame in the queue" (one frame behind, the 6 to 10 ms
//! decoder queue measured on 2026-11-13). Which one a session lands in is history (start-up, a
//! dropped frame). With phase sync the client waits at most half a frame and shows the newest ready
//! frame, so for a target margin below (frame - half a frame - jitter) only one state exists.
//!
//! Pure logic, no clocks: times are passed in seconds.

/// Largest phase shift per second (ms/s). 1 ms/s = the server's frame period 0.1 % longer/shorter.
pub const MAX_RATE_MS_PER_S: f32 = 1.0;
/// Largest effective target (target + back-off), ms: must stay below frame - client wait window
/// (11.1 - 5.6 ms at 90 Hz) by more than the jitter, else the "loop waits" state comes back
pub const MAX_EFFECTIVE_TARGET_MS: f32 = 4.0;
/// Largest frequency correction (ms/s = 1000 ppm); PC/headset clocks differ by far less
const MAX_FREQ_MS_PER_S: f32 = 0.8;
/// Proportional gain (1/s) and integral gain (1/s^2): damping kp / (2 sqrt(ki)) = 0.88
const KP: f32 = 0.25;
const KI: f32 = 0.02;
/// The proportional term is applied only while the last report is this recent
const FRESH_S: f64 = 1.5;
/// Lock: |error| below LOCK_ERR_MS in LOCK_REPORTS reports in a row; unlock: above UNLOCK_ERR_MS in
/// UNLOCK_REPORTS reports in a row
const LOCK_ERR_MS: f32 = 1.0;
const LOCK_REPORTS: u32 = 3;
const UNLOCK_ERR_MS: f32 = 3.0;
const UNLOCK_REPORTS: u32 = 4;
/// Back-off: extra margin while frames miss their slot too often (arrive later than the loop's
/// wait window), judged every EXTRA_RAISE_INTERVAL_S, lowered again after a quiet period
const EXTRA_STEP_MS: f32 = 0.5;
const EXTRA_RAISE_INTERVAL_S: f64 = 5.0;
const EXTRA_DECAY_MS: f32 = 0.25;
const EXTRA_DECAY_INTERVAL_S: f64 = 30.0;
/// Missed slots above this fraction of loops (and more than 3 in a judging interval)
const MISS_FRACTION_LIMIT: f32 = 0.02;

/// One client report, as the controller needs it
#[derive(Clone, Copy, Debug, Default)]
pub struct PhaseSample {
    /// Client time covered by the report, seconds
    pub window_s: f32,
    /// Headset display period, seconds
    pub client_frame_interval_s: f32,
    pub loops: u32,
    pub frames: u32,
    pub skipped_loops: u32,
    pub missed_slots: u32,
    /// The controlled value (median margin of the report, ms)
    pub margin_p50_ms: f32,
    /// Frames the server presented during about the same window
    pub server_frames: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HoldReason {
    /// The client skips the stream (stream unused): no samples
    StreamSkipped,
    /// Too few frames picked (no scene / SteamVR not presenting, decoder restarting)
    TooFewFrames,
    /// The client's render loop runs much slower than its display (app too slow)
    ClientSlow,
    /// Malformed report
    Invalid,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PhaseSyncUpdate {
    pub hold: Option<HoldReason>,
    /// Some(true) when the controller just locked, Some(false) when it lost the lock
    pub lock_changed: Option<bool>,
    /// The back-off margin was raised or lowered
    pub extra_changed: bool,
    /// Error used (ms), when not held
    pub error_ms: f32,
}

#[derive(Clone, Debug)]
pub struct PhaseSyncController {
    target_margin_ms: f32,
    extra_margin_ms: f32,
    /// Integral term = frequency correction, ms/s
    freq_ms_per_s: f32,
    /// Proportional term, ms/s
    p_ms_per_s: f32,
    last_update_s: Option<f64>,
    locked: bool,
    in_lock_band: u32,
    out_of_lock_band: u32,
    last_extra_change_s: f64,
    miss_window: MissWindow,
}

/// Loops, missed slots and server shortfall since the last back-off decision
#[derive(Clone, Debug, Default)]
struct MissWindow {
    start_s: Option<f64>,
    loops: u32,
    missed: u32,
    shortfall: u32,
}

impl PhaseSyncController {
    pub fn new(target_margin_ms: f32) -> Self {
        Self {
            target_margin_ms: target_margin_ms.clamp(0.0, MAX_EFFECTIVE_TARGET_MS),
            extra_margin_ms: 0.0,
            freq_ms_per_s: 0.0,
            p_ms_per_s: 0.0,
            last_update_s: None,
            locked: false,
            in_lock_band: 0,
            out_of_lock_band: 0,
            last_extra_change_s: f64::NEG_INFINITY,
            miss_window: MissWindow::default(),
        }
    }

    pub fn effective_target_ms(&self) -> f32 {
        self.target_margin_ms + self.extra_margin_ms
    }

    pub fn extra_margin_ms(&self) -> f32 {
        self.extra_margin_ms
    }

    pub fn is_locked(&self) -> bool {
        self.locked
    }

    /// Integral term: the frequency correction in ms/s (= 1000 ppm per ms/s)
    pub fn freq_ms_per_s(&self) -> f32 {
        self.freq_ms_per_s
    }

    /// Phase shift rate to apply now (ms per second; positive = frames later). Without fresh
    /// reports only the frequency correction stays (the phase is held, drift still compensated).
    pub fn rate_at(&self, now_s: f64) -> f32 {
        let fresh = self
            .last_update_s
            .is_some_and(|last| now_s - last <= FRESH_S);
        let rate = if fresh {
            self.p_ms_per_s + self.freq_ms_per_s
        } else {
            self.freq_ms_per_s
        };
        rate.clamp(-MAX_RATE_MS_PER_S, MAX_RATE_MS_PER_S)
    }

    /// Frame period to use for a nominal period (seconds in, seconds out)
    pub fn frame_interval_s(&self, nominal_s: f64, now_s: f64) -> f64 {
        nominal_s * (1.0 + f64::from(self.rate_at(now_s)) / 1000.0)
    }

    pub fn update(&mut self, now_s: f64, sample: &PhaseSample) -> PhaseSyncUpdate {
        let mut out = PhaseSyncUpdate::default();

        let hold = if !(sample.window_s > 0.0
            && sample.client_frame_interval_s > 0.0
            && sample.margin_p50_ms.is_finite())
        {
            Some(HoldReason::Invalid)
        } else if sample.skipped_loops > 0 {
            Some(HoldReason::StreamSkipped)
        } else if (sample.loops as f32) < 0.8 * sample.window_s / sample.client_frame_interval_s {
            Some(HoldReason::ClientSlow)
        } else if sample.frames < 5 || (sample.frames as f32) < 0.5 * sample.loops as f32 {
            Some(HoldReason::TooFewFrames)
        } else {
            None
        };
        if let Some(reason) = hold {
            // Keep the frequency correction, drop the phase correction
            self.p_ms_per_s = 0.0;
            self.in_lock_band = 0;
            out.hold = Some(reason);
            return out;
        }

        let frame_ms = sample.client_frame_interval_s * 1000.0;
        let error = (sample.margin_p50_ms - self.effective_target_ms()).clamp(-frame_ms, frame_ms);
        let dt = sample.window_s.clamp(0.05, 2.0);

        self.p_ms_per_s = KP * error;
        let total = self.p_ms_per_s + self.freq_ms_per_s;
        // Anti-windup: no integration while the output saturates in the error's direction
        let saturated = total.abs() >= MAX_RATE_MS_PER_S && total.signum() == error.signum();
        if !saturated {
            self.freq_ms_per_s =
                (self.freq_ms_per_s + KI * error * dt).clamp(-MAX_FREQ_MS_PER_S, MAX_FREQ_MS_PER_S);
        }
        self.last_update_s = Some(now_s);
        out.error_ms = error;

        // Lock state (for the log and the back-off)
        if error.abs() < LOCK_ERR_MS {
            self.in_lock_band += 1;
            self.out_of_lock_band = 0;
        } else if error.abs() > UNLOCK_ERR_MS {
            self.out_of_lock_band += 1;
            self.in_lock_band = 0;
        } else {
            self.in_lock_band = 0;
            self.out_of_lock_band = 0;
        }
        if !self.locked && self.in_lock_band >= LOCK_REPORTS {
            self.locked = true;
            out.lock_changed = Some(true);
        } else if self.locked && self.out_of_lock_band >= UNLOCK_REPORTS {
            self.locked = false;
            out.lock_changed = Some(false);
        }

        // Back-off, judged over EXTRA_RAISE_INTERVAL_S: frames miss their slot too often at this
        // margin. Misses explained by frames the server did not deliver (game slower than the
        // headset) do not count: those empty loops are not a phase problem. A frame the loop waited
        // for (negative margin) is not a problem either: it is shown in its slot, with less latency.
        self.miss_window.loops += sample.loops;
        self.miss_window.missed += sample.missed_slots;
        self.miss_window.shortfall += sample.loops.saturating_sub(sample.server_frames);
        let window_start = *self.miss_window.start_s.get_or_insert(now_s);
        if now_s - window_start >= EXTRA_RAISE_INTERVAL_S {
            let window = std::mem::take(&mut self.miss_window);
            let unexplained = window.missed.saturating_sub(window.shortfall);
            let too_many_misses =
                unexplained as f32 > (MISS_FRACTION_LIMIT * window.loops as f32).max(3.0);
            let max_extra = MAX_EFFECTIVE_TARGET_MS - self.target_margin_ms;
            if self.locked && too_many_misses {
                if self.extra_margin_ms < max_extra {
                    self.extra_margin_ms = (self.extra_margin_ms + EXTRA_STEP_MS).min(max_extra);
                    self.last_extra_change_s = now_s;
                    out.extra_changed = true;
                }
            } else if self.extra_margin_ms > 0.0
                && unexplained == 0
                && now_s - self.last_extra_change_s >= EXTRA_DECAY_INTERVAL_S
            {
                self.extra_margin_ms = (self.extra_margin_ms - EXTRA_DECAY_MS).max(0.0);
                self.last_extra_change_s = now_s;
                out.extra_changed = true;
            }
        }

        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Deterministic pseudo random numbers
    struct Rng(u64);
    impl Rng {
        fn uniform(&mut self) -> f64 {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            (self.0 >> 11) as f64 / (1u64 << 53) as f64
        }
        // Roughly normal, sigma 1
        fn normal(&mut self) -> f64 {
            (0..12).map(|_| self.uniform()).sum::<f64>() - 6.0
        }
    }

    // The client's wait for a frame while phase sync is on (client_openxr stream.rs)
    const CLIENT_WAIT_FRAMES: f64 = 0.5;

    struct Scenario {
        seconds: f64,
        /// Headset period = server nominal period * (1 + client_ppm / 1e6)
        client_ppm: f64,
        /// Pipeline time from virtual vsync to decoded, ms, as a function of time
        latency_ms: fn(f64) -> f64,
        jitter_sigma_ms: f64,
        /// Extra delay of single frames (decoder hiccups), ms
        hiccup_ms: fn(&mut Rng) -> f64,
        /// Server presents no frame at this vsync (game too slow, idle)
        server_drops: fn(f64, &mut Rng) -> bool,
        /// Client skips the stream (stream unused) at this time
        client_skips: fn(f64) -> bool,
        target_ms: f32,
        /// Margin at the start (positive = frames wait decoded; above frame - wait window this is
        /// the "one frame behind" state of today)
        initial_margin_ms: f64,
    }

    impl Default for Scenario {
        fn default() -> Self {
            Self {
                seconds: 90.0,
                client_ppm: 0.0,
                latency_ms: |_| 40.0,
                jitter_sigma_ms: 0.6,
                hiccup_ms: |_| 0.0,
                server_drops: |_, _| false,
                client_skips: |_| false,
                target_ms: 2.0,
                initial_margin_ms: 2.0,
            }
        }
    }

    #[derive(Default)]
    struct Trace {
        // (time, report median margin (NaN when held), rate)
        reports: Vec<(f64, f32, f32)>,
        locks: Vec<f64>,
        unlocks: Vec<f64>,
        repeats: u32,
        drops: u32,
        extra: f32,
        freq: f32,
    }

    impl Trace {
        fn margins(&self, from: f64, to: f64) -> Vec<f32> {
            self.reports
                .iter()
                .filter(|(t, m, _)| *t >= from && *t < to && m.is_finite())
                .map(|(_, m, _)| *m)
                .collect()
        }
        fn mean_margin(&self, from: f64, to: f64) -> f32 {
            let v = self.margins(from, to);
            assert!(!v.is_empty(), "no reports in {from}..{to}");
            v.iter().sum::<f32>() / v.len() as f32
        }
        fn max_abs_error(&self, from: f64, to: f64, target: f32) -> f32 {
            self.margins(from, to)
                .iter()
                .map(|m| (m - target).abs())
                .fold(0.0, f32::max)
        }
        fn rate_spread(&self, from: f64, to: f64) -> f32 {
            let rates: Vec<f32> = self
                .reports
                .iter()
                .filter(|(t, _, _)| *t >= from && *t < to)
                .map(|(_, _, r)| *r)
                .collect();
            rates.iter().copied().fold(f32::MIN, f32::max)
                - rates.iter().copied().fold(f32::MAX, f32::min)
        }
    }

    // Discrete simulation: the server's virtual vsync grid (period from the controller), the
    // pipeline to "decoded" (latency + jitter + hiccups, decoder output in order), and the client
    // render loop at the headset's period with phase sync's pick rules: newest ready frame (older
    // ones dropped), else wait up to half a frame for the next one, else show the previous frame.
    // Reports every 45 loops go back to the controller.
    fn simulate(s: &Scenario) -> Trace {
        let mut rng = Rng(12345);
        let nominal = 1.0 / 90.0;
        let tc = nominal * (1.0 + s.client_ppm * 1e-6);
        let poll_at = |k: u64| k as f64 * tc + 0.003;
        let mut controller = PhaseSyncController::new(s.target_ms);
        let mut trace = Trace::default();

        // First frame decoded initial_margin_ms before poll 10; the loops start at the first poll
        // that finds it ready (so a large initial margin starts "one frame behind")
        let first_vsync = poll_at(10) - (s.initial_margin_ms + (s.latency_ms)(0.0)) / 1000.0;
        let mut next_vsync = first_vsync;
        let first_loop = 10;
        let mut last_decoded = f64::NEG_INFINITY;
        let mut queue: Vec<f64> = vec![];
        let mut prev_loop_had_frame = true;

        let (mut loops, mut frames, mut skipped, mut missed) = (0u32, 0u32, 0u32, 0u32);
        let mut margins: Vec<f32> = vec![];
        let mut server_frames = 0u32;
        let mut window_start = poll_at(first_loop - 1);

        let last_loop = (s.seconds / tc) as u64;
        for k in first_loop..last_loop {
            let poll = poll_at(k);
            while next_vsync < poll + tc {
                let v = next_vsync;
                next_vsync += controller.frame_interval_s(nominal, v);
                if (s.server_drops)(v, &mut rng) {
                    continue;
                }
                server_frames += 1;
                let delay =
                    (s.latency_ms)(v) + s.jitter_sigma_ms * rng.normal() + (s.hiccup_ms)(&mut rng);
                let decoded = (v + delay / 1000.0).max(last_decoded + 0.0002);
                last_decoded = decoded;
                queue.push(decoded);
            }

            loops += 1;
            if (s.client_skips)(poll) {
                queue.retain(|d| *d > poll);
                skipped += 1;
                prev_loop_had_frame = true;
            } else {
                let ready = queue.iter().filter(|d| **d <= poll).count();
                if ready > 0 {
                    let newest = queue[ready - 1];
                    trace.drops += ready as u32 - 1;
                    queue.drain(..ready);
                    margins.push(((poll - newest) * 1000.0) as f32);
                    frames += 1;
                    if !prev_loop_had_frame {
                        missed += 1;
                    }
                    prev_loop_had_frame = true;
                } else if queue
                    .first()
                    .is_some_and(|d| *d <= poll + CLIENT_WAIT_FRAMES * tc)
                {
                    let d = queue.remove(0);
                    margins.push(((poll - d) * 1000.0) as f32);
                    frames += 1;
                    prev_loop_had_frame = true;
                } else {
                    trace.repeats += 1;
                    prev_loop_had_frame = false;
                }
            }

            if loops == 45 {
                let mut sorted = margins.clone();
                sorted.sort_by(f32::total_cmp);
                let p50 = sorted.get(sorted.len() / 2).copied().unwrap_or(0.0);
                let sample = PhaseSample {
                    window_s: (poll - window_start) as f32,
                    client_frame_interval_s: tc as f32,
                    loops,
                    frames,
                    skipped_loops: skipped,
                    missed_slots: missed,
                    margin_p50_ms: p50,
                    server_frames,
                };
                // the report reaches the server a little later
                let now = poll + 0.002;
                let update = controller.update(now, &sample);
                match update.lock_changed {
                    Some(true) => trace.locks.push(now),
                    Some(false) => trace.unlocks.push(now),
                    None => (),
                }
                trace.reports.push((
                    now,
                    if update.hold.is_none() { p50 } else { f32::NAN },
                    controller.rate_at(now),
                ));
                (loops, frames, skipped, missed) = (0, 0, 0, 0);
                margins.clear();
                server_frames = 0;
                window_start = poll;
            }
        }
        trace.extra = controller.extra_margin_ms();
        trace.freq = controller.freq_ms_per_s();
        trace
    }

    fn assert_settled(trace: &Trace, from: f64, to: f64, target: f32) {
        let mean = trace.mean_margin(from, to);
        assert!((mean - target).abs() < 0.3, "settled margin {mean}");
        let max_error = trace.max_abs_error(from, to, target);
        assert!(max_error < 1.0, "max error {max_error}");
        // No oscillation: the rate barely moves once settled
        let spread = trace.rate_spread(from, to);
        assert!(spread < 0.3, "rate spread {spread}");
    }

    #[test]
    fn leaves_the_one_frame_behind_state() {
        // Today's typical session: every frame waits ~8 ms decoded (one frame behind)
        let trace = simulate(&Scenario {
            initial_margin_ms: 8.0,
            ..Default::default()
        });
        let start = trace.mean_margin(0.0, 0.7);
        assert!(start > 7.0, "start margin {start}");
        assert_eq!(trace.locks.len(), 1, "locks {:?}", trace.locks);
        assert!(trace.locks[0] < 25.0, "locked at {}", trace.locks[0]);
        assert!(trace.unlocks.is_empty());
        assert_settled(&trace, 40.0, 90.0, 2.0);
    }

    #[test]
    fn converges_from_frames_the_loop_waits_for() {
        // Frames arrive ~4 ms after the loop starts looking (the loop waits for them)
        let trace = simulate(&Scenario {
            initial_margin_ms: -4.0,
            ..Default::default()
        });
        let start = trace.mean_margin(0.0, 0.7);
        assert!(start < -3.0, "start margin {start}");
        assert_eq!(trace.locks.len(), 1);
        assert_settled(&trace, 40.0, 90.0, 2.0);
    }

    #[test]
    fn tracks_clock_drift_both_ways() {
        for ppm in [-300.0, 200.0] {
            let trace = simulate(&Scenario {
                client_ppm: ppm,
                seconds: 150.0,
                ..Default::default()
            });
            assert_settled(&trace, 60.0, 150.0, 2.0);
            // The integral term is the frequency offset: the server runs at the headset's rate
            let expected = (ppm / 1000.0) as f32;
            assert!(
                (trace.freq - expected).abs() < 0.03,
                "ppm {ppm}: freq {} expected {expected}",
                trace.freq
            );
            assert!(trace.unlocks.is_empty());
        }
    }

    #[test]
    fn without_control_drift_walks_through_all_phases() {
        // Reference: the same drift with the controller frozen at zero would wrap the phase
        let mut c = PhaseSyncController::new(2.0);
        c.p_ms_per_s = 0.0;
        assert_eq!(c.rate_at(0.0), 0.0);
        // 200 ppm = 0.2 ms/s: a full 11.1 ms wrap in 56 s (the saw-tooth seen on 2026-11-13)
        assert!((11.1 / 0.2 - 55.5_f64).abs() < 0.1);
    }

    #[test]
    fn follows_a_step_in_decode_time() {
        // Decode gets 3 ms slower at t = 50 s (e.g. a bitrate change)
        let trace = simulate(&Scenario {
            latency_ms: |t| if t < 50.0 { 40.0 } else { 43.0 },
            seconds: 110.0,
            ..Default::default()
        });
        let dip = trace.mean_margin(50.5, 51.5);
        assert!(dip < 0.0, "step visible {dip}");
        assert_eq!(
            trace.repeats, 0,
            "frames shown late in their slot, none missed"
        );
        assert_settled(&trace, 80.0, 110.0, 2.0);
    }

    #[test]
    fn server_frame_drops_do_not_move_the_phase() {
        // The game misses 10 % of its frames; the frames that come still have the right phase
        let trace = simulate(&Scenario {
            server_drops: |_, rng| rng.uniform() < 0.1,
            ..Default::default()
        });
        let mean = trace.mean_margin(40.0, 90.0);
        assert!(
            (mean - 2.0).abs() < 0.3,
            "settled margin {mean}, extra {}, freq {}, locks {:?}",
            trace.extra,
            trace.freq,
            trace.locks
        );
        // The empty loops come from the server, not from the phase: no back-off
        assert_eq!(trace.extra, 0.0);
    }

    #[test]
    fn idle_and_skipped_periods_hold_the_phase() {
        // Locked, then 20 s without frames (no scene app), then 20 s stream unused, then frames
        let trace = simulate(&Scenario {
            client_ppm: 150.0,
            seconds: 130.0,
            server_drops: |t, _| (50.0..70.0).contains(&t),
            client_skips: |t| (75.0..95.0).contains(&t),
            ..Default::default()
        });
        // Right after each pause the phase is still close (drift compensated meanwhile)
        let after_idle = trace.mean_margin(71.0, 73.0);
        assert!((after_idle - 2.0).abs() < 1.0, "after idle {after_idle}");
        let after_skip = trace.mean_margin(96.0, 98.0);
        assert!((after_skip - 2.0).abs() < 1.0, "after skip {after_skip}");
        assert_settled(&trace, 105.0, 130.0, 2.0);
        assert!(trace.unlocks.is_empty(), "unlocks {:?}", trace.unlocks);
    }

    #[test]
    fn decode_jitter_keeps_the_median_on_target() {
        // Jitter sigma 1.5 ms: a quarter of the frames make the loop wait, which is harmless
        let trace = simulate(&Scenario {
            jitter_sigma_ms: 1.5,
            seconds: 120.0,
            ..Default::default()
        });
        let mean = trace.mean_margin(60.0, 120.0);
        assert!((mean - 2.0).abs() < 0.4, "margin {mean}");
        assert_eq!(trace.extra, 0.0);
    }

    #[test]
    fn missed_slots_raise_the_margin() {
        // 5 % of the frames are 9 ms late (decoder hiccups): beyond 2 ms + half a frame of wait
        let trace = simulate(&Scenario {
            hiccup_ms: |rng| if rng.uniform() < 0.05 { 9.0 } else { 0.0 },
            seconds: 120.0,
            ..Default::default()
        });
        assert!(trace.extra >= 1.0, "extra {}", trace.extra);
        assert!(trace.extra <= MAX_EFFECTIVE_TARGET_MS - 2.0);
        // Late frames never pile up: the newest ready frame is shown, so the median stays at the
        // (raised) target instead of drifting one frame behind
        let mean = trace.mean_margin(90.0, 120.0);
        assert!((mean - (2.0 + trace.extra)).abs() < 0.6, "margin {mean}");
    }

    #[test]
    fn held_reports_drop_the_phase_term_and_keep_the_frequency() {
        let mut c = PhaseSyncController::new(2.0);
        let good = PhaseSample {
            window_s: 0.5,
            client_frame_interval_s: 1.0 / 90.0,
            loops: 45,
            frames: 45,
            margin_p50_ms: 4.0,
            server_frames: 45,
            ..Default::default()
        };
        for i in 0..4 {
            assert_eq!(c.update(f64::from(i) * 0.5, &good).hold, None);
        }
        let freq = c.freq_ms_per_s();
        assert!(freq > 0.0);
        assert!(c.rate_at(1.6) > freq);
        for (sample, reason) in [
            (
                PhaseSample {
                    skipped_loops: 45,
                    frames: 0,
                    ..good
                },
                HoldReason::StreamSkipped,
            ),
            (PhaseSample { frames: 3, ..good }, HoldReason::TooFewFrames),
            (PhaseSample { loops: 20, ..good }, HoldReason::ClientSlow),
            (
                PhaseSample {
                    margin_p50_ms: f32::NAN,
                    ..good
                },
                HoldReason::Invalid,
            ),
        ] {
            assert_eq!(c.update(2.0, &sample).hold, Some(reason));
            assert_eq!(c.rate_at(2.0), freq);
        }
        // Stale reports: only the frequency term
        let mut c2 = PhaseSyncController::new(2.0);
        c2.update(0.0, &good);
        assert!(c2.rate_at(1.0) > c2.freq_ms_per_s());
        assert_eq!(c2.rate_at(5.0), c2.freq_ms_per_s());
        // Never beyond the slew limit
        let far = PhaseSample {
            margin_p50_ms: 11.0,
            ..good
        };
        let mut c3 = PhaseSyncController::new(2.0);
        for i in 0..200 {
            c3.update(f64::from(i) * 0.5, &far);
            assert!(c3.rate_at(f64::from(i) * 0.5).abs() <= MAX_RATE_MS_PER_S);
        }
        // A controller that never got a report is the nominal grid
        let idle = PhaseSyncController::new(2.0);
        assert_eq!(idle.rate_at(10.0), 0.0);
        assert_eq!(idle.frame_interval_s(1.0 / 90.0, 10.0), 1.0 / 90.0);
        // The target is capped below the client's wait window
        assert_eq!(
            PhaseSyncController::new(9.0).effective_target_ms(),
            MAX_EFFECTIVE_TARGET_MS
        );
    }
}
