//! Late pickup (fork, with phase sync): pick the main stream frame as late as the render loop's
//! work still finishes before the runtime's compositor takes the app's frame ("latch").
//!
//! Why (D:\wkspaces\ScreenVR\docs\StreamingProfile-2026-11-13.md A.2/A.3): PICO wakes the render
//! loop (xrWaitFrame returns) about 12.5 ms before its compositor latches the frame, and the loop
//! needs about 7 ms (CPU + GPU) for it. Picking the decoded frame right after the wake-up leaves
//! about 5 ms of idle time before the latch, i.e. the shown frame is 5 ms older than necessary.
//!
//! How: per loop a DEADLINE after the wake-up,
//!   deadline = latch estimate - work estimate - safety margin - back-off,
//! clamped to [0, MAX_DEADLINE_FRACTION of a frame]. The loop shows a frame that belongs to this
//! loop as soon as it is decoded, waits for one until the deadline, and never waits past it.
//! - Latch estimate (the app cannot see the compositor): from the runtime's frame period and the
//!   headset model (latch_after_wake), measured on the PICO 4 Ultra Enterprise 2026-11-13 (wake to
//!   latch 12.54 ms, std 0.44, at 90 Hz); other models get a conservative fraction of a frame.
//! - Work estimate: the loop's own "pick -> GPU work of the whole frame done" time (an EGL fence
//!   after the last GL command of the frame, waited for on a helper thread), WORK_PERCENTILE of the
//!   last WORK_WINDOW frames. Everything drawn after the pick (stream pass, key probe collection,
//!   hands, loading screen, quads) is inside it, so the other content is bounded by the same
//!   deadline. Until MIN_SAMPLES frames were measured (or when measurements stop) the planner is
//!   not ready and the loop behaves as without late pickup.
//! - Fail safe: a frame whose GPU work finished after the latch estimate is "GPU late"; more than
//!   LATE_LIMIT of them within the last LATE_WINDOW deadline-mode frames raise the back-off by
//!   BACKOFF_STEP_MS (earlier pickup), it decays by BACKOFF_DECAY_MS per quiet BACKOFF_DECAY_AFTER.
//!   The deadline moves earlier at once and later only at DEADLINE_RISE_MS_PER_S, so the server's
//!   phase controller can follow it.
//!
//! The server aims frames at "deadline - target margin": margins are reported against the
//! deadline (PhaseSyncReport v2, reference DEADLINE), wrapped into (target - frame / 2,
//! target + frame / 2] so that a frame that came just after the previous loop's deadline counts
//! as late (negative) and not as a frame that waited almost a whole frame.
//!
//! Pure logic: no clocks, no GL; times are passed in.

use alvr_packets::LatePickupAnnouncement;
use alvr_system_info::Platform;
use std::{
    collections::VecDeque,
    time::{Duration, Instant},
};

/// Work samples kept (about 2 s at 90 Hz)
pub const WORK_WINDOW: usize = 180;
/// The work estimate is this percentile of the window
pub const WORK_PERCENTILE: f32 = 0.95;
/// Samples needed before the planner is ready
pub const MIN_SAMPLES: usize = 45;
/// Not ready when the last sample is older than this (fence measurements stopped)
pub const STALE_AFTER: Duration = Duration::from_secs(1);
/// Work assumed (fraction of a frame) while the window holds few samples
const INITIAL_WORK_FRACTION: f32 = 0.6;
/// The deadline is never later than this fraction of a frame after the wake-up (the old phase
/// sync wait ended about there: poll start + half a frame)
pub const MAX_DEADLINE_FRACTION: f32 = 0.5;
/// GPU late frames: judged over this many deadline-mode frames, more than LATE_LIMIT back off
pub const LATE_WINDOW: usize = 90;
pub const LATE_LIMIT: usize = 1;
pub const BACKOFF_STEP_MS: f32 = 1.0;
pub const BACKOFF_DECAY_MS: f32 = 0.25;
pub const BACKOFF_DECAY_AFTER: Duration = Duration::from_secs(10);
/// The deadline moves later at most this fast (ms per s); earlier at once
pub const DEADLINE_RISE_MS_PER_S: f32 = 0.5;

/// Wake-up (xrWaitFrame return) to compositor latch, estimated per headset model. PICO 4 Ultra
/// (incl. Enterprise): measured 12.54 ms at 90 Hz (= frame + 1.43 ms); frame + 1.0 ms is used.
/// Any other model: 0.8 frame (no measurement; the GPU late back-off still applies).
pub fn latch_after_wake(platform: Platform, frame_interval: Duration) -> Duration {
    match platform {
        Platform::Pico4Ultra => frame_interval + Duration::from_micros(1000),
        _ => frame_interval.mul_f32(0.8),
    }
}

/// What the render loop does with the newest ready frame now
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PickDecision {
    /// Take the newest ready frame now (none ready: show the previous frame again)
    Take,
    /// Look again shortly (before the deadline)
    Wait,
}

/// Logged by the caller
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PlannerEvent {
    BackedOff {
        backoff_ms: f32,
        late_frames: usize,
        window: usize,
    },
    Recovered {
        backoff_ms: f32,
    },
}

pub struct LatePickupPlanner {
    announcement: LatePickupAnnouncement,
    work_ms: VecDeque<f32>,
    last_sample_at: Option<Instant>,
    // GPU late flags of the last deadline-mode frames
    late: VecDeque<bool>,
    backoff_ms: f32,
    last_late_or_change: Option<Instant>,
    applied_deadline_ms: Option<f32>,
    last_deadline_at: Option<Instant>,
    // Since the last take_counters()
    gpu_late_frames: u32,
}

fn percentile(values: impl Iterator<Item = f32>, fraction: f32) -> Option<f32> {
    let mut sorted = values.collect::<Vec<_>>();
    if sorted.is_empty() {
        return None;
    }
    sorted.sort_by(f32::total_cmp);
    let index = ((sorted.len() - 1) as f32 * fraction).round() as usize;
    Some(sorted[index.min(sorted.len() - 1)])
}

fn ms(duration: Duration) -> f32 {
    duration.as_secs_f32() * 1000.0
}

impl LatePickupPlanner {
    pub fn new(announcement: LatePickupAnnouncement) -> Self {
        Self {
            announcement,
            work_ms: VecDeque::with_capacity(WORK_WINDOW),
            last_sample_at: None,
            late: VecDeque::with_capacity(LATE_WINDOW),
            backoff_ms: 0.0,
            last_late_or_change: None,
            applied_deadline_ms: None,
            last_deadline_at: None,
            gpu_late_frames: 0,
        }
    }

    pub fn announcement(&self) -> LatePickupAnnouncement {
        self.announcement
    }

    /// A new announcement (settings changed on the server): the measurements are kept
    pub fn set_announcement(&mut self, announcement: LatePickupAnnouncement) {
        self.announcement = announcement;
    }

    pub fn backoff_ms(&self) -> f32 {
        self.backoff_ms
    }

    pub fn samples(&self) -> usize {
        self.work_ms.len()
    }

    /// Pick -> GPU done, WORK_PERCENTILE of the window; a conservative fraction of a frame while
    /// the window is short
    pub fn work_estimate_ms(&self, frame_interval: Duration) -> f32 {
        let measured = percentile(self.work_ms.iter().copied(), WORK_PERCENTILE);
        let initial = ms(frame_interval) * INITIAL_WORK_FRACTION;
        match measured {
            Some(work) if self.work_ms.len() >= MIN_SAMPLES => work,
            Some(work) => work.max(initial),
            None => initial,
        }
    }

    pub fn is_ready(&self, now: Instant) -> bool {
        self.work_ms.len() >= MIN_SAMPLES
            && self
                .last_sample_at
                .is_some_and(|at| now.saturating_duration_since(at) <= STALE_AFTER)
    }

    /// One frame's GPU work finished. since_wake / since_pick: from the loop's wake-up / from the
    /// end of its wait for a stream frame to the GPU completion of the whole frame. deadline_mode:
    /// the frame was picked with a late pickup deadline (only those can make the planner back
    /// off). latch: latch_after_wake() for this frame.
    pub fn on_frame_done(
        &mut self,
        now: Instant,
        since_wake: Duration,
        since_pick: Duration,
        latch: Duration,
        deadline_mode: bool,
    ) -> Option<PlannerEvent> {
        if self.work_ms.len() == WORK_WINDOW {
            self.work_ms.pop_front();
        }
        self.work_ms.push_back(ms(since_pick));
        self.last_sample_at = Some(now);

        let gpu_late = since_wake > latch;
        if gpu_late {
            self.gpu_late_frames += 1;
        }
        if !deadline_mode {
            return None;
        }
        if self.late.len() == LATE_WINDOW {
            self.late.pop_front();
        }
        self.late.push_back(gpu_late);
        let last_change = *self.last_late_or_change.get_or_insert(now);
        if gpu_late {
            let late_frames = self.late.iter().filter(|late| **late).count();
            if late_frames > LATE_LIMIT {
                self.backoff_ms += BACKOFF_STEP_MS;
                self.late.clear();
                self.last_late_or_change = Some(now);
                return Some(PlannerEvent::BackedOff {
                    backoff_ms: self.backoff_ms,
                    late_frames,
                    window: LATE_WINDOW,
                });
            }
            self.last_late_or_change = Some(now);
        } else if self.backoff_ms > 0.0
            && now.saturating_duration_since(last_change) >= BACKOFF_DECAY_AFTER
        {
            self.backoff_ms = (self.backoff_ms - BACKOFF_DECAY_MS).max(0.0);
            self.last_late_or_change = Some(now);
            return Some(PlannerEvent::Recovered {
                backoff_ms: self.backoff_ms,
            });
        }
        None
    }

    /// The deadline after the wake-up for this loop (None: not ready, behave as without late
    /// pickup). Moves earlier at once, later at most DEADLINE_RISE_MS_PER_S.
    pub fn deadline_after_wake(
        &mut self,
        now: Instant,
        frame_interval: Duration,
        latch: Duration,
    ) -> Option<Duration> {
        if !self.is_ready(now) {
            // The last deadline is kept: after a pause (stream unused, a decoder restart) the
            // loop goes on from it instead of rising from the start again (the server's phase
            // still fits it)
            self.last_deadline_at = None;
            return None;
        }
        let frame_ms = ms(frame_interval);
        let wanted = (ms(latch)
            - self.work_estimate_ms(frame_interval)
            - self.announcement.safety_margin_ms.max(0.0)
            - self.backoff_ms)
            .clamp(0.0, frame_ms * MAX_DEADLINE_FRACTION);
        let applied = match (self.applied_deadline_ms, self.last_deadline_at) {
            // First deadline: about where the loop used to look (1 ms after the wake-up), rising
            // like any other
            (None, _) => wanted.min(frame_ms * 0.1),
            (Some(previous), Some(at)) if wanted > previous => {
                let dt = now.saturating_duration_since(at).as_secs_f32();
                wanted.min(previous + DEADLINE_RISE_MS_PER_S * dt)
            }
            // Ready again after a pause: no later than before it
            (Some(previous), None) => wanted.min(previous),
            _ => wanted,
        };
        self.applied_deadline_ms = Some(applied);
        self.last_deadline_at = Some(now);
        Some(Duration::from_secs_f32(applied / 1000.0))
    }

    /// Whether to take the newest ready frame now. newest_ready: None = no frame ready;
    /// Some(None) = ready, decode time unknown; Some(Some(t)) = ready, decoded at t.
    /// A frame decoded after "deadline - target - frame / 2" belongs to this loop (taken at once);
    /// an older one came after the previous loop's deadline: wait for a newer one until the
    /// deadline (taking it at once would keep every later frame one loop behind).
    pub fn pick_decision(
        now: Instant,
        deadline: Instant,
        newest_ready: Option<Option<Instant>>,
        frame_interval: Duration,
        target_margin_ms: f32,
    ) -> PickDecision {
        if now >= deadline {
            return PickDecision::Take;
        }
        match newest_ready {
            None => PickDecision::Wait,
            Some(None) => PickDecision::Take,
            Some(Some(decoded)) => {
                let margin_ms = signed_ms(deadline, decoded);
                if margin_ms <= target_margin_ms + ms(frame_interval) / 2.0 {
                    PickDecision::Take
                } else {
                    PickDecision::Wait
                }
            }
        }
    }

    /// The reported margin against the deadline, wrapped into
    /// (target - frame / 2, target + frame / 2]
    pub fn wrap_margin_ms(margin_ms: f32, frame_interval: Duration, target_margin_ms: f32) -> f32 {
        let frame_ms = ms(frame_interval);
        if frame_ms <= 0.0 {
            return margin_ms;
        }
        let mut margin = margin_ms;
        while margin > target_margin_ms + frame_ms / 2.0 {
            margin -= frame_ms;
        }
        while margin <= target_margin_ms - frame_ms / 2.0 {
            margin += frame_ms;
        }
        margin
    }

    /// GPU late frames since the last call
    pub fn take_gpu_late_frames(&mut self) -> u32 {
        std::mem::take(&mut self.gpu_late_frames)
    }
}

/// a - b in ms (negative when b is later)
pub fn signed_ms(a: Instant, b: Instant) -> f32 {
    if a >= b {
        ms(a.duration_since(b))
    } else {
        -ms(b.duration_since(a))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FRAME: Duration = Duration::from_micros(11_111);

    fn announcement() -> LatePickupAnnouncement {
        LatePickupAnnouncement {
            safety_margin_ms: 1.0,
            target_margin_ms: 2.0,
        }
    }

    fn latch() -> Duration {
        latch_after_wake(Platform::Pico4Ultra, FRAME)
    }

    // One simulated frame: GPU done `work` after the pick, the pick at `deadline` (or 1 ms after
    // the wake-up without a deadline). Returns (deadline ms, event).
    struct Sim {
        planner: LatePickupPlanner,
        t0: Instant,
        frame: u32,
    }

    impl Sim {
        fn new() -> Self {
            Self {
                planner: LatePickupPlanner::new(announcement()),
                t0: Instant::now(),
                frame: 0,
            }
        }

        fn step(&mut self, work_ms: f32) -> (Option<f32>, Option<PlannerEvent>) {
            let wake = self.t0 + FRAME * self.frame;
            self.frame += 1;
            let deadline = self.planner.deadline_after_wake(wake, FRAME, latch());
            let pick_after_wake = deadline.map_or(Duration::from_millis(1), |d| {
                d.max(Duration::from_millis(1))
            });
            let work = Duration::from_secs_f32(work_ms / 1000.0);
            let since_wake = pick_after_wake + work;
            let event = self.planner.on_frame_done(
                wake + since_wake,
                since_wake,
                work,
                latch(),
                deadline.is_some(),
            );
            (deadline.map(ms), event)
        }

        fn run(
            &mut self,
            frames: u32,
            work_ms: impl Fn(u32) -> f32,
        ) -> Vec<(Option<f32>, Option<PlannerEvent>)> {
            (0..frames)
                .map(|i| {
                    let work = work_ms(i);
                    self.step(work)
                })
                .collect()
        }
    }

    #[test]
    fn not_ready_until_enough_frames_were_measured() {
        let mut sim = Sim::new();
        let steps = sim.run(MIN_SAMPLES as u32, |_| 6.0);
        assert!(steps.iter().all(|(deadline, _)| deadline.is_none()));
        let (deadline, _) = sim.step(6.0);
        assert!(deadline.is_some());
    }

    #[test]
    fn stable_work_converges_to_latch_minus_work_minus_safety() {
        let mut sim = Sim::new();
        // 6 ms +- 0.3 ms of work: deadline = 12.11 - 6.3 - 1.0 = 4.8 ms, but at most half a frame
        // (5.56 ms); it rises at 0.5 ms/s, so give it 15 s
        let steps = sim.run(90 * 15, |i| 6.0 + 0.3 * ((i % 7) as f32 / 3.0 - 1.0));
        let (deadline, _) = steps.last().copied().unwrap();
        let deadline = deadline.unwrap();
        assert!((deadline - 4.81).abs() < 0.1, "{deadline}");
        // No GPU late frame, no back-off, the deadline never moved down
        assert_eq!(sim.planner.backoff_ms(), 0.0);
        assert!(steps.iter().all(|(_, event)| event.is_none()));
        let deadlines = steps.iter().filter_map(|(d, _)| *d).collect::<Vec<_>>();
        assert!(deadlines.windows(2).all(|w| w[1] >= w[0] - 1e-4));
        // Short work: the deadline is capped at half a frame
        let mut sim = Sim::new();
        let steps = sim.run(90 * 20, |_| 2.0);
        let deadline = steps.last().unwrap().0.unwrap();
        assert!((deadline - ms(FRAME) * MAX_DEADLINE_FRACTION).abs() < 1e-3);
    }

    #[test]
    fn a_single_spike_does_not_move_the_deadline_or_back_off() {
        let mut sim = Sim::new();
        sim.run(90 * 15, |_| 6.0);
        let before = sim.step(6.0).0.unwrap();
        // One frame of 9 ms (GPU late: 4.8 + 9 > 12.1)
        let (_, event) = sim.step(9.0);
        assert_eq!(event, None);
        let steps = sim.run(90, |_| 6.0);
        assert!(steps.iter().all(|(_, event)| event.is_none()));
        let after = steps.last().unwrap().0.unwrap();
        assert!((after - before).abs() < 0.05, "{before} -> {after}");
        assert_eq!(sim.planner.backoff_ms(), 0.0);
        assert_eq!(sim.planner.take_gpu_late_frames(), 1);
    }

    #[test]
    fn a_step_up_in_work_moves_the_deadline_earlier_quickly() {
        let mut sim = Sim::new();
        sim.run(90 * 15, |_| 5.0);
        let before = sim.step(5.0).0.unwrap();
        // Work jumps from 5 to 7 ms: once 5 % of the window has it (p95) the deadline follows;
        // the frames before that finish after the latch estimate (5.56 + 7 > 12.11) and make
        // the planner back off at the second one
        let steps = sim.run(30, |_| 7.0);
        let after = steps.last().unwrap().0.unwrap();
        assert!(before - after > 1.4, "{before} -> {after}");
        assert!(
            steps
                .iter()
                .any(|(_, e)| matches!(e, Some(PlannerEvent::BackedOff { .. })))
        );
        assert!(after + 7.0 + 1.0 <= ms(latch()) + 1e-3);
        // Step down: the deadline rises slowly (0.5 ms/s) once the window forgot the 7 ms
        let steps = sim.run(90 * 3, |_| 5.0);
        let risen = steps.last().unwrap().0.unwrap();
        assert!(
            risen > after && risen - after <= 0.5 * 1.2 + 0.05,
            "{after} -> {risen}"
        );
    }

    #[test]
    fn repeated_late_frames_back_off_and_recover() {
        let mut sim = Sim::new();
        sim.run(90 * 15, |_| 6.0);
        let normal = sim.step(6.0).0.unwrap();
        // The latch estimate is too optimistic for this frame source: every 20th frame finishes
        // 2 ms after it (5 % GPU late), without moving the 95th percentile much
        let steps = sim.run(90 * 3, |i| if i % 20 == 0 { 8.5 } else { 6.0 });
        let backoffs = steps
            .iter()
            .filter(|(_, e)| matches!(e, Some(PlannerEvent::BackedOff { .. })))
            .count();
        assert!(backoffs >= 1);
        let backed = steps.last().unwrap().0.unwrap();
        assert!(
            backed <= normal - BACKOFF_STEP_MS + 0.05,
            "{normal} -> {backed}"
        );
        assert!(sim.planner.backoff_ms() >= BACKOFF_STEP_MS);
        // With the earlier pickup the 8.5 ms frames are no longer late: no further back-off
        let steps = sim.run(90 * 5, |i| if i % 20 == 0 { 8.5 } else { 6.0 });
        let late = steps
            .iter()
            .filter(|(_, e)| matches!(e, Some(PlannerEvent::BackedOff { .. })))
            .count();
        assert!(late <= 1);
        // Quiet: the back-off decays (0.25 ms per 10 s) back to 0
        let held = sim.planner.backoff_ms();
        let steps = sim.run(90 * 10 * 10, |_| 6.0);
        assert!(
            steps
                .iter()
                .any(|(_, e)| matches!(e, Some(PlannerEvent::Recovered { .. })))
        );
        assert!(sim.planner.backoff_ms() < held);
        assert_eq!(sim.planner.backoff_ms(), 0.0);
    }

    #[test]
    fn stale_measurements_make_the_planner_unready() {
        let mut sim = Sim::new();
        sim.run(90 * 2, |_| 6.0);
        let wake = sim.t0 + FRAME * sim.frame;
        assert!(
            sim.planner
                .deadline_after_wake(wake, FRAME, latch())
                .is_some()
        );
        // No fence result for more than STALE_AFTER: back to the old behaviour
        let later = wake + STALE_AFTER + Duration::from_millis(50);
        assert!(
            sim.planner
                .deadline_after_wake(later, FRAME, latch())
                .is_none()
        );
        // Measured again: the deadline goes on from where it was, not from the start
        let mut sim = Sim::new();
        sim.run(90 * 15, |_| 6.0);
        let before = sim.step(6.0).0.unwrap();
        sim.t0 += Duration::from_secs(5);
        assert_eq!(sim.step(6.0).0, None);
        let after = sim.step(6.0).0.unwrap();
        assert!((after - before).abs() < 0.01, "{before} -> {after}");
    }

    #[test]
    fn pick_takes_this_loops_frame_and_waits_past_a_stale_one() {
        let t0 = Instant::now();
        let deadline = t0 + Duration::from_millis(15);
        let at = |ms_after: f32| t0 + Duration::from_secs_f32((ms_after + 10.0) / 1000.0);
        // Nothing ready: wait; at or past the deadline: take (whatever is there)
        assert_eq!(
            LatePickupPlanner::pick_decision(at(1.0), deadline, None, FRAME, 2.0),
            PickDecision::Wait
        );
        assert_eq!(
            LatePickupPlanner::pick_decision(deadline, deadline, None, FRAME, 2.0),
            PickDecision::Take
        );
        // Decoded 0.5 ms ago (this loop's): take at once
        assert_eq!(
            LatePickupPlanner::pick_decision(at(2.0), deadline, Some(Some(at(1.5))), FRAME, 2.0),
            PickDecision::Take
        );
        // Decoded 9 ms before the deadline (just after the previous loop's deadline): wait for
        // a newer one
        assert_eq!(
            LatePickupPlanner::pick_decision(at(1.0), deadline, Some(Some(at(-4.0))), FRAME, 2.0),
            PickDecision::Wait
        );
        // Unknown decode time: take
        assert_eq!(
            LatePickupPlanner::pick_decision(at(1.0), deadline, Some(None), FRAME, 2.0),
            PickDecision::Take
        );
    }

    #[test]
    fn margins_wrap_around_the_target() {
        let wrap = |m| LatePickupPlanner::wrap_margin_ms(m, FRAME, 2.0);
        assert_eq!(wrap(2.0), 2.0);
        assert_eq!(wrap(0.0), 0.0);
        assert_eq!(wrap(7.5), 7.5);
        // 10.6 ms before the deadline = 0.5 ms after the previous loop's deadline
        assert!((wrap(10.611) - (-0.5)).abs() < 1e-3);
        assert!((wrap(-3.0) - (-3.0)).abs() < 1e-3);
        assert!((wrap(-3.0 - 11.111) - (-3.0)).abs() < 1e-3);
        // Further out than half a frame before the target: the nearer slot
        assert!((wrap(-4.0) - 7.111).abs() < 1e-3);
    }

    #[test]
    fn latch_estimate_per_model() {
        assert_eq!(
            latch_after_wake(Platform::Pico4Ultra, FRAME),
            FRAME + Duration::from_millis(1)
        );
        assert!(latch_after_wake(Platform::Pico4Enterprise, FRAME) < FRAME);
    }
}
