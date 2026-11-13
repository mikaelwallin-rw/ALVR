//! Phase sync (fork), client side: per render loop, how the decoded main stream frame met the loop,
//! summed up in a PhaseSyncReport every REPORT_INTERVAL for the server's controller
//! (server_core phase_sync.rs). Active only while the server announces phase sync
//! (RealTimeConfig.ext_str EXT_PHASE_SYNC).

use alvr_packets::{PHASE_SYNC_REFERENCE_POLL_START, PhaseSyncReport};
use std::time::{Duration, Instant};

pub const REPORT_INTERVAL: Duration = Duration::from_millis(500);

/// What one render loop got
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LoopOutcome {
    /// A new frame. margin_ms = the loop's reference (poll start, or the late pickup deadline,
    /// wrapped) minus frame decoded (None when the decode time is unknown); ready_at_poll = it was already decoded when the loop started looking;
    /// dropped = older ready frames dropped for it
    Picked {
        margin_ms: Option<f32>,
        ready_at_poll: bool,
        dropped: u32,
    },
    /// No new frame within the wait: the previous frame is shown again
    NoFrame,
    /// The stream is not rendered in this loop (stream unused)
    Skipped,
}

pub struct PhaseSyncReporter {
    enabled: bool,
    sequence: u32,
    window_start: Option<Instant>,
    frame_interval: Duration,
    prev_loop_had_frame: bool,
    loops: u32,
    frames: u32,
    skipped: u32,
    missed: u32,
    late: u32,
    dropped: u32,
    margins_ms: Vec<f32>,
    // Late pickup (v2 fields): the last loop's reference, deadlines of this window, the last
    // estimates, GPU late frames of this window
    reference: u32,
    deadline_sum_ms: f32,
    deadline_count: u32,
    work_estimate_ms: f32,
    backoff_ms: f32,
    gpu_late: u32,
}

impl Default for PhaseSyncReporter {
    fn default() -> Self {
        Self {
            enabled: false,
            sequence: 0,
            window_start: None,
            frame_interval: Duration::from_secs_f32(1.0 / 90.0),
            prev_loop_had_frame: true,
            loops: 0,
            frames: 0,
            skipped: 0,
            missed: 0,
            late: 0,
            dropped: 0,
            margins_ms: Vec::with_capacity(128),
            reference: PHASE_SYNC_REFERENCE_POLL_START,
            deadline_sum_ms: 0.0,
            deadline_count: 0,
            work_estimate_ms: 0.0,
            backoff_ms: 0.0,
            gpu_late: 0,
        }
    }
}

fn percentile(sorted: &[f32], fraction: f32) -> f32 {
    if sorted.is_empty() {
        return 0.0;
    }
    let index = ((sorted.len() - 1) as f32 * fraction).round() as usize;
    sorted[index.min(sorted.len() - 1)]
}

impl PhaseSyncReporter {
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Enabling or disabling starts a new window (nothing of an older one is reported)
    pub fn set_enabled(&mut self, enabled: bool) {
        if enabled != self.enabled {
            let sequence = self.sequence;
            *self = Self {
                enabled,
                sequence,
                ..Default::default()
            };
        }
    }

    /// Late pickup state of the coming loop (call before on_loop): its margin reference, its
    /// deadline after the wake-up (deadline reference only), the planner's estimates, and GPU late
    /// frames measured since the last call
    pub fn note_late_pickup(
        &mut self,
        reference: u32,
        deadline_ms: Option<f32>,
        work_estimate_ms: f32,
        backoff_ms: f32,
        gpu_late: u32,
    ) {
        if !self.enabled {
            return;
        }
        self.reference = reference;
        if let Some(deadline) = deadline_ms.filter(|d| d.is_finite()) {
            self.deadline_sum_ms += deadline;
            self.deadline_count += 1;
        }
        if work_estimate_ms.is_finite() {
            self.work_estimate_ms = work_estimate_ms;
        }
        if backoff_ms.is_finite() {
            self.backoff_ms = backoff_ms;
        }
        self.gpu_late += gpu_late;
    }

    /// One render loop. Returns a report when one is due (only while enabled).
    pub fn on_loop(
        &mut self,
        now: Instant,
        outcome: LoopOutcome,
        frame_interval: Option<Duration>,
    ) -> Option<PhaseSyncReport> {
        if !self.enabled {
            return None;
        }
        if let Some(interval) = frame_interval.filter(|i| !i.is_zero()) {
            self.frame_interval = interval;
        }
        let window_start = *self.window_start.get_or_insert(now);

        self.loops += 1;
        match outcome {
            LoopOutcome::Picked {
                margin_ms,
                ready_at_poll,
                dropped,
            } => {
                self.dropped += dropped;
                if ready_at_poll && !self.prev_loop_had_frame {
                    self.missed += 1;
                }
                // frames = margin samples: a frame without a known decode time is not counted
                // (the server holds the phase when too few samples come)
                if let Some(margin) = margin_ms.filter(|m| m.is_finite()) {
                    self.frames += 1;
                    if margin < 0.0 {
                        self.late += 1;
                    }
                    self.margins_ms.push(margin);
                }
                self.prev_loop_had_frame = true;
            }
            LoopOutcome::NoFrame => self.prev_loop_had_frame = false,
            LoopOutcome::Skipped => {
                self.skipped += 1;
                self.prev_loop_had_frame = true;
            }
        }

        let window = now.saturating_duration_since(window_start);
        if window < REPORT_INTERVAL {
            return None;
        }

        self.margins_ms.sort_by(f32::total_cmp);
        let mean = if self.margins_ms.is_empty() {
            0.0
        } else {
            self.margins_ms.iter().sum::<f32>() / self.margins_ms.len() as f32
        };
        self.sequence = self.sequence.wrapping_add(1);
        let report = PhaseSyncReport {
            sequence: self.sequence,
            window_us: window.as_micros().min(u32::MAX as u128) as u32,
            frame_interval_us: self.frame_interval.as_micros().min(u32::MAX as u128) as u32,
            loops: self.loops,
            frames: self.frames,
            skipped_loops: self.skipped,
            missed_slots: self.missed,
            late_frames: self.late,
            dropped_frames: self.dropped,
            margin_mean_ms: mean,
            margin_p10_ms: percentile(&self.margins_ms, 0.1),
            margin_p50_ms: percentile(&self.margins_ms, 0.5),
            margin_p90_ms: percentile(&self.margins_ms, 0.9),
            reference: self.reference,
            gpu_late_frames: self.gpu_late,
            deadline_mean_ms: if self.deadline_count > 0 {
                self.deadline_sum_ms / self.deadline_count as f32
            } else {
                0.0
            },
            work_estimate_ms: self.work_estimate_ms,
            backoff_ms: self.backoff_ms,
        };

        self.window_start = Some(now);
        self.loops = 0;
        self.frames = 0;
        self.skipped = 0;
        self.missed = 0;
        self.late = 0;
        self.dropped = 0;
        self.margins_ms.clear();
        self.deadline_sum_ms = 0.0;
        self.deadline_count = 0;
        self.gpu_late = 0;

        Some(report)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn picked(margin: f32, ready_at_poll: bool) -> LoopOutcome {
        LoopOutcome::Picked {
            margin_ms: Some(margin),
            ready_at_poll,
            dropped: 0,
        }
    }

    #[test]
    fn silent_while_disabled() {
        let mut r = PhaseSyncReporter::default();
        let t0 = Instant::now();
        for i in 0..200 {
            let now = t0 + Duration::from_millis(11 * i);
            assert!(r.on_loop(now, picked(2.0, true), None).is_none());
        }
    }

    #[test]
    fn reports_every_500_ms_with_counts_and_percentiles() {
        let mut r = PhaseSyncReporter::default();
        r.set_enabled(true);
        let t0 = Instant::now();
        let period = Duration::from_micros(11_111);
        let mut reports = vec![];
        for i in 0..100u32 {
            let now = t0 + period * i;
            let outcome = match i {
                // loop 10 gets nothing, loop 11 finds its frame waiting: a missed slot
                10 => LoopOutcome::NoFrame,
                11 => LoopOutcome::Picked {
                    margin_ms: Some(1.5),
                    ready_at_poll: true,
                    dropped: 1,
                },
                // the loop waits for these
                20..=24 => picked(-1.0, false),
                _ => picked(2.0 + (i % 3) as f32 * 0.5, true),
            };
            if let Some(report) = r.on_loop(now, outcome, Some(period)) {
                reports.push(report);
            }
        }
        assert_eq!(reports.len(), 2);
        let first = reports[0];
        assert_eq!(first.sequence, 1);
        assert_eq!(first.frame_interval_us, 11_111);
        // loops 0..=46 (46 * 11.111 ms is the first time >= 500 ms)
        assert_eq!(first.loops, 47);
        assert_eq!(first.frames, 46);
        assert_eq!(first.missed_slots, 1);
        assert_eq!(first.late_frames, 5);
        assert_eq!(first.dropped_frames, 1);
        assert_eq!(first.skipped_loops, 0);
        assert_eq!(first.window_us, 511_106);
        // 46 samples: 5 x -1.0, one 1.5, the rest 2.0 to 3.0; p10 = sorted[round(4.5)]
        assert_eq!(first.margin_p10_ms, 1.5);
        assert_eq!(first.margin_p50_ms, 2.5);
        assert_eq!(first.margin_p90_ms, 3.0);
        assert!(first.margin_mean_ms > 1.5 && first.margin_mean_ms < 2.5);
        assert_eq!(reports[1].sequence, 2);
        assert_eq!(reports[1].missed_slots, 0);
        // Without late pickup: the v1 meaning
        assert_eq!(first.reference, PHASE_SYNC_REFERENCE_POLL_START);
        assert_eq!(first.deadline_mean_ms, 0.0);
    }

    #[test]
    fn late_pickup_fields_are_reported_per_window() {
        let mut r = PhaseSyncReporter::default();
        r.set_enabled(true);
        let t0 = Instant::now();
        let period = Duration::from_micros(11_111);
        let mut reports = vec![];
        for i in 0..100u32 {
            r.note_late_pickup(
                alvr_packets::PHASE_SYNC_REFERENCE_DEADLINE,
                Some(if i % 2 == 0 { 4.0 } else { 5.0 }),
                6.5,
                0.5,
                u32::from(i == 3),
            );
            if let Some(report) = r.on_loop(t0 + period * i, picked(2.0, false), Some(period)) {
                reports.push(report);
            }
        }
        assert_eq!(reports.len(), 2);
        assert_eq!(
            reports[0].reference,
            alvr_packets::PHASE_SYNC_REFERENCE_DEADLINE
        );
        assert!((reports[0].deadline_mean_ms - 4.5).abs() < 0.05);
        assert_eq!(reports[0].work_estimate_ms, 6.5);
        assert_eq!(reports[0].backoff_ms, 0.5);
        assert_eq!(reports[0].gpu_late_frames, 1);
        assert_eq!(reports[1].gpu_late_frames, 0);
        // Waiting for a frame is normal with the deadline: those are not late (positive margin)
        assert_eq!(reports[0].late_frames, 0);
    }

    #[test]
    fn skipped_loops_are_counted_and_toggling_restarts_the_window() {
        let mut r = PhaseSyncReporter::default();
        r.set_enabled(true);
        let t0 = Instant::now();
        let period = Duration::from_micros(11_111);
        for i in 0..20u32 {
            assert!(
                r.on_loop(t0 + period * i, LoopOutcome::Skipped, None)
                    .is_none()
            );
        }
        // Disable + enable: the 20 skipped loops are forgotten
        r.set_enabled(false);
        r.set_enabled(true);
        let mut report = None;
        for i in 0..60u32 {
            let outcome = if i < 5 {
                LoopOutcome::Skipped
            } else {
                picked(2.0, true)
            };
            report = report.or(r.on_loop(t0 + period * (100 + i), outcome, None));
        }
        let report = report.unwrap();
        assert_eq!(report.skipped_loops, 5);
        assert_eq!(report.loops, 47);
        // after skipped loops a waiting frame is not a missed slot
        assert_eq!(report.missed_slots, 0);
        // Unknown decode times are no samples
        let mut r = PhaseSyncReporter::default();
        r.set_enabled(true);
        let mut report = None;
        for i in 0..50u32 {
            report = report.or(r.on_loop(
                t0 + period * i,
                LoopOutcome::Picked {
                    margin_ms: None,
                    ready_at_poll: true,
                    dropped: 0,
                },
                None,
            ));
        }
        let report = report.unwrap();
        assert_eq!(report.frames, 0);
        assert_eq!(report.loops, 47);
    }
}
