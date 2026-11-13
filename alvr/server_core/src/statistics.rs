use crate::phase_sync::{HoldReason, PhaseSample, PhaseSyncController};
use alvr_common::{HEAD_ID, SlidingWindowAverage, info};
use alvr_events::{BitrateDirectives, EventType, GraphStatistics, StatisticsSummary};
use alvr_packets::{ClientStatistics, PhaseSyncReport};
use std::{
    collections::{HashMap, VecDeque},
    time::{Duration, Instant},
};

const FULL_REPORT_INTERVAL: Duration = Duration::from_millis(500);
/// Phase sync status line in the session log
const PHASE_SYNC_LOG_INTERVAL: Duration = Duration::from_secs(10);
const EPS_INTERVAL: Duration = Duration::from_micros(1);

pub struct HistoryFrame {
    target_timestamp: Duration,
    tracking_received: Instant,
    frame_present: Instant,
    frame_composed: Instant,
    frame_encoded: Instant,
    video_packet_bytes: usize,
    total_pipeline_latency: Duration,
}

impl Default for HistoryFrame {
    fn default() -> Self {
        let now = Instant::now();
        Self {
            target_timestamp: Duration::ZERO,
            tracking_received: now,
            frame_present: now,
            frame_composed: now,
            frame_encoded: now,
            video_packet_bytes: 0,
            total_pipeline_latency: Duration::ZERO,
        }
    }
}

#[derive(Default, Clone)]
struct BatteryData {
    gauge_value: f32,
    is_plugged: bool,
}

pub struct StatisticsManager {
    history_buffer: VecDeque<HistoryFrame>,
    max_history_size: usize,
    last_full_report_instant: Instant,
    last_frame_present_instant: Instant,
    last_frame_present_interval: Duration,
    video_packets_total: usize,
    video_packets_partial_sum: usize,
    video_bytes_total: usize,
    video_bytes_partial_sum: usize,
    battery_gauges: HashMap<u64, BatteryData>,
    steamvr_pipeline_latency: Duration,
    motion_to_photon_latency_average: SlidingWindowAverage<Duration>,
    last_vsync_time: Instant,
    frame_interval: Duration,
    last_throughput_directives: BitrateDirectives,
    // Fork: steers the virtual vsync phase from the client's reports (None = upstream behaviour)
    phase_sync: Option<PhaseSyncState>,
}

struct PhaseSyncState {
    controller: PhaseSyncController,
    origin: Instant,
    presents_since_report: u32,
    reports_received: u64,
    last_hold: Option<HoldReason>,
    last_log: Instant,
    // Sums since the last status line
    log_reports: u32,
    log_loops: u32,
    log_frames: u32,
    log_late: u32,
    log_missed: u32,
    log_dropped: u32,
    log_skipped: u32,
    log_margin_mean_sum: f32,
    // Late pickup (PSYN v2, reference DEADLINE): GPU late frames since the status line
    log_gpu_late: u32,
    // The margin reference of the last report (logged on change)
    last_reference: Option<u32>,
    last_report: PhaseSyncReport,
}

impl PhaseSyncState {
    fn new(target_margin_ms: f32) -> Self {
        Self {
            controller: PhaseSyncController::new(target_margin_ms),
            origin: Instant::now(),
            presents_since_report: 0,
            reports_received: 0,
            last_hold: None,
            last_log: Instant::now(),
            log_reports: 0,
            log_loops: 0,
            log_frames: 0,
            log_late: 0,
            log_missed: 0,
            log_dropped: 0,
            log_skipped: 0,
            log_margin_mean_sum: 0.0,
            log_gpu_late: 0,
            last_reference: None,
            last_report: PhaseSyncReport::default(),
        }
    }

    fn now_s(&self) -> f64 {
        self.origin.elapsed().as_secs_f64()
    }
}

impl StatisticsManager {
    // history size used to calculate average total pipeline latency
    /// phase_sync_target_ms: Some(target margin) to steer the virtual vsync from the client's
    /// phase sync reports (only once reports arrive; without them the grid is the nominal one)
    pub fn new(
        max_history_size: usize,
        nominal_server_frame_interval: Duration,
        steamvr_pipeline_frames: f32,
        phase_sync_target_ms: Option<f32>,
    ) -> Self {
        Self {
            history_buffer: VecDeque::new(),
            max_history_size,
            last_full_report_instant: Instant::now(),
            last_frame_present_instant: Instant::now(),
            last_frame_present_interval: Duration::ZERO,
            video_packets_total: 0,
            video_packets_partial_sum: 0,
            video_bytes_total: 0,
            video_bytes_partial_sum: 0,
            battery_gauges: HashMap::new(),
            steamvr_pipeline_latency: Duration::from_secs_f32(
                steamvr_pipeline_frames * nominal_server_frame_interval.as_secs_f32(),
            ),
            motion_to_photon_latency_average: SlidingWindowAverage::new(
                Duration::ZERO,
                max_history_size,
            ),
            last_vsync_time: Instant::now(),
            frame_interval: nominal_server_frame_interval,
            last_throughput_directives: BitrateDirectives::default(),
            phase_sync: phase_sync_target_ms.map(PhaseSyncState::new),
        }
    }

    pub fn report_tracking_received(&mut self, target_timestamp: Duration) {
        if !self
            .history_buffer
            .iter()
            .any(|frame| frame.target_timestamp == target_timestamp)
        {
            self.history_buffer.push_front(HistoryFrame {
                target_timestamp,
                tracking_received: Instant::now(),
                ..Default::default()
            });
        }

        if self.history_buffer.len() > self.max_history_size {
            self.history_buffer.pop_back();
        }
    }

    pub fn report_frame_present(&mut self, target_timestamp: Duration, offset: Duration) {
        if let Some(frame) = self
            .history_buffer
            .iter_mut()
            .find(|frame| frame.target_timestamp == target_timestamp)
        {
            let now = Instant::now() - offset;

            self.last_frame_present_interval =
                now.saturating_duration_since(self.last_frame_present_instant);
            self.last_frame_present_instant = now;

            frame.frame_present = now;
        }
        if let Some(phase_sync) = &mut self.phase_sync {
            phase_sync.presents_since_report += 1;
        }
    }

    pub fn report_frame_composed(&mut self, target_timestamp: Duration, offset: Duration) {
        if let Some(frame) = self
            .history_buffer
            .iter_mut()
            .find(|frame| frame.target_timestamp == target_timestamp)
        {
            frame.frame_composed = Instant::now() - offset;
        }
    }

    // returns encoding interval
    pub fn report_frame_encoded(
        &mut self,
        target_timestamp: Duration,
        bytes_count: usize,
    ) -> Duration {
        self.video_packets_total += 1;
        self.video_packets_partial_sum += 1;
        self.video_bytes_total += bytes_count;
        self.video_bytes_partial_sum += bytes_count;

        if let Some(frame) = self
            .history_buffer
            .iter_mut()
            .find(|frame| frame.target_timestamp == target_timestamp)
        {
            frame.frame_encoded = Instant::now();

            frame.video_packet_bytes = bytes_count;

            frame
                .frame_encoded
                .saturating_duration_since(frame.frame_composed)
        } else {
            Duration::ZERO
        }
    }

    pub fn report_battery(&mut self, device_id: u64, gauge_value: f32, is_plugged: bool) {
        *self.battery_gauges.entry(device_id).or_default() = BatteryData {
            gauge_value,
            is_plugged,
        };
    }

    pub fn report_throughput_stats(&mut self, stats: BitrateDirectives) {
        self.last_throughput_directives = stats;
    }

    // Called every frame. Some statistics are reported once every frame
    // Returns (network latency, game time latency)
    pub fn report_statistics(&mut self, client_stats: ClientStatistics) -> (Duration, Duration) {
        self.motion_to_photon_latency_average
            .submit_sample(client_stats.total_pipeline_latency);

        if let Some(frame) = self
            .history_buffer
            .iter_mut()
            .find(|frame| frame.target_timestamp == client_stats.target_timestamp)
        {
            frame.total_pipeline_latency = client_stats.total_pipeline_latency;

            let game_time_latency = frame
                .frame_present
                .saturating_duration_since(frame.tracking_received);

            let server_compositor_latency = frame
                .frame_composed
                .saturating_duration_since(frame.frame_present);

            let encoder_latency = frame
                .frame_encoded
                .saturating_duration_since(frame.frame_composed);

            // The network latency cannot be estiamed directly. It is what's left of the total
            // latency after subtracting all other latency intervals. In particular it contains the
            // transport latency of the tracking packet and the interval between the first video
            // packet is sent and the last video packet is received for a specific frame.
            // For safety, use saturating_sub to avoid a crash if for some reason the network
            // latency is miscalculated as negative.
            let network_latency = frame.total_pipeline_latency.saturating_sub(
                game_time_latency
                    + server_compositor_latency
                    + encoder_latency
                    + client_stats.video_decode
                    + client_stats.video_decoder_queue
                    + client_stats.rendering
                    + client_stats.vsync_queue,
            );

            let client_fps =
                1.0 / Duration::max(client_stats.frame_interval, EPS_INTERVAL).as_secs_f32();
            let server_fps =
                1.0 / Duration::max(self.last_frame_present_interval, EPS_INTERVAL).as_secs_f32();

            if self.last_full_report_instant + FULL_REPORT_INTERVAL < Instant::now() {
                self.last_full_report_instant += FULL_REPORT_INTERVAL;

                let interval_secs = FULL_REPORT_INTERVAL.as_secs_f32();

                alvr_events::send_event(EventType::StatisticsSummary(StatisticsSummary {
                    video_packets_total: self.video_packets_total,
                    video_packets_per_sec: (self.video_packets_partial_sum as f32 / interval_secs)
                        as _,
                    video_mbytes_total: (self.video_bytes_total as f32 / 1e6) as usize,
                    video_mbits_per_sec: self.video_bytes_partial_sum as f32 * 8.
                        / 1e6
                        / interval_secs,
                    total_latency_ms: client_stats.total_pipeline_latency.as_secs_f32() * 1000.,
                    network_latency_ms: network_latency.as_secs_f32() * 1000.,
                    encode_latency_ms: encoder_latency.as_secs_f32() * 1000.,
                    decode_latency_ms: client_stats.video_decode.as_secs_f32() * 1000.,
                    client_fps: client_fps as _,
                    server_fps: server_fps as _,
                    battery_hmd: (self
                        .battery_gauges
                        .get(&HEAD_ID)
                        .cloned()
                        .unwrap_or_default()
                        .gauge_value
                        * 100.) as u32,
                    hmd_plugged: self
                        .battery_gauges
                        .get(&HEAD_ID)
                        .cloned()
                        .unwrap_or_default()
                        .is_plugged,
                }));

                self.video_packets_partial_sum = 0;
                self.video_bytes_partial_sum = 0;
            }

            let packet_bits = frame.video_packet_bytes as f32 * 8.0;
            let throughput_bps =
                packet_bits / Duration::max(network_latency, EPS_INTERVAL).as_secs_f32();
            let bitrate_bps = packet_bits
                / Duration::max(self.last_frame_present_interval, EPS_INTERVAL).as_secs_f32();

            // todo: use target timestamp in nanoseconds. the dashboard needs to use the first
            // timestamp as the graph time origin.
            alvr_events::send_event(EventType::GraphStatistics(GraphStatistics {
                total_pipeline_latency_s: client_stats.total_pipeline_latency.as_secs_f32(),
                game_time_s: game_time_latency.as_secs_f32(),
                server_compositor_s: server_compositor_latency.as_secs_f32(),
                encoder_s: encoder_latency.as_secs_f32(),
                network_s: network_latency.as_secs_f32(),
                decoder_s: client_stats.video_decode.as_secs_f32(),
                decoder_queue_s: client_stats.video_decoder_queue.as_secs_f32(),
                client_compositor_s: client_stats.rendering.as_secs_f32(),
                vsync_queue_s: client_stats.vsync_queue.as_secs_f32(),
                client_fps,
                server_fps,
                bitrate_directives: self.last_throughput_directives.clone(),
                throughput_bps,
                bitrate_bps,
            }));

            (network_latency, game_time_latency)
        } else {
            (Duration::ZERO, Duration::ZERO)
        }
    }

    pub fn motion_to_photon_latency_average(&self) -> Duration {
        self.motion_to_photon_latency_average.get_average()
    }

    pub fn tracker_pose_time_offset(&self) -> Duration {
        // This is the opposite of the client's StatisticsManager::tracker_prediction_offset().
        self.steamvr_pipeline_latency
    }

    /// Fork: a client phase sync report (ReservedBuffer 'PSYN'). Ignored when phase sync is off.
    pub fn report_phase_sync(&mut self, report: PhaseSyncReport) {
        let Some(state) = &mut self.phase_sync else {
            return;
        };
        let now_s = state.now_s();
        let sample = PhaseSample {
            window_s: report.window_us as f32 / 1e6,
            client_frame_interval_s: report.frame_interval_us as f32 / 1e6,
            loops: report.loops,
            frames: report.frames,
            skipped_loops: report.skipped_loops,
            missed_slots: report.missed_slots,
            margin_p50_ms: report.margin_p50_ms,
            server_frames: state.presents_since_report,
        };
        state.presents_since_report = 0;
        state.reports_received += 1;
        if state.reports_received == 1 {
            info!(
                "Phase sync: client reports arriving, target margin {:.1} ms, margin now {:.1} ms (median)",
                state.controller.effective_target_ms(),
                report.margin_p50_ms
            );
        }

        if state.last_reference != Some(report.reference) {
            // The client moved its margin reference (late pickup on / off, or warming up): the
            // controller sees a step in the margin and follows it at its rate limit
            info!(
                "Phase sync: client margins now against {}",
                if report.reference == alvr_packets::PHASE_SYNC_REFERENCE_DEADLINE {
                    format!(
                        "its late pickup deadline (wake + {:.1} ms, work estimate {:.1} ms, back-off {:.1} ms)",
                        report.deadline_mean_ms, report.work_estimate_ms, report.backoff_ms
                    )
                } else {
                    "its poll start".to_string()
                }
            );
            state.last_reference = Some(report.reference);
        }

        let update = state.controller.update(now_s, &sample);
        if update.hold != state.last_hold {
            match update.hold {
                Some(reason) => info!("Phase sync: holding the phase ({reason:?})"),
                None => info!("Phase sync: steering again"),
            }
            state.last_hold = update.hold;
        }
        match update.lock_changed {
            Some(true) => info!(
                "Phase sync: LOCKED, margin {:.1} ms (median; target {:.1}), clock correction {:+.0} ppm",
                report.margin_p50_ms,
                state.controller.effective_target_ms(),
                state.controller.freq_ms_per_s() * 1000.0
            ),
            Some(false) => info!(
                "Phase sync: lock lost, margin {:.1} ms (median; target {:.1})",
                report.margin_p50_ms,
                state.controller.effective_target_ms()
            ),
            None => (),
        }
        if update.extra_changed {
            info!(
                "Phase sync: target margin now {:.1} ms (back-off {:.1} ms)",
                state.controller.effective_target_ms(),
                state.controller.extra_margin_ms(),
            );
        }

        state.log_reports += 1;
        state.log_loops += report.loops;
        state.log_frames += report.frames;
        state.log_late += report.late_frames;
        state.log_missed += report.missed_slots;
        state.log_dropped += report.dropped_frames;
        state.log_skipped += report.skipped_loops;
        state.log_margin_mean_sum += report.margin_mean_ms;
        state.log_gpu_late += report.gpu_late_frames;
        state.last_report = report;
        if state.last_log.elapsed() >= PHASE_SYNC_LOG_INTERVAL {
            let r = &state.last_report;
            // With the late pickup reference, "late" frames arrived after their loop's deadline
            // (the loop waiting for a frame is normal then); "GPU late" frames finished their
            // GPU work after the client's estimated compositor latch
            let late_pickup = if r.reference == alvr_packets::PHASE_SYNC_REFERENCE_DEADLINE {
                format!(
                    "; late pickup: deadline wake + {:.1} ms, work {:.1} ms, back-off {:.1} ms, GPU late {}",
                    r.deadline_mean_ms, r.work_estimate_ms, r.backoff_ms, state.log_gpu_late
                )
            } else {
                String::new()
            };
            info!(
                "Phase sync: margin p10/p50/p90 {:.1}/{:.1}/{:.1} ms, mean over {} reports {:.1} ms, target {:.1}, {}; rate {:+.2} ms/s, clock {:+.0} ppm; loops {}, frames {}, {} {}, missed {}, dropped {}, skipped {}{}",
                r.margin_p10_ms,
                r.margin_p50_ms,
                r.margin_p90_ms,
                state.log_reports,
                state.log_margin_mean_sum / state.log_reports.max(1) as f32,
                state.controller.effective_target_ms(),
                if state.controller.is_locked() {
                    "locked"
                } else {
                    "not locked"
                },
                state.controller.rate_at(now_s),
                state.controller.freq_ms_per_s() * 1000.0,
                state.log_loops,
                state.log_frames,
                if r.reference == alvr_packets::PHASE_SYNC_REFERENCE_DEADLINE {
                    "late"
                } else {
                    "waited for"
                },
                state.log_late,
                state.log_missed,
                state.log_dropped,
                state.log_skipped,
                late_pickup,
            );
            state.last_log = Instant::now();
            state.log_reports = 0;
            state.log_loops = 0;
            state.log_frames = 0;
            state.log_late = 0;
            state.log_missed = 0;
            state.log_dropped = 0;
            state.log_skipped = 0;
            state.log_margin_mean_sum = 0.0;
            state.log_gpu_late = 0;
        }
    }

    // NB: this call is non-blocking, waiting should be done externally
    pub fn duration_until_next_vsync(&mut self) -> Duration {
        let now = Instant::now();

        // Fork phase sync: the period is nominal * (1 + rate), |rate| <= 0.1 %; exactly the
        // nominal period while phase sync is off or before the first client report
        let frame_interval = match &self.phase_sync {
            Some(state) if state.reports_received > 0 => Duration::from_secs_f64(
                state
                    .controller
                    .frame_interval_s(self.frame_interval.as_secs_f64(), state.now_s()),
            ),
            _ => self.frame_interval,
        };

        // update the last vsync if it's too old
        while self.last_vsync_time + frame_interval < now {
            self.last_vsync_time += frame_interval;
        }

        (self.last_vsync_time + frame_interval).saturating_duration_since(now)
    }
}
