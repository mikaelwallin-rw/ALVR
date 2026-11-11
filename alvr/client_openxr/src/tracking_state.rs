// HEAD TRACKING CONTINUITY ("tracking epoch") for overlay apps that keep poses in STAGE.
//
// Measured on the PICO 4 Ultra Enterprise (2026-11-11, tracking_diag.rs logs): STAGE is rigidly
// tied to LOCAL, and its horizontal origin and yaw are RE-CREATED AT THE HEAD whenever head
// position tracking is lost and comes back (sleep longer than ~2 min: tracking stops ~120 s after
// the session goes IDLE; app restart; reboot). Short sleeps and awake pauses keep tracking and the
// origin. The persistent anchor and the stage identity do not see this (the anchor moves with the
// origin). Rule for consumers: "position tracking was lost at any time since the poses were
// measured => stored STAGE poses are invalid".
//
// The client's epoch starts at 1 per process (a restarted process cannot prove continuity:
// nothing is persisted) and is incremented when (EpochMachine::on_sample and the event hooks):
//  1. TRACKING_LOST: a VIEW-in-STAGE sample without POSITION_TRACKED after one with it (also
//     while the session is IDLE: the sampler keeps sampling at 2 Hz while asleep);
//  2. TRACKING_REGAINED: POSITION_TRACKED back after a sample without it;
//  3. REANCHOR_JUMP: the head pose deviates from the constant-velocity prediction of the two
//     previous samples (<= 100 ms apart) by more than JUMP_METERS, and the jump is inside a
//     "tracking (re)start window": one of the two samples lacks POSITION_TRACKED, or it is less
//     than WINDOW after a regain, a session READY, a session (re)creation or the process start.
//     Outside the window a jump only clears SETTLED for 2 s (fast head turns never change the
//     epoch; a head cannot accelerate 50 mm off the prediction within 20 ms anyway);
//  4. STAGE_CHANGED: a STAGE XrEventDataReferenceSpaceChangePending, unless it is a pure
//     floor-height change while settled (horizontal <= 5 mm, rotation <= 0.2 deg, measured from
//     STAGE-in-LOCAL before and after changeTime, no LOCAL change within 1 s): that one is
//     carried over (stage_change_counter, stage_from_previous) without an epoch change;
//  5. CONTINUITY_UNKNOWN: a gap of more than GAP between samples, or the device suspended
//     between two samples (CLOCK_BOOTTIME advanced more than CLOCK_MONOTONIC by > SUSPEND):
//     a loss and regain could have been missed;
//  6. SESSION_RESTART: a new OpenXR session in the same process (not the first).
// NOT incremented: LOCAL recenter (XrEventDataReferenceSpaceChangePending for LOCAL; STAGE
// stays fixed in the room), awake pauses, sleeps where POSITION_TRACKED never dropped.
//
// SETTLED: position tracked continuously for >= SETTLE, no jump > JUMP_METERS and no reference
// space change (any type) within SETTLE, the epoch unchanged for SETTLE, and, once the runtime has
// sent PICO's undocumented event 1200007323 in this process, its latest value is 3 (it sends 1 at
// session start/wake and 3 when 6DoF tracking is established).

use alvr_common::{Pose, glam::Quat};
use alvr_packets::{
    CameraTrackingStamp, HEAD_ORIENTATION_TRACKED, HEAD_POSITION_TRACKED, HEAD_SETTLED,
    HEAD_STAMP_KNOWN, TrackingStateReport, epoch_reason,
};
use std::collections::VecDeque;

pub const SETTLE_NANOS: i64 = 2_000_000_000;
pub const WINDOW_NANOS: i64 = 5_000_000_000;
pub const JUMP_METERS: f32 = 0.05;
const JUMP_MAX_GAP_NANOS: i64 = 100_000_000;
pub const GAP_NANOS: i64 = 3_000_000_000;
pub const SUSPEND_NANOS: i64 = 1_000_000_000;
const PURE_FLOOR_METERS: f32 = 0.005;
const PURE_FLOOR_DEGREES: f32 = 0.2;
const LOCAL_CHANGE_NEAR_NANOS: i64 = 1_000_000_000;
// A STAGE change whose new STAGE pose is not located within this long counts as a real change
const STAGE_CHANGE_TIMEOUT_NANOS: i64 = 2_000_000_000;
const STAGE_HISTORY_NANOS: i64 = 2_000_000_000;
const STAMP_HISTORY_NANOS: i64 = 5_000_000_000;
// A camera stamp needs a sample at most this much before the capture time
const STAMP_MAX_AGE_NANOS: i64 = 100_000_000;

// PICO vendor event (undocumented): u32 at offset 8 of the event payload, 1 = tracking starting,
// 3 = 6DoF tracking established (2026-11-11 logs)
pub const PICO_TRACKING_EVENT_TYPE: i32 = 1_200_007_323;
pub const PICO_TRACKING_READY_VALUE: u32 = 3;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpaceKind {
    Stage,
    Local,
    Other,
}

#[derive(Clone, Copy, Debug)]
pub struct Sample {
    /// CLOCK_BOOTTIME and CLOCK_MONOTONIC, nanoseconds (a growing difference = suspend)
    pub boot_ns: i64,
    pub mono_ns: i64,
    /// XrTime the spaces were located at
    pub xr_ns: i64,
    pub position_tracked: bool,
    pub orientation_tracked: bool,
    pub view_in_stage: Option<Pose>,
    pub stage_in_local: Option<Pose>,
}

struct PendingStage {
    change_xr_ns: i64,
    received_boot_ns: i64,
    settled_at_event: bool,
}

pub fn reason_name(reason: u32) -> &'static str {
    match reason {
        epoch_reason::CLIENT_START => "client start",
        epoch_reason::TRACKING_LOST => "tracking lost",
        epoch_reason::TRACKING_REGAINED => "tracking regained",
        epoch_reason::REANCHOR_JUMP => "re-anchor jump",
        epoch_reason::STAGE_CHANGED => "STAGE changed",
        epoch_reason::CONTINUITY_UNKNOWN => "continuity unknown",
        epoch_reason::SESSION_RESTART => "session restart",
        _ => "unknown",
    }
}

pub struct EpochMachine {
    instance: u64,
    epoch: u32,
    reason: u32,
    epoch_changed_boot_ns: i64,
    last: Option<Sample>,
    views: [Option<(i64, Pose)>; 2],
    position_tracked_since: Option<i64>,
    window_until_boot_ns: i64,
    window_at_next_sample: bool,
    last_jump_boot_ns: Option<i64>,
    last_reference_change_boot_ns: Option<i64>,
    last_local_change_boot_ns: Option<i64>,
    vendor_seen: bool,
    vendor_ready: bool,
    sessions: u32,
    pending_stage: Option<PendingStage>,
    stage_history: VecDeque<(i64, Pose)>,
    stage_change_counter: u32,
    stage_from_previous: Pose,
    flags: u32,
    stamps: VecDeque<(i64, u32, u32)>,
    // Diagnostic lines produced since the last drain
    lines: Vec<String>,
}

fn pose_brief(pose: &Pose) -> String {
    let p = pose.position;
    let (yaw, pitch, roll) = pose.orientation.to_euler(alvr_common::glam::EulerRot::YXZ);
    format!(
        "({:.4},{:.4},{:.4}) ypr ({:.2},{:.2},{:.2})",
        p.x,
        p.y,
        p.z,
        yaw.to_degrees(),
        pitch.to_degrees(),
        roll.to_degrees()
    )
}

impl EpochMachine {
    pub fn new(instance: u64, boot_ns: i64) -> Self {
        Self {
            instance,
            epoch: 1,
            reason: epoch_reason::CLIENT_START,
            epoch_changed_boot_ns: boot_ns,
            last: None,
            views: [None, None],
            position_tracked_since: None,
            window_until_boot_ns: boot_ns + WINDOW_NANOS,
            window_at_next_sample: true,
            last_jump_boot_ns: None,
            last_reference_change_boot_ns: None,
            last_local_change_boot_ns: None,
            vendor_seen: false,
            vendor_ready: false,
            sessions: 0,
            pending_stage: None,
            stage_history: VecDeque::new(),
            stage_change_counter: 0,
            stage_from_previous: Pose::IDENTITY,
            flags: 0,
            stamps: VecDeque::new(),
            lines: vec![format!(
                "EPOCH 1 (client start, instance {instance:016x}): a new client process cannot \
                 prove tracking continuity"
            )],
        }
    }

    #[cfg_attr(not(test), allow(dead_code))]
    pub fn epoch(&self) -> u32 {
        self.epoch
    }

    #[cfg_attr(not(test), allow(dead_code))]
    pub fn flags(&self) -> u32 {
        self.flags
    }

    #[cfg_attr(not(test), allow(dead_code))]
    pub fn stage_change_counter(&self) -> u32 {
        self.stage_change_counter
    }

    pub fn drain_lines(&mut self) -> Vec<String> {
        std::mem::take(&mut self.lines)
    }

    fn change(&mut self, reason: u32, boot_ns: i64, detail: &str) {
        self.epoch = self.epoch.wrapping_add(1).max(1);
        self.reason = reason;
        self.epoch_changed_boot_ns = boot_ns;
        self.lines.push(format!(
            "EPOCH {} -> {} ({}): {detail}",
            self.epoch.wrapping_sub(1),
            self.epoch,
            reason_name(reason)
        ));
    }

    fn open_window(&mut self) {
        self.window_at_next_sample = true;
    }

    /// A new OpenXR session (the first one of the process changes nothing)
    pub fn on_session_created(&mut self, boot_ns: i64) {
        self.sessions += 1;
        if self.sessions > 1 {
            self.change(
                epoch_reason::SESSION_RESTART,
                boot_ns,
                "new OpenXR session in this process",
            );
        }
        self.open_window();
    }

    /// Session state READY (start or wake)
    pub fn on_session_ready(&mut self) {
        self.open_window();
    }

    pub fn on_reference_space_change(&mut self, boot_ns: i64, kind: SpaceKind, change_xr_ns: i64) {
        self.last_reference_change_boot_ns = Some(boot_ns);
        match kind {
            SpaceKind::Stage => {
                if let Some(pending) = self.pending_stage.take() {
                    // Two STAGE changes before the first could be measured: not a pure change
                    self.change(
                        epoch_reason::STAGE_CHANGED,
                        boot_ns,
                        &format!(
                            "STAGE changed again before the change at {} was measured",
                            pending.change_xr_ns
                        ),
                    );
                }
                self.pending_stage = Some(PendingStage {
                    change_xr_ns,
                    received_boot_ns: boot_ns,
                    settled_at_event: self.flags & HEAD_SETTLED != 0,
                });
            }
            SpaceKind::Local => self.last_local_change_boot_ns = Some(boot_ns),
            SpaceKind::Other => (),
        }
    }

    /// PICO vendor event 1200007323 (value 1 = tracking starting, 3 = established)
    pub fn on_vendor_tracking_event(&mut self, value: u32) {
        self.vendor_seen = true;
        self.vendor_ready = value == PICO_TRACKING_READY_VALUE;
    }

    fn in_window(&self, sample: &Sample) -> bool {
        !sample.position_tracked
            || self.last.is_some_and(|last| !last.position_tracked)
            || sample.boot_ns < self.window_until_boot_ns
    }

    pub fn on_sample(&mut self, sample: Sample) {
        if self.window_at_next_sample {
            self.window_at_next_sample = false;
            self.window_until_boot_ns = sample.boot_ns + WINDOW_NANOS;
        }

        if let Some(last) = self.last {
            let gap = sample.boot_ns - last.boot_ns;
            let suspended =
                (sample.boot_ns - sample.mono_ns) - (last.boot_ns - last.mono_ns);
            if gap > GAP_NANOS || suspended > SUSPEND_NANOS {
                self.change(
                    epoch_reason::CONTINUITY_UNKNOWN,
                    sample.boot_ns,
                    &format!(
                        "no tracking sample for {:.1} s ({:.1} s suspended): a loss could have \
                         been missed",
                        gap as f64 / 1e9,
                        suspended as f64 / 1e9
                    ),
                );
                self.window_until_boot_ns = sample.boot_ns + WINDOW_NANOS;
                self.views = [None, None];
            }

            match (last.position_tracked, sample.position_tracked) {
                (true, false) => self.change(
                    epoch_reason::TRACKING_LOST,
                    sample.boot_ns,
                    &format!(
                        "head position no longer tracked (pose {})",
                        sample
                            .view_in_stage
                            .as_ref()
                            .map_or("invalid".into(), pose_brief)
                    ),
                ),
                (false, true) => {
                    self.change(
                        epoch_reason::TRACKING_REGAINED,
                        sample.boot_ns,
                        &format!(
                            "head position tracked again (pose {})",
                            sample
                                .view_in_stage
                                .as_ref()
                                .map_or("invalid".into(), pose_brief)
                        ),
                    );
                    self.window_until_boot_ns = sample.boot_ns + WINDOW_NANOS;
                }
                _ => (),
            }
        }

        if sample.position_tracked {
            self.position_tracked_since.get_or_insert(sample.boot_ns);
        } else {
            self.position_tracked_since = None;
        }

        self.check_jump(&sample);
        self.check_stage_change(&sample);

        // Settled
        let older = |t: Option<i64>| t.is_none_or(|t| sample.boot_ns - t >= SETTLE_NANOS);
        let settled = sample.position_tracked
            && self
                .position_tracked_since
                .is_some_and(|since| sample.boot_ns - since >= SETTLE_NANOS)
            && older(self.last_jump_boot_ns)
            && older(self.last_reference_change_boot_ns)
            && older(Some(self.epoch_changed_boot_ns))
            && (!self.vendor_seen || self.vendor_ready)
            && self.pending_stage.is_none();
        let flags = u32::from(sample.position_tracked) * HEAD_POSITION_TRACKED
            | u32::from(sample.orientation_tracked) * HEAD_ORIENTATION_TRACKED
            | u32::from(settled) * HEAD_SETTLED;
        if (flags ^ self.flags) & HEAD_SETTLED != 0 {
            self.lines.push(format!(
                "SETTLED {} (epoch {})",
                if settled { "yes" } else { "no" },
                self.epoch
            ));
        }
        self.flags = flags;

        self.stamps.push_back((sample.xr_ns, self.epoch, flags));
        while self
            .stamps
            .front()
            .is_some_and(|(t, _, _)| sample.xr_ns - t > STAMP_HISTORY_NANOS)
        {
            self.stamps.pop_front();
        }
        self.last = Some(sample);
    }

    fn check_jump(&mut self, sample: &Sample) {
        let Some(pose) = sample.view_in_stage else {
            self.views = [None, None];
            return;
        };
        let now = sample.xr_ns;
        if let [Some((t1, p1)), Some((t0, p0))] = self.views
            && now > t1
            && t1 > t0
            && now - t1 <= JUMP_MAX_GAP_NANOS
            && t1 - t0 <= JUMP_MAX_GAP_NANOS
        {
            let ratio = (now - t1) as f32 / (t1 - t0) as f32;
            let predicted = p1.position + (p1.position - p0.position) * ratio;
            let error = pose.position.distance(predicted);
            if error > JUMP_METERS {
                self.last_jump_boot_ns = Some(sample.boot_ns);
                let detail = format!(
                    "head pose {:.1} mm off the prediction in {:.1} ms, {} -> {}",
                    error * 1000.0,
                    (now - t1) as f64 / 1e6,
                    pose_brief(&p1),
                    pose_brief(&pose)
                );
                if self.in_window(sample) {
                    self.change(epoch_reason::REANCHOR_JUMP, sample.boot_ns, &detail);
                } else {
                    self.lines.push(format!(
                        "JUMP outside a tracking (re)start window, epoch kept, not settled for 2 \
                         s: {detail}"
                    ));
                }
                // A step is one jump: the next prediction must not extrapolate across it
                self.views = [Some((now, pose)), None];
                return;
            }
        }
        if self.views[0].is_none_or(|(t, _)| now > t) {
            self.views = [Some((now, pose)), self.views[0]];
        }
    }

    fn check_stage_change(&mut self, sample: &Sample) {
        let Some(pending) = &self.pending_stage else {
            if let Some(stage) = sample.stage_in_local {
                self.push_stage(sample.xr_ns, stage);
            }
            return;
        };
        let (change_xr_ns, received, settled_at_event) = (
            pending.change_xr_ns,
            pending.received_boot_ns,
            pending.settled_at_event,
        );

        match sample.stage_in_local {
            Some(new) if sample.xr_ns >= change_xr_ns => {
                self.pending_stage = None;
                let old = self
                    .stage_history
                    .iter()
                    .rev()
                    .find(|(t, _)| *t < change_xr_ns)
                    .map(|(_, pose)| *pose);
                let local_changed = self
                    .last_local_change_boot_ns
                    .is_some_and(|t| (t - received).abs() <= LOCAL_CHANGE_NEAR_NANOS);
                match old {
                    Some(old) => {
                        // p_local = old * p_previous = new * p_new => p_new = new^-1 old p_previous
                        let transform = new.inverse() * old;
                        let horizontal = transform.position.x.hypot(transform.position.z);
                        let degrees = transform.orientation.angle_between(Quat::IDENTITY).to_degrees();
                        let pure = horizontal <= PURE_FLOOR_METERS
                            && degrees <= PURE_FLOOR_DEGREES
                            && settled_at_event
                            && !local_changed;
                        let detail = format!(
                            "STAGE new-from-previous {} (horizontal {:.1} mm, {degrees:.2} deg, \
                             settled at the event {settled_at_event}, LOCAL changed too \
                             {local_changed})",
                            pose_brief(&transform),
                            horizontal * 1000.0
                        );
                        if pure {
                            self.stage_change_counter = self.stage_change_counter.wrapping_add(1);
                            self.stage_from_previous = transform;
                            self.lines.push(format!(
                                "STAGE floor-height change {} carried over, epoch {} kept: \
                                 {detail}",
                                self.stage_change_counter, self.epoch
                            ));
                        } else {
                            self.change(epoch_reason::STAGE_CHANGED, sample.boot_ns, &detail);
                        }
                    }
                    None => self.change(
                        epoch_reason::STAGE_CHANGED,
                        sample.boot_ns,
                        "STAGE changed (no STAGE pose from before the change)",
                    ),
                }
                self.stage_history.clear();
                self.push_stage(sample.xr_ns, new);
            }
            _ => {
                if sample.boot_ns - received > STAGE_CHANGE_TIMEOUT_NANOS {
                    self.pending_stage = None;
                    self.change(
                        epoch_reason::STAGE_CHANGED,
                        sample.boot_ns,
                        "STAGE changed (not located after the change)",
                    );
                }
                // Samples from before changeTime still describe the old STAGE
                if let Some(stage) = sample.stage_in_local
                    && sample.xr_ns < change_xr_ns
                {
                    self.push_stage(sample.xr_ns, stage);
                }
            }
        }
    }

    fn push_stage(&mut self, xr_ns: i64, pose: Pose) {
        self.stage_history.push_back((xr_ns, pose));
        while self
            .stage_history
            .front()
            .is_some_and(|(t, _)| xr_ns - t > STAGE_HISTORY_NANOS)
        {
            self.stage_history.pop_front();
        }
    }

    pub fn report(&self, boot_ns: i64) -> TrackingStateReport {
        TrackingStateReport {
            client_instance: self.instance,
            client_epoch: self.epoch,
            epoch_reason: self.reason,
            flags: self.flags,
            epoch_age_ms: ((boot_ns - self.epoch_changed_boot_ns).max(0) / 1_000_000)
                .min(i64::from(u32::MAX)) as u32,
            stage_change_counter: self.stage_change_counter,
            stage_from_previous: self.stage_from_previous,
        }
    }

    /// The tracking state at a camera frame's capture time (XrTime)
    pub fn stamp(&self, capture_ns: i64) -> CameraTrackingStamp {
        let entry = self
            .stamps
            .iter()
            .rev()
            .find(|(t, _, _)| *t <= capture_ns)
            .filter(|(t, _, _)| capture_ns - t <= STAMP_MAX_AGE_NANOS);
        CameraTrackingStamp {
            capture_ns: capture_ns.max(0) as u64,
            client_instance: self.instance,
            // A later epoch change (e.g. a loss noticed after the capture) still invalidates the
            // frame: the reader compares this with the current epoch
            client_epoch: entry.map_or(self.epoch, |(_, epoch, _)| *epoch),
            flags: entry.map_or(0, |(_, _, flags)| flags | HEAD_STAMP_KNOWN),
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Process-wide instance (the sampler feeds it, the render and camera threads read it)

mod global {
    use super::*;
    use alvr_client_core::ClientCoreContext;
    use std::{
        hash::{BuildHasher, Hasher},
        sync::{Mutex, OnceLock},
        time::{Duration, Instant},
    };

    static MACHINE: OnceLock<Mutex<EpochMachine>> = OnceLock::new();
    static LAST_SENT: Mutex<Option<(TrackingStateReport, Instant)>> = Mutex::new(None);
    const RESEND_INTERVAL: Duration = Duration::from_millis(250);

    fn random_u64() -> u64 {
        loop {
            let mut hasher = std::collections::hash_map::RandomState::new().build_hasher();
            hasher.write_u128(
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_nanos(),
            );
            hasher.write_u32(std::process::id());
            let value = hasher.finish();
            if value != 0 {
                return value;
            }
        }
    }

    fn machine() -> &'static Mutex<EpochMachine> {
        MACHINE.get_or_init(|| {
            Mutex::new(EpochMachine::new(
                random_u64(),
                crate::tracking_diag::boot_ns(),
            ))
        })
    }

    // Runs `f` and writes the diagnostic lines it produced. Never panics on a poisoned lock.
    fn with<R>(f: impl FnOnce(&mut EpochMachine) -> R) -> Option<R> {
        let mut machine = machine().lock().ok()?;
        let result = f(&mut machine);
        for line in machine.drain_lines() {
            crate::tracking_diag::log(line);
        }
        Some(result)
    }

    // Events from the render thread are queued (a send never blocks) and applied by the sampler
    // before its next sample, in order
    enum Event {
        SessionCreated(i64),
        SessionReady,
        ReferenceSpace(i64, SpaceKind, i64),
        Vendor(u32),
    }

    struct Queue {
        sender: std::sync::mpsc::Sender<Event>,
        receiver: Mutex<std::sync::mpsc::Receiver<Event>>,
    }

    static QUEUE: OnceLock<Queue> = OnceLock::new();

    fn queue() -> &'static Queue {
        QUEUE.get_or_init(|| {
            let (sender, receiver) = std::sync::mpsc::channel();
            Queue {
                sender,
                receiver: Mutex::new(receiver),
            }
        })
    }

    fn post(event: Event) {
        queue().sender.send(event).ok();
    }

    /// Sampler thread
    pub fn on_sample(sample: Sample) {
        let events: Vec<Event> = queue()
            .receiver
            .lock()
            .map(|receiver| receiver.try_iter().collect())
            .unwrap_or_default();
        with(|m| {
            for event in events {
                match event {
                    Event::SessionCreated(boot_ns) => m.on_session_created(boot_ns),
                    Event::SessionReady => m.on_session_ready(),
                    Event::ReferenceSpace(boot_ns, kind, change) => {
                        m.on_reference_space_change(boot_ns, kind, change)
                    }
                    Event::Vendor(value) => m.on_vendor_tracking_event(value),
                }
            }
            m.on_sample(sample);
        });
    }

    pub fn on_session_created() {
        post(Event::SessionCreated(crate::tracking_diag::boot_ns()));
    }

    pub fn on_session_ready() {
        post(Event::SessionReady);
    }

    pub fn on_reference_space_change(kind: SpaceKind, change_xr_ns: i64) {
        post(Event::ReferenceSpace(
            crate::tracking_diag::boot_ns(),
            kind,
            change_xr_ns,
        ));
    }

    pub fn on_vendor_tracking_event(value: u32) {
        post(Event::Vendor(value));
    }

    /// Render thread, while streaming: on change, else every RESEND_INTERVAL. Never blocks.
    pub fn send_report(core_context: &ClientCoreContext) {
        let Some(report) = machine()
            .try_lock()
            .ok()
            .map(|m| m.report(crate::tracking_diag::boot_ns()))
        else {
            return;
        };
        let Ok(mut last) = LAST_SENT.try_lock() else {
            return;
        };
        let due = last.as_ref().is_none_or(|(sent, at)| {
            at.elapsed() >= RESEND_INTERVAL
                || TrackingStateReport {
                    epoch_age_ms: 0,
                    ..*sent
                } != TrackingStateReport {
                    epoch_age_ms: 0,
                    ..report
                }
        });
        if due {
            core_context.send_reserved_buffer(report.encode());
            *last = Some((report, Instant::now()));
        }
    }

    /// Camera thread: sent right before the CameraFrame packet it describes
    pub fn send_camera_stamp(core_context: &ClientCoreContext, capture_time: Duration) {
        let capture_ns = capture_time.as_nanos().min(i64::MAX as u128) as i64;
        if let Some(stamp) = machine().lock().ok().map(|m| m.stamp(capture_ns)) {
            core_context.send_reserved_buffer(stamp.encode());
        }
    }
}

pub use global::*;

#[cfg(test)]
mod tests {
    use super::*;
    use alvr_common::glam::Vec3;

    const MS: i64 = 1_000_000;
    const S: i64 = 1_000_000_000;

    fn pose(x: f32, y: f32, z: f32) -> Pose {
        Pose {
            orientation: Quat::IDENTITY,
            position: Vec3::new(x, y, z),
        }
    }

    struct Rig {
        m: EpochMachine,
        t: i64,
        // CLOCK_BOOTTIME - CLOCK_MONOTONIC (grows when the device suspends)
        suspended: i64,
        head: Pose,
        stage_in_local: Pose,
        tracked: bool,
    }

    impl Rig {
        fn new() -> Self {
            Self {
                m: EpochMachine::new(1, 0),
                t: 0,
                suspended: 0,
                head: pose(-0.05, 1.63, 0.05),
                stage_in_local: pose(0.0, -1.65, 0.0),
                tracked: true,
            }
        }

        fn sample(&mut self) {
            self.m.on_sample(Sample {
                boot_ns: self.t,
                mono_ns: self.t - self.suspended,
                xr_ns: self.t,
                position_tracked: self.tracked,
                orientation_tracked: true,
                view_in_stage: Some(self.head),
                stage_in_local: Some(self.stage_in_local),
            });
        }

        // Samples every `step` for `duration`
        fn run(&mut self, duration: i64, step: i64) {
            let end = self.t + duration;
            while self.t < end {
                self.t += step;
                self.sample();
            }
        }

        fn awake(&mut self, duration: i64) {
            self.run(duration, 20 * MS);
        }

        fn idle(&mut self, duration: i64) {
            self.run(duration, 500 * MS);
        }

        fn settled(&self) -> bool {
            self.m.flags() & HEAD_SETTLED != 0
        }

        fn reason(&self) -> u32 {
            self.m.report(self.t).epoch_reason
        }
    }

    // A started process that tracked for a while (PICO: PT from the first sample after a reboot)
    fn running() -> Rig {
        let mut rig = Rig::new();
        rig.m.on_session_created(0);
        rig.m.on_vendor_tracking_event(1);
        rig.awake(300 * MS);
        rig.m.on_vendor_tracking_event(3);
        rig.awake(10 * S);
        rig
    }

    #[test]
    fn process_start_is_epoch_one_and_settles_after_two_seconds() {
        let mut rig = Rig::new();
        rig.m.on_session_created(0);
        rig.m.on_vendor_tracking_event(1);
        rig.awake(1 * S);
        assert!(!rig.settled(), "vendor event still 1");
        rig.m.on_vendor_tracking_event(3);
        rig.awake(1100 * MS);
        assert!(rig.settled());
        assert_eq!(rig.m.epoch(), 1);
        assert_eq!(rig.reason(), epoch_reason::CLIENT_START);
    }

    // 12:53:53 sleep -> PT lost 12:55:54 (IDLE) -> 12:58:20.989 READY: placeholder pose without PT,
    // 144 mm re-anchor jump at +1.5 s, PT at +3.2 s, vendor 3 at +3.3 s
    #[test]
    fn wake_after_long_sleep() {
        let mut rig = running();
        let before = rig.m.epoch();
        rig.idle(120 * S);
        assert_eq!(rig.m.epoch(), before, "IDLE alone changes nothing");
        // Tracking stops while asleep
        rig.tracked = false;
        rig.head = pose(-0.0506, 1.6313, 0.0519);
        rig.idle(145 * S);
        assert_eq!(rig.m.epoch(), before + 1);
        assert_eq!(rig.reason(), epoch_reason::TRACKING_LOST);
        // Wake: still untracked, first pose (the placeholder changes too)
        rig.m.on_session_ready();
        rig.m.on_vendor_tracking_event(1);
        rig.head = pose(-0.0021, 1.6480, -0.0823);
        rig.awake(1500 * MS);
        let at_wake = rig.m.epoch();
        assert!(at_wake >= before + 1);
        rig.head = pose(-0.0499, 1.6313, 0.0525); // 144 mm re-anchor
        rig.awake(1700 * MS);
        assert_eq!(rig.m.epoch(), at_wake + 1);
        assert_eq!(rig.reason(), epoch_reason::REANCHOR_JUMP);
        assert!(!rig.settled());
        rig.tracked = true;
        rig.awake(70 * MS);
        rig.m.on_vendor_tracking_event(3);
        assert_eq!(rig.m.epoch(), at_wake + 2);
        assert_eq!(rig.reason(), epoch_reason::TRACKING_REGAINED);
        rig.awake(1800 * MS);
        assert!(!rig.settled(), "less than 2 s since PT");
        rig.awake(400 * MS);
        assert!(rig.settled());
        assert_eq!(rig.m.epoch(), at_wake + 2);
    }

    // 12:51:11 sleep, 12:52:20 wake: PT never dropped; vendor 1 then 3 after 0.3 s
    #[test]
    fn short_sleep_keeps_the_epoch() {
        let mut rig = running();
        let before = rig.m.epoch();
        rig.idle(69 * S);
        rig.m.on_session_ready();
        rig.m.on_vendor_tracking_event(1);
        rig.awake(300 * MS);
        assert!(!rig.settled(), "not settled until the vendor event says 3");
        rig.m.on_vendor_tracking_event(3);
        rig.awake(100 * MS);
        assert!(rig.settled());
        assert_eq!(rig.m.epoch(), before);
    }

    // Awake pause, fast head turns, and a 62 mm snap while tracked outside any window (13:13:53)
    #[test]
    fn awake_pause_and_head_motion_keep_the_epoch() {
        let mut rig = running();
        let before = rig.m.epoch();
        // Fast head turn: 600 deg/s about a neck 10 cm behind the eyes
        let turned = |yaw: f32| Pose {
            orientation: Quat::from_rotation_y(yaw.to_radians()),
            position: Vec3::new(0.0, 1.6, 0.0)
                + Quat::from_rotation_y(yaw.to_radians()) * Vec3::new(0.0, 0.0, -0.1),
        };
        rig.head = turned(0.0);
        rig.awake(60 * S);
        assert!(rig.settled());
        for i in 0..50 {
            rig.head = turned(i as f32 * 12.0);
            rig.awake(20 * MS);
            assert!(rig.settled(), "turn step {i}");
        }
        assert_eq!(rig.m.epoch(), before);
        // Tracking snap while tracked
        rig.head.position.y -= 0.062;
        rig.awake(20 * MS);
        assert_eq!(rig.m.epoch(), before, "outside a window a jump does not change the epoch");
        assert!(!rig.settled());
        rig.awake(2100 * MS);
        assert!(rig.settled());
    }

    // Recenter (long press Home) x6: LOCAL changes, STAGE stays in the room
    #[test]
    fn recenter_keeps_the_epoch() {
        let mut rig = running();
        let before = rig.m.epoch();
        for i in 0..6 {
            rig.m.on_reference_space_change(rig.t, SpaceKind::Local, rig.t + 10 * MS);
            // STAGE seen from the new LOCAL moves
            rig.stage_in_local = Pose {
                orientation: Quat::from_rotation_y((30.0 * i as f32).to_radians()),
                position: Vec3::new(0.1, -1.9, 0.02 * i as f32),
            };
            rig.awake(500 * MS);
            assert!(!rig.settled());
            rig.awake(2 * S);
        }
        assert!(rig.settled());
        assert_eq!(rig.m.epoch(), before);
    }

    // 13:05 reboot: new process with PT from the start, STAGE change at +0.4 s (floor 1.65 m
    // assumed -> 0), then STAGE + LOCAL change at +8.5 s with 4 cm horizontally
    #[test]
    fn reboot_with_two_stage_changes() {
        let mut rig = Rig::new();
        rig.m.on_session_created(0);
        rig.awake(400 * MS);
        assert_eq!(rig.m.epoch(), 1);
        rig.m.on_reference_space_change(rig.t, SpaceKind::Stage, rig.t);
        rig.stage_in_local = pose(0.0, 0.0, 0.0);
        rig.awake(100 * MS);
        assert_eq!(rig.m.epoch(), 2);
        assert_eq!(rig.reason(), epoch_reason::STAGE_CHANGED);
        rig.m.on_session_ready();
        rig.awake(8 * S);
        rig.m.on_reference_space_change(rig.t, SpaceKind::Stage, rig.t);
        rig.m.on_reference_space_change(rig.t, SpaceKind::Local, rig.t);
        rig.stage_in_local = Pose {
            orientation: Quat::from_rotation_y(0.03_f32.to_radians()),
            position: Vec3::new(-0.0003, -1.3151, 0.0001),
        };
        rig.head = pose(-0.002, 1.3145, 0.0003);
        rig.awake(100 * MS);
        assert_eq!(rig.m.epoch(), 3);
        assert_eq!(rig.reason(), epoch_reason::STAGE_CHANGED);
        assert_eq!(rig.m.stage_change_counter(), 0);
    }

    // A pure floor-height change while settled is carried over; the same change right after a
    // wake (13:11:30, 0.55 s after PT) changes the epoch
    #[test]
    fn floor_change_during_a_pass() {
        let mut rig = running();
        let before = rig.m.epoch();
        assert!(rig.settled());
        rig.m.on_reference_space_change(rig.t, SpaceKind::Stage, rig.t + 5 * MS);
        rig.stage_in_local.position.y += 0.3686;
        rig.awake(100 * MS);
        assert_eq!(rig.m.epoch(), before);
        assert_eq!(rig.m.stage_change_counter(), 1);
        let report = rig.m.report(rig.t);
        // p_new = T p_previous: a point at the old floor is 0.3686 m below the new floor
        let moved = report.stage_from_previous.position;
        assert!((moved.y + 0.3686).abs() < 1e-4 && moved.x.abs() < 1e-4, "{moved:?}");
        assert!(!rig.settled());
        rig.awake(2100 * MS);
        assert!(rig.settled());

        // Same change 0.55 s after a regain
        rig.tracked = false;
        rig.awake(3 * S);
        rig.tracked = true;
        rig.awake(550 * MS);
        let after_regain = rig.m.epoch();
        rig.m.on_reference_space_change(rig.t, SpaceKind::Stage, rig.t);
        rig.stage_in_local.position.y += 0.3686;
        rig.awake(100 * MS);
        assert_eq!(rig.m.epoch(), after_regain + 1);
        assert_eq!(rig.reason(), epoch_reason::STAGE_CHANGED);
        assert_eq!(rig.m.stage_change_counter(), 1);
    }

    #[test]
    fn suspend_or_sampling_gap_means_continuity_unknown() {
        let mut rig = running();
        let before = rig.m.epoch();
        rig.suspended += 5 * S;
        rig.t += 5 * S;
        rig.idle(1 * S);
        assert_eq!(rig.m.epoch(), before + 1);
        assert_eq!(rig.reason(), epoch_reason::CONTINUITY_UNKNOWN);
        rig.t += 4 * S;
        rig.idle(1 * S);
        assert_eq!(rig.m.epoch(), before + 2);
    }

    #[test]
    fn second_session_and_camera_stamps() {
        let mut rig = running();
        let capture = rig.t - 30 * MS;
        let stamp = rig.m.stamp(capture);
        assert_eq!(stamp.client_epoch, rig.m.epoch());
        assert_ne!(stamp.flags & HEAD_STAMP_KNOWN, 0);
        assert_ne!(stamp.flags & HEAD_SETTLED, 0);
        // No sample close to the capture time
        assert_eq!(rig.m.stamp(rig.t - 60 * S).flags, 0);

        let before = rig.m.epoch();
        rig.m.on_session_created(rig.t);
        assert_eq!(rig.m.epoch(), before + 1);
        assert_eq!(rig.reason(), epoch_reason::SESSION_RESTART);
        // The frame captured before the change keeps the old epoch: readers discard it
        assert_eq!(rig.m.stamp(capture).client_epoch, before);
    }
}
