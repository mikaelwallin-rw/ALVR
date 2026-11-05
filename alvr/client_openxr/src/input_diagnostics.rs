// Controller vs hand tracking diagnostics, enabled with the log_input_diagnostics setting.
// Logs per-hand OpenXR input state on change and on a heartbeat. Does not affect what is sent.
// Lines go to logcat (client logs are always printed there) prefixed with INPUT_DIAG.
use crate::interaction::HandInteraction;
use alvr_common::info;
use alvr_packets::{ButtonEntry, ButtonValue};
use openxr as xr;
use std::time::{Duration, Instant};

const HEARTBEAT_INTERVAL: Duration = Duration::from_secs(5);
const TAG: &str = "INPUT_DIAG";

const HAND_NAMES: [&str; 2] = ["left", "right"];
const HAND_PATHS: [&str; 2] = ["/user/hand/left", "/user/hand/right"];

#[derive(Clone, PartialEq, Eq)]
struct HandInputState {
    interaction_profile: String,
    grip_active: bool,
    hand_interaction_active: bool,
    grip_flags: String,
    hand_tracker_active: bool,
    joints_position_valid: usize,
    joints_position_tracked: usize,
    sent_controller: bool,
    sent_skeleton: bool,
}

pub struct InputDiagnostics {
    last_states: [Option<HandInputState>; 2],
    last_heartbeat: Instant,
}

impl InputDiagnostics {
    pub fn new() -> Self {
        Self {
            last_states: [None, None],
            last_heartbeat: Instant::now(),
        }
    }

    // Call while diagnostics are disabled, so that the full state is logged again as soon as they
    // get enabled
    pub fn reset(&mut self) {
        self.last_states = [None, None];
    }

    pub fn update(
        &mut self,
        xr_session: &xr::Session<xr::OpenGlEs>,
        reference_space: &xr::Space,
        time: xr::Time,
        hands: &[HandInteraction; 2],
        sent_controller: [bool; 2],
        sent_skeleton: [bool; 2],
    ) {
        let heartbeat = self.last_heartbeat.elapsed() >= HEARTBEAT_INTERVAL;
        if heartbeat {
            self.last_heartbeat = Instant::now();
        }

        for idx in 0..2 {
            let state = HandInputState {
                interaction_profile: current_interaction_profile(xr_session, HAND_PATHS[idx]),
                grip_active: hands[idx]
                    .grip_action
                    .is_active(xr_session, xr::Path::NULL)
                    .unwrap_or(false),
                hand_interaction_active: hands[idx]
                    .hand_interaction_grip_action
                    .as_ref()
                    .is_some_and(|action| {
                        action
                            .is_active(xr_session, xr::Path::NULL)
                            .unwrap_or(false)
                    }),
                grip_flags: hands[idx]
                    .grip_space
                    .locate(reference_space, time)
                    .map(|location| format_flags(location.location_flags))
                    .unwrap_or_else(|e| format!("err:{e}")),
                hand_tracker_active: false,
                joints_position_valid: 0,
                joints_position_tracked: 0,
                sent_controller: sent_controller[idx],
                sent_skeleton: sent_skeleton[idx],
            };
            let state = locate_joints(reference_space, time, &hands[idx], state);

            if heartbeat || self.last_states[idx].as_ref() != Some(&state) {
                info!(
                    "{TAG} hand={} {}profile={} grip_active={} hand_ia_active={} grip_flags={} \
                    tracker_active={} joints_pos_valid={}/26 joints_pos_tracked={}/26 \
                    sent_ctrl={} sent_skel={}",
                    HAND_NAMES[idx],
                    if heartbeat { "[hb] " } else { "" },
                    state.interaction_profile,
                    state.grip_active,
                    state.hand_interaction_active,
                    state.grip_flags,
                    state.hand_tracker_active,
                    state.joints_position_valid,
                    state.joints_position_tracked,
                    state.sent_controller,
                    state.sent_skeleton,
                );
                self.last_states[idx] = Some(state);
            }
        }
    }
}

pub fn log_buttons(xr_session: &xr::Session<xr::OpenGlEs>, entries: &[ButtonEntry]) {
    for entry in entries {
        if let ButtonValue::Binary(value) = entry.value {
            let path = alvr_common::BUTTON_INFO
                .get(&entry.path_id)
                .map_or("unknown", |info| info.path);
            info!(
                "{TAG} button {path}={value} profile_left={} profile_right={}",
                current_interaction_profile(xr_session, HAND_PATHS[0]),
                current_interaction_profile(xr_session, HAND_PATHS[1]),
            );
        }
    }
}

pub fn log_interaction_profiles(xr_session: &xr::Session<xr::OpenGlEs>) {
    info!(
        "{TAG} InteractionProfileChanged left={} right={}",
        current_interaction_profile(xr_session, HAND_PATHS[0]),
        current_interaction_profile(xr_session, HAND_PATHS[1]),
    );
}

fn current_interaction_profile(xr_session: &xr::Session<xr::OpenGlEs>, hand_path: &str) -> String {
    let instance = xr_session.instance();
    let Ok(user_path) = instance.string_to_path(hand_path) else {
        return "err:path".into();
    };

    match xr_session.current_interaction_profile(user_path) {
        Ok(path) if path == xr::Path::NULL => "null".into(),
        Ok(path) => instance
            .path_to_string(path)
            .unwrap_or_else(|e| format!("err:{e}")),
        Err(e) => format!("err:{e}"),
    }
}

fn locate_joints(
    reference_space: &xr::Space,
    time: xr::Time,
    hand: &HandInteraction,
    mut state: HandInputState,
) -> HandInputState {
    let Some(tracker) = &hand.skeleton_tracker else {
        return state;
    };

    if let Ok(Some(joints)) = reference_space.locate_hand_joints(tracker, time) {
        state.hand_tracker_active = true;
        state.joints_position_valid = joints
            .iter()
            .filter(|j| {
                j.location_flags
                    .contains(xr::SpaceLocationFlags::POSITION_VALID)
            })
            .count();
        state.joints_position_tracked = joints
            .iter()
            .filter(|j| {
                j.location_flags
                    .contains(xr::SpaceLocationFlags::POSITION_TRACKED)
            })
            .count();
    }

    state
}

fn format_flags(flags: xr::SpaceLocationFlags) -> String {
    let mut out = String::new();
    for (flag, name) in [
        (xr::SpaceLocationFlags::ORIENTATION_VALID, "OV"),
        (xr::SpaceLocationFlags::POSITION_VALID, "PV"),
        (xr::SpaceLocationFlags::ORIENTATION_TRACKED, "OT"),
        (xr::SpaceLocationFlags::POSITION_TRACKED, "PT"),
    ] {
        if flags.contains(flag) {
            if !out.is_empty() {
                out.push('|');
            }
            out.push_str(name);
        }
    }
    if out.is_empty() { "none".into() } else { out }
}
