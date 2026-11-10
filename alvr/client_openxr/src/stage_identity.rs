// Identity of the STAGE space for an overlay app that keeps poses in STAGE across sessions (e.g.
// physically placed screens): a random number persisted on the headset that changes only when
// STAGE itself moves, with the transform from the previous STAGE when it is known. Evidence of a
// move: the runtime's STAGE change event (beyond tolerance), or the persistent anchor (normally at
// the touchscreen) found at another STAGE pose than where it was last seen.

use crate::extra_extensions::{
    ANCHOR_LOCATED, ANCHOR_UNSUPPORTED, AnchorCommand, PersistentAnchor,
};
use alvr_client_core::ClientCoreContext;
use alvr_common::{Pose, info, warn};
use alvr_packets::{AnchorRequest, AnchorRequestKind, StageInfo};
use openxr as xr;
use std::{
    collections::hash_map::RandomState,
    fs,
    hash::{BuildHasher, Hasher},
    time::{Duration, Instant},
};

const STATE_FILE: &str = "/sdcard/Android/data/alvr.client.dev/files/screenvr_stage.txt";
// A STAGE change smaller than this is the same stage (runtime events at wake/recenter often carry
// an identity or near identity transform)
const SAME_STAGE_METERS: f32 = 0.001;
const SAME_STAGE_DEGREES: f32 = 0.05;
// The anchor seen this far from where it was last seen means STAGE moved...
const ANCHOR_MOVED_METERS: f32 = 0.002;
const ANCHOR_MOVED_DEGREES: f32 = 0.2;
// ...when this many locates in a row agree (500 ms apart)
const ANCHOR_MOVED_CONFIRMATIONS: u32 = 3;
// Also keeps the server's anchorLocatedMs fresh (contract: Located = located within the last
// second)
const RESEND_INTERVAL: Duration = Duration::from_millis(500);

#[derive(Default)]
struct Stored {
    identity: u64,
    previous_identity: u64,
    change_sequence: u32,
    request_served: u32,
    anchor_uuid: [u8; 16],
    anchor_generation: u32,
    // Where the anchor was last seen in this STAGE
    anchor_reference: Option<Pose>,
}

fn random_u64() -> u64 {
    loop {
        let mut hasher = RandomState::new().build_hasher();
        hasher.write_u128(
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos(),
        );
        let value = hasher.finish();
        if value != 0 {
            return value;
        }
    }
}

// Build.MODEL, e.g. "PICO 4 Ultra Enterprise" (Build.PRODUCT is a code name)
#[cfg(target_os = "android")]
fn model_name() -> String {
    alvr_system_info::android::model_name()
}

#[cfg(not(target_os = "android"))]
fn model_name() -> String {
    String::new()
}

fn pose_to_text(pose: &Pose) -> String {
    let q = pose.orientation;
    let p = pose.position;
    format!("{} {} {} {} {} {} {}", q.x, q.y, q.z, q.w, p.x, p.y, p.z)
}

fn pose_from_text(text: &str) -> Option<Pose> {
    let values = text
        .split_whitespace()
        .map(str::parse::<f32>)
        .collect::<Result<Vec<_>, _>>()
        .ok()?;
    let [qx, qy, qz, qw, x, y, z] = values[..] else {
        return None;
    };
    Some(Pose {
        orientation: alvr_common::glam::Quat::from_xyzw(qx, qy, qz, qw).normalize(),
        position: alvr_common::glam::Vec3::new(x, y, z),
    })
}

impl Stored {
    fn load() -> Self {
        let mut stored = Stored::default();
        let Ok(text) = fs::read_to_string(STATE_FILE) else {
            return stored;
        };
        for line in text.lines() {
            let Some((key, value)) = line.split_once('=') else {
                continue;
            };
            let value = value.trim();
            match key.trim() {
                "identity" => stored.identity = u64::from_str_radix(value, 16).unwrap_or(0),
                "previous_identity" => {
                    stored.previous_identity = u64::from_str_radix(value, 16).unwrap_or(0)
                }
                "change_sequence" => stored.change_sequence = value.parse().unwrap_or(0),
                "request_served" => stored.request_served = value.parse().unwrap_or(0),
                "anchor_generation" => stored.anchor_generation = value.parse().unwrap_or(0),
                "anchor_uuid" if value.len() == 32 => {
                    for (i, byte) in stored.anchor_uuid.iter_mut().enumerate() {
                        *byte = u8::from_str_radix(&value[i * 2..i * 2 + 2], 16).unwrap_or(0);
                    }
                }
                "anchor_reference" => stored.anchor_reference = pose_from_text(value),
                _ => (),
            }
        }
        stored
    }

    fn save(&self) {
        let uuid: String = self
            .anchor_uuid
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect();
        let text = format!(
            "identity={:016x}\nprevious_identity={:016x}\nchange_sequence={}\nrequest_served={}\n\
             anchor_generation={}\nanchor_uuid={uuid}\nanchor_reference={}\n",
            self.identity,
            self.previous_identity,
            self.change_sequence,
            self.request_served,
            self.anchor_generation,
            self.anchor_reference
                .as_ref()
                .map(pose_to_text)
                .unwrap_or_default(),
        );
        // Write then rename: never a half written file
        let temp = format!("{STATE_FILE}.tmp");
        if let Err(e) = fs::write(&temp, text).and_then(|()| fs::rename(&temp, STATE_FILE)) {
            warn!("Stage identity: cannot save {STATE_FILE}: {e}");
        }
    }
}

fn pose_difference(a: &Pose, b: &Pose) -> (f32, f32) {
    let meters = a.position.distance(b.position);
    let degrees = a.orientation.angle_between(b.orientation).to_degrees();
    (meters, degrees)
}

pub struct StageIdentity {
    stored: Stored,
    from_previous: Option<Pose>,
    anchor: Option<PersistentAnchor>,
    moved_confirmations: u32,
    product_name: String,
    last_sent: Option<Instant>,
    dirty: bool,
}

impl StageIdentity {
    pub fn new(session: &xr::Session<xr::OpenGlEs>) -> Self {
        let mut stored = Stored::load();
        if stored.identity == 0 {
            stored.identity = random_u64();
            stored.change_sequence = stored.change_sequence.wrapping_add(1);
            info!("Stage identity: none stored, new {:016x}", stored.identity);
            stored.save();
        } else {
            info!("Stage identity: {:016x}", stored.identity);
        }

        // PICO spatial anchors need the spatial data permission (asked once; a kiosk/MDM can
        // pre-grant it)
        #[cfg(target_os = "android")]
        alvr_system_info::try_get_permission("com.picovr.permission.SPATIAL_DATA");

        let stored_uuid = (stored.anchor_uuid != [0; 16]).then_some(stored.anchor_uuid);
        let anchor = PersistentAnchor::new(session, stored_uuid, stored.anchor_generation);

        Self {
            stored,
            from_previous: None,
            anchor,
            moved_confirmations: 0,
            product_name: model_name(),
            last_sent: None,
            dirty: true,
        }
    }

    fn change_stage(&mut self, from_previous: Option<Pose>, reason: &str) {
        self.stored.previous_identity = self.stored.identity;
        self.stored.identity = random_u64();
        self.stored.change_sequence = self.stored.change_sequence.wrapping_add(1);
        self.from_previous = from_previous;
        info!(
            "Stage identity: {:016x} -> {:016x} ({reason}), from previous: {:?}",
            self.stored.previous_identity, self.stored.identity, from_previous
        );
        self.dirty = true;
    }

    /// xrEventDataReferenceSpaceChangePending
    pub fn on_reference_space_change(&mut self, event: &xr::ReferenceSpaceChangePending) {
        if event.reference_space_type() != xr::ReferenceSpaceType::STAGE {
            return;
        }
        if !event.pose_valid() {
            self.change_stage(None, "STAGE change event without pose");
        } else {
            // Pose of the new STAGE origin in the previous STAGE: p_previous = P * p_new
            let pose_in_previous = crate::from_xr_pose(event.pose_in_previous_space());
            let (meters, degrees) = pose_difference(&pose_in_previous, &Pose::IDENTITY);
            if meters <= SAME_STAGE_METERS && degrees <= SAME_STAGE_DEGREES {
                info!(
                    "Stage identity: STAGE change event within tolerance ({:.2} mm, {degrees:.3} \
                     deg), same stage",
                    meters * 1000.0
                );
                return;
            }
            self.change_stage(
                Some(pose_in_previous.inverse()),
                &format!(
                    "STAGE change event, {:.1} mm, {degrees:.2} deg",
                    meters * 1000.0
                ),
            );
        }
        // The anchor will be seen elsewhere in the new STAGE: that is the same move
        if self.stored.anchor_reference.is_some() {
            self.stored.anchor_reference = None;
        }
        self.moved_confirmations = 0;
        self.stored.save();
    }

    pub fn on_anchor_request(&mut self, request: AnchorRequest) {
        if request.sequence == self.stored.request_served {
            return;
        }
        info!(
            "Stage identity: anchor request {} {:?}",
            request.sequence, request.kind
        );
        if let Some(anchor) = &mut self.anchor {
            anchor.command(match request.kind {
                AnchorRequestKind::Create(pose) => AnchorCommand::Create(pose),
                AnchorRequestKind::Forget => AnchorCommand::Forget,
            });
        }
        self.stored.request_served = request.sequence;
        self.stored.save();
        self.dirty = true;
    }

    /// Once per frame while streaming
    pub fn update(&mut self, core_context: &ClientCoreContext, stage: &xr::Space, time: xr::Time) {
        if let Some(anchor) = &mut self.anchor
            && anchor.poll(stage, time)
        {
            self.dirty = true;
            self.on_anchor_status();
        }

        if self.dirty
            || self
                .last_sent
                .is_none_or(|t| t.elapsed() >= RESEND_INTERVAL)
        {
            core_context.send_stage_info(self.info());
            self.last_sent = Some(Instant::now());
            self.dirty = false;
        }
    }

    fn on_anchor_status(&mut self) {
        let Some(anchor) = &self.anchor else {
            return;
        };
        let status = anchor.status.clone();

        if status.generation != self.stored.anchor_generation {
            self.stored.anchor_generation = status.generation;
            self.stored.save();
        }
        if status.uuid != self.stored.anchor_uuid {
            // Created, replaced or forgotten: a new reference
            self.stored.anchor_uuid = status.uuid;
            self.stored.anchor_reference = None;
            self.moved_confirmations = 0;
            self.stored.save();
        }

        if status.state != ANCHOR_LOCATED {
            return;
        }
        let Some(pose) = status.pose else {
            return;
        };
        let Some(reference) = self.stored.anchor_reference else {
            self.stored.anchor_reference = Some(pose);
            self.stored.save();
            return;
        };

        let (meters, degrees) = pose_difference(&pose, &reference);
        if meters > ANCHOR_MOVED_METERS || degrees > ANCHOR_MOVED_DEGREES {
            self.moved_confirmations += 1;
            if self.moved_confirmations >= ANCHOR_MOVED_CONFIRMATIONS {
                // A physical point: p_reference_stage -> anchor local -> p_current_stage
                self.change_stage(
                    Some(pose * reference.inverse()),
                    &format!("anchor moved {:.1} mm, {degrees:.2} deg", meters * 1000.0),
                );
                self.stored.anchor_reference = Some(pose);
                self.moved_confirmations = 0;
                self.stored.save();
            }
        } else {
            self.moved_confirmations = 0;
        }
    }

    pub fn info(&self) -> StageInfo {
        let status = self.anchor.as_ref().map(|anchor| anchor.status.clone());
        StageInfo {
            stage_identity: self.stored.identity,
            previous_stage_identity: self.stored.previous_identity,
            stage_change_sequence: self.stored.change_sequence,
            stage_from_previous: self.from_previous,
            anchor_state: status
                .as_ref()
                .map_or(ANCHOR_UNSUPPORTED, |status| status.state),
            anchor_uuid: status.as_ref().map_or([0; 16], |status| status.uuid),
            anchor_pose: status.as_ref().and_then(|status| status.pose),
            anchor_generation: status.as_ref().map_or(0, |status| status.generation),
            anchor_request_served: self.stored.request_served,
            product_name: self.product_name.clone(),
        }
    }
}
