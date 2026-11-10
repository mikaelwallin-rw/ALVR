// Hand display transform (overlay app contract: VrTrackingRequestBlock v4 in ScreenVR's
// VrTrackingExtras.h, which has the full semantics): the DRAWN hands near the touch display are
// moved by the transform the touch quad underwent (true pose -> published, passthrough-compensated
// pose), so that the drawn hand meets the drawn display where the real finger meets the real glass
// in passthrough. F(x) = c' + R s (x - c); one weight per hand from the distance of F(raw index tip)
// to the published quad rectangle (full near it, smoothstep to none at the falloff); partial
// transform F_w(x) = c_w + R_w s_w (x - c) with c_w = c + w (c' - c). A jump of the target (active
// toggled, e.g. scene mode, or stale) glides over GLIDE like the quad; continuous changes apply
// directly. Order: finger length first (the hand's own frame), then this. Reported joints stay raw.

use alvr_common::{
    Pose,
    glam::{Quat, Vec2, Vec3},
};
use alvr_packets::HandDisplay;
use std::time::{Duration, Instant};

pub const GLIDE: Duration = Duration::from_millis(500);

/// F(x) = published_centre + rotation * scale * (x - true_centre)
#[derive(Clone, Copy, Debug)]
pub struct DisplayTransform {
    pub rotation: Quat,
    pub scale: f32,
    pub true_centre: Vec3,
    pub published_centre: Vec3,
}

impl DisplayTransform {
    pub const IDENTITY: Self = Self {
        rotation: Quat::IDENTITY,
        scale: 1.0,
        true_centre: Vec3::ZERO,
        published_centre: Vec3::ZERO,
    };

    pub fn apply_point(&self, p: Vec3) -> Vec3 {
        self.published_centre + self.rotation * ((p - self.true_centre) * self.scale)
    }

    pub fn apply_pose(&self, pose: Pose) -> Pose {
        Pose {
            orientation: self.rotation * pose.orientation,
            position: self.apply_point(pose.position),
        }
    }

    /// Weight w: R_w = slerp(I, R, w), s_w = 1 + w (s - 1), c_w = c + w (c' - c)
    pub fn partial(&self, w: f32) -> Self {
        let w = w.clamp(0.0, 1.0);
        Self {
            rotation: Quat::IDENTITY.slerp(self.rotation, w),
            scale: 1.0 + (self.scale - 1.0) * w,
            true_centre: self.true_centre,
            published_centre: self.true_centre.lerp(self.published_centre, w),
        }
    }

    /// Between two transforms, as functions: interpolated rotation and scale, and the image of a
    /// common point (b's true centre), so that both ends match exactly
    pub fn blend(a: &Self, b: &Self, k: f32) -> Self {
        let k = k.clamp(0.0, 1.0);
        // The display's true centre (from whichever end is not the identity): hands near the display
        // then glide on straight lines
        let anchor = if b.true_centre == Vec3::ZERO && b.published_centre == Vec3::ZERO {
            a.true_centre
        } else {
            b.true_centre
        };
        Self {
            rotation: a.rotation.slerp(b.rotation, k),
            scale: a.scale + (b.scale - a.scale) * k,
            true_centre: anchor,
            published_centre: a.apply_point(anchor).lerp(b.apply_point(anchor), k),
        }
    }
}

fn smoothstep(x: f32) -> f32 {
    let x = x.clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

fn ease_in_out(x: f32) -> f32 {
    0.5 - 0.5 * (std::f32::consts::PI * x.clamp(0.0, 1.0)).cos()
}

/// Distance from a point to a rectangle (centre pose, local XY plane, full size); 0 on it
pub fn distance_to_rect(point: Vec3, pose: Pose, size: [f32; 2]) -> f32 {
    let local = pose.orientation.inverse() * (point - pose.position);
    let half = Vec2::new(size[0], size[1]) * 0.5;
    let clamped = local.truncate().clamp(-half, half);
    local.distance(clamped.extend(0.0))
}

fn target(display: Option<&HandDisplay>) -> DisplayTransform {
    match display {
        Some(display) if display.active => {
            let published_centre = display.quad_pose.position;
            // c = R^T (c' - t)
            let true_centre = display.transform.orientation.inverse()
                * (published_centre - display.transform.position);
            DisplayTransform {
                rotation: display.transform.orientation,
                scale: if display.scale.is_finite() && display.scale > 0.0 {
                    display.scale
                } else {
                    1.0
                },
                true_centre,
                published_centre,
            }
        }
        _ => DisplayTransform::IDENTITY,
    }
}

#[derive(Default)]
pub struct HandDisplayState {
    display: Option<HandDisplay>,
    // The effective transform when the last jump started
    from: Option<DisplayTransform>,
    glide_start: Option<Instant>,
}

impl HandDisplayState {
    /// The current (unweighted) transform
    pub fn current(&self, now: Instant) -> DisplayTransform {
        let to = target(self.display.as_ref());
        match (self.from, self.glide_start) {
            (Some(from), Some(start)) if now.saturating_duration_since(start) < GLIDE => {
                DisplayTransform::blend(
                    &from,
                    &to,
                    ease_in_out(
                        now.saturating_duration_since(start).as_secs_f32() / GLIDE.as_secs_f32(),
                    ),
                )
            }
            _ => to,
        }
    }

    pub fn set(&mut self, display: HandDisplay, now: Instant) {
        let was_active = self.display.is_some_and(|previous| previous.active);
        if display.active != was_active {
            // A jump (scene mode on/off, stale): glide like the touch quad
            self.from = Some(self.current(now));
            self.glide_start = Some(now);
        }
        self.display = Some(display);
    }

    /// The transform for a hand whose raw index tip is at `index_tip`: weight from the distance of
    /// F(index tip) to the published quad rectangle
    pub fn for_hand(&self, index_tip: Vec3, now: Instant) -> DisplayTransform {
        let current = self.current(now);
        let Some(display) = &self.display else {
            return current;
        };
        let distance = distance_to_rect(
            current.apply_point(index_tip),
            display.quad_pose,
            display.quad_size,
        );
        let full = display.full_meters.max(0.0);
        let weight = if display.falloff_meters <= full {
            if distance <= full { 1.0 } else { 0.0 }
        } else {
            1.0 - smoothstep((distance - full) / (display.falloff_meters - full))
        };
        current.partial(weight)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn same(a: &DisplayTransform, b: &DisplayTransform) -> bool {
        let points = [Vec3::ZERO, Vec3::X, Vec3::new(0.3, 1.2, -0.6)];
        points
            .iter()
            .all(|&p| a.apply_point(p).distance(b.apply_point(p)) < 1e-5)
            && a.rotation.angle_between(b.rotation) < 1e-5
    }

    fn display(active: bool) -> HandDisplay {
        let true_pose = Pose {
            orientation: Quat::IDENTITY,
            position: Vec3::new(0.3, 1.24, -0.63),
        };
        let transform = Pose {
            orientation: Quat::from_rotation_x(2f32.to_radians()),
            position: Vec3::new(0.0, -0.04, 0.03),
        };
        HandDisplay {
            sequence: 1,
            active,
            transform,
            scale: 1.04,
            // The published quad = R * (true pose)
            quad_pose: transform * true_pose,
            quad_size: [0.22, 0.14],
            full_meters: 0.05,
            falloff_meters: 0.25,
        }
    }

    #[test]
    fn the_true_display_centre_maps_to_the_published_one() {
        let display = display(true);
        let transform = target(Some(&display));
        let true_centre = Vec3::new(0.3, 1.24, -0.63);
        assert!(transform.true_centre.distance(true_centre) < 1e-5);
        assert!(
            transform
                .apply_point(true_centre)
                .distance(display.quad_pose.position)
                < 1e-5
        );
        assert!(
            transform
                .partial(0.0)
                .apply_point(Vec3::X)
                .distance(Vec3::X)
                < 1e-6
        );
    }

    #[test]
    fn weight_full_near_the_panel_none_beyond_the_falloff() {
        let now = Instant::now();
        let mut state = HandDisplayState::default();
        state.set(display(true), now - GLIDE * 2);
        let full = target(Some(&display(true)));
        // A raw tip that F maps onto the published panel's area: full weight
        let on_panel = full.true_centre + Vec3::new(0.0, 0.0, 0.02);
        assert!(same(&state.for_hand(on_panel, now), &full));
        // Far away: identity
        let far = state.for_hand(full.true_centre + Vec3::new(0.0, 0.0, 0.6), now);
        let p = Vec3::new(0.1, 0.2, 0.3);
        assert!(far.apply_point(p).distance(p) < 1e-6);
        // Beside the panel counts from its edge
        let pose = display(true).quad_pose;
        let beside = pose.position + pose.orientation * Vec3::new(0.11 + 0.03, 0.0, 0.0);
        assert!((distance_to_rect(beside, pose, [0.22, 0.14]) - 0.03).abs() < 1e-5);
    }

    #[test]
    fn a_jump_glides_continuous_changes_do_not() {
        let now = Instant::now();
        let mut state = HandDisplayState::default();
        state.set(display(true), now - GLIDE * 2);
        // Continuous change while active: applied at once
        let mut moved = display(true);
        moved.sequence = 2;
        moved.transform.position.y -= 0.002;
        moved.quad_pose.position.y -= 0.002;
        state.set(moved, now);
        assert!(same(&state.current(now), &target(Some(&moved))));
        // Scene mode: inactive; halfway through the glide, halfway between
        let mut off = moved;
        off.active = false;
        state.set(off, now);
        let p = target(Some(&moved)).true_centre;
        let halfway = state.current(now + GLIDE / 2).apply_point(p);
        let expected = p.lerp(target(Some(&moved)).apply_point(p), 0.5);
        assert!(halfway.distance(expected) < 1e-4);
        let after = state.current(now + GLIDE * 2);
        assert!(after.apply_point(p).distance(p) < 1e-6);
    }
}
