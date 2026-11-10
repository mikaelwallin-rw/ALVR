// Hand-tracked hands drawn natively by the client, in their own alpha-blended projection layer
// above the stream (and above the native quads). The joints are located at the frame's predicted
// display time on the render thread, so the hands have no stream latency and do not depend on what
// the PC application renders.

use crate::{graphics, interaction::InteractionContext};
use alvr_common::{
    Pose, RelaxedAtomic, ViewParams,
    glam::{UVec2, Vec3},
    info,
    parking_lot::RwLock,
    warn,
};
use alvr_graphics::{
    GraphicsContext, HAND_JOINT_COUNT, HandShape, HandsRenderer, HandsViewParams,
    finger_length_factors, gl_format_to_wgpu, lengthen_fingers,
};
use alvr_packets::{HandCorrection, HandDisplay, HandShapeInfo};
use openxr as xr;
use std::time::Instant;
use std::{rc::Rc, time::Duration};

// Defaults of the hands layer: resolution relative to the stream view resolution, and MSAA sample
// count (debug switches screenvr_hands_scale.txt / screenvr_hands_msaa.txt override them live)
const DEFAULT_RESOLUTION_SCALE: f32 = 0.5;
const DEFAULT_SAMPLE_COUNT: u32 = 4;
// A hand counts as in view when one of these joints (wrist, fingertips) is inside an eye's FoV
// widened by this margin
const IN_VIEW_JOINTS: [usize; 6] = [1, 5, 10, 15, 20, 25];
const IN_VIEW_MARGIN_RAD: f32 = 0.1;
// XR_EXT_hand_tracking joints: wrist, then the middle finger metacarpal..tip
const MIDDLE_FINGER_CHAIN: [usize; 6] = [1, 11, 12, 13, 14, 15];

// Live hand shape (debug switch screenvr_hands.txt)
#[derive(Clone, Copy, Debug, PartialEq)]
struct ShapeSettings {
    thickness: f32,
    scale: f32,
    // Finger bone length factor
    fingers: f32,
    // Overlay app's hand display transform applied (A/B switch display_transform=off)
    display_transform: bool,
    // Girth per joint from the tracked joint radii (relative to the mesh's own)
    from_radii: bool,
}

impl Default for ShapeSettings {
    fn default() -> Self {
        Self {
            thickness: 1.0,
            scale: 1.0,
            fingers: 1.0,
            display_transform: true,
            from_radii: false,
        }
    }
}

impl ShapeSettings {
    fn parse(text: &str) -> Self {
        let mut settings = Self::default();
        for line in text.lines() {
            let Some((key, value)) = line.split_once('=') else {
                continue;
            };
            let value = value.trim();
            let number = |current: f32, min: f32, max: f32| {
                value
                    .parse::<f32>()
                    .ok()
                    .filter(|v| (min..=max).contains(v))
                    .unwrap_or(current)
            };
            match key.trim() {
                "thickness" => settings.thickness = number(settings.thickness, 0.5, 3.0),
                "scale" => settings.scale = number(settings.scale, 0.8, 1.3),
                "fingers" => settings.fingers = number(settings.fingers, 0.9, 1.3),
                "girth" => settings.from_radii = value == "radii",
                "display_transform" => settings.display_transform = value != "off",
                _ => (),
            }
        }
        settings
    }

    fn shape(
        &self,
        tracked_radii: Option<&[f32; HAND_JOINT_COUNT]>,
        mesh_radii: &[f32; HAND_JOINT_COUNT],
        fingers: f32,
    ) -> HandShape {
        let girth = std::array::from_fn(|joint| {
            let ratio = match tracked_radii {
                Some(radii) if self.from_radii && mesh_radii[joint] > 0.0 && radii[joint] > 0.0 => {
                    (radii[joint] / mesh_radii[joint]).clamp(0.5, 2.5)
                }
                _ => 1.0,
            };
            self.thickness * ratio
        });
        HandShape {
            girth,
            length: finger_length_factors(fingers),
            scale: self.scale,
        }
    }
}

// Overlay app correction factor for one hand: full within half `near_meters` of the plane, fading
// (smoothstep) to none at `near_meters`
fn correction_factor(correction: &HandCorrection, hand: usize, index_tip: Vec3) -> f32 {
    let scale = correction.finger_scale[hand];
    if !scale.is_finite() || correction.near_meters <= 0.0 {
        return 1.0;
    }
    let normal = correction.plane.orientation * Vec3::Z;
    let distance = normal.dot(index_tip - correction.plane.position).abs();
    let near = correction.near_meters;
    let t = ((near - distance) / (near * 0.5)).clamp(0.0, 1.0);
    let t = t * t * (3.0 - 2.0 * t);
    1.0 + (scale.clamp(0.8, 1.5) - 1.0) * t
}

// Sum of the bone lengths from the wrist to the middle finger tip
fn middle_finger_length(positions: &[Vec3; HAND_JOINT_COUNT]) -> f32 {
    MIDDLE_FINGER_CHAIN
        .windows(2)
        .map(|pair| positions[pair[0]].distance(positions[pair[1]]))
        .sum()
}

// Swapchains, their resolution and the renderer drawing into them
type HandsLayer = ([xr::Swapchain<xr::OpenGlEs>; 2], UVec2, HandsRenderer);

pub struct NativeHands {
    xr_session: xr::Session<xr::OpenGlEs>,
    gfx_ctx: Rc<GraphicsContext>,
    stream_view_resolution: UVec2,
    // (resolution scale, sample count) of the current layer
    settings: (f32, u32),
    swapchains: [xr::Swapchain<xr::OpenGlEs>; 2],
    resolution: UVec2,
    renderer: HandsRenderer,
    in_view_log_state: Option<bool>,
    shape_text: String,
    shape_settings: ShapeSettings,
    // Overlay app's automatic correction, and what was applied in the last frame
    correction: Option<HandCorrection>,
    applied_correction: [f32; 2],
    display: crate::hand_display::HandDisplayState,
    // View poses and FoVs of the last rendered frame, None when no hand was drawn
    rendered_views: Option<[xr::View; 2]>,
    visible_log_state: [bool; 2],
}

impl NativeHands {
    pub fn new(
        xr_session: &xr::Session<xr::OpenGlEs>,
        gfx_ctx: &Rc<GraphicsContext>,
        stream_view_resolution: UVec2,
    ) -> Option<Self> {
        let settings = Self::wanted_settings();
        let (swapchains, resolution, renderer) =
            Self::create_layer(xr_session, gfx_ctx, stream_view_resolution, settings)?;

        Some(Self {
            xr_session: xr_session.clone(),
            gfx_ctx: Rc::clone(gfx_ctx),
            stream_view_resolution,
            settings,
            swapchains,
            resolution,
            renderer,
            in_view_log_state: None,
            shape_text: String::new(),
            shape_settings: ShapeSettings::default(),
            correction: None,
            applied_correction: [1.0; 2],
            display: crate::hand_display::HandDisplayState::default(),
            rendered_views: None,
            visible_log_state: [false; 2],
        })
    }

    fn wanted_settings() -> (f32, u32) {
        let switches = crate::debug_switches::get();
        (
            switches.hands_scale.unwrap_or(DEFAULT_RESOLUTION_SCALE),
            switches.hands_msaa.unwrap_or(DEFAULT_SAMPLE_COUNT),
        )
    }

    fn create_layer(
        xr_session: &xr::Session<xr::OpenGlEs>,
        gfx_ctx: &Rc<GraphicsContext>,
        stream_view_resolution: UVec2,
        (scale, sample_count): (f32, u32),
    ) -> Option<HandsLayer> {
        let resolution = (stream_view_resolution.as_vec2() * scale)
            .round()
            .as_uvec2()
            .max(UVec2::ONE);

        gfx_ctx.make_current();
        let (format, _) = graphics::srgb_swapchain_format(xr_session);
        let swapchains = [
            graphics::create_swapchain(xr_session, gfx_ctx, resolution, format, None),
            graphics::create_swapchain(xr_session, gfx_ctx, resolution, format, None),
        ];
        let images = swapchains.each_ref().map(|swapchain| {
            swapchain
                .enumerate_images()
                .unwrap_or_default()
                .into_iter()
                .collect::<Vec<_>>()
        });

        let wgpu_format = gl_format_to_wgpu(format);
        match HandsRenderer::new(
            Rc::clone(gfx_ctx),
            resolution,
            images,
            wgpu_format,
            sample_count,
        ) {
            Ok(renderer) => {
                info!(
                    "Native hands: layer {}x{} (scale {scale}), {sample_count}x MSAA, \
                     {wgpu_format:?}",
                    resolution.x, resolution.y
                );

                Some((swapchains, resolution, renderer))
            }
            Err(e) => {
                warn!("Native hands: disabled, {e}");

                None
            }
        }
    }

    /// The overlay app's automatic hand correction (latest replaces earlier)
    pub fn set_correction(&mut self, correction: HandCorrection) {
        info!("Native hands: correction {correction:?}");
        self.correction = Some(correction);
    }

    /// The overlay app's hand display transform (glides from the current one)
    pub fn set_display(&mut self, display: HandDisplay) {
        info!("Native hands: display transform {display:?}");
        self.display.set(display, Instant::now());
    }

    /// What the hands are drawn with (reported to the server for the overlay app)
    pub fn shape_info(&self) -> HandShapeInfo {
        HandShapeInfo {
            thickness: self.shape_settings.thickness,
            scale: self.shape_settings.scale,
            fingers: self.shape_settings.fingers,
            finger_correction: self.applied_correction,
            correction_sequence: self.correction.map_or(0, |correction| correction.sequence),
        }
    }

    // Joints of a tracked hand and their radii (meters), None when the hand is not tracked or the
    // runtime selected the controller for it.
    fn locate_hand(
        stage_space: &xr::Space,
        tracker: Option<&xr::HandTracker>,
        selected: &RelaxedAtomic,
        time: xr::Time,
    ) -> Option<([Pose; HAND_JOINT_COUNT], [f32; HAND_JOINT_COUNT])> {
        if !selected.value() {
            return None;
        }

        let locations = stage_space.locate_hand_joints(tracker?, time).ok()??;
        let all_valid = locations.iter().all(|location| {
            location.location_flags.contains(
                xr::SpaceLocationFlags::ORIENTATION_VALID | xr::SpaceLocationFlags::POSITION_VALID,
            )
        });

        all_valid.then(|| {
            (
                locations.map(|location| crate::from_xr_pose(location.pose)),
                locations.map(|location| location.radius),
            )
        })
    }

    pub fn render(
        &mut self,
        stage_space: &xr::Space,
        interaction_ctx: &RwLock<InteractionContext>,
        hands_selected: &[RelaxedAtomic; 2],
        display_time: Duration,
    ) {
        self.rendered_views = None;
        let xr_time = crate::to_xr_time(display_time);

        // Debug switches: rebuild the layer when its resolution scale or MSAA changes
        let settings = Self::wanted_settings();
        if settings != self.settings {
            if let Some((swapchains, resolution, renderer)) = Self::create_layer(
                &self.xr_session,
                &self.gfx_ctx,
                self.stream_view_resolution,
                settings,
            ) {
                self.swapchains = swapchains;
                self.resolution = resolution;
                self.renderer = renderer;
            }
            self.settings = settings;
        }

        let located = {
            let int_ctx = interaction_ctx.read();
            [0, 1].map(|idx| {
                Self::locate_hand(
                    stage_space,
                    int_ctx.hands_interaction[idx].skeleton_tracker.as_ref(),
                    &hands_selected[idx],
                    xr_time,
                )
            })
        };
        let radii = located.map(|hand| hand.map(|(_, radii)| radii));
        let mut hands = located.map(|hand| hand.map(|(joints, _)| joints));

        // Debug switch screenvr_hands.txt: hand shape, live
        let shape_text = crate::debug_switches::get().hands_shape;
        if shape_text != self.shape_text {
            self.shape_settings = ShapeSettings::parse(&shape_text);
            info!("Native hands: shape {:?}", self.shape_settings);
            self.shape_text = shape_text;
        }

        // TEST ONLY (debug switch screenvr_fake_hands.txt): a synthetic left hand in front of the
        // STAGE origin, so that the per-frame uploads run on an unworn headset
        if hands.iter().all(Option::is_none)
            && crate::debug_switches::get().fake_hands
            && let Ok((_, views)) = self.xr_session.locate_views(
                xr::ViewConfigurationType::PRIMARY_STEREO,
                xr_time,
                stage_space,
            )
            && let Some(view) = views.first()
        {
            // 35 cm in front of and 12 cm below the left eye, at arm's length
            let head = crate::from_xr_pose(view.pose);
            let anchor = head.position + head.orientation * Vec3::new(0.05, -0.12, -0.35);
            hands[0] = Some(self.renderer.mesh_joint_positions(0).map(|position| Pose {
                orientation: head.orientation,
                position: anchor + head.orientation * position,
            }));
        }

        let visible = [hands[0].is_some(), hands[1].is_some()];
        for (idx, joints) in hands.iter().enumerate() {
            if let Some(joints) = joints
                && !self.visible_log_state[idx]
            {
                let tracked = middle_finger_length(&joints.map(|joint| joint.position));
                let mesh = middle_finger_length(&self.renderer.mesh_joint_positions(idx));
                let check = self.renderer.check_skinning(idx, joints);
                info!(
                    "Native hands: {} hand, wrist to middle tip: tracked {:.1} cm, mesh {:.1} cm; \
                     skinned fingertips max {:.1} cm from the tracked tips, {} non-finite vertices",
                    if idx == 0 { "left" } else { "right" },
                    tracked * 100.0,
                    mesh * 100.0,
                    check.max_tip_error * 100.0,
                    check.non_finite_vertices
                );
                // Joint radii (XR_EXT_hand_tracking) against the mesh's own girth, in mm
                let mesh_radii = self.renderer.mesh_joint_radii(idx);
                info!(
                    "Native hands: {} hand joint radii mm (tracked/mesh): {}",
                    if idx == 0 { "left" } else { "right" },
                    (0..HAND_JOINT_COUNT)
                        .map(|joint| format!(
                            "{joint}:{:.1}/{:.1}",
                            radii[idx].map_or(0.0, |radii| radii[joint]) * 1000.0,
                            mesh_radii[joint] * 1000.0
                        ))
                        .collect::<Vec<_>>()
                        .join(" ")
                );
            }
        }
        if visible != self.visible_log_state {
            info!(
                "Native hands: visible left {}, right {}",
                visible[0], visible[1]
            );
            self.visible_log_state = visible;
        }
        if hands.iter().all(Option::is_none) {
            return;
        }

        let Ok((flags, views)) = self.xr_session.locate_views(
            xr::ViewConfigurationType::PRIMARY_STEREO,
            xr_time,
            stage_space,
        ) else {
            return;
        };
        if !flags
            .contains(xr::ViewStateFlags::ORIENTATION_VALID | xr::ViewStateFlags::POSITION_VALID)
        {
            return;
        }
        let Ok(views): Result<[xr::View; 2], _> = views.try_into() else {
            return;
        };

        // Tracked but outside both eyes' FoV (e.g. hands down): no draw, no layer
        let in_view = hands
            .iter()
            .flatten()
            .any(|joints| hand_in_view(joints, &views));
        if self.in_view_log_state != Some(in_view) {
            info!("Native hands: in view {in_view}");
            self.in_view_log_state = Some(in_view);
        }
        if !in_view {
            return;
        }

        let mut swapchain_indices = [0; 2];
        for (swapchain, index) in self.swapchains.iter_mut().zip(&mut swapchain_indices) {
            let Ok(acquired) = swapchain.acquire_image() else {
                return;
            };
            *index = acquired;
            swapchain.wait_image(xr::Duration::INFINITE).ok();
        }

        // Finger length: own setting times the overlay app's correction near the touchscreen. Only
        // the DRAWN joints change; the joints reported to the server are the raw tracked ones.
        let fingers = [0, 1].map(|idx| {
            let correction = match (&self.correction, &hands[idx]) {
                (Some(correction), Some(joints)) => {
                    correction_factor(correction, idx, joints[10].position)
                }
                _ => 1.0,
            };
            self.applied_correction[idx] = correction;
            self.shape_settings.fingers * correction
        });
        // Then the overlay app's display transform, weighted by the raw index tip's distance from
        // the published touch display (sizes scale with it)
        let now = Instant::now();
        let display = [0, 1].map(|idx| match &hands[idx] {
            Some(joints) if self.shape_settings.display_transform => {
                self.display.for_hand(joints[10].position, now)
            }
            _ => crate::hand_display::DisplayTransform::IDENTITY,
        });
        let drawn = [0, 1].map(|idx| {
            hands[idx].map(|joints| {
                let joints = if (fingers[idx] - 1.0).abs() > 1e-4 {
                    lengthen_fingers(&joints, fingers[idx])
                } else {
                    joints
                };
                joints.map(|joint| display[idx].apply_pose(joint))
            })
        });

        self.renderer.render(
            [0, 1].map(|idx| HandsViewParams {
                swapchain_index: swapchain_indices[idx],
                view_params: ViewParams {
                    pose: crate::from_xr_pose(views[idx].pose),
                    fov: crate::from_xr_fov(views[idx].fov),
                },
            }),
            drawn,
            [0, 1].map(|idx| {
                let mut shape = self.shape_settings.shape(
                    radii[idx].as_ref(),
                    &self.renderer.mesh_joint_radii(idx),
                    fingers[idx],
                );
                let size = display[idx].scale;
                shape.girth = shape.girth.map(|girth| girth * size);
                shape.length = shape.length.map(|length| length * size);
                shape
            }),
        );

        for swapchain in &mut self.swapchains {
            swapchain.release_image().ok();
        }

        self.rendered_views = Some(views);
    }

    /// Layer of the last rendered frame, None when no hand is visible
    pub fn build<'a>(
        &'a self,
        stage_space: &'a xr::Space,
    ) -> Option<graphics::ProjectionLayerBuilder<'a>> {
        let views = self.rendered_views?;

        let rect = xr::Rect2Di {
            offset: xr::Offset2Di { x: 0, y: 0 },
            extent: xr::Extent2Di {
                width: self.resolution.x as _,
                height: self.resolution.y as _,
            },
        };

        Some(graphics::ProjectionLayerBuilder::new(
            stage_space,
            [0, 1].map(|idx| {
                xr::CompositionLayerProjectionView::new()
                    .pose(views[idx].pose)
                    .fov(views[idx].fov)
                    .sub_image(
                        xr::SwapchainSubImage::new()
                            .swapchain(&self.swapchains[idx])
                            .image_array_index(0)
                            .image_rect(rect),
                    )
            }),
            Some(graphics::ProjectionLayerAlphaConfig {
                premultiplied: true,
            }),
            None,
        ))
    }
}

// Whether one of the hand's key joints is inside an eye's FoV (widened by IN_VIEW_MARGIN_RAD)
fn hand_in_view(joints: &[Pose; HAND_JOINT_COUNT], views: &[xr::View; 2]) -> bool {
    views.iter().any(|view| {
        let eye = crate::from_xr_pose(view.pose);
        let eye_from_stage = eye.orientation.inverse();
        let fov = view.fov;
        IN_VIEW_JOINTS.iter().any(|&joint| {
            let p = eye_from_stage * (joints[joint].position - eye.position);
            if p.z > -0.02 {
                return false; // behind the eye
            }
            let (h, v) = ((p.x / -p.z).atan(), (p.y / -p.z).atan());
            h >= fov.angle_left - IN_VIEW_MARGIN_RAD
                && h <= fov.angle_right + IN_VIEW_MARGIN_RAD
                && v >= fov.angle_down - IN_VIEW_MARGIN_RAD
                && v <= fov.angle_up + IN_VIEW_MARGIN_RAD
        })
    })
}
