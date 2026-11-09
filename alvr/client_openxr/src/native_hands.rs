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
    GraphicsContext, HAND_JOINT_COUNT, HandsRenderer, HandsViewParams, gl_format_to_wgpu,
};
use openxr as xr;
use std::{rc::Rc, time::Duration};

// Resolution of the hands layer relative to the stream view resolution (rendered with 4x MSAA)
const RESOLUTION_SCALE: f32 = 0.5;
// XR_EXT_hand_tracking joints: wrist, then the middle finger metacarpal..tip
const MIDDLE_FINGER_CHAIN: [usize; 6] = [1, 11, 12, 13, 14, 15];

// Sum of the bone lengths from the wrist to the middle finger tip
fn middle_finger_length(positions: &[Vec3; HAND_JOINT_COUNT]) -> f32 {
    MIDDLE_FINGER_CHAIN
        .windows(2)
        .map(|pair| positions[pair[0]].distance(positions[pair[1]]))
        .sum()
}

pub struct NativeHands {
    xr_session: xr::Session<xr::OpenGlEs>,
    swapchains: [xr::Swapchain<xr::OpenGlEs>; 2],
    resolution: UVec2,
    renderer: HandsRenderer,
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
        let resolution = (stream_view_resolution.as_vec2() * RESOLUTION_SCALE)
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
        match HandsRenderer::new(Rc::clone(gfx_ctx), resolution, images, wgpu_format) {
            Ok(renderer) => {
                info!(
                    "Native hands: layer {}x{}, {wgpu_format:?}",
                    resolution.x, resolution.y
                );

                Some(Self {
                    xr_session: xr_session.clone(),
                    swapchains,
                    resolution,
                    renderer,
                    rendered_views: None,
                    visible_log_state: [false; 2],
                })
            }
            Err(e) => {
                warn!("Native hands: disabled, {e}");

                None
            }
        }
    }

    // Joints of a tracked hand, None when the hand is not tracked or the runtime selected the
    // controller for it.
    fn locate_hand(
        stage_space: &xr::Space,
        tracker: Option<&xr::HandTracker>,
        selected: &RelaxedAtomic,
        time: xr::Time,
    ) -> Option<[Pose; HAND_JOINT_COUNT]> {
        if !selected.value() {
            return None;
        }

        let locations = stage_space.locate_hand_joints(tracker?, time).ok()??;
        let all_valid = locations.iter().all(|location| {
            location.location_flags.contains(
                xr::SpaceLocationFlags::ORIENTATION_VALID | xr::SpaceLocationFlags::POSITION_VALID,
            )
        });

        all_valid.then(|| locations.map(|location| crate::from_xr_pose(location.pose)))
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

        let hands = {
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

        let mut swapchain_indices = [0; 2];
        for (swapchain, index) in self.swapchains.iter_mut().zip(&mut swapchain_indices) {
            let Ok(acquired) = swapchain.acquire_image() else {
                return;
            };
            *index = acquired;
            swapchain.wait_image(xr::Duration::INFINITE).ok();
        }

        self.renderer.render(
            [0, 1].map(|idx| HandsViewParams {
                swapchain_index: swapchain_indices[idx],
                view_params: ViewParams {
                    pose: crate::from_xr_pose(views[idx].pose),
                    fov: crate::from_xr_fov(views[idx].fov),
                },
            }),
            hands,
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
