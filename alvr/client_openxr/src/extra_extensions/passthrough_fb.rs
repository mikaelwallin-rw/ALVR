use alvr_common::{Pose, info, warn};
use alvr_system_info::Platform;
use openxr::{
    self as xr, raw,
    sys::{self, Handle},
};
use std::ptr;

pub struct PassthroughFB {
    handle: sys::PassthroughFB,
    layer_handle: sys::PassthroughLayerFB,
    layer: sys::CompositionLayerPassthroughFB,
    ext_fns: raw::PassthroughFB,
    session: sys::Session,
    // PROTOTYPE (debug switch screenvr_projected_passthrough.txt): passthrough projected onto given
    // rectangles (the calibrated monitors) instead of the runtime's reconstruction
    projected: Option<ProjectedPassthrough>,
}

/// A rectangle to project passthrough onto: centre pose (STAGE; the rectangle lies in its local XY
/// plane) and full size in metres
#[derive(Clone, Copy, Debug)]
pub struct ProjectionRect {
    pub pose: Pose,
    pub width: f32,
    pub height: f32,
}

struct ProjectedPassthrough {
    layer_handle: sys::PassthroughLayerFB,
    layer: sys::CompositionLayerPassthroughFB,
    mesh_fns: raw::TriangleMeshFB,
    // Unit square in the XY plane, both windings (no culling either way)
    mesh: sys::TriangleMeshFB,
    instances: Vec<sys::GeometryInstanceFB>,
    // Rects (incl. margin) the instances were made for
    rects: Vec<ProjectionRect>,
}

impl PassthroughFB {
    pub fn new(session: &xr::Session<xr::OpenGlEs>, platform: Platform) -> xr::Result<Self> {
        let ext_fns = session
            .instance()
            .exts()
            .fb_passthrough
            .ok_or(sys::Result::ERROR_EXTENSION_NOT_PRESENT)?;

        let mut handle = sys::PassthroughFB::NULL;
        let info = sys::PassthroughCreateInfoFB {
            ty: sys::PassthroughCreateInfoFB::TYPE,
            next: ptr::null(),
            flags: sys::PassthroughFlagsFB::IS_RUNNING_AT_CREATION,
        };
        unsafe {
            super::xr_res((ext_fns.create_passthrough)(
                session.as_raw(),
                &raw const info,
                &raw mut handle,
            ))?
        };

        let mut layer_handle = sys::PassthroughLayerFB::NULL;
        let info = sys::PassthroughLayerCreateInfoFB {
            ty: sys::PassthroughLayerCreateInfoFB::TYPE,
            next: ptr::null(),
            passthrough: handle,
            flags: sys::PassthroughFlagsFB::IS_RUNNING_AT_CREATION,
            purpose: sys::PassthroughLayerPurposeFB::RECONSTRUCTION,
        };
        unsafe {
            super::xr_res((ext_fns.create_passthrough_layer)(
                session.as_raw(),
                &raw const info,
                &raw mut layer_handle,
            ))?
        };

        let layer = sys::CompositionLayerPassthroughFB {
            ty: sys::CompositionLayerPassthroughFB::TYPE,
            next: ptr::null(),
            flags: xr::CompositionLayerFlags::BLEND_TEXTURE_SOURCE_ALPHA,
            space: sys::Space::NULL,
            layer_handle,
        };

        // HACK: YVR runtime seems to ignore IS_RUNNING_AT_CREATION on versions <= 3.0.1
        if platform.is_yvr() {
            unsafe { super::xr_res((ext_fns.passthrough_start)(handle))? };
        }

        Ok(Self {
            handle,
            layer_handle,
            layer,
            ext_fns,
            session: session.as_raw(),
            projected: None,
        })
    }

    // return reference to make sure the passthrough handle is not dropped while the layer is in use
    pub fn layer(&self) -> &sys::CompositionLayerPassthroughFB {
        &self.layer
    }

    /// The projected layer, if enabled and it has geometry
    pub fn projected_layer(&self) -> Option<&sys::CompositionLayerPassthroughFB> {
        self.projected
            .as_ref()
            .filter(|projected| !projected.instances.is_empty())
            .map(|projected| &projected.layer)
    }

    /// PROTOTYPE: project passthrough onto these rectangles (None: off). Geometry instances are
    /// created or moved only when the rectangles change. Returns true when the projection was
    /// turned off: the caller must recreate the whole passthrough (PICO 5.15.7 breaks the remaining
    /// reconstruction layer when the projected layer is destroyed: "passthrough layer count is
    /// illegal", passthrough black until the passthrough object is recreated).
    pub fn set_projection(
        &mut self,
        instance: &xr::Instance,
        stage: &xr::Space,
        time: xr::Time,
        rects: Option<&[ProjectionRect]>,
    ) -> bool {
        let Some(rects) = rects else {
            if self.projected.is_some() {
                info!("Projected passthrough: off, passthrough to be recreated");
                return true;
            }
            return false;
        };
        if self.projected.is_none() {
            match ProjectedPassthrough::new(self, instance) {
                Ok(projected) => {
                    info!("Projected passthrough: layer and mesh created");
                    self.projected = Some(projected);
                }
                Err(e) => {
                    warn!("Projected passthrough: cannot create: {e}");
                    return false;
                }
            }
        }
        let ext_fns = self.ext_fns;
        let session = self.session;
        if let Some(projected) = &mut self.projected {
            projected.update(&ext_fns, session, stage, time, rects);
        }
        false
    }
}

fn to_sys_pose(pose: Pose) -> sys::Posef {
    crate::to_xr_pose(pose)
}

fn same_rect(a: &ProjectionRect, b: &ProjectionRect) -> bool {
    a.pose.position == b.pose.position
        && a.pose.orientation == b.pose.orientation
        && a.width.to_bits() == b.width.to_bits()
        && a.height.to_bits() == b.height.to_bits()
}

impl ProjectedPassthrough {
    fn new(fb: &PassthroughFB, instance: &xr::Instance) -> xr::Result<Self> {
        let mesh_fns = instance
            .exts()
            .fb_triangle_mesh
            .ok_or(sys::Result::ERROR_EXTENSION_NOT_PRESENT)?;

        let mut layer_handle = sys::PassthroughLayerFB::NULL;
        let info = sys::PassthroughLayerCreateInfoFB {
            ty: sys::PassthroughLayerCreateInfoFB::TYPE,
            next: ptr::null(),
            passthrough: fb.handle,
            flags: sys::PassthroughFlagsFB::IS_RUNNING_AT_CREATION,
            purpose: sys::PassthroughLayerPurposeFB::PROJECTED,
        };
        unsafe {
            super::xr_res((fb.ext_fns.create_passthrough_layer)(
                fb.session,
                &raw const info,
                &raw mut layer_handle,
            ))?
        };

        let vertices = [
            sys::Vector3f {
                x: -0.5,
                y: -0.5,
                z: 0.0,
            },
            sys::Vector3f {
                x: 0.5,
                y: -0.5,
                z: 0.0,
            },
            sys::Vector3f {
                x: 0.5,
                y: 0.5,
                z: 0.0,
            },
            sys::Vector3f {
                x: -0.5,
                y: 0.5,
                z: 0.0,
            },
        ];
        // Counter-clockwise seen from +Z, then the same triangles reversed
        let indices: [u32; 12] = [0, 1, 2, 0, 2, 3, 0, 2, 1, 0, 3, 2];
        let info = sys::TriangleMeshCreateInfoFB {
            ty: sys::TriangleMeshCreateInfoFB::TYPE,
            next: ptr::null(),
            flags: sys::TriangleMeshFlagsFB::EMPTY,
            winding_order: sys::WindingOrderFB::CCW,
            vertex_count: vertices.len() as u32,
            vertex_buffer: vertices.as_ptr(),
            triangle_count: (indices.len() / 3) as u32,
            index_buffer: indices.as_ptr(),
        };
        let mut mesh = sys::TriangleMeshFB::NULL;
        let result = unsafe {
            super::xr_res((mesh_fns.create_triangle_mesh)(
                fb.session,
                &raw const info,
                &raw mut mesh,
            ))
        };
        if let Err(e) = result {
            unsafe { (fb.ext_fns.destroy_passthrough_layer)(layer_handle) };
            return Err(e);
        }

        Ok(Self {
            layer_handle,
            layer: sys::CompositionLayerPassthroughFB {
                ty: sys::CompositionLayerPassthroughFB::TYPE,
                next: ptr::null(),
                flags: xr::CompositionLayerFlags::BLEND_TEXTURE_SOURCE_ALPHA,
                space: sys::Space::NULL,
                layer_handle,
            },
            mesh_fns,
            mesh,
            instances: Vec::new(),
            rects: Vec::new(),
        })
    }

    fn update(
        &mut self,
        ext_fns: &raw::PassthroughFB,
        session: sys::Session,
        stage: &xr::Space,
        time: xr::Time,
        rects: &[ProjectionRect],
    ) {
        if rects.len() == self.rects.len()
            && rects.iter().zip(&self.rects).all(|(a, b)| same_rect(a, b))
        {
            return;
        }
        if rects.len() != self.instances.len() {
            for instance in self.instances.drain(..) {
                unsafe { (ext_fns.destroy_geometry_instance)(instance) };
            }
            for rect in rects {
                let info = sys::GeometryInstanceCreateInfoFB {
                    ty: sys::GeometryInstanceCreateInfoFB::TYPE,
                    next: ptr::null(),
                    layer: self.layer_handle,
                    mesh: self.mesh,
                    base_space: stage.as_raw(),
                    pose: to_sys_pose(rect.pose),
                    scale: sys::Vector3f {
                        x: rect.width,
                        y: rect.height,
                        z: 1.0,
                    },
                };
                let mut instance = sys::GeometryInstanceFB::NULL;
                let result = unsafe {
                    super::xr_res((ext_fns.create_geometry_instance)(
                        session,
                        &raw const info,
                        &raw mut instance,
                    ))
                };
                match result {
                    Ok(()) => self.instances.push(instance),
                    Err(e) => warn!("Projected passthrough: create_geometry_instance: {e}"),
                }
            }
            info!(
                "Projected passthrough: {} of {} rectangle(s) as geometry instances: {:?}",
                self.instances.len(),
                rects.len(),
                rects
                    .iter()
                    .map(|rect| (rect.pose.position, rect.width, rect.height))
                    .collect::<Vec<_>>()
            );
        } else {
            for (instance, rect) in self.instances.iter().zip(rects) {
                let transform = sys::GeometryInstanceTransformFB {
                    ty: sys::GeometryInstanceTransformFB::TYPE,
                    next: ptr::null(),
                    base_space: stage.as_raw(),
                    time,
                    pose: to_sys_pose(rect.pose),
                    scale: sys::Vector3f {
                        x: rect.width,
                        y: rect.height,
                        z: 1.0,
                    },
                };
                let result = unsafe {
                    (ext_fns.geometry_instance_set_transform)(*instance, &raw const transform)
                };
                if result != sys::Result::SUCCESS {
                    warn!("Projected passthrough: geometry_instance_set_transform: {result:?}");
                }
            }
        }
        self.rects = rects.to_vec();
    }

    fn destroy(&mut self, ext_fns: &raw::PassthroughFB) {
        unsafe {
            for instance in self.instances.drain(..) {
                (ext_fns.destroy_geometry_instance)(instance);
            }
            (self.mesh_fns.destroy_triangle_mesh)(self.mesh);
            (ext_fns.destroy_passthrough_layer)(self.layer_handle);
        }
    }
}

impl Drop for PassthroughFB {
    fn drop(&mut self) {
        if let Some(mut projected) = self.projected.take() {
            projected.destroy(&self.ext_fns);
        }
        unsafe {
            (self.ext_fns.destroy_passthrough_layer)(self.layer_handle);
            (self.ext_fns.destroy_passthrough)(self.handle);
        }
    }
}
