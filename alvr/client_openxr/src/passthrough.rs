use crate::extra_extensions::{PassthroughFB, PassthroughHTC, ProjectionRect};
use alvr_common::anyhow::{Result, bail};
use alvr_system_info::Platform;
use openxr as xr;
use std::{marker::PhantomData, ops::Deref, ptr};

pub struct PassthroughLayer<'a> {
    handle_fb: Option<PassthroughFB>,
    handle_htc: Option<PassthroughHTC>,
    _marker: PhantomData<&'a ()>,
}

impl PassthroughLayer<'_> {
    pub fn new(session: &xr::Session<xr::OpenGlEs>, platform: Platform) -> Result<Self> {
        let mut handle_fb = None;
        let mut handle_htc = None;

        let exts = session.instance().exts();
        if exts.fb_passthrough.is_some() {
            handle_fb = Some(PassthroughFB::new(session, platform)?);
        } else if exts.htc_passthrough.is_some() {
            handle_htc = Some(PassthroughHTC::new(session)?);
        } else {
            bail!("No passthrough extension available");
        };

        Ok(Self {
            handle_fb,
            handle_htc,
            _marker: PhantomData,
        })
    }

    /// PROTOTYPE (XR_FB_passthrough only): project passthrough onto these rectangles; None = off
    pub fn set_projection(
        &mut self,
        instance: &xr::Instance,
        stage: &xr::Space,
        time: xr::Time,
        rects: Option<&[ProjectionRect]>,
    ) -> bool {
        self.handle_fb
            .as_mut()
            .is_some_and(|handle| handle.set_projection(instance, stage, time, rects))
    }

    /// The projected passthrough layer, to submit right after this one
    pub fn projected_layer(&self) -> Option<&xr::CompositionLayerBase<'_, xr::OpenGlEs>> {
        self.handle_fb
            .as_ref()
            .and_then(PassthroughFB::projected_layer)
            .map(|layer| unsafe { &*ptr::from_ref(layer).cast() })
    }
}

impl<'a> Deref for PassthroughLayer<'a> {
    type Target = xr::CompositionLayerBase<'a, xr::OpenGlEs>;

    fn deref(&self) -> &Self::Target {
        if let Some(handle) = &self.handle_fb {
            unsafe { &*ptr::from_ref(handle.layer()).cast() }
        } else if let Some(handle) = &self.handle_htc {
            unsafe { &*ptr::from_ref(handle.layer()).cast() }
        } else {
            panic!("No passthrough extension available");
        }
    }
}
