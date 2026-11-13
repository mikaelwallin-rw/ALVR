//! When the GPU finished a frame (fork, late pickup): an EGL fence after the frame's last GL
//! command, waited for on a helper thread (eglClientWaitSyncKHR needs no GL context), which records
//! the instant the fence signalled. The same pattern as PICO's own `gpu_frame_end` thread. The
//! render thread never blocks on it: it drains the results in later loops.

use super::GraphicsContext;
use glow::HasContext;
use khronos_egl as egl;
use std::{
    ffi::c_void,
    ptr,
    sync::mpsc::{self, Receiver, Sender},
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

const EGL_SYNC_FENCE_KHR: u32 = 0x30F9;
const EGL_CONDITION_SATISFIED_KHR: i32 = 0x30F6;
// A frame's GPU work that takes longer than this is reported as unknown
const WAIT_TIMEOUT: Duration = Duration::from_millis(200);

type CreateSyncFn = unsafe extern "C" fn(egl::EGLDisplay, u32, *const egl::Int) -> *mut c_void;
type ClientWaitSyncFn =
    unsafe extern "C" fn(egl::EGLDisplay, *mut c_void, egl::Int, u64) -> egl::Int;
type DestroySyncFn = unsafe extern "C" fn(egl::EGLDisplay, *mut c_void) -> egl::Boolean;

// EGL handles are process-global; the display is valid for the context's lifetime
struct Pending {
    id: u64,
    display: usize,
    sync: usize,
}

pub struct GpuFenceTimer {
    create_sync: CreateSyncFn,
    sender: Option<Sender<Pending>>,
    results: Receiver<(u64, Option<Instant>)>,
    thread: Option<JoinHandle<()>>,
}

impl GpuFenceTimer {
    /// None when the EGL fence sync functions are not available (EGL_KHR_fence_sync)
    pub fn new(context: &GraphicsContext) -> Option<Self> {
        let create = context.egl_proc_address("eglCreateSyncKHR");
        let wait = context.egl_proc_address("eglClientWaitSyncKHR");
        let destroy = context.egl_proc_address("eglDestroySyncKHR");
        if create.is_null() || wait.is_null() || destroy.is_null() {
            return None;
        }
        let (create_sync, client_wait_sync, destroy_sync) = unsafe {
            (
                std::mem::transmute::<*const c_void, CreateSyncFn>(create),
                std::mem::transmute::<*const c_void, ClientWaitSyncFn>(wait),
                std::mem::transmute::<*const c_void, DestroySyncFn>(destroy),
            )
        };

        let (sender, pending) = mpsc::channel::<Pending>();
        let (result_sender, results) = mpsc::channel();
        let thread = thread::Builder::new()
            .name("gpu_fence_timer".into())
            .spawn(move || {
                while let Ok(item) = pending.recv() {
                    let display = item.display as egl::EGLDisplay;
                    let sync = item.sync as *mut c_void;
                    let status = unsafe {
                        client_wait_sync(display, sync, 0, WAIT_TIMEOUT.as_nanos() as u64)
                    };
                    let done = Instant::now();
                    unsafe { destroy_sync(display, sync) };
                    let result = (status == EGL_CONDITION_SATISFIED_KHR).then_some(done);
                    if result_sender.send((item.id, result)).is_err() {
                        break;
                    }
                }
            })
            .ok()?;

        Some(Self {
            create_sync,
            sender: Some(sender),
            results,
            thread: Some(thread),
        })
    }

    /// After the frame's last GL command, on the render thread (the context current): inserts
    /// the fence and flushes so that it reaches the GPU. False if the fence could not be made.
    pub fn submit(&self, context: &GraphicsContext, id: u64) -> bool {
        let display = context.egl_display.as_ptr();
        let sync = unsafe { (self.create_sync)(display, EGL_SYNC_FENCE_KHR, ptr::null()) };
        if sync.is_null() {
            return false;
        }
        unsafe { context.gl_context.flush() };
        self.sender.as_ref().is_some_and(|sender| {
            sender
                .send(Pending {
                    id,
                    display: display as usize,
                    sync: sync as usize,
                })
                .is_ok()
        })
    }

    /// Finished fences since the last call: (id, GPU done; None = wait failed or timed out)
    pub fn poll(&self) -> Vec<(u64, Option<Instant>)> {
        self.results.try_iter().collect()
    }
}

impl Drop for GpuFenceTimer {
    fn drop(&mut self) {
        // The thread ends once the queue is empty and the sender is gone
        self.sender.take();
        if let Some(thread) = self.thread.take() {
            thread.join().ok();
        }
    }
}
