#![allow(
    non_upper_case_globals,
    non_snake_case,
    clippy::missing_safety_doc,
    clippy::unseparated_literal_suffix
)]

mod c_api;
mod connection;
mod logging_backend;
mod sockets;
mod statistics;
mod storage;

#[cfg(target_os = "android")]
mod audio;

pub mod video_decoder;

use alvr_common::{
    ConnectionState, LifecycleState, ViewParams, dbg_client_core, error,
    glam::{UVec2, Vec2},
    info,
    parking_lot::{Mutex, RwLock},
    warn,
};
use alvr_packets::{
    AnchorRequest, BatteryInfo, ButtonEntry, CameraFrame, CameraFramesConfig, ClientControlPacket,
    ClientStreamConfig, HandShapeInfo, NativeQuad, OpaqueRegion, RealTimeConfig,
    ScreenVideoPacketHeader, StageInfo, TrackingData,
};
use alvr_session::CodecType;
use alvr_system_info::Platform;
use connection::{ConnectionContext, DecoderCallback};
use std::{
    collections::{HashSet, VecDeque},
    sync::Arc,
    thread::{self, JoinHandle},
    time::Duration,
};
use storage::Config;

pub use logging_backend::init_logging;

pub enum ClientCoreEvent {
    UpdateHudMessage(String),
    StreamingStarted(Box<ClientStreamConfig>),
    StreamingStopped,
    Haptics {
        device_id: u64,
        duration: Duration,
        frequency: f32,
        amplitude: f32,
    },
    // Note: All subsequent DecoderConfig events should be ignored until reconnection
    DecoderConfig {
        codec: CodecType,
        config_nal: Vec<u8>,
    },
    RealTimeConfig(RealTimeConfig),
    // None stops the camera frames
    CameraFrames(Option<CameraFramesConfig>),
    // Debug reference quads to draw as native layers (empty = none)
    NativeQuads(Vec<NativeQuad>),
    // Overlay app's persistent anchor command
    AnchorRequest(AnchorRequest),
}

// Note: this struct may change without breaking network protocol changes
#[derive(Clone)]
pub struct ClientCapabilities {
    pub platform: Platform,
    pub default_view_resolution: UVec2,
    pub max_view_resolution: UVec2,
    pub refresh_rates: Vec<f32>,
    pub foveated_encoding: bool,
    pub encoder_high_profile: bool,
    pub encoder_10_bits: bool,
    pub encoder_av1: bool,
    pub prefer_10bit: bool,
    pub preferred_encoding_gamma: f32,
    pub prefer_hdr: bool,
}

#[derive(Clone, Copy)]
pub struct VideoFrameMetadata {
    pub view_params: [ViewParams; 2],
    /// If absent, reuse the previous centers or the negotiated centers before the first update.
    pub foveation_center_shifts: Option<[[f32; 2]; 2]>,
}

pub struct ClientCoreContext {
    platform: Platform,
    lifecycle_state: Arc<RwLock<LifecycleState>>,
    event_queue: Arc<Mutex<VecDeque<ClientCoreEvent>>>,
    connection_context: Arc<ConnectionContext>,
    connection_thread: Arc<Mutex<Option<JoinHandle<()>>>>,
    last_good_opaque_regions: Mutex<Arc<Vec<OpaqueRegion>>>,
}

impl ClientCoreContext {
    /// On Android, requests microphone permission together with the additional permissions.
    #[cfg_attr(not(target_os = "android"), expect(unused_variables))]
    pub fn new(capabilities: ClientCapabilities, additional_permissions: Vec<&str>) -> Self {
        dbg_client_core!("Create");

        // Make sure to reset config in case of version compat mismatch.
        if Config::load().protocol_id != alvr_common::protocol_id() {
            // NB: Config::default() sets the current protocol ID
            Config::default().store();
        }

        #[cfg(target_os = "android")]
        {
            let mut permissions = additional_permissions;
            permissions.push(alvr_system_info::MICROPHONE_PERMISSION);
            alvr_system_info::try_get_permissions(&permissions);
            alvr_system_info::set_wifi_lock(true);
        }

        let platform = capabilities.platform;

        let lifecycle_state = Arc::new(RwLock::new(LifecycleState::Idle));
        let event_queue = Arc::new(Mutex::new(VecDeque::new()));
        let connection_context = Arc::new(ConnectionContext::default());
        let connection_thread = thread::spawn({
            let lifecycle_state = Arc::clone(&lifecycle_state);
            let connection_context = Arc::clone(&connection_context);
            let event_queue = Arc::clone(&event_queue);
            move || {
                connection::connection_lifecycle_loop(
                    capabilities,
                    connection_context,
                    lifecycle_state,
                    event_queue,
                )
            }
        });

        Self {
            platform,
            lifecycle_state,
            event_queue,
            connection_context,
            connection_thread: Arc::new(Mutex::new(Some(connection_thread))),
            last_good_opaque_regions: Mutex::new(Arc::new(Vec::new())),
        }
    }

    pub fn resume(&self) {
        dbg_client_core!("resume");

        *self.lifecycle_state.write() = LifecycleState::Resumed;
    }

    pub fn pause(&self) {
        dbg_client_core!("pause");

        let mut connection_state_lock = self.connection_context.state.write();

        *self.lifecycle_state.write() = LifecycleState::Idle;

        // Wait until the connection is actually torn down before returning to
        // the OpenXR main thread (this is called from the XR_STOPPING handler).
        //
        // The original implementation used a single `wait_rwlock(...)` here.
        // That has a lost-wakeup race: `wait_rwlock` releases the outer
        // RwLock BEFORE parking the thread on the inner Condvar, so a
        // `notify_one` that fires in that window is dropped (Condvar wakeups
        // are edge-triggered, not buffered). When the wakeup is lost, the main
        // UI thread blocks indefinitely, Android raises an ANR, and the Pico
        // runtime steals the display (visible to the user as the Pico loading
        // ring and a fully wedged ALVR client that requires `am force-stop`
        // to recover).
        //
        // The predicate loop below is immune to that race: it re-checks the
        // authoritative `connection_state` after every wake / timeout. Even
        // if every notification gets lost, the loop exits as soon as the
        // state is observed to be Disconnected. Worst-case latency is the
        // wait_rwlock_timeout argument (500 ms).
        const WAIT_POLL: Duration = Duration::from_millis(500);
        const HEARTBEAT_EVERY: u32 = 4; // every ~2s
        let start = std::time::Instant::now();
        let initial_state = (*connection_state_lock).clone();
        let mut iter = 0u32;

        while *connection_state_lock != ConnectionState::Disconnected {
            if iter == 0 {
                info!(
                    "pause: connection_state is {initial_state:?}, waiting for it to reach Disconnected"
                );
            } else if iter.is_multiple_of(HEARTBEAT_EVERY) {
                warn!(
                    "pause: still waiting after {} ms (iter {iter}, state={:?})",
                    start.elapsed().as_millis(),
                    *connection_state_lock,
                );
            }
            iter += 1;
            alvr_common::wait_rwlock_timeout(
                &self.connection_context.disconnected_notif,
                &mut connection_state_lock,
                WAIT_POLL,
            );
        }

        if iter > 0 {
            info!(
                "pause: connection reached Disconnected after {} ms ({iter} wait iterations, initial state {initial_state:?})",
                start.elapsed().as_millis(),
            );
        } else {
            dbg_client_core!("pause: already Disconnected on entry, returning immediately");
        }
    }

    pub fn poll_event(&self) -> Option<ClientCoreEvent> {
        dbg_client_core!("poll_event");

        self.event_queue.lock().pop_front()
    }

    pub fn send_battery(&self, device_id: u64, gauge_value: f32, is_plugged: bool) {
        dbg_client_core!("send_battery");

        if let Some(sender) = &mut *self.connection_context.control_sender.lock() {
            sender
                .send(&ClientControlPacket::Battery(BatteryInfo {
                    device_id,
                    gauge_value,
                    is_plugged,
                }))
                .ok();
        }
    }

    pub fn send_playspace(&self, area: Option<Vec2>) {
        dbg_client_core!("send_playspace");

        if let Some(sender) = &mut *self.connection_context.control_sender.lock() {
            sender.send(&ClientControlPacket::PlayspaceSync(area)).ok();
        }
    }

    pub fn send_active_interaction_profile(
        &self,
        device_id: u64,
        profile_id: u64,
        input_ids: HashSet<u64>,
    ) {
        dbg_client_core!("send_active_interaction_profile");

        if let Some(sender) = &mut *self.connection_context.control_sender.lock() {
            sender
                .send(&ClientControlPacket::ActiveInteractionProfile {
                    device_id,
                    profile_id,
                    input_ids,
                })
                .ok();
        }
    }

    pub fn send_buttons(&self, entries: Vec<ButtonEntry>) {
        dbg_client_core!("send_buttons");

        if let Some(sender) = &mut *self.connection_context.control_sender.lock() {
            sender.send(&ClientControlPacket::Buttons(entries)).ok();
        }
    }

    // These must be in its local space, as if the head pose is in the origin.
    pub fn send_view_params(&self, views: [ViewParams; 2]) {
        dbg_client_core!("send_view_params");

        if let Some(sender) = &mut *self.connection_context.control_sender.lock() {
            sender
                .send(&ClientControlPacket::LocalViewParams(views))
                .ok();
        }
    }

    pub fn send_tracking(&self, data: TrackingData) {
        dbg_client_core!("send_tracking");

        if let Some(sender) = &mut *self.connection_context.tracking_sender.lock() {
            sender.send_header(&data).ok();

            if let Some(stats) = &mut *self.connection_context.statistics_manager.lock() {
                stats.report_input_acquired(data.poll_timestamp);
            }
        }
    }

    pub fn send_camera_frame(&self, frame: CameraFrame) {
        dbg_client_core!("send_camera_frame");

        if let Some(sender) = &mut *self.connection_context.control_sender.lock() {
            sender
                .send(&ClientControlPacket::CameraFrame(Box::new(frame)))
                .ok();
        }
    }

    /// Next screen video access unit (Annex B) received from the server, in arrival order
    pub fn poll_screen_video_frame(&self) -> Option<(ScreenVideoPacketHeader, Vec<u8>)> {
        self.connection_context
            .screen_video_queue
            .lock()
            .pop_front()
    }

    /// Asks the server for a keyframe of a screen video stream (e.g. after a decoder error)
    pub fn request_screen_idr(&self, stream_id: u32) {
        if let Some(sender) = &mut *self.connection_context.control_sender.lock() {
            sender
                .send(&ClientControlPacket::RequestScreenIdr { stream_id })
                .ok();
        }
    }

    pub fn send_hand_shape(&self, info: HandShapeInfo) {
        if let Some(sender) = &mut *self.connection_context.control_sender.lock() {
            sender.send(&ClientControlPacket::HandShape(info)).ok();
        }
    }

    /// Tagged payloads (e.g. alvr_packets::TrackingStateReport) that any server can ignore
    pub fn send_reserved_buffer(&self, bytes: Vec<u8>) {
        if let Some(sender) = &mut *self.connection_context.control_sender.lock() {
            sender
                .send(&ClientControlPacket::ReservedBuffer(bytes))
                .ok();
        }
    }

    pub fn send_stage_info(&self, info: StageInfo) {
        if let Some(sender) = &mut *self.connection_context.control_sender.lock() {
            sender.send(&ClientControlPacket::StageInfo(info)).ok();
        }
    }

    pub fn send_proximity_state(&self, headset_is_worn: bool) {
        if let Some(sender) = &mut *self.connection_context.control_sender.lock() {
            sender
                .send(&ClientControlPacket::ProximityState(headset_is_worn))
                .ok();
        }
    }

    pub fn get_total_prediction_offset(&self) -> Duration {
        dbg_client_core!("get_total_prediction_offset");

        if let Some(stats) = &*self.connection_context.statistics_manager.lock() {
            Duration::min(
                stats.average_total_pipeline_latency(),
                *self.connection_context.max_prediction.read(),
            )
        } else {
            Duration::ZERO
        }
    }

    /// The callback should return true if the frame was successfully submitted to the decoder
    pub fn set_decoder_input_callback(&self, callback: Box<DecoderCallback>) {
        dbg_client_core!("set_decoder_input_callback");

        *self.connection_context.decoder_callback.lock() = Some(callback);

        if let Some(sender) = &mut *self.connection_context.control_sender.lock() {
            sender.send(&ClientControlPacket::RequestIdr).ok();
        }
    }

    pub fn report_frame_decoded(&self, timestamp: Duration) {
        dbg_client_core!("report_frame_decoded");

        if let Some(stats) = &mut *self.connection_context.statistics_manager.lock() {
            stats.report_frame_decoded(timestamp);
        }
    }

    pub fn report_fatal_decoder_error(&self, error: &str) {
        error!("Fatal decoder error, restarting connection: {error}");

        // The connection loop observes changes on this value
        *self.connection_context.state.write() = ConnectionState::Disconnecting;
    }

    /// Returns metadata matching this decoded frame, if available.
    /// If no match is found, the caller can display the frame using its previous metadata.
    pub fn report_compositor_start(&self, timestamp: Duration) -> Option<VideoFrameMetadata> {
        dbg_client_core!("report_compositor_start");

        if let Some(stats) = &mut *self.connection_context.statistics_manager.lock() {
            stats.report_compositor_start(timestamp);
        }

        if let Some((_, regions)) = self
            .connection_context
            .opaque_regions_queue
            .lock()
            .iter()
            .rev()
            .find(|(frame_timestamp, _)| *frame_timestamp == timestamp)
        {
            *self.last_good_opaque_regions.lock() = Arc::clone(regions);
        }

        self.connection_context
            .video_frame_metadata_queue
            .lock()
            .iter()
            .rev()
            .find_map(|(frame_timestamp, metadata)| {
                (*frame_timestamp == timestamp).then_some(*metadata)
            })
    }

    /// Opaque regions of the frame last passed to `report_compositor_start`, in the same space as
    /// its view params.
    pub fn last_frame_opaque_regions(&self) -> Arc<Vec<OpaqueRegion>> {
        Arc::clone(&self.last_good_opaque_regions.lock())
    }

    pub fn report_submit(&self, timestamp: Duration, vsync_queue: Duration) {
        dbg_client_core!("report_submit");

        if let Some(stats) = &mut *self.connection_context.statistics_manager.lock() {
            stats.report_submit(timestamp, vsync_queue);

            if let Some(sender) = &mut *self.connection_context.statistics_sender.lock() {
                if let Some(stats) = stats.summary(timestamp) {
                    sender.send_header(&stats).ok();
                } else {
                    warn!("Statistics summary not ready!");
                }
            }
        }
    }

    pub fn platform(&self) -> Platform {
        self.platform
    }
}

impl Drop for ClientCoreContext {
    fn drop(&mut self) {
        dbg_client_core!("Drop");

        *self.lifecycle_state.write() = LifecycleState::ShuttingDown;

        if let Some(thread) = self.connection_thread.lock().take() {
            thread.join().ok();
        }

        #[cfg(target_os = "android")]
        alvr_system_info::set_wifi_lock(false);
    }
}
