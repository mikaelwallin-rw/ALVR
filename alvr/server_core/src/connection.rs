use crate::{
    ConnectionContext, FILESYSTEM_LAYOUT, SESSION_MANAGER, ServerCoreEvent,
    ServerNegotiatedStreamingConfig,
    bitrate::BitrateManager,
    camera_frames,
    input_mapping::ButtonMappingManager,
    native_quads::NativeQuadsReader,
    screen_video::ScreenVideoReader,
    sockets::WelcomeSocket,
    statistics::StatisticsManager,
    tracking::{self, TrackingManager},
    tracking_extras,
};
use alvr_adb::{WiredConnection, WiredConnectionStatus};
use alvr_common::{
    AlvrFoveatedEncodingParams, AnyhowToCon, BUTTON_INFO, CONTROLLER_PROFILE_INFO, ConResult,
    ConnectionError, ConnectionState, LifecycleState, QUEST_CONTROLLER_PROFILE_PATH, RelaxedAtomic,
    con_bail, dbg_connection, debug, error,
    glam::{UVec2, Vec2},
    info,
    parking_lot::{Condvar, Mutex, RwLock},
    settings_schema::Switch,
    warn,
};
use alvr_events::{AdbEvent, ButtonEvent, EventType};
use alvr_packets::{
    AUDIO, CameraEye, CameraFramesConfig, ClientConnectionResult, ClientConnectionsAction,
    ClientControlPacket, ClientNegotiatedStreamingConfig, ClientStatistics, HAPTICS, NativeQuad,
    NegotiatedStreamingConfigExt, RealTimeConfig, SCREEN_VIDEO, STATISTICS,
    ScreenVideoPacketHeader, ServerControlPacket, StreamConfigPacket, TRACKING, TrackingData,
    VIDEO, VideoPacketHeader,
};
use alvr_session::{
    BodyTrackingSinkConfig, CodecType, ControllersEmulationMode, FrameSize, H264Profile, Settings,
    SocketProtocol, SteamvrHmdInitConfig,
};
use alvr_sockets::{
    CONTROL_PORT, KEEPALIVE_INTERVAL, KEEPALIVE_TIMEOUT, ProtoControlSocket, SocketConnection,
    StreamSocketConfig, WIRED_CLIENT_HOSTNAME,
};
use std::{
    collections::{HashMap, hash_map::DefaultHasher},
    hash::{Hash, Hasher},
    net::{IpAddr, Ipv4Addr},
    process::Command,
    sync::{
        Arc,
        mpsc::{self, RecvTimeoutError},
    },
    thread,
    time::{Duration, Instant},
};

const RETRY_CONNECT_MIN_INTERVAL: Duration = Duration::from_secs(1);
const HANDSHAKE_ACTION_TIMEOUT: Duration = Duration::from_secs(2);
pub const STREAMING_RECV_TIMEOUT: Duration = Duration::from_millis(500);
// Also the reaction time to scene mode changes (passthrough off while a scene app presents)
const REAL_TIME_UPDATE_INTERVAL: Duration = Duration::from_millis(100);
// Headset camera frames for overlay calibration (see VrCameraFrame.h): full rectified resolution,
// a few frames per second
const CAMERA_FRAME_SIZE: u32 = 2048;
// ~12 fps: the headset needs ~78 ms per frame (SecureMR capture 35 ms + pose/JPEG 41 ms + send
// 2 ms, measured 2026-11-10), so a calibration pass gets its frames 3x faster than at 250 ms
const CAMERA_FRAME_INTERVAL: Duration = Duration::from_millis(83);
// Screen thread: wait for a new screen video frame at most this long, poll the native quads at
// this interval (pose trims must reach the headset quickly)
const SCREEN_VIDEO_WAIT: Duration = Duration::from_millis(5);
const NATIVE_QUADS_POLL_INTERVAL: Duration = Duration::from_millis(100);
// Screen video bitrate measurement, subtracted from the main stream's bitrate. The window spans
// two keyframe periods of the overlay app (2 s), so keyframes don't make it oscillate.
const SCREEN_VIDEO_BITRATE_INTERVAL: Duration = Duration::from_secs(4);
const SCREEN_VIDEO_BITRATE_SMOOTHING: f32 = 0.5;

const MAX_UNREAD_PACKETS: usize = 10; // Applies per stream

pub struct VideoPacket {
    pub header: VideoPacketHeader,
    pub payload: Vec<u8>,
}

fn align32(value: f32) -> u32 {
    ((value / 32.).floor() * 32.) as u32
}

fn is_streaming(client_hostname: &str) -> bool {
    SESSION_MANAGER
        .read()
        .client_list()
        .get(client_hostname)
        .is_some_and(|c| c.connection_state == ConnectionState::Streaming)
}

pub fn compute_restart_settings_hash(
    steamvr_hmd_init_config: &SteamvrHmdInitConfig,
    settings: &Settings,
) -> u64 {
    let mut controller_is_tracker = false;
    let mut controller_profile = 0;
    let mut use_separate_hand_trackers = false;
    let controllers_enabled = if let Switch::Enabled(config) = &settings.headset.controllers {
        controller_is_tracker =
            matches!(config.emulation_mode, ControllersEmulationMode::ViveTracker);
        controller_profile = match config.emulation_mode {
            ControllersEmulationMode::RiftSTouch => 0,
            ControllersEmulationMode::Quest1Touch => 1,
            ControllersEmulationMode::Quest2Touch => 2,
            ControllersEmulationMode::Quest3Plus => 3,
            ControllersEmulationMode::QuestPro => 4,
            ControllersEmulationMode::Pico4 => 10,
            ControllersEmulationMode::ValveIndex => 20,
            ControllersEmulationMode::ViveWand => 40,
            ControllersEmulationMode::ViveTracker => 41,
            ControllersEmulationMode::PSVR2Sense => 60,
            ControllersEmulationMode::Custom { .. } => 500,
        };
        use_separate_hand_trackers = config
            .hand_skeleton
            .as_option()
            .is_some_and(|c| c.steamvr_input_2_0);
        true
    } else {
        false
    };

    let body_tracking_vive_enabled =
        if let Switch::Enabled(config) = &settings.headset.body_tracking {
            matches!(config.sink, BodyTrackingSinkConfig::FakeViveTracker)
        } else if let Switch::Enabled(config) = &settings.headset.multimodal_tracking {
            config.detached_controllers_steamvr_sink
        } else {
            false
        };

    let body_tracking_has_legs = settings
        .headset
        .body_tracking
        .as_option()
        .map(|c| c.sources.meta.prefer_full_body)
        .unwrap_or(false);

    let mut foveation_center_size_x = 0.0_f32;
    let mut foveation_center_size_y = 0.0_f32;
    let mut foveation_center_shift_x = 0.0_f32;
    let mut foveation_center_shift_y = 0.0_f32;
    let mut foveation_edge_ratio_x = 0.0_f32;
    let mut foveation_edge_ratio_y = 0.0_f32;
    let enable_foveated_encoding =
        if let Switch::Enabled(config) = &settings.video.foveated_encoding {
            [foveation_center_size_x, foveation_center_size_y] = config.center_size;
            [foveation_center_shift_x, foveation_center_shift_y] = config.center_shift;
            [foveation_edge_ratio_x, foveation_edge_ratio_y] = config.edge_ratio;
            true
        } else {
            false
        };

    let mut brightness = 0.0_f32;
    let mut contrast = 0.0_f32;
    let mut saturation = 0.0_f32;
    let mut gamma = 0.0_f32;
    let mut sharpening = 0.0_f32;
    let enable_color_correction = if let Switch::Enabled(config) = &settings.video.color_correction
    {
        brightness = config.brightness;
        contrast = config.contrast;
        saturation = config.saturation;
        gamma = config.gamma;
        sharpening = config.sharpening;
        true
    } else {
        false
    };

    let nvenc = &settings.video.encoder_config.nvenc;
    let amf = &settings.video.encoder_config.amf;
    let hdr = &settings.video.encoder_config.hdr;
    let enc = &settings.video.encoder_config;
    let dbg = &settings.extra.logging.debug_groups;

    let mut h = DefaultHasher::new();

    // Negotiated fields (persisted in SteamvrHmdInitConfig)
    steamvr_hmd_init_config.eye_resolution_width.hash(&mut h);
    steamvr_hmd_init_config.eye_resolution_height.hash(&mut h);
    steamvr_hmd_init_config
        .target_eye_resolution_width
        .hash(&mut h);
    steamvr_hmd_init_config
        .target_eye_resolution_height
        .hash(&mut h);
    steamvr_hmd_init_config.refresh_rate.hash(&mut h);
    // Pre-init settings fields (read directly from settings)
    settings.video.adapter_index.hash(&mut h);
    settings.headset.tracking_ref_only.hash(&mut h);
    settings.headset.enable_vive_tracker_proxy.hash(&mut h);
    settings.extra.patches.linux_async_compute.hash(&mut h);
    settings.extra.patches.linux_async_reprojection.hash(&mut h);
    // Encoder / codec
    (settings.video.preferred_codec as u8).hash(&mut h);
    (enc.h264_profile as u32).hash(&mut h);
    (enc.rate_control_mode as u32).hash(&mut h);
    enc.filler_data.hash(&mut h);
    (enc.entropy_coding as u32).hash(&mut h);
    (enc.quality_preset as u32).hash(&mut h);
    enc.enable_vbaq.hash(&mut h);
    enc.use_10bit.hash(&mut h);
    enc.encoding_gamma.map(f32::to_bits).hash(&mut h);
    enc.software.force_software_encoding.hash(&mut h);
    enc.software.thread_count.hash(&mut h);
    // HDR
    hdr.enable.hash(&mut h);
    hdr.force_hdr_srgb_correction.hash(&mut h);
    hdr.clamp_hdr_extended_range.hash(&mut h);
    // AMF
    amf.enable_pre_analysis.hash(&mut h);
    amf.enable_hmqb.hash(&mut h);
    amf.use_preproc.hash(&mut h);
    amf.preproc_sigma.hash(&mut h);
    amf.preproc_tor.hash(&mut h);
    // NVENC
    (nvenc.quality_preset as u32).hash(&mut h);
    (nvenc.tuning_preset as u32).hash(&mut h);
    (nvenc.multi_pass as u32).hash(&mut h);
    (nvenc.adaptive_quantization_mode as u32).hash(&mut h);
    nvenc.low_delay_key_frame_scale.hash(&mut h);
    nvenc.refresh_rate.hash(&mut h);
    nvenc.enable_intra_refresh.hash(&mut h);
    nvenc.intra_refresh_period.hash(&mut h);
    nvenc.intra_refresh_count.hash(&mut h);
    nvenc.max_num_ref_frames.hash(&mut h);
    nvenc.gop_length.hash(&mut h);
    nvenc.p_frame_strategy.hash(&mut h);
    nvenc.rate_control_mode.hash(&mut h);
    nvenc.rc_buffer_size.hash(&mut h);
    nvenc.rc_initial_delay.hash(&mut h);
    nvenc.rc_max_bitrate.hash(&mut h);
    nvenc.rc_average_bitrate.hash(&mut h);
    nvenc.enable_weighted_prediction.hash(&mut h);
    // Foveated encoding
    enable_foveated_encoding.hash(&mut h);
    foveation_center_size_x.to_bits().hash(&mut h);
    foveation_center_size_y.to_bits().hash(&mut h);
    foveation_center_shift_x.to_bits().hash(&mut h);
    foveation_center_shift_y.to_bits().hash(&mut h);
    foveation_edge_ratio_x.to_bits().hash(&mut h);
    foveation_edge_ratio_y.to_bits().hash(&mut h);
    // Color correction
    enable_color_correction.hash(&mut h);
    brightness.to_bits().hash(&mut h);
    contrast.to_bits().hash(&mut h);
    saturation.to_bits().hash(&mut h);
    gamma.to_bits().hash(&mut h);
    sharpening.to_bits().hash(&mut h);
    // Controllers
    controllers_enabled.hash(&mut h);
    controller_is_tracker.hash(&mut h);
    controller_profile.hash(&mut h);
    use_separate_hand_trackers.hash(&mut h);
    // Body tracking
    body_tracking_vive_enabled.hash(&mut h);
    body_tracking_has_legs.hash(&mut h);
    // Misc
    settings.connection.minimum_idr_interval_ms.hash(&mut h);
    settings.extra.capture.capture_frame_dir.hash(&mut h);
    settings.video.bitrate.image_corruption_fix.hash(&mut h);
    // Debug groups
    dbg.server_impl.hash(&mut h);
    dbg.client_impl.hash(&mut h);
    dbg.server_core.hash(&mut h);
    dbg.client_core.hash(&mut h);
    dbg.connection.hash(&mut h);
    dbg.sockets.hash(&mut h);
    dbg.server_gfx.hash(&mut h);
    dbg.client_gfx.hash(&mut h);
    dbg.encoder.hash(&mut h);
    dbg.decoder.hash(&mut h);

    h.finish()
}

// Alternate connection trials with manual IPs and clients discovered on the local network
pub fn handshake_loop(ctx: Arc<ConnectionContext>, lifecycle_state: Arc<RwLock<LifecycleState>>) {
    dbg_connection!("handshake_loop: Begin");

    let welcome_socket = match WelcomeSocket::new() {
        Ok(socket) => socket,
        Err(e) => {
            error!("Failed to create discovery socket: {e:?}");
            return;
        }
    };

    let mut wired_connection = None;

    while *lifecycle_state.read() != LifecycleState::ShuttingDown {
        dbg_connection!("handshake_loop: Try connect to wired device");

        // Self-heal any leaked non-Disconnected state on the wired client.
        // Connection states are normally driven by the connection_pipeline
        // worker thread (set to Connecting on entry, Streaming/Disconnecting
        // in flight, Disconnected by the post-cleanup at the end of
        // try_connect). If that worker terminates abnormally (panic, hang,
        // killed) the state is stranded at whatever it was when the worker
        // died and the eligibility check below (state == Disconnected) skips
        // wired forever. Detect "no live worker for wired AND no removal
        // pending AND state != Disconnected" and force a reset so the loop
        // can retry. active_pipelines is populated by try_connect and cleared
        // by a Drop guard inside the worker closure (panic-safe).
        let wired_state_to_reset: Option<ConnectionState> = {
            let session = SESSION_MANAGER.read();
            let pending_remove = ctx.clients_to_be_removed.lock();
            let active = ctx.active_pipelines.lock();
            session
                .client_list()
                .get(WIRED_CLIENT_HOSTNAME)
                .and_then(|info| {
                    let stuck = info.connection_state != ConnectionState::Disconnected
                        && !pending_remove.contains(WIRED_CLIENT_HOSTNAME)
                        && !active.contains(WIRED_CLIENT_HOSTNAME);
                    stuck.then(|| info.connection_state.clone())
                })
        };
        if let Some(stuck_state) = wired_state_to_reset {
            info!(
                "handshake_loop: clearing leaked {stuck_state:?} on {WIRED_CLIENT_HOSTNAME} (no active worker, no pending removal)"
            );
            SESSION_MANAGER.write().update_client_connections(
                WIRED_CLIENT_HOSTNAME.to_owned(),
                ClientConnectionsAction::SetConnectionState(ConnectionState::Disconnected),
            );
        }

        let mut wired_client_ips = HashMap::new();
        if SESSION_MANAGER
            .read()
            .client_list()
            .iter()
            .any(|(hostname, info)| {
                info.connection_state == ConnectionState::Disconnected
                    && hostname.as_str() == WIRED_CLIENT_HOSTNAME
            })
        {
            // Make sure the wired connection is created once and kept alive
            let wired_connection = if let Some(connection) = &wired_connection {
                connection
            } else {
                let connection = match WiredConnection::new(
                    FILESYSTEM_LAYOUT.get().unwrap(),
                    |downloaded, maybe_total| {
                        if let Some(total) = maybe_total {
                            alvr_events::send_event(EventType::Adb(AdbEvent {
                                download_progress: downloaded as f32 / total as f32,
                            }));
                        };
                    },
                ) {
                    Ok(connection) => connection,
                    Err(e) => {
                        error!("{e:?}");
                        thread::sleep(RETRY_CONNECT_MIN_INTERVAL);
                        continue;
                    }
                };

                wired_connection = Some(connection);

                wired_connection.as_ref().unwrap()
            };

            let stream_port;
            let client_type;
            let client_autolaunch;
            {
                let session_manager_lock = SESSION_MANAGER.read();
                let connection = &session_manager_lock.settings().connection;
                stream_port = connection.stream_port;
                client_type = connection.wired_client_type.clone();
                client_autolaunch = connection.wired_client_autolaunch.as_option().cloned();
            }

            let status = match wired_connection.setup(
                CONTROL_PORT,
                stream_port,
                &client_type,
                client_autolaunch,
            ) {
                Ok(status) => status,
                Err(e) => {
                    error!("{e:?}");
                    thread::sleep(RETRY_CONNECT_MIN_INTERVAL);
                    continue;
                }
            };

            #[cfg_attr(not(debug_assertions), expect(unused_variables))]
            if let WiredConnectionStatus::NotReady(s) = status {
                dbg_connection!("handshake_loop: Wired connection not ready: {s}");
                thread::sleep(RETRY_CONNECT_MIN_INTERVAL);
                continue;
            }

            let client_ip = IpAddr::V4(Ipv4Addr::LOCALHOST);
            wired_client_ips.insert(client_ip, WIRED_CLIENT_HOSTNAME.to_owned());
        }

        if !wired_client_ips.is_empty()
            && try_connect(
                Arc::clone(&ctx),
                Arc::clone(&lifecycle_state),
                wired_client_ips,
            )
            .is_ok()
        {
            thread::sleep(RETRY_CONNECT_MIN_INTERVAL);
            continue;
        }

        dbg_connection!("handshake_loop: Try connect to manual IPs");

        let available_manual_client_ips = {
            let mut manual_client_ips = HashMap::new();
            for (hostname, connection_info) in
                SESSION_MANAGER
                    .read()
                    .client_list()
                    .iter()
                    .filter(|(hostname, info)| {
                        info.connection_state == ConnectionState::Disconnected
                            && hostname.as_str() != WIRED_CLIENT_HOSTNAME
                    })
            {
                for ip in &connection_info.manual_ips {
                    manual_client_ips.insert(*ip, hostname.clone());
                }
            }
            manual_client_ips
        };

        if !available_manual_client_ips.is_empty()
            && try_connect(
                Arc::clone(&ctx),
                Arc::clone(&lifecycle_state),
                available_manual_client_ips,
            )
            .is_ok()
        {
            thread::sleep(RETRY_CONNECT_MIN_INTERVAL);
            continue;
        }

        let discovery_config = SESSION_MANAGER
            .read()
            .settings()
            .connection
            .client_discovery
            .clone();
        if let Switch::Enabled(config) = discovery_config {
            dbg_connection!("handshake_loop: Discovering clients");

            let clients = match welcome_socket.recv_all() {
                Ok(clients) => clients,
                Err(e) => {
                    warn!("mDNS listening error: {e:?}");

                    thread::sleep(RETRY_CONNECT_MIN_INTERVAL);
                    continue;
                }
            };

            if clients.is_empty() {
                thread::sleep(RETRY_CONNECT_MIN_INTERVAL);
                continue;
            }

            for (client_hostname, client_ip) in clients {
                let trusted = {
                    let mut session_manager = SESSION_MANAGER.write();

                    session_manager.update_client_connections(
                        client_hostname.clone(),
                        ClientConnectionsAction::AddIfMissing {
                            trusted: false,
                            manual_ips: vec![],
                        },
                    );

                    if config.auto_trust_clients {
                        session_manager.update_client_connections(
                            client_hostname.clone(),
                            ClientConnectionsAction::Trust,
                        );
                    }

                    session_manager
                        .client_list()
                        .get(&client_hostname)
                        .is_some_and(|c| c.trusted)
                };

                // do not attempt connection if the client is already connected
                if trusted
                    && SESSION_MANAGER
                        .read()
                        .client_list()
                        .get(&client_hostname)
                        .is_some_and(|c| c.connection_state == ConnectionState::Disconnected)
                    && let Err(e) = try_connect(
                        Arc::clone(&ctx),
                        Arc::clone(&lifecycle_state),
                        [(client_ip, client_hostname.clone())].into_iter().collect(),
                    )
                {
                    error!("Could not initiate connection for {client_hostname}: {e}");
                }

                thread::sleep(RETRY_CONNECT_MIN_INTERVAL);
            }
        } else {
            thread::sleep(RETRY_CONNECT_MIN_INTERVAL);
        }
    }

    alvr_common::dbg_connection!("handshake_loop: Joining connection threads");

    // At this point, LIFECYCLE_STATE == ShuttingDown, so all threads are already terminating
    for thread in ctx.connection_threads.lock().drain(..) {
        thread.join().ok();
    }

    alvr_common::dbg_connection!("handshake_loop: End");
}

fn try_connect(
    ctx: Arc<ConnectionContext>,
    lifecycle_state: Arc<RwLock<LifecycleState>>,
    mut client_ips: HashMap<IpAddr, String>,
) -> ConResult {
    dbg_connection!("try_connect: Finding client and creating control socket");

    let (socket, client_ip, connection_result) = alvr_sockets::connect_to_client(
        client_ips.keys().cloned().collect(),
        Duration::from_secs(1),
    )?;

    let Some(client_hostname) = client_ips.remove(&client_ip) else {
        con_bail!("unreachable");
    };

    dbg_connection!("try_connect: Pushing new client connection thread");

    // Track this hostname as having a live pipeline before spawning, so
    // handshake_loop's self-heal can tell "state Connecting and active worker"
    // (legitimate, leave alone) apart from "state Connecting but worker dead"
    // (stuck, reset). Removal is handled by a Drop guard inside the spawned
    // closure so it fires on panic unwind too -- panics are exactly the case
    // where the post-cleanup below would otherwise be skipped.
    ctx.active_pipelines.lock().insert(client_hostname.clone());

    struct PipelineGuard {
        ctx: Arc<ConnectionContext>,
        hostname: String,
    }
    impl Drop for PipelineGuard {
        fn drop(&mut self) {
            self.ctx.active_pipelines.lock().remove(&self.hostname);
        }
    }

    ctx.connection_threads.lock().push(thread::spawn({
        let ctx = Arc::clone(&ctx);
        move || {
            // Drop runs last in the closure (declared first = dropped last),
            // so by the time active_pipelines is cleared, the post-cleanup
            // below has already written the final state to the session.
            let _guard = PipelineGuard {
                ctx: Arc::clone(&ctx),
                hostname: client_hostname.clone(),
            };

            if let Err(e) = connection_pipeline(
                Arc::clone(&ctx),
                lifecycle_state,
                socket,
                connection_result,
                client_hostname.clone(),
                client_ip,
            ) {
                error!("Handshake error for {client_hostname}: {e}");
            }

            let mut clients_to_be_removed = ctx.clients_to_be_removed.lock();

            let action = if clients_to_be_removed.contains(&client_hostname) {
                clients_to_be_removed.remove(&client_hostname);

                ClientConnectionsAction::RemoveEntry
            } else {
                ClientConnectionsAction::SetConnectionState(ConnectionState::Disconnected)
            };
            SESSION_MANAGER
                .write()
                .update_client_connections(client_hostname, action);
        }
    }));

    Ok(())
}

fn connection_pipeline(
    ctx: Arc<ConnectionContext>,
    lifecycle_state: Arc<RwLock<LifecycleState>>,
    socket: ProtoControlSocket,
    connection_result: ClientConnectionResult,
    client_hostname: String,
    client_ip: IpAddr,
) -> ConResult {
    dbg_connection!("connection_pipeline: Begin");

    // This session lock will make sure settings and client list cannot be changed while connecting
    // to thos client, no other client can connect until handshake is finished. It will then be
    // temporarily relocked while shutting down the threads.
    let mut session_manager_lock = SESSION_MANAGER.write();

    dbg_connection!("connection_pipeline: Setting client state in session");
    session_manager_lock.update_client_connections(
        client_hostname.clone(),
        ClientConnectionsAction::SetConnectionState(ConnectionState::Connecting),
    );
    session_manager_lock.update_client_connections(
        client_hostname.clone(),
        ClientConnectionsAction::UpdateCurrentIp(Some(client_ip)),
    );

    let maybe_streaming_caps =
        if let ClientConnectionResult::ConnectionAccepted(info) = connection_result {
            session_manager_lock.update_client_connections(
                client_hostname.clone(),
                ClientConnectionsAction::SetDisplayName(info.platform_string),
            );

            if info.client_protocol_id != alvr_common::protocol_id_u64() {
                warn!(
                    "Trusted client is incompatible! Expected protocol ID: {}, found: {}",
                    alvr_common::protocol_id_u64(),
                    info.client_protocol_id,
                );

                return Ok(());
            }

            info.streaming_capabilities
        } else {
            debug!("Found client in standby. Retrying");
            return Ok(());
        };

    let Some(streaming_caps) = maybe_streaming_caps else {
        con_bail!("Only streaming clients are supported for now");
    };

    let initial_settings = session_manager_lock.settings().clone();

    fn get_view_res(config: FrameSize, default_res: UVec2) -> UVec2 {
        let res = match config {
            FrameSize::Scale(scale) => default_res.as_vec2() * scale,
            FrameSize::Absolute { width, height } => {
                let width = width as f32;
                Vec2::new(
                    width,
                    height.map_or_else(
                        || {
                            let default_res = default_res.as_vec2();
                            width * default_res.y / default_res.x
                        },
                        |h| h as f32,
                    ),
                )
            }
        };

        UVec2::new(align32(res.x), align32(res.y))
    }

    let mut transcoding_view_resolution = get_view_res(
        initial_settings.video.transcoding_view_resolution.clone(),
        streaming_caps.default_view_resolution,
    );
    if transcoding_view_resolution.x > streaming_caps.max_view_resolution.x
        || transcoding_view_resolution.y > streaming_caps.max_view_resolution.y
    {
        warn!(
            "Chosen resolution {}x{} exceeds client maximum supported resolution of {}x{}. \
            Using maximum supported resolution at same aspect ratio.",
            transcoding_view_resolution.x,
            transcoding_view_resolution.y,
            streaming_caps.max_view_resolution.x,
            streaming_caps.max_view_resolution.y,
        );

        let transcoding_ratio =
            transcoding_view_resolution.x as f32 / transcoding_view_resolution.y as f32;

        if transcoding_ratio
            > streaming_caps.max_view_resolution.x as f32
                / streaming_caps.max_view_resolution.y as f32
        {
            transcoding_view_resolution = UVec2::new(
                align32(streaming_caps.max_view_resolution.x as f32),
                align32(streaming_caps.max_view_resolution.x as f32 / transcoding_ratio),
            );
        } else {
            transcoding_view_resolution = UVec2::new(
                align32(streaming_caps.max_view_resolution.y as f32 * transcoding_ratio),
                align32(streaming_caps.max_view_resolution.y as f32),
            );
        }
    }

    let emulated_headset_view_resolution = get_view_res(
        initial_settings
            .video
            .emulated_headset_view_resolution
            .clone(),
        streaming_caps.default_view_resolution,
    );

    let fps = {
        let mut best_match = 0_f32;
        let mut min_diff = f32::MAX;
        for rate in &streaming_caps.refresh_rates {
            let diff = (*rate - initial_settings.video.preferred_fps).abs();
            if diff < min_diff {
                best_match = *rate;
                min_diff = diff;
            }
        }
        best_match
    };

    if !streaming_caps
        .refresh_rates
        .contains(&initial_settings.video.preferred_fps)
    {
        warn!("Chosen refresh rate not supported. Using {fps}Hz");
    }

    let foveated_encoding = if let Switch::Enabled(config) =
        &initial_settings.video.foveated_encoding
    {
        if streaming_caps.foveated_encoding || config.force_enable {
            let mut params = AlvrFoveatedEncodingParams {
                edge_ratio: config.edge_ratio,
                ..Default::default()
            };

            for (axis, resolution) in transcoding_view_resolution
                .to_array()
                .into_iter()
                .enumerate()
            {
                let resolution = resolution as f32;
                // # Safety: `axis` comes from a two-element resolution array, and each
                // configuration array also contains exactly two axes.
                let (center_size, center_shift, edge_ratio) = (
                    config.center_size[axis],
                    config.center_shift[axis],
                    config.edge_ratio[axis],
                );
                let edge_size = resolution - center_size * resolution;

                // Preserve the existing C++ encoder's operation order and double intermediates.
                let center_size = (1.0
                    - (edge_size as f64 / (edge_ratio as f64 * 2.0)).ceil()
                        * (edge_ratio as f64 * 2.0)
                        / resolution as f64) as f32;
                let edge_size = resolution - center_size * resolution;
                let center_shift = if !center_shift.is_finite()
                    || !edge_size.is_finite()
                    || !edge_ratio.is_finite()
                    || edge_size <= 0.0
                    || edge_ratio <= 0.0
                {
                    0.0
                } else {
                    let step = edge_ratio * 2.0 / edge_size;
                    if !step.is_finite() || step >= 1.0 {
                        0.0
                    } else {
                        // Reserve one alignment step on each edge to avoid singular inverse coefficients.
                        // Do not replace this with division by `step`: f32 rounding can change the ceiling.
                        let aligned = (center_shift * edge_size / (edge_ratio * 2.0)).ceil()
                            * (edge_ratio * 2.0)
                            / edge_size;

                        aligned.clamp(-1.0 + step, 1.0 - step)
                    }
                };

                let scale =
                    (center_size as f64 + (1.0 - center_size as f64) / edge_ratio as f64) as f32;
                let optimized_resolution = scale * resolution;
                let encoded_resolution = (optimized_resolution / 32.0).ceil() as u32 * 32;

                // # Safety: `axis` is 0 or 1; all output axis arrays and the eye array have length 2.
                (
                    params.encoded_view_resolution[axis],
                    params.view_ratio[axis],
                    params.center_size[axis],
                    params.center_shifts[0][axis],
                    params.center_shifts[1][axis],
                ) = (
                    encoded_resolution,
                    optimized_resolution / encoded_resolution as f32,
                    center_size,
                    center_shift,
                    center_shift,
                );
            }

            Some(params)
        } else {
            warn!("Foveated encoding is not supported by the client.");

            None
        }
    } else {
        None
    };

    let encoder_profile = if initial_settings.video.encoder_config.h264_profile == H264Profile::High
    {
        let profile = if streaming_caps.encoder_high_profile {
            H264Profile::High
        } else {
            H264Profile::Main
        };

        if profile != H264Profile::High {
            warn!("High profile encoding is not supported by the client.");
        }

        profile
    } else {
        initial_settings.video.encoder_config.h264_profile
    };

    let mut enable_10_bits_encoding = initial_settings
        .video
        .encoder_config
        .use_10bit
        .unwrap_or(streaming_caps.prefer_10bit);

    if enable_10_bits_encoding && !streaming_caps.encoder_10_bits {
        warn!("10 bits encoding is not supported by the client.");
        enable_10_bits_encoding = false
    }

    let enable_hdr = initial_settings
        .video
        .encoder_config
        .hdr
        .enable
        .unwrap_or(streaming_caps.prefer_hdr);

    let encoding_gamma = initial_settings
        .video
        .encoder_config
        .encoding_gamma
        .unwrap_or(streaming_caps.preferred_encoding_gamma);

    let codec = if initial_settings.video.preferred_codec == CodecType::AV1 {
        let codec = if streaming_caps.encoder_av1 {
            CodecType::AV1
        } else {
            CodecType::Hevc
        };

        if codec != CodecType::AV1 {
            warn!("AV1 encoding is not supported by the client.");
        }

        codec
    } else {
        initial_settings.video.preferred_codec
    };

    #[cfg(not(target_os = "windows"))]
    let game_audio_sample_rate = 44100;

    #[cfg(target_os = "windows")]
    let game_audio_sample_rate =
        if let Switch::Enabled(game_audio_config) = &initial_settings.audio.game_audio {
            let game_audio_device =
                alvr_audio::new_output(game_audio_config.device.as_ref()).to_con()?;

            if let Switch::Enabled(microphone_config) = &initial_settings.audio.microphone
                && matches!(
                    microphone_config.devices,
                    alvr_session::MicrophoneDevicesConfig::VAC
                        | alvr_session::MicrophoneDevicesConfig::VBCable
                )
            {
                let (sink, _) =
                    alvr_audio::new_virtual_microphone_pair(microphone_config.devices.clone())
                        .to_con()?;

                // VoiceMeeter and Custom devices may have arbitrary internal routing.
                // Therefore, we cannot detect the loopback issue without knowing the routing.
                if alvr_audio::is_same_device(&game_audio_device, &sink) {
                    con_bail!("Game audio and microphone cannot point to the same device!");
                }
            }

            alvr_audio::input_sample_rate(&game_audio_device).to_con()?
        } else {
            0
        };

    let wired = client_ip.is_loopback();

    dbg_connection!("connection_pipeline: send streaming config");
    let stream_config_packet = StreamConfigPacket::new(
        session_manager_lock.session(),
        ClientNegotiatedStreamingConfig {
            view_resolution: transcoding_view_resolution,
            refresh_rate_hint: fps,
            game_audio_sample_rate,
            foveated_encoding,
            encoding_gamma,
            enable_hdr,
            wired,
            ext_str: String::new(),
        }
        .with_ext(NegotiatedStreamingConfigExt {}),
    )
    .to_con()?;

    let new_steamvr_hmd_init_config = SteamvrHmdInitConfig {
        eye_resolution_width: transcoding_view_resolution.x,
        eye_resolution_height: transcoding_view_resolution.y,
        target_eye_resolution_width: emulated_headset_view_resolution.x,
        target_eye_resolution_height: emulated_headset_view_resolution.y,
        refresh_rate: fps as _,
    };
    let new_hash = compute_restart_settings_hash(&new_steamvr_hmd_init_config, &initial_settings);
    if session_manager_lock.session().restart_settings_hash != new_hash {
        let mut session = session_manager_lock.session_mut();
        session.steamvr_hmd_init_config = new_steamvr_hmd_init_config;
        session.restart_settings_hash = new_hash;

        alvr_sockets::send_restart_signal(socket, stream_config_packet)?;

        crate::notify_restart_driver();

        *lifecycle_state.write() = LifecycleState::ShuttingDown;

        return Ok(());
    }

    let stream_protocol = if wired {
        SocketProtocol::Tcp
    } else {
        initial_settings.connection.stream_protocol
    };

    dbg_connection!("connection_pipeline: Finishing handshake");
    let mut socket = SocketConnection::from_client_connection(
        socket,
        HANDSHAKE_ACTION_TIMEOUT,
        stream_config_packet,
        StreamSocketConfig {
            protocol: stream_protocol,
            port: initial_settings.connection.stream_port,
            buffer_config: initial_settings.connection.server_buffer_config,
            max_packet_size: initial_settings.connection.packet_size as _,
            dscp: initial_settings.connection.dscp,
        },
    )?;

    dbg_connection!("connection_pipeline: Handshake successful, spawning threads");

    let disconnect_notif = Arc::new(Condvar::new());

    // Fork phase sync: steers the virtual vsync that paces SteamVR (PostPresent, Windows only, and
    // only with server frame pacing); announced to the client with RealTimeConfig EXT_PHASE_SYNC
    let phase_sync_target_ms = initial_settings
        .video
        .phase_sync
        .as_option()
        .filter(|_| cfg!(windows) && initial_settings.video.enforce_server_frame_pacing)
        .map(|config| config.target_margin_ms);
    // Late pickup (fork, with phase sync only): announced with RealTimeConfig EXT_LATE_PICKUP;
    // only a client that knows the token changes its pick rule and margin reference
    let late_pickup_token = initial_settings
        .video
        .phase_sync
        .as_option()
        .filter(|_| phase_sync_target_ms.is_some())
        .and_then(|config| {
            config.late_pickup.as_option().map(|late_pickup| {
                alvr_packets::late_pickup_token(alvr_packets::LatePickupAnnouncement {
                    safety_margin_ms: late_pickup.safety_margin_ms.clamp(0.0, 4.0),
                    target_margin_ms: config.target_margin_ms.clamp(0.0, 4.0),
                })
            })
        });
    info!(
        "Phase sync: {}",
        match phase_sync_target_ms {
            Some(target) => format!(
                "on, target margin {target:.1} ms, late pickup {}",
                late_pickup_token.as_deref().unwrap_or("off")
            ),
            None => "off (setting, or server frame pacing off)".into(),
        }
    );
    *ctx.statistics_manager.write() = Some(StatisticsManager::new(
        initial_settings.connection.statistics_history_size,
        Duration::from_secs_f32(1.0 / fps),
        if let Switch::Enabled(config) = &initial_settings.headset.controllers {
            config.steamvr_pipeline_frames
        } else {
            0.0
        },
        phase_sync_target_ms,
    ));
    *ctx.bitrate_manager.lock() =
        BitrateManager::new(initial_settings.video.bitrate.history_size, fps);
    *ctx.tracking_manager.write() =
        TrackingManager::new(initial_settings.connection.statistics_history_size);

    let control_sender = Arc::new(Mutex::new(socket.request_reliable_stream()?));
    let mut video_sender = socket.request_unreliable_stream(VIDEO);
    let game_audio_sender: alvr_sockets::StreamSender<()> = socket.request_unreliable_stream(AUDIO);
    let haptics_sender = socket.request_unreliable_stream(HAPTICS);

    let mut screen_video_sender =
        socket.request_unreliable_stream::<ScreenVideoPacketHeader>(SCREEN_VIDEO);
    let mut control_receiver = socket.subscribe_to_reliable_stream()?;
    let mut microphone_receiver: alvr_sockets::StreamReceiver<()> =
        socket.subscribe_to_unreliable_stream(AUDIO, MAX_UNREAD_PACKETS);
    let tracking_receiver =
        socket.subscribe_to_unreliable_stream::<TrackingData>(TRACKING, MAX_UNREAD_PACKETS);
    let mut statics_receiver =
        socket.subscribe_to_unreliable_stream::<ClientStatistics>(STATISTICS, MAX_UNREAD_PACKETS);

    let (video_channel_sender, video_channel_receiver) =
        std::sync::mpsc::sync_channel(initial_settings.connection.max_queued_server_video_frames);
    *ctx.video_channel_sender.lock() = Some(video_channel_sender);
    *ctx.haptics_sender.lock() = Some(haptics_sender);

    let video_send_thread = thread::spawn({
        let ctx = Arc::clone(&ctx);
        let client_hostname = client_hostname.clone();
        move || {
            while is_streaming(&client_hostname) {
                let VideoPacket {
                    mut header,
                    payload,
                } = match video_channel_receiver.recv_timeout(STREAMING_RECV_TIMEOUT) {
                    Ok(packet) => packet,
                    Err(RecvTimeoutError::Timeout) => continue,
                    Err(RecvTimeoutError::Disconnected) => return,
                };

                {
                    let tracking_manager_lock = ctx.tracking_manager.read();
                    tracking_manager_lock.unrecenter_view_params(&mut header.global_view_params);
                    tracking_manager_lock.unrecenter_opaque_regions(&mut header.opaque_regions);
                }

                // todo: use get_buffer and make encoder write to socket buffers directly to avoid copy
                video_sender
                    .send_header_with_payload(&header, &payload)
                    .ok();
            }
        }
    });

    #[cfg_attr(target_os = "linux", expect(unused_variables))]
    let game_audio_thread = if let Switch::Enabled(config) =
        initial_settings.audio.game_audio.clone()
    {
        #[cfg(windows)]
        let ctx = Arc::clone(&ctx);

        let client_hostname = client_hostname.clone();
        thread::spawn(move || {
            #[cfg(not(target_os = "linux"))]
            while is_streaming(&client_hostname) {
                {
                    let device = match alvr_audio::new_output(config.device.as_ref()) {
                        Ok(data) => data,
                        Err(e) => {
                            warn!("New audio device failed: {e:?}");
                            thread::sleep(RETRY_CONNECT_MIN_INTERVAL);
                            continue;
                        }
                    };

                    #[cfg(windows)]
                    if let Ok(id) = alvr_audio::get_windows_device_id(&device) {
                        let prop = alvr_session::OpenvrProperty {
                            key: alvr_session::OpenvrPropKey::AudioDefaultPlaybackDeviceIdString,
                            value: id,
                        };
                        ctx.events_sender
                            .send(ServerCoreEvent::SetOpenvrProperty {
                                device_id: *alvr_common::HEAD_ID,
                                prop,
                            })
                            .ok();
                    } else {
                        continue;
                    };

                    if let Err(e) = alvr_audio::record_audio_blocking(
                        Arc::new({
                            let client_hostname = client_hostname.clone();
                            move || is_streaming(&client_hostname)
                        }),
                        game_audio_sender.clone(),
                        &device,
                        2,
                        config.mute_when_streaming,
                    ) {
                        error!("Audio record error: {e:?}");
                    }

                    #[cfg(windows)]
                    if let Ok(id) = alvr_audio::new_output(None)
                        .and_then(|d| alvr_audio::get_windows_device_id(&d))
                    {
                        let prop = alvr_session::OpenvrProperty {
                            key: alvr_session::OpenvrPropKey::AudioDefaultPlaybackDeviceIdString,
                            value: id,
                        };
                        ctx.events_sender
                            .send(ServerCoreEvent::SetOpenvrProperty {
                                device_id: *alvr_common::HEAD_ID,
                                prop,
                            })
                            .ok();
                    }
                }
            }
        })
    } else {
        thread::spawn(|| ())
    };

    #[cfg(not(target_os = "linux"))]
    let microphone_thread = if let Switch::Enabled(config) =
        initial_settings.audio.microphone.clone()
    {
        #[allow(unused_variables)]
        let (sink, source) = alvr_audio::new_virtual_microphone_pair(config.devices).to_con()?;

        #[cfg(windows)]
        if let Ok(id) = alvr_audio::get_windows_device_id(&source) {
            ctx.events_sender
                .send(ServerCoreEvent::SetOpenvrProperty {
                    device_id: *alvr_common::HEAD_ID,
                    prop: alvr_session::OpenvrProperty {
                        key: alvr_session::OpenvrPropKey::AudioDefaultRecordingDeviceIdString,
                        value: id,
                    },
                })
                .ok();
        }

        let client_hostname = client_hostname.clone();
        thread::spawn(move || {
            alvr_common::show_err(alvr_audio::play_audio_loop(
                {
                    let client_hostname = client_hostname.clone();
                    move || is_streaming(&client_hostname)
                },
                &sink,
                1,
                streaming_caps.microphone_sample_rate,
                config.buffering,
                &mut microphone_receiver,
            ));
        })
    } else {
        thread::spawn(|| ())
    };

    #[cfg(target_os = "linux")]
    let microphone_thread = {
        use alvr_audio::linux::{self, AudioInfo};
        let mic = if let Switch::Enabled(config) = initial_settings.audio.microphone.clone() {
            Some((
                AudioInfo {
                    sample_rate: streaming_caps.microphone_sample_rate,
                    channel_count: 1,
                },
                config.buffering,
            ))
        } else {
            None
        };

        let audio_info = initial_settings
            .audio
            .game_audio
            .enabled()
            .then_some(AudioInfo {
                sample_rate: game_audio_sample_rate,
                channel_count: 2,
            });

        if mic.is_some() || audio_info.is_some() {
            let client_hostname = client_hostname.clone();
            thread::spawn(move || {
                linux::audio_loop(
                    {
                        let client_hostname = client_hostname.clone();
                        move || is_streaming(&client_hostname)
                    },
                    game_audio_sender,
                    audio_info,
                    &mut microphone_receiver,
                    mic,
                );
            })
        } else {
            thread::spawn(|| ())
        }
    };

    let tracking_receive_thread = thread::spawn({
        let ctx = Arc::clone(&ctx);
        let initial_settings = initial_settings.clone();
        let client_hostname = client_hostname.clone();
        move || {
            tracking::tracking_loop(&ctx, initial_settings, tracking_receiver, || {
                is_streaming(&client_hostname)
            });
        }
    });

    let statistics_thread = thread::spawn({
        let ctx = Arc::clone(&ctx);
        let client_hostname = client_hostname.clone();
        move || {
            while is_streaming(&client_hostname) {
                let data = match statics_receiver.recv(STREAMING_RECV_TIMEOUT) {
                    Ok(stats) => stats,
                    Err(ConnectionError::TryAgain(_)) => continue,
                    Err(ConnectionError::Other(_)) => return,
                };
                let Ok(client_stats) = data.get_header() else {
                    return;
                };

                if let Some(stats) = &mut *ctx.statistics_manager.write() {
                    let timestamp = client_stats.target_timestamp;
                    let decoder_latency = client_stats.video_decode;
                    let (network_latency, game_latency) = stats.report_statistics(client_stats);

                    ctx.events_sender
                        .send(ServerCoreEvent::GameRenderLatencyFeedback(game_latency))
                        .ok();

                    let session_manager_lock = SESSION_MANAGER.read();
                    ctx.bitrate_manager.lock().report_frame_latencies(
                        &session_manager_lock.settings().video.bitrate.mode,
                        timestamp,
                        network_latency,
                        decoder_latency,
                    );
                }
            }
        }
    });

    // Scene mode from the overlay app's native quads block (written by the screen thread): a
    // scene app (V3, full VR) is presenting, so passthrough and chroma key are turned off
    let scene_mode = Arc::new(RelaxedAtomic::new(false));
    // Nothing of the overlay app's is in the stream (native quads block flag): forwarded to the
    // client in RealTimeConfig.ext_str, which then skips rendering the stream
    let stream_unused = Arc::new(RelaxedAtomic::new(false));
    // A scene app is starting or shutting down (native quads block flag): forwarded to the client in
    // RealTimeConfig.ext_str, which then shows its loading screen
    let scene_loading = Arc::new(RelaxedAtomic::new(false));

    let real_time_update_thread = thread::spawn({
        let control_sender = Arc::clone(&control_sender);
        let client_hostname = client_hostname.clone();
        let scene_mode = Arc::clone(&scene_mode);
        let stream_unused = Arc::clone(&stream_unused);
        let scene_loading = Arc::clone(&scene_loading);
        move || {
            let mut previous_config = None;
            let mut previous_scene_mode = false;
            let mut previous_stream_unused = false;
            let mut previous_scene_loading = false;
            while is_streaming(&client_hostname) {
                let mut config = {
                    let session_manager_lock = SESSION_MANAGER.read();
                    let settings = session_manager_lock.settings();

                    RealTimeConfig::from_settings(settings)
                };

                let scene_mode = scene_mode.value();
                if scene_mode != previous_scene_mode {
                    if scene_mode {
                        info!("Scene mode on: passthrough and chroma key off");
                    } else {
                        info!(
                            "Scene mode off: passthrough restored ({})",
                            if config.passthrough.is_some() {
                                "configured"
                            } else {
                                "none configured"
                            }
                        );
                    }
                    previous_scene_mode = scene_mode;
                }
                if scene_mode {
                    config.passthrough = None;
                }

                let stream_unused = stream_unused.value();
                if stream_unused != previous_stream_unused {
                    info!(
                        "Stream unused flag {}: the client {} the stream",
                        if stream_unused { "on" } else { "off" },
                        if stream_unused { "skips" } else { "renders" }
                    );
                    previous_stream_unused = stream_unused;
                }
                let scene_loading = scene_loading.value();
                if scene_loading != previous_scene_loading {
                    info!(
                        "Scene loading flag {}: the client {} its loading screen",
                        if scene_loading { "on" } else { "off" },
                        if scene_loading { "shows" } else { "may hide" }
                    );
                    previous_scene_loading = scene_loading;
                }

                let mut ext_tokens = vec![];
                if stream_unused {
                    ext_tokens.push(alvr_packets::EXT_STREAM_UNUSED);
                }
                if scene_mode {
                    ext_tokens.push(alvr_packets::EXT_SCENE_MODE);
                }
                if scene_loading {
                    ext_tokens.push(alvr_packets::EXT_SCENE_LOADING);
                }
                // Phase sync needs the server's frame pacing (real-time setting): without it the
                // client keeps its upstream frame queue behaviour
                if phase_sync_target_ms.is_some()
                    && SESSION_MANAGER
                        .read()
                        .settings()
                        .video
                        .enforce_server_frame_pacing
                {
                    ext_tokens.push(alvr_packets::EXT_PHASE_SYNC);
                    if let Some(token) = &late_pickup_token {
                        ext_tokens.push(token.as_str());
                    }
                }
                config.ext_str = ext_tokens.join(",");

                let same_config = previous_config.as_ref().is_some_and(|prev| config == *prev);
                if !same_config {
                    previous_config = Some(config.clone());

                    control_sender
                        .lock()
                        .send(&ServerControlPacket::RealTimeConfig(config))
                        .ok();
                }

                thread::sleep(REAL_TIME_UPDATE_INTERVAL);
            }
        }
    });

    let keepalive_thread = thread::spawn({
        let control_sender = Arc::clone(&control_sender);
        let disconnect_notif = Arc::clone(&disconnect_notif);
        let client_hostname = client_hostname.clone();
        move || {
            // Last camera frames request sent to the client (the client starts without one)
            let mut sent_camera_frames = None;
            while is_streaming(&client_hostname) {
                let camera_frames = tracking_extras::camera_frames_request().map(|right_camera| {
                    CameraFramesConfig {
                        image_size: CAMERA_FRAME_SIZE,
                        interval: CAMERA_FRAME_INTERVAL,
                        eye: if right_camera {
                            CameraEye::Right
                        } else {
                            CameraEye::Left
                        },
                    }
                });
                if camera_frames != sent_camera_frames
                    && control_sender
                        .lock()
                        .send(&ServerControlPacket::CameraFrames(camera_frames))
                        .is_ok()
                {
                    info!("Camera frames request sent to the client: {camera_frames:?}");
                    sent_camera_frames = camera_frames;
                }

                if let Err(e) = control_sender.lock().send(&ServerControlPacket::KeepAlive) {
                    info!("Client disconnected. Cause: {e:?}");

                    disconnect_notif.notify_one();

                    return;
                }

                thread::sleep(KEEPALIVE_INTERVAL);
            }
        }
    });

    // Keyframe requests for screen video streams, from the control thread to the screen thread
    let (screen_idr_sender, screen_idr_receiver) = mpsc::channel::<u32>();

    // Native quads and screen video from the overlay app (ScreenVR)
    let screen_thread = thread::spawn({
        let ctx = Arc::clone(&ctx);
        let control_sender = Arc::clone(&control_sender);
        let client_hostname = client_hostname.clone();
        let scene_mode = Arc::clone(&scene_mode);
        let stream_unused = Arc::clone(&stream_unused);
        let scene_loading = Arc::clone(&scene_loading);
        move || {
            let mut screen_video_reader = ScreenVideoReader::new();
            let mut native_quads_reader = NativeQuadsReader::new();
            // Last native quads sent to the client, as Debug text for comparison (Pose has no
            // PartialEq). The client starts without any.
            let mut sent_native_quads = format!("{:?}", Vec::<NativeQuad>::new());
            let mut last_native_quads_poll = None::<Instant>;
            // Anchor request sequence last forwarded on this connection (the client dedups across
            // connections with its persisted served sequence)
            let mut sent_anchor_sequence = None::<u32>;
            let mut sent_frames = 0_u64;
            let mut bitrate_window_start = Instant::now();
            let mut bitrate_window_bytes = 0_usize;
            let mut screen_video_bps = 0.0_f32;

            while is_streaming(&client_hostname) {
                for frame in screen_video_reader.read(SCREEN_VIDEO_WAIT) {
                    if sent_frames == 0
                        || frame.header.is_keyframe && sent_frames.is_multiple_of(1000)
                    {
                        info!(
                            "Screen video: forwarding stream {} frame {} ({} bytes, keyframe {})",
                            frame.header.stream_id,
                            frame.header.frame_index,
                            frame.data.len(),
                            frame.header.is_keyframe
                        );
                    }
                    sent_frames += 1;
                    bitrate_window_bytes += frame.data.len();
                    screen_video_sender
                        .send_header_with_payload(&frame.header, &frame.data)
                        .ok();
                }

                let window = bitrate_window_start.elapsed();
                if window >= SCREEN_VIDEO_BITRATE_INTERVAL {
                    let bps = bitrate_window_bytes as f32 * 8.0 / window.as_secs_f32();
                    screen_video_bps += (bps - screen_video_bps) * SCREEN_VIDEO_BITRATE_SMOOTHING;
                    if bps == 0.0 && screen_video_bps < 1e5 {
                        screen_video_bps = 0.0;
                    }
                    bitrate_window_start = Instant::now();
                    bitrate_window_bytes = 0;

                    if ctx
                        .bitrate_manager
                        .lock()
                        .report_screen_video_bitrate(screen_video_bps)
                    {
                        info!(
                            "Screen video: {:.1} Mbps, subtracted from the main stream bitrate (keeping at least half)",
                            screen_video_bps / 1e6
                        );
                    }
                }

                while let Ok(stream_id) = screen_idr_receiver.try_recv() {
                    screen_video_reader.request_idr(stream_id);
                }

                if last_native_quads_poll
                    .is_none_or(|instant| instant.elapsed() >= NATIVE_QUADS_POLL_INTERVAL)
                {
                    last_native_quads_poll = Some(Instant::now());

                    let native_quads = native_quads_reader.read(tracking_extras::stage_epoch());
                    scene_mode.set(native_quads_reader.scene_mode());
                    stream_unused.set(native_quads_reader.stream_unused());
                    scene_loading.set(native_quads_reader.scene_loading());
                    let native_quads_text = format!("{native_quads:?}");
                    let count = native_quads.len();
                    if native_quads_text != sent_native_quads
                        && control_sender
                            .lock()
                            .send(&ServerControlPacket::NativeQuads(native_quads))
                            .is_ok()
                    {
                        debug!("Native quads sent to the client: {count}");
                        sent_native_quads = native_quads_text;
                    }

                    if let Some(request) = tracking_extras::anchor_request()
                        && sent_anchor_sequence != Some(request.sequence)
                        && control_sender
                            .lock()
                            .send(&ServerControlPacket::AnchorRequest(request))
                            .is_ok()
                    {
                        info!("Anchor request {} sent to the client", request.sequence);
                        sent_anchor_sequence = Some(request.sequence);
                    }
                }
            }
        }
    });

    let control_receive_thread = thread::spawn({
        let ctx = Arc::clone(&ctx);

        let controllers_config = session_manager_lock
            .settings()
            .headset
            .controllers
            .as_option();
        let mut controller_button_mapping_manager = controllers_config.map(|config| {
            if let Some(mappings) = &config.button_mappings {
                ButtonMappingManager::new_manual(mappings)
            } else {
                ButtonMappingManager::new_automatic(
                    &CONTROLLER_PROFILE_INFO
                        .get(&alvr_common::hash_string(QUEST_CONTROLLER_PROFILE_PATH))
                        .unwrap()
                        .button_set,
                    &config.emulation_mode,
                    &config.button_mapping_config,
                )
            }
        });
        let controllers_emulation_mode =
            controllers_config.map(|config| config.emulation_mode.clone());

        let disconnect_notif = Arc::clone(&disconnect_notif);
        let control_sender = Arc::clone(&control_sender);
        let client_hostname = client_hostname.clone();
        move || {
            let mut disconnection_deadline = Instant::now() + KEEPALIVE_TIMEOUT;
            while is_streaming(&client_hostname) {
                let packet = match control_receiver.recv(STREAMING_RECV_TIMEOUT) {
                    Ok(packet) => packet,
                    Err(ConnectionError::TryAgain(_)) => {
                        if Instant::now() > disconnection_deadline {
                            info!("Client disconnected. Timeout");
                            break;
                        } else {
                            continue;
                        }
                    }
                    Err(e) => {
                        info!("Client disconnected. Cause: {e}");
                        break;
                    }
                };

                match packet {
                    ClientControlPacket::PlayspaceSync(packet) => {
                        // Sent on connection and whenever the client's STAGE is re-established
                        tracking_extras::bump_stage_epoch();

                        if !initial_settings.headset.tracking_ref_only {
                            let session_manager_lock = SESSION_MANAGER.read();
                            let config = &session_manager_lock.settings().headset;
                            ctx.tracking_manager
                                .write()
                                .recenter(&config.recentering_mode);

                            let area = packet.unwrap_or(Vec2::new(2.0, 2.0));
                            let wh = area.x * area.y;
                            if wh.is_finite() && wh > 0.0 {
                                info!("Received new playspace with size: {}", area);
                                ctx.events_sender
                                    .send(ServerCoreEvent::PlayspaceSync(area))
                                    .ok();
                            } else {
                                warn!("Received invalid playspace size: {}", area);
                                ctx.events_sender
                                    .send(ServerCoreEvent::PlayspaceSync(Vec2::new(2.0, 2.0)))
                                    .ok();
                            }
                        }
                    }
                    ClientControlPacket::RequestScreenIdr { stream_id } => {
                        screen_idr_sender.send(stream_id).ok();
                    }
                    ClientControlPacket::RequestIdr => {
                        if let Some(config) = ctx.decoder_config.lock().clone() {
                            control_sender
                                .lock()
                                .send(&ServerControlPacket::DecoderConfig(config))
                                .ok();
                        }
                        ctx.events_sender.send(ServerCoreEvent::RequestIDR).ok();
                    }
                    ClientControlPacket::LocalViewParams(params) => {
                        ctx.events_sender
                            .send(ServerCoreEvent::LocalViewParams(params))
                            .ok();
                    }
                    ClientControlPacket::Battery(packet) => {
                        ctx.events_sender
                            .send(ServerCoreEvent::Battery(packet.clone()))
                            .ok();

                        if let Some(stats) = &mut *ctx.statistics_manager.write() {
                            stats.report_battery(
                                packet.device_id,
                                packet.gauge_value,
                                packet.is_plugged,
                            );
                        }
                    }
                    ClientControlPacket::Buttons(entries) => {
                        {
                            let session_manager_lock = SESSION_MANAGER.read();
                            if session_manager_lock
                                .settings()
                                .extra
                                .logging
                                .log_button_presses
                            {
                                alvr_events::send_event(EventType::Buttons(
                                    entries
                                        .iter()
                                        .map(|e| ButtonEvent {
                                            path: BUTTON_INFO.get(&e.path_id).map_or_else(
                                                || format!("Unknown (ID: {:#16x})", e.path_id),
                                                |info| info.path.to_owned(),
                                            ),
                                            value: e.value,
                                        })
                                        .collect(),
                                ));
                            }
                        }

                        if let Some(manager) = &mut controller_button_mapping_manager {
                            let button_entries = entries
                                .iter()
                                .flat_map(|entry| manager.map_button(entry))
                                .collect::<Vec<_>>();

                            if !button_entries.is_empty() {
                                ctx.events_sender
                                    .send(ServerCoreEvent::Buttons(button_entries))
                                    .ok();
                            }
                        };
                    }
                    ClientControlPacket::ActiveInteractionProfile { input_ids, .. } => {
                        controller_button_mapping_manager = if let Switch::Enabled(config) =
                            &SESSION_MANAGER.read().settings().headset.controllers
                        {
                            if let Some(mappings) = &config.button_mappings {
                                Some(ButtonMappingManager::new_manual(mappings))
                            } else {
                                controllers_emulation_mode.as_ref().map(|emulation_mode| {
                                    ButtonMappingManager::new_automatic(
                                        &input_ids,
                                        emulation_mode,
                                        &config.button_mapping_config,
                                    )
                                })
                            }
                        } else {
                            None
                        };
                    }
                    ClientControlPacket::Log { level, message } => {
                        info!("Client {client_hostname}: [{level:?}] {message}")
                    }
                    ClientControlPacket::KeepAlive | ClientControlPacket::StreamReady => (),
                    ClientControlPacket::ProximityState(headset_is_worn) => {
                        ctx.events_sender
                            .send(ServerCoreEvent::ProximityState(headset_is_worn))
                            .ok();
                    }
                    ClientControlPacket::StageInfo(info) => {
                        tracking_extras::report_stage_info(info);
                    }
                    ClientControlPacket::HandShape(info) => {
                        tracking_extras::report_hand_shape(info);
                    }
                    ClientControlPacket::CameraFrame(frame) => {
                        if let Err(e) = camera_frames::publish_frame(&frame) {
                            warn!("Camera frame dropped: {e:?}");
                        }
                    }
                    // Tagged payloads of newer clients (head tracking continuity, camera frame
                    // tracking stamps); unknown ones are ignored
                    ClientControlPacket::ReservedBuffer(bytes) => {
                        if let Some(report) = alvr_packets::PhaseSyncReport::decode(&bytes) {
                            if let Some(stats) = &mut *ctx.statistics_manager.write() {
                                stats.report_phase_sync(report);
                            }
                        } else {
                            tracking_extras::report_reserved_buffer(&bytes);
                        }
                    }
                    ClientControlPacket::Reserved(_) => (),
                }

                disconnection_deadline = Instant::now() + KEEPALIVE_TIMEOUT;
            }

            disconnect_notif.notify_one()
        }
    });

    let stream_receive_thread = thread::spawn({
        let disconnect_notif = Arc::clone(&disconnect_notif);
        let client_hostname = client_hostname.clone();
        move || {
            while is_streaming(&client_hostname) {
                match socket.recv_poll() {
                    Ok(()) => (),
                    Err(ConnectionError::TryAgain(_)) => continue,
                    Err(e) => {
                        info!("Client disconnected. Cause: {e}");

                        disconnect_notif.notify_one();

                        return;
                    }
                }
            }
        }
    });

    let lifecycle_check_thread = thread::spawn({
        let disconnect_notif = Arc::clone(&disconnect_notif);
        let client_hostname = client_hostname.clone();
        move || {
            while SESSION_MANAGER
                .read()
                .client_list()
                .get(&client_hostname)
                .is_some_and(|c| c.connection_state == ConnectionState::Streaming)
                && *lifecycle_state.read() == LifecycleState::Resumed
            {
                thread::sleep(STREAMING_RECV_TIMEOUT);
            }

            disconnect_notif.notify_one()
        }
    });

    if initial_settings.connection.enable_on_connect_script {
        let on_connect_script = FILESYSTEM_LAYOUT.get().map(|l| l.connect_script()).unwrap();
        info!(
            "Running on connect script (connect): {}",
            on_connect_script.display()
        );
        if let Err(e) = Command::new(&on_connect_script)
            .env("ACTION", "connect")
            .spawn()
        {
            warn!("Failed to run connect script: {e}");
        }
    }
    if initial_settings.extra.capture.startup_video_recording {
        info!("Creating recording file");
        crate::create_recording_file(&ctx, session_manager_lock.settings());
    }

    session_manager_lock.update_client_connections(
        client_hostname.clone(),
        ClientConnectionsAction::SetConnectionState(ConnectionState::Streaming),
    );

    ctx.events_sender
        .send(ServerCoreEvent::ClientConnected(
            ServerNegotiatedStreamingConfig {
                transcoding_view_resolution,
                emulated_headset_view_resolution: transcoding_view_resolution,
                refresh_rate: fps as _,
                foveated_encoding,
                codec,
                h264_profile: encoder_profile,
                use_10bit_encoder: enable_10_bits_encoding,
                encoding_gamma,
                enable_hdr,
            },
        ))
        .ok();

    dbg_connection!("connection_pipeline: Threads initialized; unlocking streams");
    alvr_common::wait_rwlock(&disconnect_notif, &mut session_manager_lock);
    dbg_connection!("connection_pipeline: Begin connection shutdown");

    // This requests shutdown from threads
    *ctx.video_channel_sender.lock() = None;
    *ctx.haptics_sender.lock() = None;

    *ctx.video_recording_file.lock() = None;

    session_manager_lock.update_client_connections(
        client_hostname.clone(),
        ClientConnectionsAction::SetConnectionState(ConnectionState::Disconnecting),
    );

    let enable_on_disconnect_script = session_manager_lock
        .settings()
        .connection
        .enable_on_disconnect_script;
    if enable_on_disconnect_script {
        let on_disconnect_script = FILESYSTEM_LAYOUT
            .get()
            .map(|l| l.disconnect_script())
            .unwrap();
        info!(
            "Running on disconnect script (disconnect): {}",
            on_disconnect_script.display()
        );
        if let Err(e) = Command::new(&on_disconnect_script)
            .env("ACTION", "disconnect")
            .spawn()
        {
            warn!("Failed to run disconnect script: {e}");
        }
    }

    // Allow threads to shutdown correctly
    drop(session_manager_lock);

    // Ensure shutdown of threads. Each join is logged so that if any thread
    // hangs in the future (e.g. blocked on a socket write_all with no
    // timeout, the failure mode that wedged shutdown at the Disconnecting
    // state), we can see in vrserver.txt which thread is the culprit.
    let join_started = Instant::now();
    info!("connection_pipeline: starting worker-thread joins for {client_hostname}");

    fn join_logged<T>(name: &str, thread: thread::JoinHandle<T>) {
        let t0 = Instant::now();
        thread.join().ok();
        let elapsed_ms = t0.elapsed().as_millis();
        if elapsed_ms > 100 {
            warn!("connection_pipeline: join {name} took {elapsed_ms} ms");
        } else {
            info!("connection_pipeline: join {name} ok ({elapsed_ms} ms)");
        }
    }

    join_logged("video_send_thread", video_send_thread);
    join_logged("game_audio_thread", game_audio_thread);
    join_logged("microphone_thread", microphone_thread);
    join_logged("tracking_receive_thread", tracking_receive_thread);
    join_logged("statistics_thread", statistics_thread);
    join_logged("real_time_update_thread", real_time_update_thread);
    join_logged("control_receive_thread", control_receive_thread);
    join_logged("stream_receive_thread", stream_receive_thread);
    join_logged("keepalive_thread", keepalive_thread);
    join_logged("screen_thread", screen_thread);
    join_logged("lifecycle_check_thread", lifecycle_check_thread);

    info!(
        "connection_pipeline: all worker joins done for {client_hostname} in {} ms",
        join_started.elapsed().as_millis()
    );

    ctx.events_sender
        .send(ServerCoreEvent::ClientDisconnected)
        .ok();

    dbg_connection!("connection_pipeline: End");

    Ok(())
}
