use alvr_common::{
    AlvrFoveatedEncodingParams, BodySkeleton, ConnectionState, DeviceMotion, LogSeverity, Pose,
    ViewParams,
    anyhow::{Error, Result},
    glam::{Quat, UVec2, Vec2},
    semver::Version,
};
use alvr_session::{
    ClientsidePostProcessingConfig, CodecType, PassthroughMode, PerformanceLevel, SessionConfig,
    Settings,
};
use regex::Regex;
use serde::{Deserialize, Serialize};
use serde_json as json;
use std::{
    collections::HashSet,
    fmt::{self, Debug, Write as _},
    net::IpAddr,
    sync::LazyLock,
    time::Duration,
};

mod tracking_state;
pub use tracking_state::*;

pub const TRACKING: u16 = 0;
pub const HAPTICS: u16 = 1;
pub const AUDIO: u16 = 2;
pub const VIDEO: u16 = 3;
pub const STATISTICS: u16 = 4;
/// Screen video streams encoded by an overlay app (ScreenVR), forwarded as is
pub const SCREEN_VIDEO: u16 = 5;

#[derive(Serialize, Deserialize, Clone)]
pub struct VideoStreamingCapabilitiesExt {
    // Nothing for now
}

#[derive(Serialize, Deserialize, Clone)]
pub struct VideoStreamingCapabilities {
    pub default_view_resolution: UVec2,
    pub max_view_resolution: UVec2,
    pub refresh_rates: Vec<f32>,
    pub microphone_sample_rate: u32,
    pub foveated_encoding: bool,
    pub encoder_high_profile: bool,
    pub encoder_10_bits: bool,
    pub encoder_av1: bool,
    pub prefer_10bit: bool,
    pub preferred_encoding_gamma: f32,
    pub prefer_hdr: bool,
    pub ext_str: String,
}

impl VideoStreamingCapabilities {
    pub fn with_ext(self, ext: VideoStreamingCapabilitiesExt) -> Self {
        Self {
            ext_str: json::to_string(&ext).unwrap(),
            ..self
        }
    }

    pub fn ext(&self) -> Result<VideoStreamingCapabilitiesExt> {
        let _ext_json = json::from_str::<json::Value>(&self.ext_str)?;

        // decode values here

        Ok(VideoStreamingCapabilitiesExt {})
    }
}

#[derive(Serialize, Deserialize)]
pub struct ConnectionAcceptedInfo {
    pub client_protocol_id: u64,
    pub platform_string: String,
    pub server_ip: IpAddr, // must be unused for now
    pub streaming_capabilities: Option<VideoStreamingCapabilities>,
}

#[derive(Serialize, Deserialize)]
pub enum ClientConnectionResult {
    ConnectionAccepted(Box<ConnectionAcceptedInfo>),
    ClientStandby,
}

#[derive(Serialize, Deserialize)]
pub struct NegotiatedStreamingConfigExt {
    // Nothing for now
}

#[derive(Serialize, Deserialize, Clone)]
pub struct ClientNegotiatedStreamingConfig {
    pub view_resolution: UVec2,
    pub refresh_rate_hint: f32,
    pub game_audio_sample_rate: u32,
    pub foveated_encoding: Option<AlvrFoveatedEncodingParams>,
    pub encoding_gamma: f32,
    pub enable_hdr: bool,
    pub wired: bool,
    pub ext_str: String,
}

impl ClientNegotiatedStreamingConfig {
    pub fn with_ext(self, ext: NegotiatedStreamingConfigExt) -> Self {
        Self {
            ext_str: json::to_string(&ext).unwrap(),
            ..self
        }
    }

    pub fn ext(&self) -> Result<NegotiatedStreamingConfigExt> {
        let _ext_json = json::from_str::<json::Value>(&self.ext_str)?;

        // decode values here

        Ok(NegotiatedStreamingConfigExt {})
    }
}

#[derive(Serialize, Deserialize)]
pub struct StreamConfigPacket {
    pub session: String, // JSON session that allows for extrapolation
    pub negotiated: ClientNegotiatedStreamingConfig,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct ClientStreamConfig {
    pub server_version: Version,
    pub settings: Settings,
    pub negotiated_config: ClientNegotiatedStreamingConfig,
}

impl StreamConfigPacket {
    pub fn new(
        session: &SessionConfig,
        negotiated: ClientNegotiatedStreamingConfig,
    ) -> Result<Self> {
        Ok(Self {
            session: json::to_string(session)?,
            negotiated,
        })
    }

    pub fn to_stream_config(self) -> Result<ClientStreamConfig> {
        let mut session_config = SessionConfig::default();
        session_config.merge_from_json(&json::from_str(&self.session)?)?;
        let settings = session_config.to_settings();

        Ok(ClientStreamConfig {
            server_version: session_config.server_version,
            settings,
            negotiated_config: self.negotiated,
        })
    }
}

#[derive(Serialize, Deserialize, Clone)]
pub struct DecoderInitializationConfig {
    pub codec: CodecType,
    pub config_buffer: Vec<u8>, // e.g. SPS + PPS NALs
    pub ext_str: String,
}

/// Headset camera frames requested by an overlay app (e.g. for monitor calibration)
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
pub struct CameraFramesConfig {
    /// Side of the square, rectified camera image, in pixels
    pub image_size: u32,
    /// Minimum time between frames
    pub interval: Duration,
    pub eye: CameraEye,
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub enum CameraEye {
    Left,
    Right,
}

/// One rectified headset camera image, as grayscale JPEG
#[derive(Serialize, Deserialize)]
pub struct CameraFrame {
    /// Headset runtime time (XrTime) when the image was taken
    pub capture_time: Duration,
    pub eye: CameraEye,
    pub width: u32,
    pub height: u32,
    /// Pinhole intrinsics of exactly this image, in pixels. The image is rectified (no
    /// distortion).
    pub fx: f32,
    pub fy: f32,
    pub cx: f32,
    pub cy: f32,
    /// Camera pose in the client STAGE space at capture time, with OpenCV camera axes (+X right,
    /// +Y down, +Z viewing direction)
    pub stage_from_camera: Pose,
    /// Head angular speed at capture time, in radians per second (motion blur indicator)
    pub head_angular_speed: f32,
    pub jpeg: Vec<u8>,
}

/// Debug reference quad drawn natively by the client (OpenXR quad layer) with a procedural
/// chessboard texture, requested by an overlay app
#[derive(Serialize, Deserialize, Clone, Copy, Debug)]
pub struct NativeQuad {
    /// Quad centre in the client STAGE space. The quad lies in the local XY plane, front face +Z,
    /// texture top toward +Y.
    pub pose: Pose,
    /// Full size in meters
    pub width: f32,
    pub height: f32,
    /// Chessboard squares across the width and the height
    pub columns: u32,
    pub rows: u32,
    /// RGBA colors: squares starting top-left, the other squares, and the border
    pub dark_color: [u8; 4],
    pub light_color: [u8; 4],
    pub outline_color: [u8; 4],
    /// Border width in texture pixels
    pub outline_pixels: u32,
    /// Screen video stream shown instead of the chessboard (0 = none). The chessboard stays until
    /// the stream's first keyframe is decoded.
    pub video_stream_id: u32,
}

#[derive(Serialize, Deserialize)]
pub enum ServerControlPacket {
    StartStream,
    DecoderConfig(DecoderInitializationConfig),
    Restarting,
    KeepAlive,
    RealTimeConfig(RealTimeConfig),
    Reserved(String),
    ReservedBuffer(Vec<u8>),
    // None stops the frames
    CameraFrames(Option<CameraFramesConfig>),
    // Replaces the debug native quads (empty = none)
    NativeQuads(Vec<NativeQuad>),
    // Overlay app's persistent anchor command (create/replace or forget)
    AnchorRequest(AnchorRequest),
}

/// What the client applies when drawing the hands (reported so that the overlay app can tell the
/// raw tracked joints it gets from what the user sees)
#[derive(Serialize, Deserialize, Clone, Copy, Debug, Default)]
pub struct HandShapeInfo {
    /// Girth across the bones (1 = the mesh as modelled)
    pub thickness: f32,
    /// Uniform scale about the wrist
    pub scale: f32,
    /// Finger length factor: always 1 since the 2026-11-13 cleanup (fingers drawn as tracked)
    pub fingers: f32,
    /// Overlay app finger correction per hand (left, right): always 1 since the 2026-11-13 cleanup
    pub finger_correction: [f32; 2],
    /// Sequence of the last hand correction applied: always 0 since the 2026-11-13 cleanup
    pub correction_sequence: u32,
}

/// Persistent spatial anchor command from an overlay app. Carried out once per new sequence.
#[derive(Serialize, Deserialize, Clone, Copy, Debug)]
pub struct AnchorRequest {
    pub sequence: u32,
    pub kind: AnchorRequestKind,
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug)]
pub enum AnchorRequestKind {
    /// Create (or replace) the anchor at this client STAGE pose
    Create(Pose),
    Forget,
}

/// Identity of the client STAGE space and the state of the persistent anchor
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct StageInfo {
    /// Random, persisted on the headset; changes when STAGE changes (runtime event beyond
    /// tolerance, or the anchor seen moved in STAGE)
    pub stage_identity: u64,
    pub previous_stage_identity: u64,
    pub stage_change_sequence: u32,
    /// p_current = stage_from_previous * p_previous, when known
    pub stage_from_previous: Option<Pose>,
    /// VrAnchorState (0 none, 1 creating, 2 located, 3 lost, 4 unsupported, 5 error)
    pub anchor_state: u32,
    pub anchor_uuid: [u8; 16],
    /// Anchor pose in STAGE (last good pose when lost)
    pub anchor_pose: Option<Pose>,
    pub anchor_generation: u32,
    /// Sequence of the last anchor request carried out
    pub anchor_request_served: u32,
    pub product_name: String,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct BatteryInfo {
    pub device_id: u64,
    pub gauge_value: f32, // range [0, 1]
    pub is_plugged: bool,
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug)]
pub enum ButtonValue {
    Binary(bool),
    Scalar(f32),
}

#[derive(Serialize, Deserialize)]
pub struct ButtonEntry {
    pub path_id: u64,
    pub value: ButtonValue,
}

#[derive(Serialize, Deserialize)]
pub enum ClientControlPacket {
    PlayspaceSync(Option<Vec2>),
    RequestIdr,
    // Keyframe request for a screen video stream (after loss or a decoder error)
    RequestScreenIdr {
        stream_id: u32,
    },
    KeepAlive,
    StreamReady, // This flag notifies the server the client streaming socket is ready listening
    LocalViewParams([ViewParams; 2]), // In relation to head
    Battery(BatteryInfo),
    Buttons(Vec<ButtonEntry>),
    ActiveInteractionProfile {
        device_id: u64,
        profile_id: u64,
        input_ids: HashSet<u64>,
    },
    Log {
        level: LogSeverity,
        message: String,
    },
    ProximityState(bool),
    Reserved(String),
    ReservedBuffer(Vec<u8>),
    CameraFrame(Box<CameraFrame>),
    StageInfo(StageInfo),
    HandShape(HandShapeInfo),
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub enum FaceExpressions {
    Fb(Vec<f32>), // 70 values
    Bd(Vec<f32>), // 52 values
    Htc {
        eye: Option<Vec<f32>>, // 14 values
        lip: Option<Vec<f32>>, // 37 values
    },
}

#[derive(Serialize, Deserialize, Clone, Default, Debug)]
pub struct FaceData {
    // Can be used for foveated eye tracking
    pub eyes_combined: Option<Quat>,
    // Should be used only for social presence
    pub eyes_social: [Option<Quat>; 2],

    pub face_expressions: Option<FaceExpressions>,
}

#[derive(Serialize, Deserialize)]
pub struct TrackingData {
    pub poll_timestamp: Duration,
    pub device_motions: Vec<(u64, DeviceMotion)>,
    pub hand_skeletons: [Option<[Pose; 26]>; 2],
    pub face: FaceData,
    pub body: Option<BodySkeleton>,
    pub markers: Vec<(String, Pose)>,
    // Per hand: the index tip joint in `hand_skeletons` has a valid and tracked position
    pub index_tips_tracked: [bool; 2],
}

/// A region of the frame that must stay opaque when chroma keying, reported by an overlay app
/// (e.g. a virtual screen). It is one OpenVR overlay quad: the quad lies in the local XY plane of
/// `pose` and faces +Z. With `curvature` > 0 it bends around the local Y axis toward +Z, as an arc
/// of a cylinder with radius `width / (2 * PI * curvature)` (OpenVR overlay semantics).
#[derive(Serialize, Deserialize, Clone, Copy, Debug)]
pub struct OpaqueRegion {
    /// Overlay centre, in the same space as the frame view params
    pub pose: Pose,
    /// Full width (arc length when curved), in meters
    pub width: f32,
    pub height: f32,
    pub curvature: f32,
    /// Inward margin applied to every edge before forcing the region opaque, in meters
    pub inset: f32,
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub enum ScreenVideoCodec {
    H264,
    Hevc,
}

/// One encoded access unit (Annex B) of a screen video stream (SCREEN_VIDEO)
#[derive(Serialize, Deserialize, Clone, Copy, Debug)]
pub struct ScreenVideoPacketHeader {
    /// Changes when the encoder is recreated (size or codec change): restart the decoder at the
    /// next keyframe
    pub stream_id: u32,
    pub codec: ScreenVideoCodec,
    pub width: u32,
    pub height: u32,
    /// 1, 2, 3 ... per stream_id
    pub frame_index: u64,
    /// IDR carrying its parameter sets
    pub is_keyframe: bool,
}

#[derive(Serialize, Deserialize)]
pub struct VideoPacketHeader {
    pub timestamp: Duration,
    pub global_view_params: [ViewParams; 2],
    /// Centers used to encode this frame, already aligned. Normally absent when FFR is disabled.
    pub foveation_center_shifts: Option<[[f32; 2]; 2]>,
    pub is_idr: bool,
    // Regions that were visible when this frame was rendered. Usually empty.
    pub opaque_regions: Vec<OpaqueRegion>,
}

#[derive(Serialize, Deserialize)]
pub struct Haptics {
    pub device_id: u64,
    pub duration: Duration,
    pub frequency: f32,
    pub amplitude: f32,
}

#[derive(Serialize, Deserialize, Clone)]
pub enum PathSegment {
    Name(String),
    Index(usize),
}

impl fmt::Display for PathSegment {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PathSegment::Name(name) => write!(f, "{name}"),
            PathSegment::Index(index) => write!(f, "[{index}]"),
        }
    }
}

impl Debug for PathSegment {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self}")
    }
}

impl From<&str> for PathSegment {
    fn from(value: &str) -> Self {
        PathSegment::Name(value.to_owned())
    }
}

impl From<String> for PathSegment {
    fn from(value: String) -> Self {
        PathSegment::Name(value)
    }
}

impl From<usize> for PathSegment {
    fn from(value: usize) -> Self {
        PathSegment::Index(value)
    }
}

// A name segment + a (possibly empty) list of `[index]` segments.
static COMPOUND_SEGMENT: LazyLock<Regex> =
    LazyLock::new(|| Regex::new(r"^([A-Za-z0-9_]+)((?:\[[0-9]+\])*)").unwrap());

/// Parses a settings path like `"video.foveated_encoding.enable"` or
/// `"headset.controllers.content.gestures[2].joystick_range"` into name/index segments.
pub fn parse_path(path: &str) -> Result<Vec<PathSegment>> {
    fn parse_error(path: &str, rest: &str, expected: &str) -> Error {
        let found = rest
            .chars()
            .next()
            .map_or_else(|| "end of input".to_owned(), |c| format!("`{c}`"));
        Error::msg(format!(
            "invalid path \"{path}\" at column {}: expected {expected}, found {found}",
            path.len() - rest.len() + 1,
        ))
    }

    let mut segments = Vec::new();
    let mut rest = path;

    loop {
        let Some(caps) = COMPOUND_SEGMENT.captures(rest) else {
            return Err(parse_error(path, rest, "a name `[A-Za-z0-9_]+`"));
        };
        segments.push(PathSegment::Name(caps[1].to_owned()));
        for index in caps[2].split(['[', ']']).filter(|s| !s.is_empty()) {
            match index.parse() {
                Ok(index) => segments.push(PathSegment::Index(index)),
                Err(_) => {
                    let at = &rest[caps[1].len()..];
                    return Err(parse_error(
                        path,
                        at,
                        &format!("index `{index}` to fit in a usize"),
                    ));
                }
            }
        }
        rest = &rest[caps[0].len()..];

        if rest.is_empty() {
            return Ok(segments);
        }
        rest = rest
            .strip_prefix('.')
            .ok_or_else(|| parse_error(path, rest, "`.` or end of input"))?;
    }
}

pub fn path_to_string(path: &[PathSegment]) -> String {
    let mut string = String::new();
    for (i, segment) in path.iter().enumerate() {
        if i != 0 && matches!(segment, PathSegment::Name(_)) {
            string.push('.');
        }
        let _ = write!(string, "{segment}");
    }
    string
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub enum ClientConnectionsAction {
    AddIfMissing {
        trusted: bool,
        manual_ips: Vec<IpAddr>,
    },
    SetDisplayName(String),
    Trust,
    SetManualIps(Vec<IpAddr>),
    RemoveEntry,
    UpdateCurrentIp(Option<IpAddr>),
    SetConnectionState(ConnectionState),
}

#[derive(Serialize, Deserialize, Default, Clone)]
pub struct ClientStatistics {
    pub target_timestamp: Duration, // identifies the frame
    pub frame_interval: Duration,
    pub video_decode: Duration,
    pub video_decoder_queue: Duration,
    pub rendering: Duration,
    pub vsync_queue: Duration,
    pub total_pipeline_latency: Duration,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct PathValuePair {
    pub path: Vec<PathSegment>,
    pub value: json::Value,
}

#[derive(Serialize, Deserialize, Debug)]
pub enum FirewallRulesAction {
    Add,
    Remove,
}

/// RealTimeConfig.ext_str token: nothing of the overlay app's is in the stream (native quads block
/// flag), the client may skip rendering and submitting it
pub const EXT_STREAM_UNUSED: &str = "stream_unused";

/// RealTimeConfig.ext_str token: a scene app (V3, full VR) owns the view (native quads block flag;
/// passthrough is None in the same config). The client must never show key colour frames then.
pub const EXT_SCENE_MODE: &str = "scene_mode";

/// RealTimeConfig.ext_str token: a scene app is starting or shutting down (native quads block flag
/// kVrNativeQuadsFlagSceneLoading): the client shows its loading screen instead of the stream
pub const EXT_SCENE_LOADING: &str = "scene_loading";

/// RealTimeConfig.ext_str token: the server steers its frame phase from the client's
/// PhaseSyncReport (ReservedBuffer 'PSYN'). Only while it is present does the client send those
/// reports and show the newest ready frame (dropping an older one) instead of the oldest.
pub const EXT_PHASE_SYNC: &str = "phase_sync";

/// RealTimeConfig.ext_str token, only together with EXT_PHASE_SYNC: late pickup. The client waits
/// for the newest decoded frame until a deadline measured from its render loop's wake-up (as late
/// as its render + submit still finishes before the runtime's compositor takes the frame), and
/// reports margins against that deadline (PhaseSyncReport v2, reference DEADLINE). Carries what
/// the client needs: `late_pickup:<safety margin ms>:<target margin ms>` (format with
/// late_pickup_token, read with parse_late_pickup). Older clients ignore it (no exact match).
pub const EXT_LATE_PICKUP: &str = "late_pickup";

/// What EXT_LATE_PICKUP tells the client
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LatePickupAnnouncement {
    /// Kept between the client's estimated end of work and the compositor latch, ms
    pub safety_margin_ms: f32,
    /// The server's target margin (where the client centres the wrap of its margins), ms
    pub target_margin_ms: f32,
}

pub fn late_pickup_token(announcement: LatePickupAnnouncement) -> String {
    format!(
        "{EXT_LATE_PICKUP}:{:.2}:{:.2}",
        announcement.safety_margin_ms, announcement.target_margin_ms
    )
}

/// The EXT_LATE_PICKUP token of a RealTimeConfig.ext_str, if any (a malformed one counts as none)
pub fn parse_late_pickup(ext_str: &str) -> Option<LatePickupAnnouncement> {
    ext_str.split(',').find_map(|token| {
        let mut parts = token.split(':');
        if parts.next()? != EXT_LATE_PICKUP {
            return None;
        }
        let safety_margin_ms = parts.next()?.parse::<f32>().ok()?;
        let target_margin_ms = parts.next()?.parse::<f32>().ok()?;
        (safety_margin_ms.is_finite() && target_margin_ms.is_finite()).then_some(
            LatePickupAnnouncement {
                safety_margin_ms,
                target_margin_ms,
            },
        )
    })
}

// Note: server sends a packet to the client at low frequency, binary encoding, without ensuring
// compatibility between different versions, even if within the same major version.
#[derive(Serialize, Deserialize, PartialEq, Clone)]
pub struct RealTimeConfig {
    pub passthrough: Option<PassthroughMode>,
    pub clientside_post_processing: Option<ClientsidePostProcessingConfig>,
    pub cpu_performance_level: Option<PerformanceLevel>,
    pub gpu_performance_level: Option<PerformanceLevel>,
    pub log_input_diagnostics: bool,
    pub ext_str: String,
}

impl RealTimeConfig {
    pub fn from_settings(settings: &Settings) -> Self {
        Self {
            passthrough: settings.video.passthrough.clone().into_option(),
            clientside_post_processing: settings
                .video
                .clientside_post_processing
                .clone()
                .into_option(),
            cpu_performance_level: settings.headset.performance_level.clone().cpu.into_option(),
            gpu_performance_level: settings.headset.performance_level.clone().gpu.into_option(),
            log_input_diagnostics: settings.extra.logging.log_input_diagnostics,
            ext_str: String::new(), // No extensions for now
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{
        EXT_LATE_PICKUP, EXT_PHASE_SYNC, LatePickupAnnouncement, PathSegment, late_pickup_token,
        parse_late_pickup, parse_path, path_to_string,
    };

    fn debug_path(path: &str) -> String {
        format!("{:?}", parse_path(path).unwrap())
    }

    #[test]
    fn parses_names_only() {
        assert_eq!(debug_path("my.element.content"), "[my, element, content]");
    }

    #[test]
    fn parses_index_between_names() {
        let segments = parse_path("my.element[1].content").unwrap();
        assert!(matches!(
            segments.as_slice(),
            [
                PathSegment::Name(a),
                PathSegment::Name(b),
                PathSegment::Index(1),
                PathSegment::Name(c),
            ] if a == "my" && b == "element" && c == "content"
        ));
    }

    #[test]
    fn parses_chained_indices() {
        assert_eq!(debug_path("matrix[1][2]"), "[matrix, [1], [2]]");
        assert_eq!(debug_path("a[0][12][345]"), "[a, [0], [12], [345]]");
    }

    #[test]
    fn accepts_underscores_and_digits_in_names() {
        assert_eq!(debug_path("_priv.x1[0]._2y"), "[_priv, x1, [0], _2y]");
    }

    #[test]
    fn rejects_malformed_paths() {
        for path in [
            "",
            ".foo",
            "foo.",
            "foo..bar",
            ".[1]",
            "foo.[1]",
            "[0].foo",
            "foo[]",
            "foo[",
            "foo[1",
            "foo[1]bar",
            "foo[-1]",
            "foo[1.5]",
            "foo[key]",
            "foo-bar",
            "foo bar",
            "foo.bar!",
            "foé",
        ] {
            assert!(
                parse_path(path).is_err(),
                "expected \"{path}\" to be rejected"
            );
        }
    }

    #[test]
    fn rejects_index_overflow() {
        assert!(parse_path("foo[99999999999999999999999999]").is_err());
    }

    #[test]
    fn path_to_string_round_trips() {
        for path in [
            "a",
            "a.b.c",
            "a[0]",
            "a[1][2].b",
            "video.foveated_encoding.strength",
            "headset.controllers.content.gestures[2].joystick_range",
        ] {
            assert_eq!(path_to_string(&parse_path(path).unwrap()), path);
        }
    }

    #[test]
    fn error_points_at_the_failing_column() {
        assert_eq!(
            parse_path("foo..bar").unwrap_err().to_string(),
            "invalid path \"foo..bar\" at column 5: expected a name `[A-Za-z0-9_]+`, found `.`",
        );
        assert_eq!(
            parse_path("foo[1]x").unwrap_err().to_string(),
            "invalid path \"foo[1]x\" at column 7: expected `.` or end of input, found `x`",
        );
        assert_eq!(
            parse_path("foo[]").unwrap_err().to_string(),
            "invalid path \"foo[]\" at column 4: expected `.` or end of input, found `[`",
        );
    }

    #[test]
    fn late_pickup_token_round_trips_and_ignores_others() {
        let announcement = LatePickupAnnouncement {
            safety_margin_ms: 1.0,
            target_margin_ms: 2.5,
        };
        let ext = format!(
            "scene_mode,{EXT_PHASE_SYNC},{}",
            late_pickup_token(announcement)
        );
        assert_eq!(parse_late_pickup(&ext), Some(announcement));
        // An exact-match reader (older clients) does not see it as phase sync or anything else
        assert!(!ext.split(',').any(|t| t == EXT_LATE_PICKUP));
        assert_eq!(parse_late_pickup("phase_sync,scene_mode"), None);
        assert_eq!(parse_late_pickup("late_pickup"), None);
        assert_eq!(parse_late_pickup("late_pickup:x:2"), None);
        assert_eq!(parse_late_pickup("late_pickup:NaN:2"), None);
        assert_eq!(parse_late_pickup(""), None);
    }
}
