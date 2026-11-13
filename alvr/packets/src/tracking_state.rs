// Head tracking continuity and the screen video decode acknowledgement, client -> server,
// carried in ClientControlPacket::ReservedBuffer so that peers of different builds stay
// compatible: a server that does not know a payload ignores
// the whole ReservedBuffer (as upstream and every earlier fork server do), and a server that gets
// no report publishes "unknown". Payloads are tagged (4 bytes) and versioned (u16), little-endian;
// a decoder accepts its version or newer (fields are only ever appended) and ignores trailing
// bytes.

use alvr_common::{
    Pose,
    glam::{Quat, Vec3},
};

pub const TRACKING_STATE_TAG: [u8; 4] = *b"SVTS";
pub const CAMERA_STAMP_TAG: [u8; 4] = *b"SVCS";
pub const SCREEN_VIDEO_DECODE_TAG: [u8; 4] = *b"SVDA";
/// Phase sync report (fork-internal, not read by the overlay app)
pub const PHASE_SYNC_TAG: [u8; 4] = *b"PSYN";
const TRACKING_STATE_VERSION: u16 = 1;
const CAMERA_STAMP_VERSION: u16 = 1;
const SCREEN_VIDEO_DECODE_VERSION: u16 = 1;
const PHASE_SYNC_VERSION: u16 = 1;

/// Head tracking flags (TrackingStateReport::flags, CameraTrackingStamp::flags)
pub const HEAD_POSITION_TRACKED: u32 = 1 << 0;
pub const HEAD_ORIENTATION_TRACKED: u32 = 1 << 1;
/// Position tracked continuously for >= 2 s, no re-anchor jump and no reference space change in
/// the last 2 s, and the runtime's tracking-ready vendor event (if it sends one) confirmed
pub const HEAD_SETTLED: u32 = 1 << 2;
/// CameraTrackingStamp only: the client had a tracking sample close to the capture time
pub const HEAD_STAMP_KNOWN: u32 = 1 << 3;

/// Why the client's tracking epoch last changed (values shared with ScreenVR's
/// VrTrackingEpochReason)
pub mod epoch_reason {
    pub const UNKNOWN: u32 = 0;
    pub const CLIENT_START: u32 = 1;
    pub const TRACKING_LOST: u32 = 2;
    pub const TRACKING_REGAINED: u32 = 3;
    pub const REANCHOR_JUMP: u32 = 4;
    pub const STAGE_CHANGED: u32 = 5;
    pub const CONTINUITY_UNKNOWN: u32 = 6;
    pub const SESSION_RESTART: u32 = 7;
}

/// The client's head tracking continuity. Poses measured in STAGE stay valid only while
/// (client_instance, client_epoch) is unchanged.
#[derive(Clone, Copy, Debug)]
pub struct TrackingStateReport {
    /// Random per client process (never persisted: a restarted process cannot prove continuity)
    pub client_instance: u64,
    /// Starts at 1 per client process, incremented at every event that may have moved STAGE
    pub client_epoch: u32,
    /// epoch_reason::*
    pub epoch_reason: u32,
    /// HEAD_* bits (without HEAD_STAMP_KNOWN)
    pub flags: u32,
    /// Time since the client's epoch changed, milliseconds (saturating)
    pub epoch_age_ms: u32,
    /// Pure floor-height changes of STAGE carried over without an epoch change
    pub stage_change_counter: u32,
    /// The latest such change: p_stage_new = stage_from_previous * p_stage_previous
    pub stage_from_previous: Pose,
}

impl PartialEq for TrackingStateReport {
    fn eq(&self, other: &Self) -> bool {
        self.client_instance == other.client_instance
            && self.client_epoch == other.client_epoch
            && self.epoch_reason == other.epoch_reason
            && self.flags == other.flags
            && self.epoch_age_ms == other.epoch_age_ms
            && self.stage_change_counter == other.stage_change_counter
            && self.stage_from_previous.position == other.stage_from_previous.position
            && self.stage_from_previous.orientation == other.stage_from_previous.orientation
    }
}

/// Tracking state at a camera frame's capture time (sent just before the CameraFrame packet)
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CameraTrackingStamp {
    /// Same value as CameraFrame::capture_time (XrTime, nanoseconds)
    pub capture_ns: u64,
    pub client_instance: u64,
    pub client_epoch: u32,
    /// HEAD_* bits at capture time; HEAD_STAMP_KNOWN clear = no sample close to it
    pub flags: u32,
}

/// ScreenVideoDecodeStatus::state (values shared with ScreenVR's VrScreenVideoDecodeState)
pub mod screen_video_decode_state {
    /// No decoder (yet): waiting for a keyframe, or closed
    pub const NO_DECODER: u32 = 0;
    pub const DECODING: u32 = 1;
    /// The decoder could not be created, or too many decoder errors in a row (recreated at the
    /// next keyframe)
    pub const FAILED: u32 = 2;
}

/// ScreenVideoDecodeStatus::flags: a native quad shows this stream now
pub const SCREEN_VIDEO_DECODE_REFERENCED: u32 = 1 << 0;

/// Entries in one report (the overlay app has 4 stream slots; superseded streams may still have a
/// decoder for a while)
pub const SCREEN_VIDEO_DECODE_MAX_STREAMS: usize = 8;

/// What the client decodes of one screen video stream
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ScreenVideoDecodeStatus {
    pub stream_id: u32,
    /// screen_video_decode_state::*
    pub state: u32,
    /// SCREEN_VIDEO_DECODE_REFERENCED
    pub flags: u32,
    /// Decoder errors in a row
    pub error_count: u32,
    /// ScreenVideoPacketHeader::frame_index of the last frame that came out of the decoder and was
    /// released to the quad's surface (0 = none yet)
    pub last_decoded_frame_index: u64,
    /// Frames decoded by the current decoder of this stream
    pub frames_decoded: u64,
}

/// Screen video decode acknowledgement: the client's decoders, sent every 250 ms while it renders a
/// stream. That reports arrive at all is the capability (an older client never sends them).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ScreenVideoDecodeReport {
    /// Random per set of decoders (new per client process and whenever streaming stops)
    pub client_instance: u64,
    /// Streams shown by a quad first; at most SCREEN_VIDEO_DECODE_MAX_STREAMS
    pub streams: Vec<ScreenVideoDecodeStatus>,
}

/// Phase sync: how the decoded main stream frames met the client's render loop during one window
/// (about 500 ms). Sent only while the server announces phase sync (RealTimeConfig.ext_str token
/// EXT_PHASE_SYNC). The margin of a frame is "render loop starts looking for a frame" minus
/// "frame decoded", in the client's clock: positive = the frame waited decoded (decoder queue),
/// negative = the loop waited for it.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PhaseSyncReport {
    /// Increments per report (per client process)
    pub sequence: u32,
    /// Client time covered by this report, microseconds
    pub window_us: u32,
    /// The runtime's display period (xrWaitFrame), microseconds
    pub frame_interval_us: u32,
    /// Render loops that looked for a stream frame
    pub loops: u32,
    /// Loops that picked a new frame with a known decode time (each one a margin sample)
    pub frames: u32,
    /// Loops that skipped the stream (stream unused): no samples
    pub skipped_loops: u32,
    /// Loops that found a frame already waiting while the previous loop got none: that frame
    /// missed its slot (arrived after the previous loop stopped waiting)
    pub missed_slots: u32,
    /// Picked frames that the loop had to wait for (negative margin)
    pub late_frames: u32,
    /// Older decoded frames dropped because a newer one was ready at the same poll
    pub dropped_frames: u32,
    /// Margins in milliseconds (0 when frames == 0)
    pub margin_mean_ms: f32,
    pub margin_p10_ms: f32,
    pub margin_p50_ms: f32,
    pub margin_p90_ms: f32,
}

struct Writer(Vec<u8>);

impl Writer {
    fn new(tag: [u8; 4], version: u16) -> Self {
        let mut bytes = tag.to_vec();
        bytes.extend_from_slice(&version.to_le_bytes());
        Self(bytes)
    }
    fn u32(&mut self, v: u32) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn u64(&mut self, v: u64) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn f32(&mut self, v: f32) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn pose(&mut self, pose: &Pose) {
        let q = pose.orientation;
        let p = pose.position;
        for v in [q.x, q.y, q.z, q.w, p.x, p.y, p.z] {
            self.f32(v);
        }
    }
}

struct Reader<'a>(&'a [u8]);

impl Reader<'_> {
    fn take<const N: usize>(&mut self) -> Option<[u8; N]> {
        let (head, rest) = self.0.split_first_chunk::<N>()?;
        self.0 = rest;
        Some(*head)
    }
    fn u16(&mut self) -> Option<u16> {
        self.take().map(u16::from_le_bytes)
    }
    fn u32(&mut self) -> Option<u32> {
        self.take().map(u32::from_le_bytes)
    }
    fn u64(&mut self) -> Option<u64> {
        self.take().map(u64::from_le_bytes)
    }
    fn f32(&mut self) -> Option<f32> {
        self.take().map(f32::from_le_bytes)
    }
    fn pose(&mut self) -> Option<Pose> {
        let mut v = [0.0; 7];
        for value in &mut v {
            *value = self.f32()?;
        }
        let orientation = Quat::from_xyzw(v[0], v[1], v[2], v[3]);
        let position = Vec3::new(v[4], v[5], v[6]);
        (orientation.is_finite() && position.is_finite() && orientation.length() > 0.5).then(
            || Pose {
                orientation: orientation.normalize(),
                position,
            },
        )
    }
}

// The payload after a matching tag and a supported version
fn open(bytes: &[u8], tag: [u8; 4], version: u16) -> Option<Reader<'_>> {
    let mut reader = Reader(bytes);
    (reader.take::<4>()? == tag && reader.u16()? >= version).then_some(reader)
}

impl TrackingStateReport {
    pub fn encode(&self) -> Vec<u8> {
        let mut w = Writer::new(TRACKING_STATE_TAG, TRACKING_STATE_VERSION);
        w.u64(self.client_instance);
        w.u32(self.client_epoch);
        w.u32(self.epoch_reason);
        w.u32(self.flags);
        w.u32(self.epoch_age_ms);
        w.u32(self.stage_change_counter);
        w.pose(&self.stage_from_previous);
        w.0
    }

    pub fn decode(bytes: &[u8]) -> Option<Self> {
        let mut r = open(bytes, TRACKING_STATE_TAG, TRACKING_STATE_VERSION)?;
        Some(Self {
            client_instance: r.u64()?,
            client_epoch: r.u32()?,
            epoch_reason: r.u32()?,
            flags: r.u32()?,
            epoch_age_ms: r.u32()?,
            stage_change_counter: r.u32()?,
            stage_from_previous: r.pose()?,
        })
    }
}

impl CameraTrackingStamp {
    pub fn encode(&self) -> Vec<u8> {
        let mut w = Writer::new(CAMERA_STAMP_TAG, CAMERA_STAMP_VERSION);
        w.u64(self.capture_ns);
        w.u64(self.client_instance);
        w.u32(self.client_epoch);
        w.u32(self.flags);
        w.0
    }

    pub fn decode(bytes: &[u8]) -> Option<Self> {
        let mut r = open(bytes, CAMERA_STAMP_TAG, CAMERA_STAMP_VERSION)?;
        Some(Self {
            capture_ns: r.u64()?,
            client_instance: r.u64()?,
            client_epoch: r.u32()?,
            flags: r.u32()?,
        })
    }
}

impl ScreenVideoDecodeReport {
    pub fn encode(&self) -> Vec<u8> {
        let mut w = Writer::new(SCREEN_VIDEO_DECODE_TAG, SCREEN_VIDEO_DECODE_VERSION);
        w.u64(self.client_instance);
        let count = self.streams.len().min(SCREEN_VIDEO_DECODE_MAX_STREAMS);
        w.u32(count as u32);
        for status in &self.streams[..count] {
            w.u32(status.stream_id);
            w.u32(status.state);
            w.u32(status.flags);
            w.u32(status.error_count);
            w.u64(status.last_decoded_frame_index);
            w.u64(status.frames_decoded);
        }
        w.0
    }

    pub fn decode(bytes: &[u8]) -> Option<Self> {
        let mut r = open(bytes, SCREEN_VIDEO_DECODE_TAG, SCREEN_VIDEO_DECODE_VERSION)?;
        let client_instance = r.u64()?;
        let count = r.u32()? as usize;
        if count > SCREEN_VIDEO_DECODE_MAX_STREAMS {
            return None;
        }
        let mut streams = Vec::with_capacity(count);
        for _ in 0..count {
            streams.push(ScreenVideoDecodeStatus {
                stream_id: r.u32()?,
                state: r.u32()?,
                flags: r.u32()?,
                error_count: r.u32()?,
                last_decoded_frame_index: r.u64()?,
                frames_decoded: r.u64()?,
            });
        }

        Some(Self {
            client_instance,
            streams,
        })
    }
}

impl PhaseSyncReport {
    pub fn encode(&self) -> Vec<u8> {
        let mut w = Writer::new(PHASE_SYNC_TAG, PHASE_SYNC_VERSION);
        for v in [
            self.sequence,
            self.window_us,
            self.frame_interval_us,
            self.loops,
            self.frames,
            self.skipped_loops,
            self.missed_slots,
            self.late_frames,
            self.dropped_frames,
        ] {
            w.u32(v);
        }
        for v in [
            self.margin_mean_ms,
            self.margin_p10_ms,
            self.margin_p50_ms,
            self.margin_p90_ms,
        ] {
            w.f32(v);
        }
        w.0
    }

    pub fn decode(bytes: &[u8]) -> Option<Self> {
        let mut r = open(bytes, PHASE_SYNC_TAG, PHASE_SYNC_VERSION)?;
        let report = Self {
            sequence: r.u32()?,
            window_us: r.u32()?,
            frame_interval_us: r.u32()?,
            loops: r.u32()?,
            frames: r.u32()?,
            skipped_loops: r.u32()?,
            missed_slots: r.u32()?,
            late_frames: r.u32()?,
            dropped_frames: r.u32()?,
            margin_mean_ms: r.f32()?,
            margin_p10_ms: r.f32()?,
            margin_p50_ms: r.f32()?,
            margin_p90_ms: r.f32()?,
        };
        [
            report.margin_mean_ms,
            report.margin_p10_ms,
            report.margin_p50_ms,
            report.margin_p90_ms,
        ]
        .iter()
        .all(|v| v.is_finite())
        .then_some(report)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn phase_sync_report_round_trips() {
        let report = PhaseSyncReport {
            sequence: 9,
            window_us: 500_123,
            frame_interval_us: 11_111,
            loops: 45,
            frames: 44,
            skipped_loops: 0,
            missed_slots: 1,
            late_frames: 3,
            dropped_frames: 1,
            margin_mean_ms: 2.25,
            margin_p10_ms: -0.5,
            margin_p50_ms: 2.0,
            margin_p90_ms: 4.75,
        };
        let bytes = report.encode();
        assert_eq!(PhaseSyncReport::decode(&bytes), Some(report));
        let mut longer = bytes.clone();
        longer.extend_from_slice(&[7; 12]);
        assert_eq!(PhaseSyncReport::decode(&longer), Some(report));
        assert_eq!(PhaseSyncReport::decode(&bytes[..bytes.len() - 1]), None);
        // Other payloads are not phase sync reports and vice versa
        assert_eq!(TrackingStateReport::decode(&bytes), None);
        assert_eq!(ScreenVideoDecodeReport::decode(&bytes), None);
        assert_eq!(CameraTrackingStamp::decode(&bytes), None);
        let stamp = CameraTrackingStamp {
            capture_ns: 1,
            client_instance: 2,
            client_epoch: 3,
            flags: 0,
        };
        assert_eq!(PhaseSyncReport::decode(&stamp.encode()), None);
        // Non-finite margins are refused
        let mut nan = report;
        nan.margin_mean_ms = f32::NAN;
        assert_eq!(PhaseSyncReport::decode(&nan.encode()), None);
    }

    #[test]
    fn round_trips_and_rejects_foreign_payloads() {
        let report = TrackingStateReport {
            client_instance: 0x0123_4567_89ab_cdef,
            client_epoch: 7,
            epoch_reason: epoch_reason::TRACKING_REGAINED,
            flags: HEAD_POSITION_TRACKED | HEAD_SETTLED,
            epoch_age_ms: 1234,
            stage_change_counter: 2,
            stage_from_previous: Pose {
                orientation: Quat::IDENTITY,
                position: Vec3::new(0.0, 0.3686, 0.0),
            },
        };
        let bytes = report.encode();
        assert_eq!(TrackingStateReport::decode(&bytes), Some(report));
        // Trailing bytes of a newer version are ignored
        let mut longer = bytes.clone();
        longer.extend_from_slice(&[1, 2, 3, 4]);
        assert_eq!(TrackingStateReport::decode(&longer), Some(report));
        // Truncated, other tag, older version
        assert_eq!(TrackingStateReport::decode(&bytes[..bytes.len() - 1]), None);
        assert_eq!(CameraTrackingStamp::decode(&bytes), None);
        let mut old = bytes.clone();
        old[4] = 0;
        assert_eq!(TrackingStateReport::decode(&old), None);

        let stamp = CameraTrackingStamp {
            capture_ns: 495_492_938_843_390,
            client_instance: 42,
            client_epoch: 3,
            flags: HEAD_STAMP_KNOWN | HEAD_POSITION_TRACKED,
        };
        assert_eq!(CameraTrackingStamp::decode(&stamp.encode()), Some(stamp));
        assert_eq!(TrackingStateReport::decode(&stamp.encode()), None);
    }

    #[test]
    fn screen_video_decode_report_round_trips() {
        let report = ScreenVideoDecodeReport {
            client_instance: 0xfeed_beef,
            streams: vec![
                ScreenVideoDecodeStatus {
                    stream_id: 12,
                    state: screen_video_decode_state::DECODING,
                    flags: SCREEN_VIDEO_DECODE_REFERENCED,
                    error_count: 0,
                    last_decoded_frame_index: 4711,
                    frames_decoded: 300,
                },
                ScreenVideoDecodeStatus {
                    stream_id: 13,
                    state: screen_video_decode_state::FAILED,
                    ..Default::default()
                },
            ],
        };
        let bytes = report.encode();
        assert_eq!(
            ScreenVideoDecodeReport::decode(&bytes),
            Some(report.clone())
        );
        // Other payloads are not decode reports and vice versa
        assert_eq!(TrackingStateReport::decode(&bytes), None);
        assert_eq!(CameraTrackingStamp::decode(&bytes), None);
        // Trailing bytes of a newer version are ignored; truncated is refused
        let mut longer = bytes.clone();
        longer.extend_from_slice(&[9; 8]);
        assert_eq!(ScreenVideoDecodeReport::decode(&longer), Some(report));
        assert_eq!(
            ScreenVideoDecodeReport::decode(&bytes[..bytes.len() - 1]),
            None
        );
        // Empty report (no stream yet) still says "this client reports decode state"
        let empty = ScreenVideoDecodeReport {
            client_instance: 1,
            streams: vec![],
        };
        assert_eq!(
            ScreenVideoDecodeReport::decode(&empty.encode()),
            Some(empty)
        );
        // More entries than allowed are cut when encoding
        let many = ScreenVideoDecodeReport {
            client_instance: 2,
            streams: vec![ScreenVideoDecodeStatus::default(); SCREEN_VIDEO_DECODE_MAX_STREAMS + 3],
        };
        let decoded = ScreenVideoDecodeReport::decode(&many.encode()).unwrap();
        assert_eq!(decoded.streams.len(), SCREEN_VIDEO_DECODE_MAX_STREAMS);
    }
}
