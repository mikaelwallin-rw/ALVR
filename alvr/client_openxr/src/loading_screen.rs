// Loading screen while a scene app (V3, full VR) starts or shuts down, so that the user never sees
// the stream's key colour (SteamVR's magenta background) with passthrough off.
// UX spec: client_openxr/loading_assets/loading-spec.txt (v1, 2026-11-10).
//
// Who decides:
// - The overlay app (ScreenVR) sets the native quads block flag kVrNativeQuadsFlagSceneLoading from
//   the moment a scene app is starting until it presents, and while it shuts down until the front
//   end is restored (server: RealTimeConfig.ext_str "scene_loading"). Scene mode ("scene_mode",
//   passthrough None) means a scene app owns the view.
// - The client guards on its own as well (ScreenVR's signals are coarse): in scene mode, each new
//   stream frame is probed on a sparse grid for the key colour, and the stream is only shown after
//   CONTENT_FRAMES frames in a row without it. A key colour frame while the stream is shown cuts to
//   the loading screen in the same frame (it is never presented).
// - The probe is an asynchronous GPU readback (2026-11-13: the synchronous one stalled the render
//   thread every frame in scene mode, 1.5 ms mean, 5.8 ms worst). Its result is collected late in
//   the loop. LoadingGuard::needs_current_probe() says whether this frame's result can change what
//   is shown in this frame: then the loop waits for it (as late as possible, so the GPU has
//   usually finished); otherwise it is collected without waiting, or one loop later
//   (GuardInputs::key_probe_earlier). A late result only ever belongs to a frame that was not
//   shown: see needs_current_probe().
//
// Look (spec): the whole view opaque Tenstar Ocean (a large head-locked quad: a uniform colour looks
// the same head- or world-locked), and a ring world-locked 2 m ahead, 5 degrees below eye level:
// an 18 % Aluminium track with a Green arc turning clockwise at 180 deg/s whose length breathes
// between 60 and 150 degrees. Variants: "taking longer" (hourglass in the ring after 15 s) and
// "going back" (Orange ring with a return arrow, held 1.5 s, then passthrough; when the scene ends
// before its first frame, or after 60 s without one). Layers: Ocean, ring, (then the touchscreen
// quad and the hands, drawn by their own modules on top). Passthrough stays until the Ocean is
// opaque (it fades into it) and is restored under it on the way out.
// Sizes (user feedback 11-10 "a bit big"): ring outer diameter 8 deg (spec 12), logo 5.5 deg wide
// (spec 8) at the spec's relative distance below the ring.
// Live tuning (read by the debug switch thread once a second, applied while shown, no reinstall):
// screenvr_loading.txt with any of background=#RRGGBB, indicator=#RRGGBB (arc colour),
// distance_m, ring_deg (outer diameter), logo=on|off, logo_deg (width), below_deg (ring centre
// below eye level), period_s (seconds per turn); screenvr_loading_preview.txt = 1 (or normal),
// longer, back: shows the loading screen over passthrough (that variant) until removed.
// Images (read when the stream starts): screenvr_loading_indicator.png (replaces the ring: a
// static image turned at the same speed).
// Logo (spec "PLACEMENT"): the original Aluminium "TENSTAR" logo (never redrawn, recoloured or
// cropped; only downsampled), 8 deg wide, centred 0.31 m below the ring centre, fading with it. Overrides: screenvr_loading_logo.png, and in
// screenvr_loading.txt `logo=off`, `logo_width_m`.

use crate::graphics;
use alvr_common::{
    Pose,
    glam::{Quat, UVec2, Vec2, Vec3},
    info, warn,
};
use alvr_graphics::GraphicsContext;
use glow::{self as gl, HasContext};
use openxr as xr;
use std::{
    f32::consts::PI,
    fs,
    num::NonZeroU32,
    rc::Rc,
    time::{Duration, Instant},
};

const FILES_DIR: &str = "/sdcard/Android/data/alvr.client.dev/files";

// Key colour guard
// A frame is key colour when at least this fraction of its probe samples are
const KEY_FRAME_FRACTION: f32 = 0.5;
// Frames in a row without the key colour before the stream is shown
const CONTENT_FRAMES: u32 = 6;

// Timings (spec "APPEAR / DISAPPEAR", "TIMELINE")
// Passthrough or the simulator frame cross-fades with the Ocean background
const BACKGROUND_FADE: Duration = Duration::from_millis(500);
// The ring fades in once the background is half opaque, and fades out with the background
const INDICATOR_FADE: Duration = Duration::from_millis(300);
const INDICATOR_START_SCALE: f32 = 0.92;
// Once visible, shown at least this long (no flashes)
const MIN_VISIBLE: Duration = Duration::from_millis(600);
// After scene mode ends: passthrough is being recreated underneath (~0.4 s measured on 11-10)
const EXIT_HOLD: Duration = Duration::from_millis(700);
// Variants
const TAKING_LONGER_AFTER: Duration = Duration::from_secs(15);
const HOURGLASS_FADE: Duration = Duration::from_millis(500);
const HOURGLASS_PERIOD: Duration = Duration::from_secs(4);
const HOURGLASS_TURN: Duration = Duration::from_millis(600);
const GOING_BACK_AFTER: Duration = Duration::from_secs(60);
const GOING_BACK_CROSS_FADE: Duration = Duration::from_millis(300);
const GOING_BACK_HOLD: Duration = Duration::from_millis(1500);

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GuardInputs {
    pub scene_mode: bool,
    pub scene_loading: bool,
    // The stream was rendered this frame (not skipped)
    pub stream_rendered: bool,
    // Key colour fraction of this frame's new stream frame (None: no new frame, not probed, or
    // its readback not finished and not needed now)
    pub key_probe: Option<f32>,
    // Key colour fraction of earlier loops' frames whose readback finished only now (the largest)
    pub key_probe_earlier: Option<f32>,
    // DEBUG SWITCH screenvr_loading_preview.txt: show this variant over passthrough (tuning)
    pub preview: Option<Variant>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Variant {
    Normal,
    // No first frame after TAKING_LONGER_AFTER: hourglass in the ring
    TakingLonger,
    // The scene ended before its first frame, or none after GOING_BACK_AFTER: Orange ring, arrow
    GoingBack,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GuardOutput {
    // Ocean background opacity, 0 = hidden
    pub background_alpha: f32,
    // Ring opacity and scale (0.92 -> 1 while fading in)
    pub indicator_alpha: f32,
    pub indicator_scale: f32,
    // Time since the loading screen became visible (animation clock)
    pub visible_for: Duration,
    pub variant: Variant,
    // Time since the variant started
    pub variant_for: Duration,
    // Submit the stream layer (below the loading screen)
    pub show_stream: bool,
    // Keep (or bring back) passthrough although scene mode turned it off
    pub keep_passthrough: bool,
    // The loading screen just became visible: place the ring in front of the user
    pub became_visible: bool,
    // Probe the next stream frames for the key colour
    pub probe: bool,
}

fn step(value: f32, target_up: bool, dt: Duration, duration: Duration) -> f32 {
    let delta = dt.as_secs_f32() / duration.as_secs_f32();
    if target_up {
        (value + delta).min(1.0)
    } else {
        (value - delta).max(0.0)
    }
}

fn ease_out(x: f32) -> f32 {
    1.0 - (1.0 - x.clamp(0.0, 1.0)).powi(3)
}

fn ease_in_out_sine(x: f32) -> f32 {
    0.5 - 0.5 * (PI * x.clamp(0.0, 1.0)).cos()
}

#[derive(Default)]
pub struct LoadingGuard {
    background: f32,
    indicator: f32,
    visible_since: Option<Instant>,
    good_frames: u32,
    content_ok: bool,
    previous_scene_mode: bool,
    previous_scene_active: bool,
    // The previous update's scene loading flag (a key probe result collected a loop late is
    // applied with its own frame's flag)
    previous_scene_loading: bool,
    // Real content was shown since the scene (mode or loading flag) began
    scene_content_seen: bool,
    // The stream was submitted last frame
    stream_shown: bool,
    // ... opaque (scene mode: no chroma key), where a key colour frame would be seen as is
    stream_shown_opaque: bool,
    exit_hold_until: Option<Instant>,
    want_since: Option<Instant>,
    going_back_since: Option<Instant>,
    // Went back to passthrough: the loading screen stays away until content or the scene ends
    gave_up: bool,
    last_update: Option<Instant>,
    // The first frame without the key colour since the loading screen appeared was logged
    first_clean_logged: bool,
    last_logged: Option<(bool, bool, bool, Variant)>,
}

impl LoadingGuard {
    /// Whether this frame's key probe result can change what update() shows in this frame, so
    /// the render loop must wait for it. It can when the stream was shown opaque in the last
    /// frame (a key colour frame now cuts to the loading screen at once, also when scene mode or
    /// the loading flag changes in this frame), or in scene mode without the loading flag while
    /// content is shown or this frame could complete CONTENT_FRAMES clean frames (with one
    /// earlier result). It cannot (the stream stays hidden or keyed either way) when:
    /// - the stream is not rendered;
    /// - not in scene mode and not shown opaque: the chroma key / passthrough handles key colour,
    ///   the probe only counts frames;
    /// - scene loading: clean frames are reset every frame and the stream stays hidden;
    /// - no content yet and even this frame plus one earlier result cannot complete
    ///   CONTENT_FRAMES clean frames.
    ///
    /// So a key colour frame is still never presented: at a scene start, while the scene app
    /// shows content, and when it goes back to loading or ends.
    pub fn needs_current_probe(
        &self,
        scene_mode: bool,
        scene_loading: bool,
        stream_rendered: bool,
    ) -> bool {
        stream_rendered
            && (self.stream_shown_opaque
                || (scene_mode
                    && !scene_loading
                    && (self.content_ok || self.good_frames + 2 >= CONTENT_FRAMES)))
    }

    // One probe result: counts clean frames, resets on a key colour frame. Returns key colour.
    fn apply_probe(&mut self, now: Instant, fraction: f32, scene_loading: bool) -> bool {
        if fraction >= KEY_FRAME_FRACTION {
            self.content_ok = false;
            self.good_frames = 0;
            return true;
        }
        if !self.first_clean_logged
            && let Some(since) = self.visible_since
        {
            // When the scene app's frames really started, against the overlay app's flag
            info!(
                "Loading screen: first frame without the key colour {} ms after the loading \
                 screen appeared (scene loading flag {})",
                now.saturating_duration_since(since).as_millis(),
                if scene_loading { "still set" } else { "clear" }
            );
            self.first_clean_logged = true;
        }
        self.good_frames = self.good_frames.saturating_add(1);
        if self.good_frames >= CONTENT_FRAMES && !scene_loading {
            self.content_ok = true;
        }
        false
    }

    pub fn update(&mut self, now: Instant, inputs: GuardInputs) -> GuardOutput {
        let dt = self
            .last_update
            .map_or(Duration::ZERO, |last| now.saturating_duration_since(last));
        self.last_update = Some(now);

        // Results of earlier frames that arrived a loop late: applied first, as in their own
        // loop (before this loop's resets, with their loop's loading flag). A key colour one
        // also hides the stream now (that frame was not shown, see needs_current_probe(), and
        // this one may be key colour too)
        let key_earlier = inputs
            .key_probe_earlier
            .is_some_and(|fraction| self.apply_probe(now, fraction, self.previous_scene_loading));
        self.previous_scene_loading = inputs.scene_loading;

        if inputs.scene_mode && !self.previous_scene_mode {
            self.content_ok = false;
            self.good_frames = 0;
        }
        if !inputs.scene_mode
            && self.previous_scene_mode
            && (self.background > 0.0 || self.stream_shown)
        {
            self.exit_hold_until = Some(now + EXIT_HOLD);
        }
        self.previous_scene_mode = inputs.scene_mode;

        let scene_active = inputs.scene_mode || inputs.scene_loading;
        let scene_ended = self.previous_scene_active && !scene_active;
        if scene_active && !self.previous_scene_active {
            self.scene_content_seen = false;
        }
        self.previous_scene_active = scene_active;

        if inputs.scene_loading || !inputs.stream_rendered {
            self.content_ok = false;
            self.good_frames = 0;
        }
        let mut key_now = key_earlier;
        if let Some(fraction) = inputs.key_probe {
            key_now |= self.apply_probe(now, fraction, inputs.scene_loading);
        }
        if self.content_ok && inputs.scene_mode {
            self.scene_content_seen = true;
        }

        let exit_hold = self.exit_hold_until.is_some_and(|until| now < until);
        let want_raw = inputs.scene_loading || (inputs.scene_mode && !self.content_ok) || exit_hold;
        let preview = inputs.preview.is_some();
        if self.gave_up && (self.content_ok || !scene_active) {
            self.gave_up = false;
        }
        if want_raw && !self.gave_up {
            self.want_since.get_or_insert(now);
        } else if self.going_back_since.is_none() {
            self.want_since = None;
        }

        // Going back: the scene ended before its first frame, or no first frame for too long
        let visible = self.background > 0.0;
        if self.going_back_since.is_none()
            && !self.gave_up
            && ((scene_ended && !self.scene_content_seen && visible)
                || self
                    .want_since
                    .is_some_and(|since| now.saturating_duration_since(since) >= GOING_BACK_AFTER))
        {
            info!("Loading screen: going back to passthrough (no first frame)");
            self.going_back_since = Some(now);
        }
        let going_back_for = self
            .going_back_since
            .map(|since| now.saturating_duration_since(since));
        if going_back_for.is_some_and(|elapsed| elapsed >= GOING_BACK_CROSS_FADE + GOING_BACK_HOLD)
        {
            self.going_back_since = None;
            self.want_since = None;
            self.gave_up = scene_active;
        }
        let going_back = self.going_back_since.is_some();

        let want = (want_raw && !self.gave_up) || going_back || preview;

        let mut became_visible = false;
        if want {
            if self.background == 0.0 {
                self.visible_since = Some(now);
                self.first_clean_logged = false;
                became_visible = true;
            }
            if key_now && self.stream_shown_opaque {
                // Cover the key colour frame now: it is not presented
                self.background = 1.0;
            } else {
                self.background = step(self.background, true, dt, BACKGROUND_FADE);
            }
        } else {
            let min_visible_done = self
                .visible_since
                .is_none_or(|since| now.saturating_duration_since(since) >= MIN_VISIBLE);
            // Shown for at least MIN_VISIBLE, fully: a short request still gives a complete,
            // calm loading screen instead of a flash
            self.background = step(self.background, !min_visible_done, dt, BACKGROUND_FADE);
            if self.background == 0.0 {
                self.visible_since = None;
            }
        }
        let fading_out = !want
            && self
                .visible_since
                .is_some_and(|since| now.saturating_duration_since(since) >= MIN_VISIBLE);
        self.indicator = step(
            self.indicator,
            self.background >= 0.5 && !fading_out,
            dt,
            INDICATOR_FADE,
        );

        let show_stream = if inputs.scene_mode {
            self.content_ok && !key_now && self.background < 1.0
        } else {
            self.background < 1.0
        };
        self.stream_shown = show_stream;
        self.stream_shown_opaque = show_stream && inputs.scene_mode;

        let visible_for = self
            .visible_since
            .map_or(Duration::ZERO, |since| now.saturating_duration_since(since));
        let waiting_for = self
            .want_since
            .map_or(Duration::ZERO, |since| now.saturating_duration_since(since));
        let (variant, variant_for) = if let Some(variant) = inputs.preview {
            (variant, visible_for)
        } else if let Some(elapsed) = going_back_for.filter(|_| going_back) {
            (Variant::GoingBack, elapsed)
        } else if want && waiting_for >= TAKING_LONGER_AFTER && !self.scene_content_seen {
            (Variant::TakingLonger, waiting_for - TAKING_LONGER_AFTER)
        } else {
            (Variant::Normal, visible_for)
        };

        let state = (want, self.content_ok, self.gave_up, variant);
        if self.last_logged != Some(state) {
            info!(
                "Loading screen: wanted {want} (scene mode {}, scene loading {}, exit hold \
                 {exit_hold}), content {}, gave up {}, variant {variant:?}, stream shown \
                 {show_stream}",
                inputs.scene_mode, inputs.scene_loading, self.content_ok, self.gave_up
            );
            self.last_logged = Some(state);
        }

        GuardOutput {
            background_alpha: self.background,
            indicator_alpha: ease_out(self.indicator),
            indicator_scale: INDICATOR_START_SCALE
                + (1.0 - INDICATOR_START_SCALE) * ease_out(self.indicator),
            visible_for,
            variant,
            variant_for,
            show_stream,
            keep_passthrough: (want && self.background < 1.0)
                || going_back
                || self.gave_up
                || (preview && !inputs.scene_mode),
            became_visible,
            probe: inputs.scene_mode || inputs.scene_loading || self.background > 0.0,
        }
    }
}

// ---- Look ----

const TEXTURE: u32 = 512;
// Spec texture geometry at 512 px: centreline radius 201 px, stroke 36 px
const RING_RADIUS: f32 = 201.0;
const RING_STROKE: f32 = 36.0;
const OCEAN: [u8; 3] = [0x40, 0x50, 0x5E];
const GREEN: [u8; 3] = [0x93, 0xCB, 0x4D];
const ALUMINIUM: [u8; 3] = [0xE9, 0xE9, 0xE9];
const TRACK_ALPHA: f32 = 46.0 / 255.0;
const ARC_DEG_PER_S: f32 = 180.0;
// Hourglass crop of the "taking longer" asset (centre square, inside the ring's inner edge)
const ICON_CROP: u32 = 160;
const BACKGROUND_PIXELS: u32 = 4;
const RING_UPDATE: Duration = Duration::from_millis(33);
// The background quad, head-locked 1 m ahead, this wide and high: covers ±76° around the gaze
const BACKGROUND_SIZE_M: f32 = 8.0;

// Original file name: "Tenstar_Logo_Alu-2023 (4).png" (D:\wkspaces\ScreenVR\Logos), 3048x844,
// Aluminium #ECECEC on transparent, cropped tight (no margins)
const LOGO_PNG: &[u8] = include_bytes!("../loading_assets/tenstar-logo-alu-2023.png");
// Logo texture width (area filter): 8 deg are ~165 display pixels, so ~2x
const LOGO_TEXTURE_WIDTH: u32 = 320;
// Spec: logo centre 0.31 m below the centre of a ring of 0.41 m outer diameter
const LOGO_BELOW_RING_RATIO: f32 = 0.31 / 0.41;

const LONGER_PNG: &[u8] = include_bytes!("../loading_assets/loading-indicator-longer-512.png");
const GOING_BACK_PNG: &[u8] =
    include_bytes!("../loading_assets/loading-indicator-going-back-512.png");

/// Arc length (degrees) at time t: L(t) = 105 - 45 cos(pi t), 60..150, 2 s period
pub fn arc_length_deg(t: f32) -> f32 {
    105.0 - 45.0 * (PI * t).cos()
}

struct Look {
    background: [u8; 3],
    arc: [u8; 3],
    distance_m: f32,
    // Ring outer diameter, degrees
    ring_deg: f32,
    // Seconds per turn
    period_s: f32,
    // Ring centre below eye level, degrees
    below_deg: f32,
    logo: bool,
    // Logo width, degrees
    logo_deg: f32,
}

impl Default for Look {
    fn default() -> Self {
        Self {
            background: OCEAN,
            arc: GREEN,
            distance_m: 2.0,
            ring_deg: 8.0,
            period_s: 360.0 / ARC_DEG_PER_S,
            below_deg: 5.0,
            logo: true,
            logo_deg: 5.5,
        }
    }
}

// Ring outer diameter as a fraction of the ring quad's side (texture geometry)
const RING_OUTER_FRACTION: f32 = (2.0 * RING_RADIUS + RING_STROKE) / TEXTURE as f32;

fn angular_size_m(degrees: f32, distance_m: f32) -> f32 {
    2.0 * distance_m * (degrees.to_radians() / 2.0).tan()
}

impl Look {
    /// The defaults with the keys of a screenvr_loading.txt applied
    fn parse(text: &str) -> Self {
        let mut look = Look::default();
        for line in text.lines() {
            let Some((key, value)) = line.split_once('=') else {
                continue;
            };
            let value = value.trim();
            let colour = || {
                let hex = value.trim_start_matches('#');
                (hex.len() == 6 && hex.is_ascii())
                    .then(|| [0, 2, 4].map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap_or(0)))
            };
            let number = |current: f32, min: f32, max: f32| {
                value
                    .parse::<f32>()
                    .ok()
                    .filter(|v| (min..=max).contains(v))
                    .unwrap_or(current)
            };
            match key.trim() {
                "background" => look.background = colour().unwrap_or(look.background),
                "indicator" => look.arc = colour().unwrap_or(look.arc),
                "distance_m" => look.distance_m = number(look.distance_m, 0.5, 10.0),
                "ring_deg" => look.ring_deg = number(look.ring_deg, 1.0, 40.0),
                "period_s" => look.period_s = number(look.period_s, 0.2, 20.0),
                "below_deg" => look.below_deg = number(look.below_deg, -30.0, 45.0),
                "logo" => look.logo = !matches!(value, "off" | "0" | "false" | "no"),
                "logo_deg" => look.logo_deg = number(look.logo_deg, 1.0, 40.0),
                _ => (),
            }
        }
        look
    }

    /// Side of the ring quad, metres
    fn ring_quad_m(&self) -> f32 {
        angular_size_m(self.ring_deg, self.distance_m) / RING_OUTER_FRACTION
    }

    fn logo_width_m(&self) -> f32 {
        angular_size_m(self.logo_deg, self.distance_m)
    }

    /// Logo centre below the ring centre, metres: the spec's 0.31 m for a 0.41 m ring, scaled
    fn logo_below_ring_m(&self) -> f32 {
        angular_size_m(self.ring_deg, self.distance_m) * LOGO_BELOW_RING_RATIO
    }
}

fn read_image(name: &str) -> Option<(UVec2, Vec<u8>)> {
    let path = format!("{FILES_DIR}/{name}");
    let image = fs::read(&path).ok().and_then(|bytes| decode_png(&bytes));
    if image.is_some() {
        info!("Loading screen: {path} used");
    }
    image
}

pub(crate) fn decode_png(bytes: &[u8]) -> Option<(UVec2, Vec<u8>)> {
    let mut decoder = png::Decoder::new(std::io::Cursor::new(bytes));
    decoder.set_transformations(png::Transformations::normalize_to_color8());
    let mut reader = decoder.read_info().ok()?;
    let mut pixels = vec![0; reader.output_buffer_size()];
    let info = reader.next_frame(&mut pixels).ok()?;
    pixels.truncate(info.buffer_size());
    let rgba = match info.color_type {
        png::ColorType::Rgba => pixels,
        png::ColorType::Rgb => pixels
            .chunks_exact(3)
            .flat_map(|p| [p[0], p[1], p[2], 255])
            .collect(),
        png::ColorType::GrayscaleAlpha => pixels
            .chunks_exact(2)
            .flat_map(|p| [p[0], p[0], p[0], p[1]])
            .collect(),
        png::ColorType::Grayscale => pixels.iter().flat_map(|&g| [g, g, g, 255]).collect(),
        png::ColorType::Indexed => return None,
    };
    Some((UVec2::new(info.width, info.height), rgba))
}

// Nearest-neighbour resample (assets are exported at the target size; this only guards odd sizes)
fn resample(pixels: &[u8], from: UVec2, to: UVec2) -> Vec<u8> {
    if from == to {
        return pixels.to_vec();
    }
    let mut out = vec![0; (to.x * to.y * 4) as usize];
    for y in 0..to.y {
        for x in 0..to.x {
            let sx = (x * from.x / to.x).min(from.x - 1);
            let sy = (y * from.y / to.y).min(from.y - 1);
            let s = ((sy * from.x + sx) * 4) as usize;
            let d = ((y * to.x + x) * 4) as usize;
            out[d..d + 4].copy_from_slice(&pixels[s..s + 4]);
        }
    }
    out
}

/// Area-filter downsample of unpremultiplied RGBA8 (alpha-weighted, so transparent pixels do not
/// darken the edges), to `width` keeping the aspect ratio. Returns the new size and pixels.
fn downsample(pixels: &[u8], size: UVec2, width: u32) -> (UVec2, Vec<u8>) {
    if width >= size.x {
        return (size, pixels.to_vec());
    }
    let height = ((size.y as u64 * width as u64 + size.x as u64 / 2) / size.x as u64).max(1) as u32;
    let to = UVec2::new(width, height);
    let mut out = vec![0; (width * height * 4) as usize];
    for y in 0..height {
        let y0 = y * size.y / height;
        let y1 = ((y + 1) * size.y / height).max(y0 + 1);
        for x in 0..width {
            let x0 = x * size.x / width;
            let x1 = ((x + 1) * size.x / width).max(x0 + 1);
            let mut sum = [0.0_f64; 4];
            for sy in y0..y1 {
                for sx in x0..x1 {
                    let i = ((sy * size.x + sx) * 4) as usize;
                    let alpha = f64::from(pixels[i + 3]);
                    for c in 0..3 {
                        sum[c] += f64::from(pixels[i + c]) * alpha;
                    }
                    sum[3] += alpha;
                }
            }
            let count = f64::from((y1 - y0) * (x1 - x0));
            let o = ((y * width + x) * 4) as usize;
            if sum[3] > 0.0 {
                for c in 0..3 {
                    out[o + c] = (sum[c] / sum[3]).round() as u8;
                }
            }
            out[o + 3] = (sum[3] / count).round() as u8;
        }
    }
    (to, out)
}

fn crop(pixels: &[u8], width: u32, origin: UVec2, size: u32) -> Vec<u8> {
    let mut out = Vec::with_capacity((size * size * 4) as usize);
    for y in origin.y..origin.y + size {
        let start = ((y * width + origin.x) * 4) as usize;
        out.extend_from_slice(&pixels[start..start + (size * 4) as usize]);
    }
    out
}

/// sRGB-encoded brand colour as the bytes to upload: unchanged for an sRGB swapchain (the
/// compositor decodes it, so the display shows the hex value), linearized for a UNORM one
pub fn swapchain_bytes(colour: [u8; 3], srgb: bool) -> [u8; 3] {
    if srgb {
        colour
    } else {
        colour.map(srgb_to_linear_u8)
    }
}

pub(crate) fn srgb_to_linear_u8(value: u8) -> u8 {
    let encoded = f32::from(value) / 255.0;
    let linear = if encoded <= 0.04045 {
        encoded / 12.92
    } else {
        ((encoded + 0.055) / 1.055).powf(2.4)
    };
    (linear * 255.0).round() as u8
}

// Pixels of the ring's band, precomputed: (pixel index, position relative to the centre in pixels
// with y up, clockwise angle from 12 o'clock in degrees, radial coverage of the stroke)
struct RingBand {
    pixels: Vec<(usize, Vec2, f32, f32)>,
}

impl RingBand {
    fn new() -> Self {
        let centre = TEXTURE as f32 / 2.0;
        let mut pixels = vec![];
        for y in 0..TEXTURE {
            for x in 0..TEXTURE {
                let p = Vec2::new(x as f32 + 0.5 - centre, centre - (y as f32 + 0.5));
                let radial =
                    (RING_STROKE / 2.0 + 0.5 - (p.length() - RING_RADIUS).abs()).clamp(0.0, 1.0);
                if radial > 0.0 {
                    // Clockwise from 12 o'clock: atan2(x, y) with y up
                    let angle = p.x.atan2(p.y).to_degrees();
                    pixels.push(((y * TEXTURE + x) as usize, p, angle, radial));
                }
            }
        }
        Self { pixels }
    }

    /// Track + arc (RGBA8 unpremultiplied, sRGB-encoded, rows top-down). The arc is `length_deg`
    /// long with round caps, centred `centre_deg` clockwise from 12 o'clock.
    fn draw(&self, arc_colour: [u8; 3], length_deg: f32, centre_deg: f32, out: &mut [u8]) {
        out.fill(0);
        let half = length_deg / 2.0;
        let ends = [centre_deg - half, centre_deg + half].map(|deg| {
            let rad = deg.to_radians();
            Vec2::new(rad.sin(), rad.cos()) * RING_RADIUS
        });
        for &(index, p, angle, radial) in &self.pixels {
            let from_centre = (angle - centre_deg + 540.0).rem_euclid(360.0) - 180.0;
            let mut arc = if from_centre.abs() <= half {
                radial
            } else {
                0.0
            };
            for end in ends {
                arc = arc.max((RING_STROKE / 2.0 + 0.5 - p.distance(end)).clamp(0.0, 1.0));
            }
            let track = radial * TRACK_ALPHA;
            let alpha = arc + track * (1.0 - arc);
            let i = index * 4;
            for c in 0..3 {
                out[i + c] = ((f32::from(arc_colour[c]) * arc
                    + f32::from(ALUMINIUM[c]) * track * (1.0 - arc))
                    / alpha)
                    .round() as u8;
            }
            out[i + 3] = (alpha * 255.0).round() as u8;
        }
    }
}

pub(crate) struct QuadSwapchain {
    swapchain: xr::Swapchain<xr::OpenGlEs>,
    images: Vec<u32>,
    size: UVec2,
    // At least one image released: the layer may be submitted
    pub(crate) filled: bool,
}

impl QuadSwapchain {
    pub(crate) fn new(
        xr_session: &xr::Session<xr::OpenGlEs>,
        size: UVec2,
        format: u32,
    ) -> Option<Self> {
        let swapchain = xr_session
            .create_swapchain(&xr::SwapchainCreateInfo {
                create_flags: xr::SwapchainCreateFlags::EMPTY,
                usage_flags: xr::SwapchainUsageFlags::COLOR_ATTACHMENT
                    | xr::SwapchainUsageFlags::SAMPLED,
                format,
                sample_count: 1,
                width: size.x,
                height: size.y,
                face_count: 1,
                array_size: 1,
                mip_count: 1,
            })
            .map_err(|e| warn!("Loading screen: create_swapchain: {e}"))
            .ok()?;
        let images = swapchain.enumerate_images().ok()?;
        Some(Self {
            swapchain,
            images,
            size,
            filled: false,
        })
    }

    // RGBA8 rows top-down
    pub(crate) fn upload(&mut self, gl_context: &gl::Context, pixels: &[u8]) {
        let Ok(index) = self.swapchain.acquire_image() else {
            return;
        };
        if self.swapchain.wait_image(xr::Duration::INFINITE).is_err() {
            return;
        }
        // OpenGL images have their origin at the bottom left: rows bottom-up
        let row_bytes = self.size.x as usize * 4;
        let flipped = pixels
            .chunks_exact(row_bytes)
            .rev()
            .flatten()
            .copied()
            .collect::<Vec<_>>();
        if let Some(texture) = self
            .images
            .get(index as usize)
            .copied()
            .and_then(NonZeroU32::new)
        {
            let guard = crate::gl_state::GlStateGuard::save(gl_context);
            unsafe {
                gl_context.bind_texture(gl::TEXTURE_2D, Some(gl::NativeTexture(texture)));
                gl_context.pixel_store_i32(gl::UNPACK_ALIGNMENT, 1);
                gl_context.pixel_store_i32(gl::UNPACK_ROW_LENGTH, 0);
                gl_context.pixel_store_i32(gl::UNPACK_SKIP_ROWS, 0);
                gl_context.pixel_store_i32(gl::UNPACK_SKIP_PIXELS, 0);
                gl_context.bind_buffer(gl::PIXEL_UNPACK_BUFFER, None);
                gl_context.tex_sub_image_2d(
                    gl::TEXTURE_2D,
                    0,
                    0,
                    0,
                    self.size.x as _,
                    self.size.y as _,
                    gl::RGBA,
                    gl::UNSIGNED_BYTE,
                    gl::PixelUnpackData::Slice(Some(&flipped)),
                );
                gl_context.bind_texture(gl::TEXTURE_2D, None);
                gl_context.flush();
            }
            drop(guard);
        }
        if self.swapchain.release_image().is_ok() {
            self.filled = true;
        }
    }

    pub(crate) fn layer<'a>(
        &'a self,
        space: &'a xr::Space,
        pose: Pose,
        width_m: f32,
    ) -> xr::CompositionLayerQuad<'a, xr::OpenGlEs> {
        self.layer_rect(space, pose, width_m, UVec2::ZERO, self.size)
    }

    /// A layer showing only the `offset`/`size` rectangle of the image (atlas), rows from the top
    pub(crate) fn layer_rect<'a>(
        &'a self,
        space: &'a xr::Space,
        pose: Pose,
        width_m: f32,
        offset: UVec2,
        size: UVec2,
    ) -> xr::CompositionLayerQuad<'a, xr::OpenGlEs> {
        let height_m = width_m * size.y as f32 / size.x as f32;
        // Rows were uploaded bottom-up (GL origin): the rectangle's first row in GL coordinates
        let gl_y = self.size.y - offset.y - size.y;
        xr::CompositionLayerQuad::new()
            .layer_flags(
                xr::CompositionLayerFlags::BLEND_TEXTURE_SOURCE_ALPHA
                    | xr::CompositionLayerFlags::UNPREMULTIPLIED_ALPHA,
            )
            .space(space)
            .eye_visibility(xr::EyeVisibility::BOTH)
            .sub_image(
                xr::SwapchainSubImage::new()
                    .swapchain(&self.swapchain)
                    .image_array_index(0)
                    .image_rect(xr::Rect2Di {
                        offset: xr::Offset2Di {
                            x: offset.x as _,
                            y: gl_y as _,
                        },
                        extent: xr::Extent2Di {
                            width: size.x as _,
                            height: size.y as _,
                        },
                    }),
            )
            .pose(crate::to_xr_pose(pose))
            .size(xr::Extent2Df {
                width: width_m,
                height: height_m,
            })
    }
}

// Unpremultiplied RGBA8 buffer: alpha scaled, colours converted for the swapchain
fn finish(pixels: &mut [u8], alpha: f32, srgb: bool) {
    for pixel in pixels.chunks_exact_mut(4) {
        if !srgb {
            for channel in &mut pixel[..3] {
                *channel = srgb_to_linear_u8(*channel);
            }
        }
        pixel[3] = (f32::from(pixel[3]) * alpha).round() as u8;
    }
}

// What the ring texture currently shows, quantized: re-uploaded only when it changes
#[derive(Clone, Copy, PartialEq)]
struct RingKey {
    variant: Variant,
    length_q: i32,
    centre_q: i32,
    alpha_q: u8,
    cross_fade_q: u8,
}

pub struct LoadingScreen {
    gfx_ctx: Rc<GraphicsContext>,
    look: Look,
    // screenvr_loading.txt as last applied (debug switch thread)
    look_text: String,
    // screenvr_loading_indicator.png, resampled to the ring texture
    ring_image: Option<Vec<u8>>,
    srgb: bool,
    band: RingBand,
    going_back_image: Vec<u8>,
    hourglass_image: Vec<u8>,
    background: Option<QuadSwapchain>,
    ring: Option<QuadSwapchain>,
    icon: Option<QuadSwapchain>,
    scratch: Vec<u8>,
    background_alpha_q: Option<u8>,
    ring_key: Option<RingKey>,
    icon_alpha_q: Option<u8>,
    logo: Option<(QuadSwapchain, Vec<u8>)>,
    logo_alpha_q: Option<u8>,
    last_ring_upload: Option<Instant>,
    // World-locked placement, set when the loading screen becomes visible
    base_pose: Pose,
    // Ring turn (radians, clockwise as seen) applied to the quad pose; frozen while going back
    ring_turn: f32,
    frozen: Option<(f32, f32)>,
    icon_turn: f32,
    output: Option<GuardOutput>,
}

impl LoadingScreen {
    pub fn new(xr_session: &xr::Session<xr::OpenGlEs>, gfx_ctx: &Rc<GraphicsContext>) -> Self {
        let (format, srgb) = graphics::srgb_swapchain_format(xr_session);
        gfx_ctx.make_current();
        let look_text = crate::debug_switches::get().loading_look;
        let look = Look::parse(&look_text);
        let ring_image = read_image("screenvr_loading_indicator.png")
            .map(|(size, pixels)| resample(&pixels, size, UVec2::splat(TEXTURE)));
        let texture = UVec2::splat(TEXTURE);
        let asset = |bytes: &[u8], name: &str| {
            decode_png(bytes)
                .map(|(size, pixels)| resample(&pixels, size, texture))
                .unwrap_or_else(|| {
                    warn!("Loading screen: embedded {name} unreadable");
                    vec![0; (TEXTURE * TEXTURE * 4) as usize]
                })
        };
        let longer = asset(LONGER_PNG, "hourglass");
        let logo = read_image("screenvr_loading_logo.png")
            .or_else(|| {
                let decoded = decode_png(LOGO_PNG);
                if decoded.is_none() {
                    warn!("Loading screen: embedded logo unreadable");
                }
                decoded
            })
            .and_then(|(size, pixels)| {
                let (size, pixels) = downsample(&pixels, size, LOGO_TEXTURE_WIDTH);
                QuadSwapchain::new(xr_session, size, format).map(|swapchain| (swapchain, pixels))
            });
        let origin = UVec2::splat((TEXTURE - ICON_CROP) / 2);
        Self {
            gfx_ctx: Rc::clone(gfx_ctx),
            background: QuadSwapchain::new(xr_session, UVec2::splat(BACKGROUND_PIXELS), format),
            ring: QuadSwapchain::new(xr_session, texture, format),
            icon: QuadSwapchain::new(xr_session, UVec2::splat(ICON_CROP), format),
            look,
            look_text,
            ring_image,
            srgb,
            band: RingBand::new(),
            going_back_image: asset(GOING_BACK_PNG, "going back"),
            hourglass_image: crop(&longer, TEXTURE, origin, ICON_CROP),
            scratch: vec![0; (TEXTURE * TEXTURE * 4) as usize],
            background_alpha_q: None,
            ring_key: None,
            icon_alpha_q: None,
            logo,
            logo_alpha_q: None,
            last_ring_upload: None,
            base_pose: Pose::IDENTITY,
            ring_turn: 0.0,
            frozen: None,
            icon_turn: 0.0,
            output: None,
        }
    }

    /// Once per frame. `head` (STAGE) places the ring when the loading screen becomes visible.
    pub fn update(&mut self, output: &GuardOutput, head: Option<Pose>) {
        self.output = Some(*output);

        // Live tuning: screenvr_loading.txt changed
        let look_text = crate::debug_switches::get().loading_look;
        let mut replace = false;
        if look_text != self.look_text {
            self.look = Look::parse(&look_text);
            info!(
                "Loading screen: look updated: ring {} deg, logo {} {} deg, {} m, {} deg below eye \
                 level",
                self.look.ring_deg,
                if self.look.logo { "on" } else { "off" },
                self.look.logo_deg,
                self.look.distance_m,
                self.look.below_deg
            );
            self.look_text = look_text;
            self.ring_key = None;
            self.background_alpha_q = None;
            replace = true;
        }
        if output.background_alpha <= 0.0 {
            self.frozen = None;
            return;
        }

        if (output.became_visible || self.ring_key.is_none() || replace)
            && let Some(head) = head
        {
            // Ahead of the user, level (head yaw only), below_deg under eye level, facing them
            let forward = head.orientation * -Vec3::Z;
            let yaw = f32::atan2(-forward.x, -forward.z);
            let orientation = Quat::from_rotation_y(yaw);
            let drop = self.look.distance_m * self.look.below_deg.to_radians().tan();
            self.base_pose = Pose {
                orientation,
                position: head.position
                    + orientation * Vec3::new(0.0, -drop, -self.look.distance_m),
            };
        }

        let gl_context = &self.gfx_ctx.gl_context;
        self.gfx_ctx.make_current();

        // Background: re-uploaded only when its opacity changes
        let background_q = (output.background_alpha * 255.0).round() as u8;
        if self.background_alpha_q != Some(background_q)
            && let Some(background) = &mut self.background
        {
            let colour = swapchain_bytes(self.look.background, self.srgb);
            let pixel = [colour[0], colour[1], colour[2], background_q];
            background.upload(
                gl_context,
                &pixel.repeat((BACKGROUND_PIXELS * BACKGROUND_PIXELS) as usize),
            );
            self.background_alpha_q = Some(background_q);
        }

        // Ring: turned by the quad pose (smooth at display rate); the texture carries the arc
        // length, and the going-back cross-fade
        let t = output.visible_for.as_secs_f32();
        let turn_deg = t * 360.0 / self.look.period_s;
        let length = arc_length_deg(t);
        let (turn_deg, length) = if output.variant == Variant::GoingBack {
            *self.frozen.get_or_insert((turn_deg, length))
        } else {
            self.frozen = None;
            (turn_deg, length)
        };
        let cross_fade = if output.variant == Variant::GoingBack {
            (output.variant_for.as_secs_f32() / GOING_BACK_CROSS_FADE.as_secs_f32()).clamp(0.0, 1.0)
        } else {
            0.0
        };
        // Going back: the quad stops turning (upright arrow), the frozen arc is drawn turned instead
        let (quad_turn, arc_centre) = if output.variant == Variant::GoingBack {
            (0.0, turn_deg)
        } else {
            (turn_deg, 0.0)
        };
        self.ring_turn = quad_turn.to_radians();

        let key = RingKey {
            variant: output.variant,
            length_q: (length * 4.0).round() as i32,
            centre_q: (arc_centre * 4.0).round() as i32,
            alpha_q: (output.indicator_alpha * 255.0).round() as u8,
            cross_fade_q: (cross_fade * 255.0).round() as u8,
        };
        // At most every RING_UPDATE (~30 Hz): the turn is the quad pose (display rate), only the
        // arc length (up to 141 deg/s, i.e. each end moves <= 2.4 deg per update) and fades are in
        // the texture. Cost per update: ~48k band pixels drawn + one 1 MiB upload.
        let due = self
            .last_ring_upload
            .is_none_or(|last| last.elapsed() >= RING_UPDATE);
        if self.ring_key != Some(key)
            && due
            && let Some(ring) = &mut self.ring
        {
            if let Some(image) = &self.ring_image {
                self.scratch.copy_from_slice(image);
            } else {
                self.band
                    .draw(self.look.arc, length, arc_centre, &mut self.scratch);
            }
            if cross_fade > 0.0 {
                for (pixel, target) in self
                    .scratch
                    .chunks_exact_mut(4)
                    .zip(self.going_back_image.chunks_exact(4))
                {
                    blend_unpremultiplied(pixel, target, cross_fade);
                }
            }
            finish(&mut self.scratch, output.indicator_alpha, self.srgb);
            ring.upload(gl_context, &self.scratch);
            self.ring_key = Some(key);
            self.last_ring_upload = Some(Instant::now());
        }

        // Hourglass (taking longer): fades in, turns over every HOURGLASS_PERIOD
        let icon_alpha = if output.variant == Variant::TakingLonger {
            let since = output.variant_for.as_secs_f32();
            let cycle = since % HOURGLASS_PERIOD.as_secs_f32();
            let turns = (since / HOURGLASS_PERIOD.as_secs_f32()).floor();
            let turn_start = HOURGLASS_PERIOD.as_secs_f32() - HOURGLASS_TURN.as_secs_f32();
            let progress = ease_in_out_sine((cycle - turn_start) / HOURGLASS_TURN.as_secs_f32());
            self.icon_turn = (turns + progress) * PI;
            (since / HOURGLASS_FADE.as_secs_f32()).clamp(0.0, 1.0) * output.indicator_alpha
        } else {
            0.0
        };
        let icon_q = (icon_alpha * 255.0).round() as u8;
        if self.icon_alpha_q != Some(icon_q)
            && let Some(icon) = &mut self.icon
        {
            let mut pixels = self.hourglass_image.clone();
            finish(&mut pixels, icon_alpha, self.srgb);
            icon.upload(gl_context, &pixels);
            self.icon_alpha_q = Some(icon_q);
        }

        // Logo: fades with the ring
        let logo_q = (output.indicator_alpha * 255.0).round() as u8;
        if self.logo_alpha_q != Some(logo_q)
            && let Some((logo, image)) = &mut self.logo
        {
            let mut pixels = image.clone();
            finish(&mut pixels, output.indicator_alpha, self.srgb);
            logo.upload(gl_context, &pixels);
            self.logo_alpha_q = Some(logo_q);
        }
    }

    /// Background (head-locked, in `view_space`), ring, logo and hourglass (world-locked, in
    /// `stage_space`), in that order; empty while hidden
    pub fn build<'a>(
        &'a self,
        view_space: &'a xr::Space,
        stage_space: &'a xr::Space,
    ) -> Vec<xr::CompositionLayerQuad<'a, xr::OpenGlEs>> {
        let Some(output) = self.output.filter(|output| output.background_alpha > 0.0) else {
            return Vec::new();
        };
        let mut layers = Vec::with_capacity(4);
        if let Some(background) = self.background.as_ref().filter(|s| s.filled) {
            layers.push(background.layer(
                view_space,
                Pose {
                    orientation: Quat::IDENTITY,
                    position: Vec3::new(0.0, 0.0, -1.0),
                },
                BACKGROUND_SIZE_M,
            ));
        }
        let size = self.look.ring_quad_m() * output.indicator_scale;
        if output.indicator_alpha > 0.0
            && let Some(ring) = self.ring.as_ref().filter(|s| s.filled)
        {
            // Clockwise as seen by the user (the quad faces them along +Z)
            let pose = Pose {
                orientation: self.base_pose.orientation * Quat::from_rotation_z(-self.ring_turn),
                position: self.base_pose.position,
            };
            layers.push(ring.layer(stage_space, pose, size));
        }
        if output.indicator_alpha > 0.0
            && self.look.logo
            && let Some((logo, _)) = self.logo.as_ref().filter(|(s, _)| s.filled)
        {
            // Same depth as the ring, centred below it, level (no turn), scaled with it
            let drop = self.look.logo_below_ring_m();
            let pose = Pose {
                orientation: self.base_pose.orientation,
                position: self.base_pose.position
                    + self.base_pose.orientation * Vec3::new(0.0, -drop, 0.0),
            };
            layers.push(logo.layer(
                stage_space,
                pose,
                self.look.logo_width_m() * output.indicator_scale,
            ));
        }
        if output.variant == Variant::TakingLonger
            && self.icon_alpha_q.is_some_and(|alpha| alpha > 0)
            && let Some(icon) = self.icon.as_ref().filter(|s| s.filled)
        {
            let pose = Pose {
                orientation: self.base_pose.orientation * Quat::from_rotation_z(-self.icon_turn),
                // A hair in front of the ring
                position: self.base_pose.position + self.base_pose.orientation * Vec3::Z * 0.002,
            };
            layers.push(icon.layer(stage_space, pose, size * ICON_CROP as f32 / TEXTURE as f32));
        }
        layers
    }
}

// a := a * (1 - k) + b * k, on unpremultiplied RGBA8
fn blend_unpremultiplied(a: &mut [u8], b: &[u8], k: f32) {
    let alpha_a = f32::from(a[3]) / 255.0 * (1.0 - k);
    let alpha_b = f32::from(b[3]) / 255.0 * k;
    let alpha = alpha_a + alpha_b;
    if alpha <= 0.0 {
        a.fill(0);
        return;
    }
    for c in 0..3 {
        a[c] = ((f32::from(a[c]) * alpha_a + f32::from(b[c]) * alpha_b) / alpha).round() as u8;
    }
    a[3] = (alpha * 255.0).round() as u8;
}

#[cfg(test)]
mod tests {
    use super::*;

    const FRAME: Duration = Duration::from_millis(11);

    struct Sim {
        guard: LoadingGuard,
        now: Instant,
    }

    impl Sim {
        fn new() -> Self {
            Self {
                guard: LoadingGuard::default(),
                now: Instant::now(),
            }
        }

        fn step(&mut self, inputs: GuardInputs) -> GuardOutput {
            self.now += FRAME;
            self.guard.update(self.now, inputs)
        }

        fn run(&mut self, duration: Duration, inputs: GuardInputs) -> GuardOutput {
            let mut output = self.step(inputs);
            let mut elapsed = FRAME;
            while elapsed < duration {
                output = self.step(inputs);
                elapsed += FRAME;
            }
            output
        }
    }

    fn front_end() -> GuardInputs {
        GuardInputs {
            scene_mode: false,
            scene_loading: false,
            stream_rendered: true,
            key_probe: None,
            key_probe_earlier: None,
            preview: None,
        }
    }

    fn scene(key: Option<f32>) -> GuardInputs {
        GuardInputs {
            scene_mode: true,
            scene_loading: false,
            stream_rendered: true,
            key_probe: key,
            key_probe_earlier: None,
            preview: None,
        }
    }

    // (scene mode, scene loading, key fraction of the frame) per frame: front end, a scene start
    // with magenta while loading and after the flag cleared, content, the scene app stopping
    // (magenta before the overlay app's flag), back to loading, content again, scene end
    fn scene_script() -> Vec<(bool, bool, f32)> {
        let mut script = vec![];
        let mut add = |n: usize, mode: bool, loading: bool, key: f32| {
            script.extend(std::iter::repeat_n((mode, loading, key), n));
        };
        add(20, false, false, 0.0);
        add(50, true, true, 0.97);
        add(10, true, false, 0.97);
        add(3, true, false, 0.0);
        add(1, true, false, 0.9);
        add(300, true, false, 0.0);
        add(5, true, false, 0.97);
        add(30, true, true, 0.97);
        add(20, true, true, 0.0);
        add(200, true, false, 0.0);
        add(2, true, false, 0.97);
        add(100, true, false, 0.0);
        add(60, false, true, 0.97);
        add(100, false, false, 0.0);
        script
    }

    #[test]
    fn the_loop_waits_for_the_probe_only_when_it_can_change_this_frame() {
        let mut sim = Sim::new();
        // Front end: never
        sim.run(Duration::from_millis(100), front_end());
        assert!(!sim.guard.needs_current_probe(false, false, true));
        // Scene loading with magenta: never (the stream stays hidden)
        let loading = GuardInputs {
            scene_loading: true,
            ..scene(Some(0.97))
        };
        sim.run(Duration::from_millis(500), loading);
        assert!(!sim.guard.needs_current_probe(true, true, true));
        // Flag cleared, first clean frames: not until the next result could show the stream
        let mut needs = vec![];
        for _ in 0..CONTENT_FRAMES {
            needs.push(sim.guard.needs_current_probe(true, false, true));
            sim.step(scene(Some(0.0)));
        }
        assert_eq!(needs, [false, false, false, false, true, true]);
        // Content shown: every frame, also when the loading flag or scene mode changes now
        assert!(
            sim.run(Duration::from_secs(1), scene(Some(0.0)))
                .show_stream
        );
        assert!(sim.guard.needs_current_probe(true, false, true));
        assert!(sim.guard.needs_current_probe(true, true, true));
        assert!(sim.guard.needs_current_probe(false, true, true));
        // Not rendered: never
        assert!(!sim.guard.needs_current_probe(true, false, false));
    }

    #[test]
    fn asynchronous_probe_results_show_exactly_what_synchronous_ones_did() {
        // Synchronous (the old readback): every probed frame's result in its own loop.
        // Asynchronous: the result is ready in its own loop only sometimes; the loop waits for
        // it when needs_current_probe() says so, otherwise it arrives one loop later.
        for ready_pattern in [0_usize, 1, 2, 3] {
            let mut sync = Sim::new();
            let mut lagged = Sim::new();
            let mut sync_probe = false;
            let mut lagged_probe = false;
            let mut pending = None::<f32>;
            let mut waits = 0;
            for (i, (scene_mode, scene_loading, key)) in scene_script().into_iter().enumerate() {
                let base = GuardInputs {
                    scene_mode,
                    scene_loading,
                    stream_rendered: true,
                    key_probe: None,
                    key_probe_earlier: None,
                    preview: None,
                };
                let sync_output = sync.step(GuardInputs {
                    key_probe: sync_probe.then_some(key),
                    ..base
                });
                sync_probe = sync_output.probe;

                let earlier = pending.take();
                let ready_now = match ready_pattern {
                    0 => false,
                    1 => true,
                    2 => i % 2 == 0,
                    _ => i % 5 != 0,
                };
                let current = if lagged_probe {
                    let needed = lagged
                        .guard
                        .needs_current_probe(scene_mode, scene_loading, true);
                    if needed && !ready_now {
                        waits += 1;
                    }
                    if needed || ready_now {
                        Some(key)
                    } else {
                        pending = Some(key);
                        None
                    }
                } else {
                    None
                };
                let lagged_output = lagged.step(GuardInputs {
                    key_probe: current,
                    key_probe_earlier: earlier,
                    ..base
                });
                lagged_probe = lagged_output.probe;

                assert_eq!(
                    lagged_output, sync_output,
                    "frame {i}, pattern {ready_pattern}"
                );
                assert!(
                    !(scene_mode && key >= KEY_FRAME_FRACTION && lagged_output.show_stream),
                    "frame {i}: a key colour frame was shown"
                );
            }
            // With nothing ready in its own loop the loop waits only while content is shown or
            // about to be: not during loading
            if ready_pattern == 0 {
                assert!(waits > 0 && waits < scene_script().len() - 200, "{waits}");
            }
        }
    }

    #[test]
    fn preview_shows_the_chosen_variant_over_passthrough_until_removed() {
        let mut sim = Sim::new();
        let preview = GuardInputs {
            preview: Some(Variant::TakingLonger),
            ..front_end()
        };
        let output = sim.run(Duration::from_secs(2), preview);
        assert_eq!(output.background_alpha, 1.0);
        assert_eq!(output.variant, Variant::TakingLonger);
        assert!(output.keep_passthrough);
        // No going back however long it is shown
        let output = sim.run(GOING_BACK_AFTER + Duration::from_secs(2), preview);
        assert_eq!(output.variant, Variant::TakingLonger);
        assert_eq!(output.background_alpha, 1.0);
        let output = sim.run(BACKGROUND_FADE + FRAME * 4, front_end());
        assert_eq!(output.background_alpha, 0.0);
    }

    #[test]
    fn look_sizes_and_live_keys() {
        let look = Look::default();
        // Ring 8 deg outer diameter at 2 m: 0.280 m, on a 0.327 m quad
        assert!((angular_size_m(look.ring_deg, look.distance_m) - 0.2797).abs() < 0.001);
        assert!((look.ring_quad_m() - 0.327).abs() < 0.002);
        // Logo 5.5 deg wide: 0.192 m, centre 0.211 m below the ring centre
        assert!((look.logo_width_m() - 0.192).abs() < 0.001);
        assert!((look.logo_below_ring_m() - 0.2115).abs() < 0.002);
        let look = Look::parse(
            "ring_deg=12\nlogo=off\nlogo_deg=8\ndistance_m=1.5\nbelow_deg=7\nbackground=#000000\nring_deg=500",
        );
        assert_eq!(
            (
                look.ring_deg,
                look.logo,
                look.logo_deg,
                look.distance_m,
                look.below_deg
            ),
            (12.0, false, 8.0, 1.5, 7.0)
        );
        assert_eq!(look.background, [0, 0, 0]);
    }

    fn content(sim: &mut Sim) -> GuardOutput {
        let mut output = sim.step(scene(Some(0.0)));
        for _ in 1..CONTENT_FRAMES {
            output = sim.step(scene(Some(0.0)));
        }
        output
    }

    #[test]
    fn front_end_shows_the_stream_and_no_loading_screen() {
        let mut sim = Sim::new();
        let output = sim.run(Duration::from_secs(2), front_end());
        assert_eq!(output.background_alpha, 0.0);
        assert!(output.show_stream);
        assert!(!output.keep_passthrough);
    }

    #[test]
    fn magenta_at_scene_start_is_never_shown_and_passthrough_fades_into_loading() {
        let mut sim = Sim::new();
        sim.run(Duration::from_secs(1), front_end());

        // Scene mode on, SteamVR's magenta in the stream (13:31:40.8 in the recording)
        let output = sim.step(scene(Some(0.97)));
        assert!(!output.show_stream);
        assert!(output.became_visible);
        assert!(output.keep_passthrough, "passthrough stays until opaque");
        // The ring starts when the background is half opaque, at 92 % size
        let output = sim.run(BACKGROUND_FADE / 2 - FRAME * 2, scene(Some(0.97)));
        assert!(output.background_alpha < 0.5);
        assert_eq!(output.indicator_alpha, 0.0);
        let output = sim.run(BACKGROUND_FADE / 2 + FRAME * 2, scene(Some(0.97)));
        assert_eq!(output.background_alpha, 1.0);
        assert!(!output.keep_passthrough);
        assert!(output.indicator_alpha > 0.0 && output.indicator_scale < 1.0);
        let output = sim.run(Duration::from_secs(3), scene(Some(0.97)));
        assert!(!output.show_stream);
        assert_eq!(output.indicator_alpha, 1.0);
        assert_eq!(output.indicator_scale, 1.0);
        assert_eq!(output.variant, Variant::Normal);

        // V3 presents: shown after CONTENT_FRAMES frames, then the loading screen cross-fades out
        let output = content(&mut sim);
        assert!(output.show_stream);
        let output = sim.run(BACKGROUND_FADE + FRAME, scene(None));
        assert_eq!(output.background_alpha, 0.0);
        assert_eq!(output.indicator_alpha, 0.0);
        assert!(output.show_stream);
    }

    #[test]
    fn a_key_frame_while_the_stream_is_shown_is_covered_at_once() {
        let mut sim = Sim::new();
        sim.run(Duration::from_millis(100), scene(Some(0.9)));
        content(&mut sim);
        let output = sim.run(Duration::from_secs(3), scene(None));
        assert!(output.show_stream);
        assert_eq!(output.background_alpha, 0.0);

        // V3 quits: the compositor's magenta (86.67 s in the recording) never reaches the display
        let output = sim.step(scene(Some(0.97)));
        assert!(!output.show_stream);
        assert_eq!(output.background_alpha, 1.0);
    }

    #[test]
    fn normal_exit_holds_over_passthrough_then_fades_out_without_going_back() {
        let mut sim = Sim::new();
        sim.run(Duration::from_secs(2), scene(Some(0.97)));
        content(&mut sim);
        sim.run(Duration::from_secs(5), scene(None));
        // V3 quits: magenta covered, then scene mode off
        sim.run(Duration::from_millis(300), scene(Some(0.97)));
        let output = sim.step(front_end());
        assert_eq!(output.background_alpha, 1.0);
        assert_eq!(output.variant, Variant::Normal);
        let output = sim.run(EXIT_HOLD - FRAME * 2, front_end());
        assert_eq!(output.background_alpha, 1.0);
        let output = sim.run(BACKGROUND_FADE + FRAME * 4, front_end());
        assert_eq!(output.background_alpha, 0.0);
        assert_eq!(output.variant, Variant::Normal);
        assert!(output.show_stream);
    }

    #[test]
    fn the_flag_shows_the_loading_screen_even_with_content_and_never_flickers() {
        let mut sim = Sim::new();
        sim.run(Duration::from_millis(200), scene(Some(0.0)));
        sim.run(Duration::from_secs(2), scene(None));
        let loading = GuardInputs {
            scene_loading: true,
            ..scene(Some(0.0))
        };
        let output = sim.run(Duration::from_millis(50), loading);
        assert!(output.background_alpha > 0.0);
        assert!(!output.show_stream);
        // Flag cleared quickly: still shown, fully, for MIN_VISIBLE
        let output = sim.run(Duration::from_millis(500), scene(Some(0.0)));
        assert_eq!(output.background_alpha, 1.0);
        let output = sim.run(MIN_VISIBLE + BACKGROUND_FADE, scene(Some(0.0)));
        assert_eq!(output.background_alpha, 0.0);
    }

    #[test]
    fn taking_longer_after_15_s() {
        let mut sim = Sim::new();
        let output = sim.run(
            TAKING_LONGER_AFTER - Duration::from_millis(100),
            scene(Some(0.97)),
        );
        assert_eq!(output.variant, Variant::Normal);
        let output = sim.run(Duration::from_millis(200), scene(Some(0.97)));
        assert_eq!(output.variant, Variant::TakingLonger);
    }

    #[test]
    fn scene_ending_before_its_first_frame_goes_back() {
        let mut sim = Sim::new();
        sim.run(Duration::from_secs(5), scene(Some(0.97)));
        // V3 exits before presenting
        let output = sim.step(front_end());
        assert_eq!(output.variant, Variant::GoingBack);
        assert!(output.keep_passthrough);
        let output = sim.run(
            GOING_BACK_CROSS_FADE + GOING_BACK_HOLD - FRAME * 2,
            front_end(),
        );
        assert_eq!(output.variant, Variant::GoingBack);
        assert_eq!(output.background_alpha, 1.0);
        let output = sim.run(BACKGROUND_FADE + FRAME * 4, front_end());
        assert_eq!(output.background_alpha, 0.0);
        assert!(output.show_stream);
    }

    #[test]
    fn no_first_frame_for_60_s_goes_back_and_passthrough_stays() {
        let mut sim = Sim::new();
        let output = sim.run(GOING_BACK_AFTER + FRAME * 2, scene(Some(0.97)));
        assert_eq!(output.variant, Variant::GoingBack);
        let output = sim.run(
            GOING_BACK_CROSS_FADE + GOING_BACK_HOLD + BACKGROUND_FADE + FRAME * 4,
            scene(Some(0.97)),
        );
        assert_eq!(output.background_alpha, 0.0);
        assert!(output.keep_passthrough);
        assert!(!output.show_stream, "the magenta stays hidden");
        // V3 finally presents: the stream appears (passthrough stays until scene mode ends)
        let output = content(&mut sim);
        assert!(output.show_stream);
    }

    #[test]
    fn arc_length_follows_the_spec_key_frames() {
        let expected = [60, 66, 83, 105, 128, 144, 150, 144, 128, 105, 83, 66];
        for (k, degrees) in expected.iter().enumerate() {
            let length = arc_length_deg(k as f32 / 6.0);
            assert!((length - *degrees as f32).abs() < 0.6, "k {k}: {length}");
        }
    }

    #[test]
    fn brand_colours_reach_the_display_unchanged() {
        // sRGB swapchain: the bytes are the brand hex values (the compositor decodes sRGB)
        assert_eq!(swapchain_bytes(OCEAN, true), [0x40, 0x50, 0x5E]);
        assert_eq!(swapchain_bytes(GREEN, true), [0x93, 0xCB, 0x4D]);
        // UNORM fallback: linear values that encode back to the brand colour
        assert_eq!(swapchain_bytes(OCEAN, false), [13, 20, 29]);

        // The drawn ring: the arc's middle is exactly Green, opaque; the track 18 % Aluminium
        let band = RingBand::new();
        let mut pixels = vec![0; (TEXTURE * TEXTURE * 4) as usize];
        band.draw(GREEN, 100.0, 0.0, &mut pixels);
        let at = |x: u32, y: u32| {
            let i = ((y * TEXTURE + x) * 4) as usize;
            [pixels[i], pixels[i + 1], pixels[i + 2], pixels[i + 3]]
        };
        let centre = TEXTURE / 2;
        let top = centre - RING_RADIUS as u32;
        assert_eq!(at(centre, top), [0x93, 0xCB, 0x4D, 255]);
        assert_eq!(
            at(centre, centre + RING_RADIUS as u32),
            [0xE9, 0xE9, 0xE9, 46]
        );
        assert_eq!(at(centre, centre)[3], 0);
        // Same as the designer's export at the same spots
        let (_, export) = decode_png(include_bytes!(
            "../loading_assets/loading-indicator-longer-512.png"
        ))
        .unwrap();
        let export_at = |x: u32, y: u32| {
            let i = ((y * TEXTURE + x) * 4) as usize;
            [export[i], export[i + 1], export[i + 2], export[i + 3]]
        };
        assert_eq!(export_at(centre, top), at(centre, top));
        assert_eq!(
            export_at(centre, centre + RING_RADIUS as u32),
            at(centre, centre + RING_RADIUS as u32)
        );
    }

    #[test]
    fn logo_is_downsampled_without_recolouring() {
        let (size, pixels) = decode_png(LOGO_PNG).unwrap();
        assert_eq!(size, UVec2::new(3048, 844));
        let (small, out) = downsample(&pixels, size, LOGO_TEXTURE_WIDTH);
        assert_eq!(small, UVec2::new(320, 89));
        // Every visible pixel keeps the artwork's Aluminium colour; only coverage (alpha) changes
        for pixel in out.chunks_exact(4).filter(|p| p[3] > 0) {
            assert_eq!(&pixel[..3], &[236, 236, 236]);
        }
        assert!(out.chunks_exact(4).any(|p| p[3] == 255));
    }

    #[test]
    fn embedded_assets_decode() {
        let (size, _) = decode_png(GOING_BACK_PNG).unwrap();
        assert_eq!(size, UVec2::splat(TEXTURE));
        let (size, _) = decode_png(LONGER_PNG).unwrap();
        assert_eq!(size, UVec2::splat(TEXTURE));
    }
}

#[cfg(test)]
mod preview {
    use super::*;

    // cargo test -p alvr_client_openxr --lib loading_screen_preview -- --ignored, with
    // LOADING_PREVIEW_DIR set: writes the drawn ring at the spec's 12 key frames (raw RGBA8 512x512)
    #[test]
    #[ignore = "writes files"]
    fn loading_screen_preview() {
        let Ok(dir) = std::env::var("LOADING_PREVIEW_DIR") else {
            return;
        };
        let band = RingBand::new();
        let mut pixels = vec![0; (TEXTURE * TEXTURE * 4) as usize];
        for k in 0..12 {
            let t = k as f32 / 6.0;
            // The quad turn (pose) folded into the texture for the preview
            band.draw(GREEN, arc_length_deg(t), t * ARC_DEG_PER_S, &mut pixels);
            fs::write(format!("{dir}/ring_{k:02}.rgba"), &pixels).unwrap();
        }
    }
}
