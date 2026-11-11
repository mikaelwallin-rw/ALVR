// DEBUG SWITCHES for headset A/B tests without a reinstall or reconnect: small files in the app's
// external files dir (`adb shell` can write there), read by a background thread once a second so
// that the render thread never touches the (FUSE) file system.
// - screenvr_display_time.txt: "predicted" = xrEndFrame with the runtime's predicted display time
//   while native quads are shown; else the stream frame's timestamp (upstream).
// - screenvr_quad_settings.txt: XR_FB_composition_layer_settings flags for the video quads
//   (decimal; 0/absent off, 1/2 normal/quality super-sampling, 4/8 normal/quality sharpening).
// - screenvr_video_path.txt: "mipmapped" (default), "zero-copy" or "auto" (screen_video.rs).
// - screenvr_stall.txt: "<ms> <period ms>", e.g. "100 3000": the render thread sleeps <ms> before
//   xrEndFrame once every <period ms> (tests what the runtime does with a late frame).
// - screenvr_stream_wait.txt: how long the render loop waits for a decoded main-stream frame, as a
//   fraction of the frame interval (upstream 0.8). Lower: submit on time with the previous stream
//   frame instead of going late when the stream frame comes late.
// - screenvr_stream_unused.txt: "ignore" = render the stream even while the overlay app flags it
//   as unused (A/B of the stream-unused skip; default: honour the flag).
// - screenvr_hands_scale.txt: hands layer resolution relative to the stream view resolution
//   (0.25-1.0), screenvr_hands_msaa.txt: its MSAA sample count (1, 2 or 4). Absent: the defaults in
//   native_hands.rs. A change rebuilds the hands layer.
// - screenvr_key_probe.txt: "off" = MEASUREMENT ONLY: no key colour check of stream frames in
//   scene mode (every rendered frame counts as content, so magenta is shown again). For the A/B of the
//   check's cost (loading_screen.rs); default: on.
// - screenvr_loading.txt: loading screen look (sizes, colours; keys in loading_screen.rs), applied
//   live while it is shown.
// - screenvr_loading_preview.txt: "1"/"normal", "longer" or "back": show the loading screen in that
//   variant over passthrough until the file is removed (tuning without starting a scene app).
// - screenvr_projected_passthrough.txt: PROTOTYPE "on" (5 cm margin) or a margin in metres, e.g.
//   "0.05": passthrough PROJECTED onto each native quad's rectangle (plus the margin), over the
//   normal reconstruction passthrough (extra_extensions/passthrough_fb.rs).
// - screenvr_hands.txt: hand shape, live (native_hands.rs): `thickness=<factor>` (girth
//   perpendicular to the bones, 1 = the mesh), `scale=<factor>` (uniform about the wrist, moves the
//   fingertips), `fingers=<0.9..1.3>` (finger bones lengthened from the knuckles; the palm stays),
//   `girth=mesh|radii` (radii: per joint from the tracked joint radii; PICO reports a constant
//   15 mm, so it carries no information there).
// - screenvr_refmarkers.txt: reference markers (ref_markers.rs): lines `<png> <size m> <3x4 STAGE
//   pose row-major>`, drawn world-locked for the displayed-view calibration test.
// - screenvr_fake_hands.txt: "1" = TEST ONLY: when no hand is tracked, draw a synthetic left hand
//   in front of the STAGE origin, so that the hands renderer's per-frame uploads run on an unworn
//   headset.
// - screenvr_camera_extrinsics.txt: absent/"pico" = camera frames use PICO's own camera extrinsics
//   (capture library, CAMERA_EXTRINSICS_PICO in camera_frames.rs; the default); "fit" = the old
//   hand-eye fitted set, KNOWN WRONG legacy set kept only for A/B measurement (to be removed in the
//   cleanup). Applied live to the next camera frame; logged as CAMERA_EXTRINSICS. "pico" (and any
//   value other than "fit") is normalised to the default, so it never counts as a non-default
//   switch for the crash-loop guard below.
// CRASH-LOOP GUARD: whenever non-default switches are applied, screenvr_switches_applied.txt gets
// the time and the switches. If at start-up the same switches were applied less than
// CRASH_GUARD_WINDOW ago (the previous run died, or was restarted, right after applying them), the
// files are ignored (defaults, logged) until their content changes.

use alvr_common::{info, warn};
use std::{
    fs,
    sync::{Mutex, OnceLock},
    thread,
    time::{Duration, SystemTime, UNIX_EPOCH},
};

const DIR: &str = "/sdcard/Android/data/alvr.client.dev/files";
const POLL_INTERVAL: Duration = Duration::from_secs(1);
const APPLIED_MARKER: &str = "screenvr_switches_applied.txt";
const CRASH_GUARD_WINDOW: Duration = Duration::from_secs(60);

#[derive(Clone, Default, PartialEq, Debug)]
pub struct DebugSwitches {
    pub predicted_display_time: bool,
    pub quad_settings_flags: u64,
    pub video_path: String,
    pub stall: Option<(Duration, Duration)>,
    pub stream_wait: Option<f32>,
    pub fake_hands: bool,
    pub ignore_stream_unused: bool,
    pub key_probe_off: bool,
    pub loading_look: String,
    pub hands_shape: String,
    pub ref_markers: String,
    pub projected_passthrough_margin: Option<f32>,
    pub loading_preview: String,
    pub hands_scale: Option<f32>,
    pub hands_msaa: Option<u32>,
    pub camera_extrinsics: String,
}

fn read(name: &str) -> String {
    fs::read_to_string(format!("{DIR}/{name}"))
        .unwrap_or_default()
        .trim()
        .to_owned()
}

fn read_all() -> DebugSwitches {
    let stall = read("screenvr_stall.txt");
    let mut numbers = stall
        .split_whitespace()
        .filter_map(|v| v.parse::<u64>().ok());
    let stall = match (numbers.next(), numbers.next()) {
        (Some(ms), Some(period)) if ms > 0 && period > ms => {
            Some((Duration::from_millis(ms), Duration::from_millis(period)))
        }
        _ => None,
    };

    DebugSwitches {
        predicted_display_time: read("screenvr_display_time.txt") == "predicted",
        quad_settings_flags: read("screenvr_quad_settings.txt").parse().unwrap_or(0),
        video_path: read("screenvr_video_path.txt"),
        stall,
        stream_wait: read("screenvr_stream_wait.txt")
            .parse::<f32>()
            .ok()
            // at least one decoder poll per frame
            .filter(|v| (0.05..=1.0).contains(v)),
        fake_hands: read("screenvr_fake_hands.txt") == "1",
        ignore_stream_unused: read("screenvr_stream_unused.txt") == "ignore",
        key_probe_off: read("screenvr_key_probe.txt") == "off",
        loading_look: read("screenvr_loading.txt"),
        hands_shape: read("screenvr_hands.txt"),
        ref_markers: read("screenvr_refmarkers.txt"),
        projected_passthrough_margin: match read("screenvr_projected_passthrough.txt").as_str() {
            "on" | "1" => Some(0.05),
            value => value
                .parse::<f32>()
                .ok()
                .filter(|m| (0.0..=0.5).contains(m)),
        },
        loading_preview: read("screenvr_loading_preview.txt"),
        hands_scale: read("screenvr_hands_scale.txt")
            .parse::<f32>()
            .ok()
            .filter(|v| (0.25..=1.0).contains(v)),
        hands_msaa: read("screenvr_hands_msaa.txt")
            .parse::<u32>()
            .ok()
            .filter(|v| alvr_graphics::HANDS_SAMPLE_COUNTS.contains(v)),
        // Only "fit" (legacy A/B set) is non-default; "pico"/absent/other = default = empty
        camera_extrinsics: if read("screenvr_camera_extrinsics.txt").eq_ignore_ascii_case("fit") {
            "fit".to_owned()
        } else {
            String::new()
        },
    }
}

static SWITCHES: OnceLock<Mutex<DebugSwitches>> = OnceLock::new();

fn now_secs() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0)
}

// The switches applied less than CRASH_GUARD_WINDOW ago by a previous run, as written to the marker
fn recently_applied() -> Option<String> {
    let text = fs::read_to_string(format!("{DIR}/{APPLIED_MARKER}")).ok()?;
    let (time, switches) = text.split_once('\n')?;
    let age = now_secs().saturating_sub(time.trim().parse().ok()?);
    (age < CRASH_GUARD_WINDOW.as_secs()).then(|| switches.trim().to_owned())
}

fn poll_loop() {
    // Switches to ignore until the files change (crash-loop guard)
    let mut suppressed = recently_applied();
    let mut applied = DebugSwitches::default();
    loop {
        let mut values = read_all();
        if let Some(text) = &suppressed {
            if *text == format!("{values:?}") && values != DebugSwitches::default() {
                if !matches!(SUPPRESS_LOGGED.get(), Some(true)) {
                    warn!(
                        "Debug switches: the previous run applied {text} less than {} s before this \
                         start; ignoring the switch files until they change",
                        CRASH_GUARD_WINDOW.as_secs()
                    );
                    SUPPRESS_LOGGED.set(true).ok();
                }
                values = DebugSwitches::default();
            } else {
                suppressed = None;
            }
        }
        if values != applied {
            info!("Debug switches: {values:?}");
            if values != DebugSwitches::default() {
                fs::write(
                    format!("{DIR}/{APPLIED_MARKER}"),
                    format!("{}\n{values:?}\n", now_secs()),
                )
                .ok();
            }
            applied = values.clone();
        }
        // Every pass: the first ones may run before get() has stored the shared value
        if let Some(switches) = SWITCHES.get() {
            let mut current = switches.lock().unwrap();
            if *current != values {
                *current = values;
            }
        }
        thread::sleep(POLL_INTERVAL);
    }
}

static SUPPRESS_LOGGED: OnceLock<bool> = OnceLock::new();

/// The current switches (default until the first poll). Starts the poll thread on first use.
pub fn get() -> DebugSwitches {
    SWITCHES
        .get_or_init(|| {
            thread::Builder::new()
                .name("debug switches".into())
                .spawn(poll_loop)
                .ok();
            Mutex::new(DebugSwitches::default())
        })
        .lock()
        .unwrap()
        .clone()
}
