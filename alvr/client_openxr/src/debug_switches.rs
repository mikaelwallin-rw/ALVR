// Headset switch files for tuning without a reinstall or reconnect: small files in the app's
// external files dir (`adb shell` can write there), read by a background thread once a second so
// that the render thread never touches the (FUSE) file system. Absent = default.
// - screenvr_video_path.txt: "mipmapped" (default), "zero-copy" or "auto" (screen_video.rs).
// - screenvr_hands_scale.txt: hands layer resolution relative to the stream view resolution
//   (0.25-1.0), screenvr_hands_msaa.txt: its MSAA sample count (1, 2 or 4). Absent: the defaults in
//   native_hands.rs. A change rebuilds the hands layer.
// - screenvr_hands.txt: drawn hand size, live (native_hands.rs): `thickness=<factor>` (girth
//   perpendicular to the bones, 1 = the mesh), `scale=<factor>` (uniform about the wrist, moves the
//   fingertips).
// - screenvr_loading.txt: loading screen look (sizes, colours; keys in loading_screen.rs), applied
//   live while it is shown.
// - screenvr_loading_preview.txt: "1"/"normal", "longer" or "back": show the loading screen in that
//   variant over passthrough until the file is removed (tuning without starting a scene app).
// - screenvr_refmarkers.txt: reference markers (ref_markers.rs): lines `<png> <size m> <3x4 STAGE
//   pose row-major>`, drawn world-locked for the displayed-view check of a headset model.
// None of them affects the camera frames (camera extrinsics are fixed per model, camera_frames.rs).
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
    pub video_path: String,
    pub hands_scale: Option<f32>,
    pub hands_msaa: Option<u32>,
    pub hands_shape: String,
    pub loading_look: String,
    pub loading_preview: String,
    pub ref_markers: String,
}

fn read(name: &str) -> String {
    fs::read_to_string(format!("{DIR}/{name}"))
        .unwrap_or_default()
        .trim()
        .to_owned()
}

fn read_all() -> DebugSwitches {
    DebugSwitches {
        video_path: read("screenvr_video_path.txt"),
        hands_scale: read("screenvr_hands_scale.txt")
            .parse::<f32>()
            .ok()
            .filter(|v| (0.25..=1.0).contains(v)),
        hands_msaa: read("screenvr_hands_msaa.txt")
            .parse::<u32>()
            .ok()
            .filter(|v| alvr_graphics::HANDS_SAMPLE_COUNTS.contains(v)),
        hands_shape: read("screenvr_hands.txt"),
        loading_look: read("screenvr_loading.txt"),
        loading_preview: read("screenvr_loading_preview.txt"),
        ref_markers: read("screenvr_refmarkers.txt"),
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
    let mut suppress_logged = false;
    let mut applied = DebugSwitches::default();
    loop {
        let mut values = read_all();
        if let Some(text) = &suppressed {
            if *text == format!("{values:?}") && values != DebugSwitches::default() {
                if !suppress_logged {
                    warn!(
                        "Debug switches: the previous run applied {text} less than {} s before this \
                         start; ignoring the switch files until they change",
                        CRASH_GUARD_WINDOW.as_secs()
                    );
                    suppress_logged = true;
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
