# HANDOFF: ALVR side of ScreenVR "Physical" placement (branch `feature/screen-markers`)

Written Friday 2026-11-06 by `alvr_session`, for Monday. The ScreenVR side is in
`D:\wkspaces\ScreenVR\HANDOFF.md`; the task list is `D:\wkspaces\ScreenVR\TASKS.md` (A1–A5 are ALVR
tasks).

## Commits (on `feature/screen-markers`, NOT pushed)

| Commit | What |
|---|---|
| `bb2fe92c` | Tracking extras for overlay apps: fingertips, driverFromStage, recenterCounter, `stageEpoch`, request block (`server_core/src/tracking_extras.rs`) |
| `a9a2a0b8` | PICO's own OpenXR loader (unlocks `XR_PICO_*`), `XR_DIAG` extension logging |
| `6a2b5c9e` | Headset camera frames via PICO SecureMR → `Local\VRCameraFrame_v1` |
| `0f2fe90e` | Fix: camera pose = head pose × per-model extrinsics (SecureMR LOCAL mismatch, below) |
| `057a573a` | Debug native reference quads (`Local\VRNativeQuads_v1` → OpenXR quad layers) |

Not committed: this file and `tools/screen_markers/*.py`; the untracked `CLAUDE.md`, `HANDOVER.md`,
`resources/SM_Hand_*.FBX` and `resources/pico_hand_retarget_instructions.md` were deliberately left out
(agent config, stale notes, third-party meshes with an unclear licence).

## Deployed (verified running Friday 17:23)

- Streamer driver **71a4ea25** in `C:\Tenstar\VR\Pico\alvr\pc\alvr_server\bin\win64\driver_alvr_server.dll`
  (= commit 057a573a).
- Client APK **6a157f7c** on headset PA94Y0MGJ9180002G (`alvr.client.dev`) and as
  `C:\Tenstar\VR\Pico\alvr\alvr_client.apk` (= commit 057a573a).
- ScreenVR `debugNativeQuads` = 0 (quads off).

### Rollback

| Target | Streamer (copy over `pc\`) | APK |
|---|---|---|
| Before native quads | `pc_backup_20261106_pre_nativequads` | `alvr_client.apk.bak-7424add6` (= 0f2fe90e) |
| Before the extrinsics fix | same as above (driver unchanged by 0f2fe90e) | `alvr_client.apk.bak-597b455f` (= 6a2b5c9e, has the LOCAL bug) |
| Before camera v3 | `pc_backup_20261106_pre_camv3` | `alvr_client.apk.bak-cam-f337bf14` |
| Before any of this work | `pc_backup_20261106_pre_extras` | `alvr_client.apk.bak-20261106` (original production client) |

All under `C:\Tenstar\VR\Pico\alvr\`.

## Build and deploy

```
cargo xtask clippy                      # 195 pre-existing warnings; check none are in touched files
cargo test -p alvr_server_core --lib
cargo xtask build-streamer --release    # -> build/alvr_streamer_windows/
cargo xtask build-client --release      # -> build/alvr_client_android/alvr_client_android.apk
```

Streamer deploy (only `driver_alvr_server.dll` + `.pdb` change for server_core work):
1. Back up: `robocopy C:\Tenstar\VR\Pico\alvr\pc C:\Tenstar\VR\Pico\alvr\pc_backup_<date>_<tag> /E`.
2. Stop `ALVR Dashboard`, then `vrmonitor`, `vrserver` (stopping the dashboard alone did not stop
   vrserver on Friday).
3. Copy `build\alvr_streamer_windows\alvr_server\bin\win64\driver_alvr_server.dll` (+ `alvr_server_openvr.pdb`)
   into `pc\alvr_server\bin\win64\`.
4. Start `pc\ALVR Dashboard.exe` (working dir `pc\`); it starts SteamVR. The stage epoch restarts at 0, and
   ScreenVR recalibrates automatically.

Client deploy: `adb -s PA94Y0MGJ9180002G install -r <apk>` (keeps the CAMERA grant). The kiosk relaunches
the app; if not: `adb shell monkey -p alvr.client.dev -c android.intent.category.LAUNCHER 1`. Wake the
headset first: `adb shell input keyevent KEYCODE_WAKEUP` (it sleeps ~1 min off-head; the client stays
`resumed=false` until woken). Copy the APK to `C:\Tenstar\VR\Pico\alvr\alvr_client.apk` after backing up
the old one as `.bak-<hash>`.

Git Bash: `export MSYS_NO_PATHCONV=1` before adb commands with `/sdcard/...` paths.

## Key finding: SecureMR's LOCAL is not always the app's LOCAL

- SecureMR reports the camera pose relative to *its own* LOCAL space. In some client processes it equals
  the app's XR LOCAL; in others it was offset by (0.08, 0, 8.66) cm, same rotation (11:12–16:22 Friday;
  gone after a later restart; trigger unknown, not wear state, not the loader).
- Before 0f2fe90e every `stage_from_camera` and every ChArUco-calibrated monitor was shifted 8.7 cm along
  STAGE −Z. It looked like a "stand vs worn eye model" and made screens ~9 % too small.
- Fix: `stage_from_camera = stage_from_view(capture time) · view_from_camera[eye]` with extrinsics from a
  table keyed by Android product name: `CAMERA_EXTRINSICS` in `alvr/client_openxr/src/camera_frames.rs`.
  - Only `"PICO 4 Ultra Enterprise"` is in it (ALVR's `Platform` maps this headset to `PicoUnknown`).
  - Left (-3.215, +3.904, -3.495) cm, right (+3.197, +3.816, -3.514) cm, rotation roll -0.79°,
    pitch +0.66°, baseline 64.1 mm. Stable to 0.1 mm across both regimes (hand-eye fit, 0.2 mm residual).
  - Other models fall back to the SecureMR pose, with a warning in logcat.
- Monitoring: once per camera pass the client logs
  `CAMPOSE_DIFF eye=Left securemr - extrinsics: position (x, y, z) cm (|d| cm), rotation r deg`.
  ~0 cm = LOCALs agree; ~8.7 cm = the mismatch is back (harmless now, but worth noting).

## Diagnostics

- **Logcat**: the Pico main buffer rotates in seconds. Stream it to a file before the event:
  `adb -s PA94Y0MGJ9180002G logcat -T 1 > logcat.txt` (keep it running; reinstalling the APK can kill it).
- **Log tags**: `CAMPOSE_DIAG` (raw poses, first 10 frames of each camera pass), `CAMPOSE_DIFF`,
  `Camera frames:`, `Native quads:`, `XR_DIAG` (extensions at session start). Server side
  (`C:\Tenstar\VR\Pico\alvr\pc\session_log.txt`): `Tracking extras:`, `Native quads:`,
  `Camera frames request sent`.
- **Scripts** (`tools/screen_markers/`, need numpy):
  - `python campose_passes.py campose_analyze.py logcat.txt`: per CAMPOSE_DIAG line, the camera position
    in VIEW (cm). Worn and correct: (-3.2, +3.9, -3.5) for the left eye. Only meaningful when the LOCALs
    agree (it trusts the app's LOCAL).
  - `python campose_analyze.py logcat.txt`: per eye, H1 (constant C?) and H2 hand-eye fit (needs head
    rotation in the pass). H2 recovers view_from_camera and SecureMR's LOCAL in STAGE regardless of the
    mismatch; this is how the bug was found.

## Contracts (source of truth: ScreenVR headers)

- `Local\VRTrackingExtras_v1` (v2, written by ALVR): driverFromStage, recenterCounter, fingertips,
  markerStatus, stageEpoch. `Local\VRTrackingRequest_v1` (written by ScreenVR): bits 1 markers
  (answered Unsupported), 2 camera frames, 4 right camera. `Local\VRCameraFrame_v1`: 152-byte header +
  2048² GRAY8. See `VrTrackingExtras.h`, `VrCameraFrame.h`.
- `Local\VROpaqueRegions_v1` (pre-existing): regions forced opaque under chroma key.
- **`Local\VRNativeQuads_v1`** (new, ScreenVR `VrNativeQuads.h`, reader `server_core/src/native_quads.rs`):
  pack(4), seqlock, heartbeat ≤100 ms, stale 1000 ms. Header magic `0x4441514E`, version 1, sequence,
  count (≤16), heartbeatMs, stageEpoch, reserved; then 16 × 80-byte quads: `stageFromQuad` (HmdMatrix34,
  client STAGE, quad in XY facing +Z), width/height m, columns/rows (1..256), dark/light/outline RGBA
  (bytes R,G,B,A), outlinePixels. Drawn only while stageEpoch equals ALVR's. Server polls on the keepalive
  loop (1 s) and sends `ServerControlPacket::NativeQuads` on change. Client
  (`client_openxr/src/native_quads.rs`): one static RGBA8 swapchain per quad, `XrCompositionLayerQuad` in
  STAGE, submitted after the stream layer (always on top), unpremultiplied alpha.
  - Switch: ScreenVR `config.json` → `physical.debugNativeQuads` (0 off, 1 opaque, 2 see-through,
    3 none). Edit the file IN PLACE: a `sed -i` replace was not picked up by ScreenVR's watcher.
  - Verified rendering Friday; the alignment measurement (TASKS S2) is still to do. If the chessboard
    parity looks vertically flipped, the texture row order in `native_quads.rs` is the place.

## Headset facts worth knowing

- PICO 4 Ultra Enterprise, OS 5.15.7, product name `PICO 4 Ultra Enterprise`. Memory note
  `reference_pico_marker_tracking.md` has SecureMR details (intrinsics, timestamps, axes).
- Relevant OpenXR extensions offered (PICO loader): `XR_EXT_hand_tracking`, `XR_FB_passthrough`,
  `XR_FB_triangle_mesh`, `XR_KHR_composition_layer_depth`, `XR_KHR_composition_layer_cylinder`,
  `XR_KHR_android_surface_swapchain`, `XR_FB_composition_layer_alpha_blend`,
  `XR_EXT_composition_layer_inverted_alpha`. NOT offered: `XR_FB_composition_layer_depth_test`,
  `XR_FB_hand_tracking_mesh`, marker tracking.
- Known, deliberately untouched: `XR_PICO_configuration` / `XR_BD_motion_tracking` are never enabled
  (extension-name NUL mismatch in `lib.rs`; only the SecureMR names were fixed).

## Feasibility notes for A1–A3 (no code yet)

### A1. Hand-tracked hands drawn natively above the stream (HIGH)

Where the data is: `interaction::get_hand_data()` locates the 26 `XR_EXT_hand_tracking` joints
(`skeleton_joints`, in the stream's STAGE space). Today it runs only on the **tracking thread**
(`stream_input_loop` in `stream.rs`, predicted for the server), not on the render thread.

Recommended design:
- Locate the hand joints **on the render thread at the frame's predicted display time** (same
  `xrLocateHandJointsEXT` call with the render loop's time). That gives zero-stream-latency hands, the
  whole point of drawing them on the headset. The `HandData` arbitration (controller vs hand,
  `resolve_hand_sources`) should be reused so hands vanish while a controller is held.
- Render into **a separate projection layer** (own 2 swapchains, can be lower resolution than the
  stream, e.g. 50–70 %), alpha-blended, submitted after the stream layer: passthrough → stream → hands
  (→ native quads if enabled). Its views come from `xrLocateViews` for the same display time, so it is
  independent of the stream's pose and pacing.
  - Alternative: draw into the stream swapchain after `StreamRenderer::render`. That's cheaper (no extra
    layer), but it must use the stream frame's view poses (older), which reintroduces some latency.
    Not recommended.
- Depth: no `XR_FB_composition_layer_depth_test`, so the compositor can't occlude hands with stream
  depth (V3 doesn't provide usable depth through the stream anyway). Hands are "always on top", as the
  task list assumes. A settings toggle belongs in the session settings (`alvr_session`, sent with the
  stream config) or in RealTimeConfig if it should switch live.
- Mesh and skinning:
  - The lobby (`alvr_graphics/src/lobby.rs`) only draws the skeleton as **lines** (`line_pipeline`,
    `HAND_SKELETON_BONES`). That is reusable for a first, debug-quality version (bones/capsules), but it
    is not a skinned mesh.
  - A skinned hand needs a new wgpu pipeline (the client renders through wgpu on GLES): vertex
    positions/normals/UV + 4 joint indices/weights, and 26 joint matrices in a uniform buffer (push
    constants are capped at 128 bytes). Joint matrix = joint pose (STAGE) × inverse bind pose.
  - Mesh source: PICO's `SM_Hand_L/R.FBX` (PICO OpenXR Unreal plugin) has skin weights, and its bones
    are named after the XR_EXT_hand_tracking joints, so the mapping is direct. Convert offline (Blender;
    `resources/pico_hand_to_rendermodel.py` already reads these FBX) to a compact binary or glTF with
    inverse binds, and embed it in the APK. **Check the licence before committing the mesh**: it is
    third-party content (the FBX files are untracked for this reason). A simple procedural hand
    (capsules per bone plus palm) avoids the question and may be enough for touchscreen work.
- Effort: medium. The layer plumbing is small (the native quads layer list in `lib.rs` is the pattern);
  most of the work is the skinned pipeline and the mesh asset.

### A2. Passthrough window at the real touchscreen (MEDIUM)

- The opaque-regions path already rasterizes overlay quads into the stream with a stencil
  (`alvr_graphics/src/opaque_regions.rs`, `opaque_regions.wgsl`). A2 is the inverse: force alpha 0
  inside the region. Add a per-region mode (opaque | transparent) to the contract (there is a
  `reserved` field in the block header, but a per-region flag needs a v2 region layout) and a second
  stencil/shader pass that writes alpha 0.
- Passthrough must be on during V3 with chroma key **off** (V3 renders full VR), so only the window is
  transparent. Check the passthrough settings combination (passthrough layer + alpha-blended stream
  without chroma key) in `stream.rs` / `ProjectionLayerAlphaConfig`.
- Regions travel per video frame in driver space, so the window follows the stream's frame pose. Small
  residual latency at the window edge during head motion; a margin hides it.
- Effort: small to medium.

### A3. Passthrough hands (LOW)

- Two ways:
  1. Hand silhouettes written as alpha 0 into the stream image (same render-thread hand joints as A1,
     drawn into the stream swapchain after the stream pass). This is A1's mesh work, with a trivial
     shader.
  2. `XR_FB_passthrough` + `XR_FB_triangle_mesh` are offered: a projected-passthrough layer (geometry =
     hand mesh, updated per frame) submitted ABOVE the stream would show the real hands only inside the
     mesh. This is elegant, but PICO's support for dynamic projected-passthrough meshes is unverified;
     prototype it before relying on it.
- Passthrough hands look like the real hands (correct size and position by construction, apart from
  PICO's own passthrough geometry), whereas rendered hands need a mesh that fits the user's hand.

### A4 note

`XR_KHR_android_surface_swapchain` is offered: a decoder can render straight into a quad-layer
swapchain (zero copy) if native screen layers are pursued.
