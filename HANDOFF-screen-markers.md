# HANDOFF: ALVR side of ScreenVR "Physical" placement (branch `feature/screen-markers`)

Written Friday 2026-11-06 by `alvr_session`; brought up to date 2026-11-11. The ScreenVR side is in
`D:\wkspaces\ScreenVR\HANDOFF.md` (session log 2026-11-11 has the details behind this file); the task list
is `D:\wkspaces\ScreenVR\TASKS.md`. **The ScreenVR <-> ALVR interface (shared memory, packets, switch
files, compatibility) is described ONLY in `D:\wkspaces\ScreenVR\docs\AlvrInterface.md`.**

Sections below "Deployed (2026-11-11)" are dated history; where they disagree, the newer section wins.

## Commits (on `feature/screen-markers`, NOT pushed)

| Commit | What |
|---|---|
| `bb2fe92c` | Tracking extras for overlay apps: fingertips, driverFromStage, recenterCounter, `stageEpoch`, request block |
| `a9a2a0b8` | PICO's own OpenXR loader (unlocks `XR_PICO_*`), `XR_DIAG` extension logging |
| `6a2b5c9e` | Headset camera frames via PICO SecureMR -> `Local\VRCameraFrame_v1` |
| `0f2fe90e` | Camera pose = head pose x per-model extrinsics (SecureMR LOCAL mismatch) |
| `057a573a` | Debug native reference quads (`Local\VRNativeQuads_v1` -> OpenXR quad layers) |
| `13faa49c`, `cd78788f` | CAMPOSE tools, first handoff |
| `ccc2a548`..`77ceae8c` | sRGB native quads, natively drawn hand-tracked hands, hidden SteamVR hand models |
| `5fa0ef0b`..`f4fb56bb` | Screen video on native quads (multi-stream), scene mode/loading, STAGE identity + anchor, hand extras v4, loading screen, mip path, hand tuning |
| `4198cff9` | Handoff 2026-11-10 |
| `2e285591` | PICO capture-library camera extrinsics as the default set (`fit` A/B switch, unknown-model warning) |
| `5be46d3f` | `XR_PICO_camera_image` probe behind `screenvr_camera_probe.txt` (TEST ONLY, inert without the file, slated for removal) |
| `24acb3ea` | Packets: `TrackingStateReport` / `CameraTrackingStamp` in `ReservedBuffer` |
| `c35aa22b` | Server: tracking extras v5 (`trackingEpoch`, `headTrackingFlags`, floor-height stage change) + camera-frame tracking stamp |
| `c3d3847d` | Client: head tracking epoch state machine + tracking diagnostics log (= deployed tree) |
| (this file) | docs commit on top |

Intermediate commits of 2026-11-11 were not build-tested one by one; `c3d3847d` is the deployed tree.
Deliberately NOT committed: `CLAUDE.md`, `HANDOVER.md`, `resources/SM_Hand_*.FBX`,
`resources/pico_hand_retarget_instructions.md` (agent config, stale notes, third-party meshes with an unclear
licence).

## Deployed (2026-11-11 13:42; verified running)

- Streamer driver **`76182441`** (`C:\Tenstar\VR\Pico\alvr\pc\alvr_server\bin\win64\driver_alvr_server.dll`)
  + client APK **`1367d3f0`** (`C:\Tenstar\VR\Pico\alvr\alvr_client.apk`) = commit `c3d3847d`.
- Contents beyond 2026-11-10: PICO capture-library camera extrinsics as default; tracking extras **version 5**
  (`trackingEpoch`, `headTrackingFlags` PT/OT/SETTLED/LIVE, `clientInstanceId`/`clientTrackingEpoch`,
  floor-height stage change); 40-byte `CFTS` tracking stamp after the camera pixels; tracking diagnostics log;
  `XR_PICO_virtual_boundary` enabled when offered (read-only polling); camera-image probe (inert without its file).
- Transport of the new state is `ReservedBuffer` (ignored by older peers); the other fork packets still need
  driver and APK installed together (see AlvrInterface.md "Compatibility and deployment").
- Backups of today's steps (all under `C:\Tenstar\VR\Pico\alvr\`): streamer `pc_backup_20261111_pre_extras5`
  (before 76182441), `pc_backup_20261111_pre_picodefault` (before 61de28d6); APKs `alvr_client.apk.bak-2ce2f6e4`
  (diagnostics build, before 1367d3f0), `.bak-b51e35d1` (PICO default), `.bak-146b41f6`, `.bak-41853163`,
  `.bak-0cb31ce8`. Candidates kept: `alvr_client.apk.candidate-*`, `driver_candidate_*`.

### Headset switch files (`/sdcard/Android/data/alvr.client.dev/files`; table in AlvrInterface.md)

Read every second (`debug_switches.rs`): `screenvr_display_time.txt`, `screenvr_quad_settings.txt`,
`screenvr_video_path.txt`, `screenvr_stall.txt`, `screenvr_stream_wait.txt`, `screenvr_stream_unused.txt`,
`screenvr_hands_scale.txt`, `screenvr_hands_msaa.txt`, `screenvr_key_probe.txt`, `screenvr_loading.txt`,
`screenvr_loading_preview.txt`, `screenvr_projected_passthrough.txt`, `screenvr_hands.txt`,
`screenvr_refmarkers.txt`, `screenvr_fake_hands.txt`, `screenvr_camera_probe.txt` (test only),
`screenvr_camera_extrinsics.txt` (`fit` = legacy A/B). Written by the client: `screenvr_switches_applied.txt`
(crash-loop guard), `screenvr_stage.txt` (stage identity state; product state), `screenvr_tracking_diag.log`
(+ `.1`; pull with `adb pull /sdcard/Android/data/alvr.client.dev/files/screenvr_tracking_diag.log`).

### Superseded findings (kept below as history; do not act on them)

- The hand-eye **fitted** extrinsics (`CAMERA_EXTRINSICS`, "Key finding" section) are known wrong (SecureMR
  camera pose ~40 mm too high / ~34 mm too far back, wrong roll sign): they were a cause of the misalignment.
  Default is PICO's own set; `fit` is A/B only.
- **Stage identity and the persistent anchor are unreliable** as "STAGE moved" evidence on PICO: the anchor
  moves with a re-created origin and the identity said "unchanged" through every re-creation.
- **The tracking origin is re-created at the head whenever head position tracking is lost and comes back**
  (sleep > ~2 min, app restart, reboot); recenter changes LOCAL only. Use `trackingEpoch` / SETTLED (extras v5).
## Loading screen instead of magenta (2026-11-10 afternoon, BUILT, NOT INSTALLED)

Candidates (copied, not deployed): `C:\Tenstar\VR\Pico\alvr\alvr_client.apk.candidate-76f9e45d`,
`C:\Tenstar\VR\Pico\alvr\driver_candidate_4fbd4a14\` (driver_alvr_server.dll + pdb). Either can be
deployed alone (only RealTimeConfig.ext_str tokens are new; old peers ignore them), but the client
guard needs the new driver's `scene_mode` token.

Recording `D:\wkspaces\ScreenVR\recording\ScreenRecording_2026.11.10-13.31.18.mp4` (t = 0 at
≈13:30:59.7 PC time; frame stats in session scratchpad `loading\stats.tsv`):
- 2.7–9.3 s ChArUco calibration (13:31:02.4–09.0). 37.0 s V3 process detected (stream-unused off),
  passthrough + chroma-keyed stream still shown.
- 41.07 s one black frame, then **41.10–43.83 s full-view magenta (2.73 s)** with the touchscreen
  quad and both hands on top. Server "Scene mode on" 13:31:40.785; ScreenVR "started presenting"
  13:31:40.687 came ~3 s before V3 really drew.
- 43.83–44.17 s magenta fades into V3's own dark dialog ("TOUCH_DIALOG_VR_CALIBRATION_INFO_GEN 3",
  an untranslated V3 string key), shown until 47.9 s; 47.93 s first V3 scene frame; 49.7–50.6 s V3
  fades to black; ~51 s cab view.
- Exit: 85.6–86.45 s V3 fades to black, 86.45–86.63 s black, **86.67–86.97 s magenta (0.3 s)**
  (server "Scene mode off" 13:32:26.517), 87.0–87.4 s screens on black (passthrough being
  recreated), 87.4–87.6 s passthrough back.

Design (client `loading_screen.rs`, server ext tokens, graphics key probe):
- Server: native quads flag `kVrNativeQuadsFlagSceneLoading = 4` (any STAGE epoch, fresh v2 block)
  → ext token `scene_loading`; scene mode → `scene_mode` (with passthrough None as before); tokens
  comma-joined with `stream_unused`.
- Client guard: wanted = scene_loading OR (scene_mode AND no content yet) OR 0.7 s exit hold after
  scene mode ends. "Content" = 6 decoded frames in a row whose sparse 16×16 probe grid is < 50 % key
  colour (chroma within cos 0.85 of the last configured key, magenta by default, magnitude ≥ 0.06:
  catches full and fading magenta, not black/grey/V3 content). A key frame while the opaque stream is
  shown cuts to the loading screen in the same frame. Fades: in 0.3 s, out 0.4 s, min visible 1 s
  (fully faded in). Passthrough stays until the loading screen is opaque; restored on exit under it.
  Timeout 60 s: loading screen gives way to passthrough (touchscreen usable), stream stays hidden
  while it is key colour. Stream-unused skipping is suspended while scene mode/loading (frames must be
  probed). Reconnect: state resets with the stream; flags come back with the next RealTimeConfig.
- Look: Ocean #40505E head-locked 8×8 m quad 1 m ahead (uniform: looks world-locked), Tenstar Green
  ring of 12 dots (spinning, 1.4 s/rev) world-locked 1.5 m ahead at eye height −5 cm, 0.18 m; both
  below quads and hands. Overrides without a build in the app files dir: `screenvr_loading.txt`
  (`background=#RRGGBB`, `indicator=#RRGGBB`, `distance_m`, `size_m`, `period_s`) and
  `screenvr_loading_indicator.png` (rotated while shown).
- Not measured on the headset: the cost of the synchronous 16×16 readback per decoded frame in scene
  mode; the look.

### UX design built in (APK candidate abe90446; driver unchanged = 4fbd4a14; previous APK candidate 76f9e45d kept)

Spec + assets: `client_openxr/loading_assets/` (loading-spec.txt v1 copied from the UX worker;
`loading-indicator-longer-512.png` and `-going-back-512.png` embedded with include_bytes).
- Ocean #40505E full view; ring world-locked 2.0 m ahead, 5° below eye level, head yaw at start;
  quad 0.48 m with a 512 px texture (spec geometry halved: r 201 px, stroke 36 px). Track 18 %
  Aluminium, Green arc with round caps drawn procedurally: length L(t) = 105° − 45°·cos(πt), turned
  clockwise 180°/s by the QUAD POSE (smooth at display rate); the texture (arc length, fades) is
  redrawn at ≤ 30 Hz (~48k band pixels + 1 MiB upload). Unit test: drawn arc/track pixels equal the
  designer's export at the same spots, and the 12 key-frame arc lengths match the spec.
- Fades: Ocean in/out 0.5 s (passthrough → Ocean, Ocean → first frame / passthrough); ring fades in
  0.3 s with scale 0.92 → 1 (ease-out) once the Ocean is 50 % opaque, out 0.3 s with the Ocean;
  minimum visible 0.6 s (spec; with a full fade-in even for short requests, so still no flash).
- Taking longer: no first frame 15 s after the loading screen was wanted → hourglass (cropped from the
  export, 160 px, own quad) fades in 0.5 s and turns over every 4 s (0.6 s ease-in-out).
- Going back: scene (mode or loading flag) ends before any first frame, or 60 s without one → the
  ring stops (quad upright, frozen arc drawn turned), cross-fades to the Orange/arrow export in 0.3 s,
  holds 1.5 s, then the Ocean fades out over passthrough (kept from the trigger on). After a 60 s
  give-up the loading screen stays away until content or the scene ends; magenta stays hidden.
- sRGB: swapchains are SRGB8_ALPHA8 → brand bytes uploaded unchanged (unit test); UNORM fallback
  linearizes. On-display values still to check on the stand (screencap of the Ocean background).
- Key colour check: unchanged decisions; the 16×16 readback now happens right after the staging copy,
  before the stream pass is queued (the stall waits for the copy only). Switch
  `screenvr_key_probe.txt` = `off` (measurement only: magenta is shown again) and log
  `Key probe on|off: N frames ..., stream render() mean/max us` every 5 s; stand script
  `scratchpad\loading\measure_probe.sh` (60 s on/off × 3 with PxrMetric).

### Displayed-view calibration: stand proof (18:02–18:09, APK 0cb31ce8 + driver c5401609)

- Reference markers (client switch `screenvr_refmarkers.txt`, `ref_markers.rs`): PNGs from
  `scratchpad\composited\make_refmarkers.py` (ArUco DICT_4X4_1000 ids 900+, 6 cm, 8 markers: above/
  below the main displays on their planes and 0.2 m in front), drawn world-locked from ONE atlas
  swapchain. PICO limits: only ~8 extra swapchains (24 separate ones failed with
  XR_ERROR_RUNTIME_FAILURE) and the frame layer count (24 markers: "number of specified layers is
  greater than the supported number", every frame dropped) → keep ≤ 8 markers.
- Captures: `adb exec-out screencap` (4320×2160, both eyes, pre-warped) during ScreenVR calibration
  passes (Ctrl+Alt+C), 2–3 captures per pass, boards up for ~2–4 s. Analysis
  `scratchpad\composited\solve.py`: per eye fit (focal, k1, k2, pose) to the 8 markers' 32 corners
  (rms 0.9–1.1 px; f 1270–1278 px, k1 −0.355…−0.365, k2 +0.09…+0.10), ChArUco corners in both eyes,
  stereo triangulation, rigid fit to the board model.
- Displayed − ChArUco (SecureMR) pose, head axes (right, up, back) mm, 4 passes / 5 captures:
  left DELL (1.11 m): (−8.3…−9.7, −33.0…−33.8, −69.9…−73.1); BenQ (1.10 m): (+7.0…+7.3, −37.0…−37.2,
  −43.6…−44.5); MIMO touchscreen (0.97 m): (+12.3…+12.5, −47.0…−47.9, −46.6…−50.1); right DELL
  (1.43 m, edge of view, 8–20 corners, plane fit 10–12 mm): unreliable. ScreenVR's compensation
  shifts for the same pass: left DELL (−9.6, −32.8, −67.3), BenQ (+11.4, −33.8, −68.2), MIMO (+14.3,
  −47.0, −48.8). Repeatability ≤ 1.5 mm in-plane, ≤ 3.5 mm depth. Rotation vs ChArUco 2.2–3.3°
  (main), 9–13° (touchscreen: small board) — stereo plane orientation is noisy, positions are not.
- Without the reference markers (nominal f 1264, k1 −0.39, k2 0.14, pose-only fit): 6.6–10.3 px rms
  and depth changed by 10–40 mm → the reference markers are required.

### Research 2026-11-10 afternoon: calibrating from the passthrough's own source

- Our ChArUco input: SecureMR `rectifiedVSTAccess` (operator 23): the LEFT (or right) RGB VST
  camera, rectified by PICO, at the size we request (2048²), with PICO's 3×3 camera matrix for that
  frame and the capture timestamp; the camera pose comes from the head pose × our per-model
  extrinsics (`camera_frames.rs`, hand-eye calibrated). These are the same physical cameras as
  passthrough, but NOT the displayed passthrough: the user sees PICO's reprojection of these images
  to each eye (with PICO's own depth/mesh), which is where the few-cm passthrough offset comes from.
- Other PICO sources: Enterprise camera API through `com.pvr.tobservice` (installed on this headset):
  `OpenCameraAsyncfor4U` / `StartGetImageDatafor4U` (LEFT, RIGHT, 3D, interlaced; raw or undistorted),
  `GetCameraParametersNewfor4U(width, height)` (fx, fy, cx, cy + left/right extrinsics), frame
  timestamp (exposure start, BOOTTIME) + head pose. Unity-documented ("Project Swan Enterprise"), Java
  service binding for native use. Still raw camera geometry, not the displayed reprojection.
- Composited view: PICO's own recorder (`com.bytedance.pico.screencapture`, system permissions
  CAPTURE_VIDEO_OUTPUT) produces a RECTILINEAR 1920² single-eye view with passthrough and our layers;
  `adb screencap` gives both eyes 4320×2160, lens-pre-warped.
- Third-party MediaProjection, TESTED 18:10 (test APK `com.tenstar.screenvr.capturetest`, source
  `scratchpad\capturetest\`, uninstalled afterwards): consent dialog appears as a 2D panel over the VR
  view; `adb shell appops set <pkg> PROJECT_MEDIA allow` skips it (result OK after 68 ms). But the
  captured display is PICO's 1600×900 "Built-in Screen" (2D shell display) and the frames are BLACK:
  no passthrough, no VR layers. → In-app MediaProjection cannot see the composited view.
- Proof of concept (`scratchpad\composited\detect.py`, recording 13:31:18, t 2.5–9.5 s every 0.5 s,
  DICT_4X4_1000): markers detected in the composited view: BenQ 17/17 per frame (102 total), left
  DELL 17/17 (102), right DELL up to 17 (87), MIMO up to 9 of 12 (17); marker sides 15–33 px.

### Candidates APK b420a911 + driver c5401609 (NOT installed; install TOGETHER: new packets)

- Contract (ScreenVR `VrTrackingExtras.h`): extras v4 `VrTrackedHand hands[2]` (raw tracked wrist,
  palm, index proximal..tip, thumb tip, middle tip in driver space, server receive time, and the
  client's drawing factors); request v3 hand finger correction (`handFingerScale[2]`, sequence,
  near distance, plane in STAGE); request v4 hand display transform (written by the ScreenVR worker;
  semantics in the header). Server forwards v3/v4 as `HandCorrection` / `HandDisplay` packets; the
  client reports `HandShape` (thickness, scale, fingers, applied correction).
- Client: `screenvr_hands.txt` adds `fingers=<0.9..1.3>` (finger bones lengthened from the knuckles,
  palm stays; unit-tested chain continuity) and `display_transform=off` (A/B). Drawing order: raw
  joints → finger length (own setting × correction near the plane, smoothstep fade) → display
  transform (weight from F(raw index tip) to the published quad rectangle, partial transform as in
  the header, glide 0.5 s only when active toggles/stale) → girth/scale. Published joints stay raw.
- PICO joint radii are a constant 15 mm (no information).
- Native quads are composited in block array order (one list; video and static quads alike).

### Candidate 8cb13b5d (installed by the coordinator ~15:12): hand shape switch + joint radii log

- `screenvr_hands.txt` (live): `thickness=<0.5..3>` girth across the bones (joint-local X/Y, joint
  positions and fingertips unchanged), `scale=<0.8..1.3>` uniform about the wrist (moves the
  fingertips), `girth=radii` per-joint girth from the tracked XR_EXT_hand_tracking radii relative to
  the mesh's own (measured at load). Defaults 1 / 1 / mesh = unchanged look.
- New log on each hand's first sight: `Native hands: <hand> joint radii mm (tracked/mesh): j:t/m ...`.
- Findings (screencaps `hands\h1–h4`, mesh analysis `hands\mesh_tip.py`): the drawn glove sits inside
  the real hand outline (palm rows 0.80–0.96 of the real width, skin rim on both sides); mesh fingers
  are thin (distal index half-width 5.1 mm, half-thickness 3.3 mm, i.e. ~10 × 7 mm vs ~16 × 13 mm for
  a real finger); the mesh tip ends at the INDEX_TIP joint (0.5 mm beyond it). Tracked wrist→middle tip
  18.0–19.1 cm (logs), mesh 17.9 cm. Real fingertips extend beyond the drawn ones.

### Installed 14:51:21: APK 0b651677 = cd0b03d2 + projected passthrough PROTOTYPE (switch, default off)

- Rollback: `alvr_client.apk.bak-cd0b03d2` (no prototype). `.bak-a0553304` = first prototype build
  with the passthrough-breaking bug below (do not use).
- Switch `screenvr_projected_passthrough.txt`: `on` (5 cm margin) or a margin in metres (`0.05`, `0`):
  one PROJECTED `XR_FB_passthrough` layer over the normal RECONSTRUCTION layer, with one
  `XR_FB_triangle_mesh` unit square (both windings) and one geometry instance per native quad
  (target pose in STAGE, scaled to the quad size + 2 × margin; moved with
  `xrGeometryInstanceSetTransformFB` when the quads move). Code: `extra_extensions/passthrough_fb.rs`
  (`set_projection`), `native_quads.rs` (`rects`), `lib.rs` (layer order: reconstruction, projected,
  stream, loading, quads, hands).
- PICO OS 5.15.7 findings (14:45–14:52, headset lying on the desk under the monitors):
  - Projected layer, triangle mesh and 4 geometry instances are created without errors; frames are
    accepted (LayerCnt 8). No measurable cost: FrmGpu 6.6 vs 6.2 ms, ATW GPU 2.6 vs 4.1 ms, GPU 84 vs
    79 % (stream rendering in both, n = 17 / 7 s).
  - BUG (fixed in 0b651677): destroying only the projected layer leaves the reconstruction layer broken
    (`PassthroughFB-submit_passthrough_layer_effect passthrough layer count is illegal!`, passthrough
    black until the client restarted). Turning the projection off now recreates the whole passthrough
    object (verified: passthrough back, no errors).
  - NOT yet judged: whether the projected image lines up with the true-pose quads. ScreenVR's
    `debugNativeQuads: 2` did not make the VIDEO quads see-through, so the quads covered the monitors;
    the headset did not see the touchscreen; then someone picked it up. Screencaps `proj\run2\`.
  - Needed for the real test: the headset on the stand facing the monitors, quads see-through or
    hidden (a client "quads at 50 % / outline only" switch does not exist yet), then (a)/(b)/(c) with
    the edge scripts.

### Installed 14:27:55: APK cd0b03d2 (driver 4fbd4a14 since 14:09)

- Rollback: `alvr_client.apk.bak-045b7d74` (loading screen v1, 12 deg ring, "simulation" logo),
  `.bak-71444249` (before the loading screen); driver: `pc_backup_20261110_pre_loading` (84b8011d).
- Smaller defaults after the first user test ("a bit big"): ring outer diameter 8 deg (0.280 m at
  2 m, quad 0.327 m), plain "TENSTAR" logo (`loading_assets/tenstar-logo-alu-2023.png` = original
  "Tenstar_Logo_Alu-2023 (4).png") 5.5 deg wide (0.192 m), centre 0.211 m below the ring centre
  (spec ratio 0.31/0.41 of the ring diameter), 320 px texture.
- Live tuning (debug switch thread, applied while shown): `screenvr_loading.txt` keys
  `background, indicator, distance_m, ring_deg, logo=on|off, logo_deg, below_deg, period_s`;
  `screenvr_loading_preview.txt` = `1`|`normal`|`longer`|`back` shows it over passthrough until
  removed (verified 14:31 with screencaps `loading\caps\preview_*.png`). PNG overrides
  (`screenvr_loading_indicator.png`, `screenvr_loading_logo.png`) are read when the stream starts.
- Quad glide (native_quads.rs): a kept quad (same index or video stream) whose target moves >= 10 mm,
  0.5 deg or 1 % size glides there over 0.5 s (ease in/out); smaller updates are immediate (a small
  update during a glide retargets it). Logged as `Native quads: quad N (stream S) glides ...`.
  First real V3 run (14:15:20) jumped the touchscreen ~68 mm (right +9.3, up -50.0, back -45.3 mm),
  scale 1.040, 1.8 deg from the compensated to the true pose.
- New log: `Loading screen: first frame without the key colour N ms after the loading screen
  appeared (scene loading flag still set|clear)`.
- Black screencaps after a loading screen were the headset falling asleep (static sleep), not the
  client.

## History: deployed 2026-11-10 (superseded by "Deployed (2026-11-11)" above)

- Streamer driver **fe40f746** in `C:\Tenstar\VR\Pico\alvr\pc\alvr_server\bin\win64\driver_alvr_server.dll`
  = HEAD 5fa0ef0b + the UNCOMMITTED working tree (E4 + multi-stream, below).
- **Current 12:55: driver 84b8011d + APK 71444249: STAGE identity + persistent anchor + product
  name (tracking extras v3, request v2; contract `D:\wkspaces\ScreenVR\src\VrTrackingExtras.h`).**
  - Packets: `ClientControlPacket::StageInfo(StageInfo)` (client → server on change and every
    500 ms), `ServerControlPacket::AnchorRequest(AnchorRequest)` (server → client when the overlay's
    `anchorRequestSequence` changes; re-sent on each connection, the client skips the sequence it
    already served).
  - Client `client_openxr/src/stage_identity.rs`: random u64 identity persisted in
    `/sdcard/Android/data/alvr.client.dev/files/screenvr_stage.txt` (also served request, anchor
    UUID, generation, last-seen anchor pose). New identity on: STAGE `ReferenceSpaceChangePending`
    beyond 1 mm / 0.05° (stageFromPrevious = inverse(poseInPreviousSpace)); anchor found ≥3 locates
    in a row beyond 2 mm / 0.2° from where it was last seen (stageFromPrevious = now * last⁻¹); no
    stored identity. productName = `Build.MODEL` = **"A94Y0"** on the PICO 4 Ultra Enterprise
    (ro.product.model says A9210).
  - Client `extra_extensions/persistent_anchor.rs`: XR_EXT_spatial_entity + spatial_anchor +
    spatial_persistence(_operations), LOCAL_ANCHORS scope; discovery by persisted UUID, located
    in STAGE every 500 ms. **PICO needs the runtime permission `com.picovr.permission.SPATIAL_DATA`**
    (now in the manifest and requested at start; granted on the stand with `adb shell pm grant
    alvr.client.dev com.picovr.permission.SPATIAL_DATA`, kept across `install -r`). Without it the
    spatial context fails with ERROR_PERMISSION_INSUFFICIENT → state Unsupported, retried every 10 s.
    **PICO quirk: xrQuerySpatialComponentDataEXT returns 0 entities unless the two-call idiom is
    used** (count first, then exactly sized arrays).
  - Server `tracking_extras.rs`: writes the v3 fields (2456-byte block), reads request v2 only when
    the block's version ≥ 2, maps both sections whole (an old peer's smaller section stays usable).
    Located is downgraded to Lost when no client report confirmed it within 1 s (asleep/disconnected).
  - Measured on the stand (anchor created at the MIMO calibration pose, uuid
    4aec0a914ff0470dab0763ad0c35f48c, request 2), stage identity stayed 2a0c4ec689a0e300 throughout:

    | Event | Anchor vs creation pose | Located after |
    |---|---|---|
    | Reinstall + app restart (×3) | 0.00 mm / 0.000° | ~2 s after contexts ready |
    | Sleep/wake | 0.00 mm / 0.000° | <2 s |
    | Full ALVR/SteamVR restart (epoch reset to 1) | 0.00 mm / 0.000° | 0.5 s |
    | Fresh ScreenVR ChArUco calibration after the restart vs anchor | 0.44 mm / 0.17° | |

    The runtime returns the stored pose bit-identical while tracking. Not tested (need a person):
    recenter, boundary re-setup, tracking loss/relocalisation, reboot.
  - Test tools (session scratchpad): `anchor_tool.py` (read/watch v3 fields, write a v2 request
    block; stop ScreenVROverlay first), `anchor_sample.py`.
  - Not done: keeping the camera warm at headset-on/reconnect.
  - **End-to-end with ScreenVR build_s6 (13:03–13:11):** requests 3–17 each served. Each went
    Creating (state 1), then Lost (3) after ~3.0 s for ~20 ms until the first locate, then Located (2).
    Generation 1→15, stage identity unchanged. On the client relaunches (epochs 6, 7) the server
    re-sent request 14 and the client skipped it (already served); the anchor went Lost→Located in
    ~90 ms. Persisted anchors cannot be listed: the extension can only discover by UUID, and
    MRService's store is not readable without root. The previous UUID is unpersisted after every
    replace, but 3–16 are unverified (logcat rolled). Proposed: an anchor history file + a UUID probe
    switch.
  - 13:02:25/13:02:33 "User presence detected - resetting SteamVR origin" in screenvr.log came from
    a second, older ScreenVROverlay instance (interleaved writers in the log file; two processes seen
    ~12:55). Nothing on the ALVR side recentered; stage identity and anchor unaffected.
  - Headset reboot test: not done (stopped: the user is back).
  - Backups: `pc_backup_20261110_pre_stageid` (driver ac2dc000), `alvr_client.apk.bak-0a0e1e36`.
- 12:19: driver ac2dc000 + APK 0a0e1e36 (camera frames for calibration):
  - Server `CAMERA_FRAME_INTERVAL` 250 → 83 ms. The headset needs ~78 ms per frame: SecureMR
    capture 35, pose+JPEG 41, send 2 (logged every 20 frames as `Camera frames: N frames at F
    fps; per frame ...`).
  - The client no longer sleeps 1 s after the start-up frame stamped with another clock.
  - Measured on the stand:

    | | Before | After |
    |---|---|---|
    | First frame after the pipeline start | 1104 ms | 577 ms |
    | ScreenVR first frame after its request | 2.0 s | 0.9 s |
    | Rate | 4 fps | 12 fps for ~5 s, then ~9 fps sustained (capture/JPEG slow to ~53/57 ms, CPU) |

  - Headset load during a pass is fine (GPU 72–89 % at 492 MHz).
  - ScreenVR's pass is time-based (~10 s), so the real speed-up needs ScreenVR to stop once it has
    ≥8 good frames per monitor: at 9–12 fps that is under 1 s for monitors in view.
  - Backups: `pc_backup_20261110_pre_camrate`, `alvr_client.apk.bak-2df2465f` (instrumented,
    250 ms) and `.bak-043e8f50`.
- **STAGE persistence (12:13–12:16, stand; session scratchpad `stage\stage_result.txt`):** each
  monitor's ChArUco pose in STAGE, compared across events:

  | Event | Translation | Rotation |
  |---|---|---|
  | Two calibrations, same epoch (noise floor) | 0.2–0.3 mm (MIMO 1.0, few frames) | ≤0.09° |
  | Client app restart | ≤1.0 mm | ≤0.07° |
  | Sleep/wake | ≤1.5 mm | ≤0.11° |
  | Full ALVR/SteamVR restart (epoch counter reset to 1) | ≤1.3 mm vs the first baseline | ≤0.24° |

  - STAGE is PICO's runtime stage (`XR_REFERENCE_SPACE_TYPE_STAGE`, boundary/SLAM map) and is
    physically stable across all of these. `stageEpoch` counts PlayspaceSync messages, which the
    client sends at every stream start and every `ReferenceSpaceChangePending`; most bumps do not
    move anything.
  - Not tested (needs a person or a reboot): recenter (long-press home), boundary re-setup,
    relocalisation after the SLAM map is lost, headset reboot.
  - Persistent spatial anchors are offered by the PICO loader (`XR_EXT_spatial_anchor`,
    `XR_EXT_spatial_persistence(_operations)`, `XR_BD_spatial_anchor`), not enabled yet.
- Installed 11:44: APK 043e8f50 = c5a9ca84 plus one fix:
  - Frames of a stream that no quad shows and that has no decoder are dropped, with no decoder and
    no keyframe request (calibration failed, overlay mode, superseded stream id). Before, such
    streams cycled: created on a keyframe, closed 30 s later, keyframes requested.
  - Verified on the headset with 0 quads: 90 s without any screen-video activity, 1 MediaCodec (the
    main stream).
  - c5a9ca84 (installed 11:40): a stream closes only after no quad has shown it for 30 s.
  - Rollbacks: `alvr_client.apk.bak-c5a9ca84`, `.bak-c03c7029`.
- Installed 11:25: APK c03c7029: the mipmapped path is the built-in default for every screen
  (`screenvr_video_path.txt` = `zero-copy` / `auto` for A/B). A new stream is put on the mip path
  before its first keyframe, so it doesn't need a second one.
  - `auto` would pick mipmapped for all four screens at seated distance with the 11:23 ScreenVR
    downscale (BenQ 1920 px ~2.4x, DELL 960 ~2.8x, DELL 720 ~2.1x, MIMO ~3.6x minified at
    ~18 px/deg), so forcing it avoids keyframe-costing auto switches.
  - Verified on the stand 11:25–11:28: MediaCodec count constant at 5 over 3 recalibrations;
    stream-id change decoded 0.19 s after the switch; 960x1706 / 720x1280 / 1920x1080 mip chains
    with GL error 0.
  - Rollback: `alvr_client.apk.bak-ec28e98a`.
- **Candidate c5a9ca84 (built 11:3x, NOT installed)** =
  `C:\Tenstar\VR\Pico\alvr\alvr_client.apk.candidate-c5a9ca84`: c03c7029 plus one fix. A stream
  now closes only when no quad has shown it for 30 s. In c03c7029 a static screen whose quad
  disappeared during a calibration pass closed at once and reopened about 0.2 s after the quads
  returned: a short dark flash per screen on each calibration.
- Candidate ec28e98a (installed 11:05 by the coordinator, superseded) =
  `C:\Tenstar\VR\Pico\alvr\alvr_client.apk.candidate-ec28e98a`, fixing "screens take turns going
  black":
  - The client closed a screen-video stream after 10 s without packets (`STREAM_IDLE_TIMEOUT` in
    `client_openxr/src/screen_video.rs`). ScreenVR encodes on change, so with 30 s keyframes a
    static screen showed for ~10 s and went black (the chessboard fallback) for ~20 s. The server
    has no idle timeout.
  - Now a stream is closed only when no quad references it AND it has been idle for 30 s. A
    referenced stream keeps showing its last decoded frame indefinitely; MediaCodec keeps its
    reference frames, so a P-frame after minutes decodes normally.
  - A quad that references a stream without a decoder (never started, closed, recreated after
    errors, or frames arriving before a keyframe) requests a keyframe at once, at most once a
    second (log `Screen video: stream N has no decoder, requesting a keyframe`). The server already
    requests one for every stream when it starts reading (each client connection) and for each new
    stream id.
  - Stream-unused skip: `skip_frame` now consumes every decoded main-stream frame available (each
    reported as presented) instead of one per loop. Taking one per loop with nothing waiting could
    back up the main decoder's output (`Video frame queue overflow`, then `Decoder saturation`
    drops and RequestIdr). The other trigger was the 10 s screen-decoder close/recreate churn on the
    render thread, which this candidate removes.
  - A main-stream drop in the skipped state behaves as before: RequestIdr → one IDR; no bitrate
    reaction with ConstantMbps.
- **Current (10:35): driver 4167e40f + APK 02c70bc9** = the stream-unused skip (below) plus hands
  work (`graphics/src/hands.rs`, `client_openxr/src/native_hands.rs`):
  - MSAA sample count (1/2/4) and resolution scale are runtime parameters, with live switches
    `screenvr_hands_msaa.txt` / `screenvr_hands_scale.txt` (0.25–1.0); a change rebuilds the
    layer. Defaults are unchanged (0.5 scale, 4x).
  - The layer is not drawn or submitted when no tracked hand is inside either eye's FoV (wrist and
    fingertips, 0.1 rad margin); logged as `Native hands: in view`.
  - Ring of 3 skinned vertex buffers (it made no measurable difference, but it avoids rewriting a
    buffer that may still be in use).
  - The synthetic test hand (`screenvr_fake_hands.txt`) is now placed 35 cm in front of the head,
    in view.
  - Measured on the stand (hand in view, GPU busy at a fixed 456–492 MHz; no hand = 64 %):
    0.5/4x 82–86 % (≈ +2.4 ms); 0.7/2x 84 %; 0.5/2x 79 %; 0.35/2x 74 %; 0.7/1x 72 %; 0.5/1x
    70–71 % (≈ +0.7 ms).
  - Screencap crops (session scratchpad `profile\hand_quality*.png`): 1x is visibly jagged, 2x at
    0.5 shows slight stair-stepping, and 0.7/2x matches 0.5/4x at the same cost. Nothing cheaper
    looks the same, so the defaults stay.
  - PICO's FrmGpu is not the cost: it is wall time from our first GPU command to completion,
    including the wait behind the compositor. With the stream skipped and no hand it reads 0.04 ms;
    with the hand at 1x it reads ~5 ms. Use GPU busy % at a fixed clock for costs.
  - V3 case (stream rendered + hand, 0.5/4x): GPU 96 % at 599 MHz, close to saturation. 0.5/1x:
    86 %.
  - Rollback: `alvr_client.apk.bak-a4e4ed0b`.
- **Stand sleep, final finding (coordinator, 10:21):** with the presence sensor taped, PICO's
  "sleep when static" puts the headset to sleep 5 min 1 s after each wake (`PowerGroup: Sleeping
  power group ... reason=sleep_button`).
  - No settings key controls it (checked global/system/secure); pico-cli only wraps adb; the persist
    props need root.
  - Workaround: keep measurement windows under 5 min from a wake; each wake is a reconnect and a
    recalibration.
  - Real fix: remove the tape (unworn, the psensor sleep is disabled; ScreenVR has
    `requireUserPresence` for its own gate).
- Earlier (10:05): driver 4167e40f + APK a4e4ed0b = stream-unused skip.
  - ScreenVR's NativeQuads v2 flag `kVrNativeQuadsFlagStreamUnused` (2) is read by the server only
    from a fresh, valid v2 block of the current STAGE epoch (`native_quads.rs`, unit test
    `stream_unused_needs_a_fresh_v2_block_of_the_current_epoch`).
  - The server forwards it in `RealTimeConfig.ext_str` = `alvr_packets::EXT_STREAM_UNUSED`: no new
    packet, and changes reach the client within 100 ms.
  - Client (`lib.rs`, `stream.rs` `StreamContext::skip_frame`): no stream pass (and so no chroma-key
    or opaque-region passes) and no stream projection layer. Passthrough, quads and hands continue.
    The loop paces on xrWaitFrame alone.
  - The decoder keeps running: one decoded frame per loop is consumed and reported as presented
    (statistics and server pacing unchanged). When the flag clears, the next frame renders the
    stream again (no IDR wait).
  - Logs: server `Stream unused flag on|off`, client `Stream unused: skipping|rendering the stream`.
  - Switch: `screenvr_stream_unused.txt` = `ignore` renders the stream anyway.
  - Measured on the stand (zero copy, no hand): app GPU 6.78 → 0.74 ms, compositor 5.53 → 4.45 ms,
    GPU busy 85 → 66 %, late frames 28 → 1 per second.
  - With the synthetic hand, GPU busy drops 94 → 84 % (zero copy) and 96 → 83 % (mipmapped). App
    GPU time per frame is distorted by DVFS (the clock drops to 456–492 MHz).
  - Screencaps with the flag honoured vs ignored, taken back to back, are identical. A
    `debugSceneMode` round trip clears the flag and sets scene mode in the same server tick, and
    the stream comes back at once.
  - RISK: while the flag is set, anything else in the stream is invisible (another app's overlay,
    SteamVR notifications); ScreenVR cannot see those.
  - Backups: `pc_backup_20261110_pre_streamunused`, `alvr_client.apk.bak-2cfdbf8b`.
- **The stand "sleep" problem:** the reconnects were Android's screen-off timeout
  (`settings get system screen_off_timeout` = 600000) firing after about 10 min without user
  activity. Wake keyevents don't count as activity; `svc power stayon true` doesn't prevent it, and
  `persist.pvr.sleep_by_static` cannot be set from adb. Each wake = app resume = reconnect = new
  stage epoch = recalibration.
  - For a measurement session: `adb shell settings put system screen_off_timeout 2147483647`.
  - Restore afterwards: `adb shell settings put system screen_off_timeout 600000` (and keep
    `stay_on_while_plugged_in` at 2, its original value). Both are restored now.
- **Stall test (10:11, stand, awake):** 10 × 100 ms render stalls. The quads do not move, flicker or
  vanish: the frame-to-frame change inside the quads at stall frames equals that of other frames,
  and the quad-vs-webcam motion across the textured stalls is ≤0.26 px.
- **Head-motion test for the user:** `python tools/screen_markers/motion_test.py record <out_dir> 20`.
  - The user rotates the headset by hand for 20 s, with the MIMO showing a static, textured picture.
  - Output: quad-vs-real relative motion fitted as a position term (mm per deg at the MIMO) plus a
    velocity term (equivalent lag in ms).
  - Noise floor on a still headset: about 1.2 mm range. The script refuses to fit with less than
    3 deg of motion and warns on textureless patches.
- Earlier: client APK **2cfdbf8b** (installed 09:51 on the stand, soak-tested) on headset PA94Y0MGJ9180002G
  (`alvr.client.dev`) and as `C:\Tenstar\VR\Pico\alvr\alvr_client.apk` = HEAD 5fa0ef0b + the
  UNCOMMITTED working tree, with all the debug switches below (all default = upstream behaviour).
  Compared with cd0cd6a2 it fixes the mipmapped-path crash loop:
  - `client_openxr/src/gl_state.rs` `GlStateGuard` saves and restores every piece of GL state that
    raw GL blocks touch. The mip blit uses its own VAO: wgpu-hal binds ONE VAO at device open and
    never again, and the blit had bound VAO 0, which crashed the next wgpu upload (hands) in the
    driver's memcpy. The native-quad upload is guarded as well.
  - Crash-loop guard in `debug_switches.rs`: non-default switches write
    `screenvr_switches_applied.txt`; a start less than 60 s after the same switches were applied
    ignores the files (logged) until they change. Tested.
  - TEST-ONLY switch `screenvr_fake_hands.txt` = 1: draws a synthetic left hand so that the hands
    uploads run on an unworn headset.
  - Soak test: mipmapped + synthetic hand for about 6 min, no crash, GL error 0. Mipmapped screens
    are upright and the colours match zero copy (screencap).
  - Rollback: `alvr_client.apk.bak-cd0cd6a2`.
- **Profiling 2026-11-10** (stand, details in the session scratchpad `profile\profile_result.txt`):
  - Compositor cost per quad layer: ~0.24 ms (MIMO), ~0.34 ms (large screens). Passthrough layer
    ~1.2 ms.
  - Stream (app GPU): ~3.1 ms plain + ~3.8 ms for the chroma-key/opaque-region passes. Hands
    +1.6 ms.
  - Super-sampling/sharpening flags: no measurable effect.
  - There was no code regression in FrmGpu: the higher worn readings came from the hands, the PICO
    perf HUD (on) and DVFS/thermal.
  - Still headset: the quads are steady to 0.2 px sd over 60 s at 90 fps.
- ScreenVR at that time: `screenVideoKeyframeMs` = 30000, `passthroughModel` "profile" (set by the
  ScreenVR session); headset IPD changed by the user to 59.5 mm.

### Debug switches in the client (2026-11-10, uncommitted, `client_openxr/src/native_quads.rs`)

These are files in the app's external files dir, polled once a second. No reconnect is needed, and
every flip is logged. Git Bash: `export MSYS_NO_PATHCONV=1`.
- **Display time A/B** (the "screens swim" investigation):
  - On: `adb shell "echo predicted > /sdcard/Android/data/alvr.client.dev/files/screenvr_display_time.txt"`
  - Off: `adb shell rm -f /sdcard/Android/data/alvr.client.dev/files/screenvr_display_time.txt`
  - While native quads exist, ON makes `xrEndFrame` use `frame_state.predicted_display_time` instead
    of the stream frame's timestamp (upstream behaviour).
  - Log: `Native quads: display time mode PREDICTED|STREAM`, and once a second
    `stream timestamp - predicted display time: min/mean/max`.
  - Main stream: if PICO derives the projection layer's render pose from displayTime, ON makes the
    stream itself swim. Do not leave it on for V3 unchecked.
- **Newest candidate, BUILT, NOT INSTALLED: APK cd0cd6a2** =
  `C:\Tenstar\VR\Pico\alvr\alvr_client.apk.candidate-cd0cd6a2`. It is adc49f8d plus:
  - A new switch, `screenvr_stream_wait.txt` = fraction of the frame interval the render loop waits
    for a decoded main-stream frame (upstream 0.8 = 8.9 ms at 90 Hz; allowed 0.05–1.0). Lower
    values submit on time with the previous stream frame instead of going late.
  - Every switch change is logged once as `Debug switches: {...}`.
  - The hands layer needed no change: it is already not submitted when no hand is tracked.
  - Not done: skipping the stream layer in front-end mode needs to know that the stream carries
    nothing (no SteamVR overlay, no app). Only ScreenVR knows that; proposal: a NativeQuads block
    flag "stream content not needed" that the client uses to stop rendering/submitting the stream
    layer.
- **The ~10 s late/early bursts** (09:05, after keyframes went to 30 s):
  - In the late seconds the main-stream frame timestamps fall 6–15 ms behind the predicted display
    time (gap log mean -6 to -9 ms; normally ±1 ms), then run 4 ms ahead during the early seconds.
  - The server shows a steady 90 fps with no gaps and no decoder spikes.
  - So it is ALVR's main-stream timing control drifting in phase, with the render loop waiting up
    to 8.9 ms for those frames. It is not a timer of ours.
  - `Failed to import fence` / `Adreno: No valid context present` from the render thread are
    irregular (also on older builds) and not correlated with the bursts.
- Older candidate adc49f8d =
  `C:\Tenstar\VR\Pico\alvr\alvr_client.apk.candidate-mips-stall-adc49f8d`. It supersedes 85aea355
  and adds:
  - All switch files are read by a background thread (`client_openxr/src/debug_switches.rs`),
    never on the render thread.
  - A new switch, `screenvr_stall.txt` = `"<ms> <period ms>"` (e.g. `100 3000`): the render thread
    sleeps before xrEndFrame once per period (log `Debug stall`). It tests what PICO does with quad
    layers on a late frame.
- **Periodic hitch found 2026-11-10 09:00:** the main-stream decoder latency spikes from 6–10 ms to
  23–37 ms for about 200 ms exactly every 2.0 s (autocorrelation 0.77 at 2 s), with network
  latency normal. PICO PxrMetric shows 13–15 late frames every other second.
  - Cause: ScreenVR's `screenVideoKeyframeMs` = 2000 with all four encoders started together, so
    four IDRs (BenQ ~0.5 MB) reach the shared hardware decoder at once.
  - Fix candidates: a much longer keyframe interval plus staggered phases (ScreenVR config/restart;
    ALVR requests IDRs on loss per stream), and the render loop not waiting the full 8.9 ms for
    the main stream while quads are shown.
- **Video decode path** (in candidate adc49f8d, and in the older candidate 85aea355). File `screenvr_video_path.txt`: `zero-copy` (default, = today), `mipmapped` or `auto`.
  - Mipmapped: the decoder output moves with `MediaCodec.setOutputSurface` (no decoder restart, one
    keyframe request) to an AImageReader (PRIVATE, GPU_SAMPLED_IMAGE, 4 images). Each new frame's
    AHardwareBuffer is bound as an external OES texture (the main stream's
    `render_ahardwarebuffer_using_texture`) and drawn into an sRGB swapchain with a full mip chain
    (vertical flip; sRGB decode in the shader so the stored values match the zero-copy picture).
    Then `glGenerateMipmap`, trilinear filtering + max anisotropy set on the images.
  - Work happens only when the reader has a new frame. Pose updates touch nothing.
  - Auto (per stream, once a second): minification = stream width / (quad's angular width at the
    current head distance × display pixels per degree at the view centre, from the recommended
    width and the view FoV). Mipmapped above 2.0, zero copy below 1.5. Above about 2:1, bilinear
    sampling without mips aliases. No per-desk constants.
  - Logs: `Screen video: path mode`, `stream N WxH path Mipmapped|ZeroCopy (forced | auto: Xx
    minified at D m)`, `display P pixels per degree`, `mipmapped target ...`, and every 5 s per
    stream `mipmapped frame ... drawn and mip-mapped in T ms` (glFinish-bracketed sample).
  - UNVERIFIED on the headset: picture orientation (the flip), colour against zero copy, cost, and
    whether PICO's compositor actually samples the swapchain's mip levels.
- **Video quad filtering** (mip step 1): `echo <flags> > .../files/screenvr_quad_settings.txt` chains
  `XrCompositionLayerSettingsFB` to the video quads (2 = quality super-sampling, 1 normal, 4/8
  sharpening; 0 or no file = off). Log: `Native quads: video layer settings flags`. Whether it reduces
  the BenQ aliasing is not judged yet.
- Swim diagnosis (ALVR side, 08:31–08:33): the quads are static in STAGE.
  - 0 in-place pose updates, stage epoch stable, no recenter.
  - Layers are submitted every frame in the stream's STAGE space with the last packet's pose.
  - PICO PxrMetric shows bursty late/early frames (whole seconds of about 80 late, then about 80
    early). The level is the same with 1 or 4 decoders, but higher than on 11-09.
  - Not measured against passthrough: the headset was not worn during the screencap bursts.
- ScreenVR (deployed by the ScreenVR session): `renderer` = "native", `debugNativeQuads` = 0,
  `screenVideo` = true, **`screenVideoScreens` = "all"** (read at ScreenVR start-up only),
  `debugSceneMode` = false.

### Multi-stream SCREEN_VIDEO (2026-11-10, uncommitted)

- `client_core/src/connection.rs`:
  - `ScreenStreamTracker` detects loss per stream from `frame_index`. The server sends each stream's
    frames consecutively, except that it resumes at a keyframe after its own loss. So a gap before a
    non-keyframe, or a stream starting without a keyframe, means only THAT stream waits for a
    keyframe and requests one.
  - The channel's `had_packet_loss()` is only counted and logged, no longer blamed on the stream whose
    packet arrives next. A lost frame of any stream shows up as a gap in that stream's own indices.
  - Queue overflow: every stream that lost queued frames is marked and gets its own keyframe request.
  - The SCREEN_VIDEO channel has 32 reassembly buffers (was 10, shared by all streams).
  - Unit test: `screen_stream_loss_is_tracked_per_stream`.
- `server_core/src/bitrate.rs` + `connection.rs`:
  - The screen thread measures the screen-video bitrate (4 s window, smoothed). The main stream's
    bitrate is lowered by it, keeping at least 50 %, with 2 Mbps hysteresis.
  - Log: `Screen video: X Mbps, subtracted from the main stream bitrate`.
  - The MEASURED rate is used, not ScreenVR's caps: the caps add up to 35.6 Mbps with "all", more
    than the whole 30 Mbps ConstantMbps setting, while static desktops use ~4–7 Mbps.
  - Unit test: `screen_video_lowers_the_main_bitrate_down_to_a_minimum`.
- Verified with `screenVideoScreens: "all"` (MIMO 1280x800@60, 2 x DELL 1440x2560@8, BenQ
  3840x2160@8, all H.264):
  - The 4 NVIDIA H.264 MFT sessions start next to ALVR's encoder.
  - PICO decodes all 4 at once, including 3840x2160 H.264. Note that ScreenVR encodes the BenQ
    landscape, not 2160x3840.
  - Screencaps show live desktops on all four quads.
  - No loss or keyframe-request lines on either side. In scene mode only the MIMO remains, and the
    others return without decoder restarts.
- Main stream with all four screens vs scene mode (MIMO only), 8 s averages from `/api/events`:
  - 90 fps both.
  - Decoder 6.3 vs 5.4 ms; decoder queue 13.0 vs 9.7 ms; total pipeline 59 vs 48 ms.
  - Requested bitrate 23.7 Mbps (30 minus screens).
  - A few `Video frame queue overflow` bursts happen only at structural changes (decoder creation,
    scene switch: render-thread stalls of well under 1 s), none in steady state.
- **Proposed, not implemented: client-decoded acknowledgement** (needs the VrScreenVideo.h
  contract, v2). `VrScreenVideoStream` has no spare bytes, so append
  `struct VrScreenVideoClientAck { uint32_t streamId; uint32_t decodedFrames; uint64_t lastDecodedFrameIndex; } acks[kVrScreenVideoMaxStreams];`
  after `streams[]` (rings move to 8544), and bump the version to 2.
  - The client sends `ClientControlPacket::ScreenVideoDecoded { stream_id, frame_index }` at a
    stream's first decoded frame and then at most once a second (`screen_video.rs` `pump`).
  - The server writes the ack slot of that stream id (`screen_video.rs`).
  - ScreenVR (`ScreenRenderPolicy.h`) draws a screen natively only once `acks[i].streamId ==
    streamId && lastDecodedFrameIndex > 0`, and falls back to the overlay if it stops advancing
    while frames are produced.
- Note: the commit table above stops at Friday; 2026-11-09 added hands (cce6e3e0, 77ceae8c) and E1
  screen video + NativeQuads v2 (5fa0ef0b).

### Uncommitted work in the tree (2026-11-10, left uncommitted on request)

- **E4: scene mode turns passthrough and chroma key off** (`server_core/src/native_quads.rs`,
  `server_core/src/connection.rs`; mostly written by the previous session, and its first build,
  eba7d911, was deployed 11-09 17:06 untested).
  - `NativeQuadsReader::scene_mode()` is true for a fresh, valid v2 block with
    `kVrNativeQuadsFlagSceneMode`, whatever the STAGE epoch.
  - It is false when the block is missing, invalid, v1 or stale (writer crashed or hung, >1000 ms).
  - If the seqlock read keeps failing, the previous quads and state are kept.
  - The screen thread updates it every 100 ms. The real-time config thread (now every 100 ms, was 1 s)
    sends `RealTimeConfig { passthrough: None }` while it is set, and the configured passthrough
    when it clears.
  - The client already handled live passthrough changes: it drops or recreates the passthrough layer,
    and the stream layer becomes opaque (no chroma key, no opaque regions).
  - Logs: server `Scene mode on: passthrough and chroma key off` / `Scene mode off: passthrough
    restored (configured)`; client `Opaque regions: enabled false` and PICO `oxr_passthrough_destroy`.
  - Unit test: `scene_mode_needs_a_fresh_v2_block_with_the_flag`.
- **E2: pose/size-only changes are applied in place** (`client_openxr/src/native_quads.rs`).
  - HEAD already kept the swapchains, but only when ALL quads kept the same texture.
  - Now each quad keeps its layer when the quad at the same index, or with the same video stream id,
    has the same chessboard. Scene-mode switches therefore keep the touchscreen's layer too.
  - Screen video decoders were, and still are, independent of the quads (keyed by stream id in
    `screen_video.rs`).
  - Logs: `Native quads: N quad layer(s) (K kept, R removed)` on structural changes, and
    `Native quads: N pose/size update(s) applied in place to M layer(s), no swapchain recreated` at
    most every 5 s.
- **Verified 2026-11-10** (headset on the desk, unattended):
  - `physical.debugSceneMode` switched on and off several times.
  - ScreenVROverlay.exe suspended for 5 s (stale block): `Native quads: Stale` + `Scene mode off`,
    passthrough back in the screencap, and scene mode on again after resume.
  - Pose-only: `vstOffsetUpMeters` stepped through the config hot-reload gave in-place log lines,
    with no new swapchain and no decoder restart. `physical_trim.txt` is NOT hot-reloaded.
- **Found: SecureMR probably stops while passthrough is off.** PICO's `com.bytedance.pico.openmr`
  (SecureMR) logs `sess wait for vst status 1 ... mrservice state = 0` every second. Camera frames
  (ChArUco) therefore most likely do NOT run in scene mode: calibrate before scene mode, or turn VST
  on briefly for a pass. Not tested: no ChArUco pass was requested during scene mode.
- **One unexplained capture.** About 3 s after a scene-mode switch, which came 10 s after a
  reconnect while a camera pass was ending, a screencap still showed passthrough although the client
  had already destroyed the passthrough layer. Every steady-state switch showed opaque magenta
  within 2 s.

### Rollback

| Target | Streamer (copy over `pc\`) | APK |
|---|---|---|
| Before extras v5 / tracking epoch (driver 61de28d6, APK 2ce2f6e4) | `pc_backup_20261111_pre_extras5` | `alvr_client.apk.bak-2ce2f6e4` |
| Before the tracking diagnostics (APK b51e35d1) | unchanged (driver 61de28d6) | `alvr_client.apk.bak-b51e35d1` |
| Before the PICO-default extrinsics (driver c5401609) | `pc_backup_20261111_pre_picodefault` | `alvr_client.apk.bak-146b41f6` |
| Before stage identity/anchor (driver ac2dc000, APK 0a0e1e36) | `pc_backup_20261110_pre_stageid` | `alvr_client.apk.bak-0a0e1e36` |
| Before the camera rate change (driver 4167e40f) | `pc_backup_20261110_pre_camrate` | `alvr_client.apk.bak-043e8f50` |
| Before the hands work (APK 02c70bc9) | unchanged (driver 4167e40f) | `alvr_client.apk.bak-a4e4ed0b` |
| Before the stream-unused skip (driver 4167e40f, APK a4e4ed0b) | `pc_backup_20261110_pre_streamunused` (= fe40f746) | `alvr_client.apk.bak-2cfdbf8b` |
| Before 2cfdbf8b (GL-state fix, crash guard) | unchanged (driver fe40f746) | `alvr_client.apk.bak-cd0cd6a2` (has the mipmapped crash) |
| Before cd0cd6a2 (mips, stall, stream-wait switches) | unchanged (driver fe40f746) | `alvr_client.apk.bak-dd8f5216` |
| Before the debug switches (multi-stream: driver fe40f746, APK 592c7e78) | `pc_backup_20261110_pre_mips` (= fe40f746) | `alvr_client.apk.bak-20261110_pre_mips-592c7e78` |
| Before multi-stream (E4 + E2 layer reuse: driver 3908fa03, APK 8c52b4ee) | `pc_backup_20261110_pre_multistream` | `alvr_client.apk.bak-20261110_pre_multistream-8c52b4ee` |
| Before 2026-11-10 (E1 + first E4 server build eba7d911) | `pc_backup_20261110_pre_quadreuse` | `alvr_client.apk.bak-395590af` (= 5fa0ef0b) |
| Before E4 server (E1 only, driver 891f95d5) | `pc_backup_20261109_pre_scenemode` | `alvr_client.apk.bak-395590af` |
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

## Key finding: SecureMR's LOCAL is not always the app's LOCAL (2026-11-06; the fitted extrinsics below are SUPERSEDED, see top)

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

## Contracts

See `D:\wkspaces\ScreenVR\docs\AlvrInterface.md` (layouts, versions, writers/readers, packets, switch files,
compatibility). The contract headers are ScreenVR's `src/Vr*.h`; the old description that stood here was stale.

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
