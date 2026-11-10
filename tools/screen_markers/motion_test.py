"""Quad-vs-real motion under head rotation, from a 90 fps headset screenrecord.

Usage:
  python motion_test.py record <out_dir> [seconds=20]     record (rotate the headset by hand meanwhile)
  python motion_test.py analyze <video.mp4> [qx qy rx ry] [ppd=20]

record: 3 s countdown, then `adb shell screenrecord` (90 fps, full 4320x2160) plus logcat into
<out_dir>, then analyze. Rotate the headset slowly left-right and up-down, keeping the MIMO and the
desk in view.

analyze: right eye (left half of the frame). Tracks two patches FRAME TO FRAME (re-centred every
frame, so they follow large motion): a QUAD patch (default: centre of the MIMO quad) and a REAL patch
(default: desk just below the MIMO). Per frame:
  head angle  = real patch image position / ppd (deg; the real world is fixed, so its image motion is
                the head rotation), velocity = d(angle)/dt
  relative    = quad position - real position (px): 0 for a quad perfectly locked to the real world
Fit per axis: relative = a * angle + b * velocity + c
  a: position-proportional offset (geometry: passthrough reprojection vs world-locked quad), reported
     as px/deg, deg/deg and mm/deg at the MIMO distance
  b: velocity-proportional lag, reported as an equivalent latency in ms (positive: the quad trails)
The quad content must be static while recording (a still desktop on the MIMO).
"""
import json, os, subprocess, sys, time
import numpy as np

DEVICE = ['adb', '-s', 'PA94Y0MGJ9180002G']
SCALE = 2  # analysis on a 2x downscaled right eye (1080x1080)
PATCH = 96  # patch size in downscaled px
MIMO_DISTANCE_M = 0.65
MIN_TEXTURE = 0.25  # first-second median phase-correlation peak below this: patch unusable


def record(out_dir, seconds):
    os.makedirs(out_dir, exist_ok=True)
    video = os.path.join(out_dir, 'motion.mp4')
    subprocess.run(DEVICE + ['shell', 'input', 'keyevent', 'KEYCODE_WAKEUP'])
    log = open(os.path.join(out_dir, 'logcat_motion.txt'), 'wb')
    lc = subprocess.Popen(DEVICE + ['logcat', '-T', '1'], stdout=log, stderr=subprocess.STDOUT)
    for k in (3, 2, 1):
        print(f'start rotating in {k}...', flush=True)
        time.sleep(1)
    print(f'RECORDING {seconds} s: rotate the headset slowly left-right, then up-down', flush=True)
    subprocess.run(DEVICE + ['shell', f'screenrecord --time-limit {seconds} --bit-rate 40000000 '
                                      '/sdcard/motion.mp4'])
    print('done, pulling', flush=True)
    lc.terminate()
    log.close()
    subprocess.run(DEVICE + ['pull', '/sdcard/motion.mp4', video], check=True)
    subprocess.run(DEVICE + ['shell', 'rm', '-f', '/sdcard/motion.mp4'])
    return video


def load_frames(video):
    raw = video + '.gray'
    side = 2160 // SCALE
    subprocess.run(['ffmpeg', '-v', 'error', '-y', '-i', video, '-vf',
                    f'crop=2160:2160:0:0,scale={side}:{side},format=gray', '-fps_mode', 'passthrough',
                    '-f', 'rawvideo', '-pix_fmt', 'gray', raw], check=True)
    frames = np.fromfile(raw, np.uint8).reshape(-1, side, side)
    os.remove(raw)
    return frames


def phase_shift(a, b):
    win = np.outer(np.hanning(a.shape[0]), np.hanning(a.shape[1]))
    fa = np.fft.fft2((a - a.mean()) * win)
    fb = np.fft.fft2((b - b.mean()) * win)
    r = fa * np.conj(fb)
    r /= np.abs(r) + 1e-9
    c = np.fft.ifft2(r).real
    h, w = c.shape
    y, x = np.unravel_index(np.argmax(c), c.shape)

    def sub(arr, i, n):
        l, m, rr = arr[(i - 1) % n], arr[i], arr[(i + 1) % n]
        d = l - 2 * m + rr
        return i + (0.5 * (l - rr) / d if d != 0 else 0)
    xs, ys = sub(c[y, :], x, w), sub(c[:, x], y, h)
    if xs > w / 2:
        xs -= w
    if ys > h / 2:
        ys -= h
    return -float(xs), -float(ys), float(c[y, x])


def track(frames, x, y):
    """Patch centre per frame (downscaled px), following it frame to frame."""
    half = PATCH // 2
    pos = [(float(x), float(y))]
    conf = [1.0]
    side = frames.shape[1]
    template = frames[0, int(y) - half:int(y) + half, int(x) - half:int(x) + half].astype(np.float64)
    for i in range(1, len(frames)):
        px, py = pos[-1]
        cx = int(round(min(max(px, half), side - half)))
        cy = int(round(min(max(py, half), side - half)))
        prev = frames[i - 1, cy - half:cy + half, cx - half:cx + half].astype(np.float64)
        cur = frames[i, cy - half:cy + half, cx - half:cx + half].astype(np.float64)
        dx, dy, c = phase_shift(prev, cur)
        # Re-anchor on the first frame's template at the predicted position, so that per-frame
        # sub-pixel errors do not accumulate (drift)
        qx, qy = px + dx, py + dy
        ax = int(round(min(max(qx, half), side - half)))
        ay = int(round(min(max(qy, half), side - half)))
        here = frames[i, ay - half:ay + half, ax - half:ax + half].astype(np.float64)
        rx, ry, rcf = phase_shift(template, here)
        if rcf > 0.3 and abs(ax + rx - qx) < 3 and abs(ay + ry - qy) < 3:
            qx, qy = ax + rx, ay + ry
        pos.append((qx, qy))
        # Weak template match = the content changed or the patch is lost: excluded from the fit
        conf.append(min(c, rcf))
    return np.array(pos), np.array(conf)


def analyze(video, quad_xy=(700, 705), real_xy=(700, 800), ppd=20.0, fps=90.0):
    frames = load_frames(video)
    n = len(frames)
    quad, qc = track(frames, *quad_xy)
    real, rc = track(frames, *real_xy)
    ppd_s = ppd / SCALE  # px per degree in the downscaled image
    angle = (real - real[0]) / ppd_s  # head rotation as seen by the real world's image motion (deg)
    # Relative motion integrated from per-frame steps where both matches are confident: a content
    # change on the screen (low confidence) contributes nothing instead of a permanent jump
    # Both tracks are re-anchored on their first-frame templates (no drift); frames where either
    # match is weak (screen content change, occlusion) are left out of the fit
    # Relative to each track's typical match strength (low-texture patches peak lower)
    # (taken over the first second, before any content change)
    step_ok = (qc > 0.5 * np.median(qc[1:91])) & (rc > 0.5 * np.median(rc[1:91]))
    rel = (quad - real) - (quad[0] - real[0])  # px (downscaled)
    vel = np.gradient(angle, axis=0) * fps  # deg/s
    good = step_ok
    out = [f'{video}: {n} frames, {good.sum()} with confident tracking; ppd {ppd} (full res), '
           f'MIMO at ~{MIMO_DISTANCE_M} m']
    out.append(f'head rotation range: yaw {np.ptp(angle[good, 0]):.1f} deg, pitch '
               f'{np.ptp(angle[good, 1]):.1f} deg; peak velocity {np.abs(vel[good]).max():.0f} deg/s')
    out.append(f'frames left out (screen content change or lost tracking): {(~step_ok).sum()}; '
               f'first-second match strength quad {np.median(qc[1:91]):.2f}, real {np.median(rc[1:91]):.2f}')
    if min(np.median(qc[1:91]), np.median(rc[1:91])) < MIN_TEXTURE:
        out.append('WARNING: a patch has too little texture (e.g. a dark window on the MIMO): the result '
                   'is not valid. Show a textured, static picture (the wallpaper) or pass other patch '
                   'coordinates.')
    mm_per_deg = MIMO_DISTANCE_M * 1000 * np.pi / 180
    if max(np.ptp(angle[good, 0]), np.ptp(angle[good, 1])) < 3:
        out.append('NOT ENOUGH HEAD MOTION (< 3 deg): no fit; relative shift range '
                   f'{np.ptp(rel[good, 0]) / ppd_s * mm_per_deg:.2f} / '
                   f'{np.ptp(rel[good, 1]) / ppd_s * mm_per_deg:.2f} mm (x / y)')
        print('\n'.join(out))
        return
    for axis, name in [(0, 'horizontal (yaw)'), (1, 'vertical (pitch)')]:
        A = np.vstack([angle[good, axis], vel[good, axis], np.ones(good.sum())]).T
        coef, *_ = np.linalg.lstsq(A, rel[good, axis], rcond=None)
        resid = rel[good, axis] - A @ coef
        a, b, _ = coef
        a_deg = a / ppd_s  # deg of relative shift per deg of head rotation
        lag_ms = -b / ppd_s * 1000  # relative px per (deg/s) -> seconds of lag
        out.append(f'{name}: relative shift range {np.ptp(rel[good, axis]) / ppd_s:.3f} deg; '
                   f'position term {a_deg:+.4f} deg/deg = {a_deg * mm_per_deg:+.2f} mm per deg at the MIMO; '
                   f'velocity term = {lag_ms:+.1f} ms equivalent lag (positive: the quad trails the '
                   f'real world); residual sd {resid.std() / ppd_s * mm_per_deg:.2f} mm')
    out.append('A quad locked to the real world gives ~0 for both terms. Lag terms of a few ms are '
               'within the screenrecord timing noise; tens of ms would be a latency problem.')
    text = '\n'.join(out)
    print(text)
    base = os.path.splitext(video)[0]
    open(base + '_result.txt', 'w').write(text + '\n')
    np.savetxt(base + '_series.csv', np.column_stack([np.arange(n) / fps, angle, vel, rel / ppd_s, qc, rc]),
               delimiter=',', header='t,yaw_deg,pitch_deg,yaw_vel,pitch_vel,rel_x_deg,rel_y_deg,quad_conf,real_conf',
               comments='')


if __name__ == '__main__':
    if sys.argv[1] == 'record':
        seconds = int(sys.argv[3]) if len(sys.argv) > 3 else 20
        analyze(record(sys.argv[2], seconds))
    else:
        args = sys.argv[3:]
        kwargs = {}
        if len(args) >= 4:
            kwargs['quad_xy'] = (float(args[0]), float(args[1]))
            kwargs['real_xy'] = (float(args[2]), float(args[3]))
        if len(args) >= 5:
            kwargs['ppd'] = float(args[4])
        analyze(sys.argv[2], **kwargs)
