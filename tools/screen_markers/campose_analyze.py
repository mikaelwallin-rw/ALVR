"""Analyze CAMPOSE_DIAG lines: which space is SecureMR's LOCAL, and the camera->view extrinsic.

A_i = stage_from_view(t_i)          (app head pose, STAGE)
L_i = stage_from_local(t_i)         (app LOCAL in STAGE)
B_i = securemr_local_from_cam(t_i)  (OpenCV camera axes)

H1: SecureMR LOCAL == app LOCAL  ->  C_i = inv(A_i) * L_i * B_i is constant (= view_from_cam)
H2: SecureMR LOCAL = fixed unknown X in STAGE: A_i * Y = X * B_i  (hand-eye, Y = view_from_cam)
"""
import re
import sys

import numpy as np

POSE = r"p=\(([-\d.]+),([-\d.]+),([-\d.]+)\) q=\(([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+)\)"
LINE = re.compile(
    r"CAMPOSE_DIAG (?:eye=(\w+) )?t=(\d+) stage_from_view " + POSE + r" \| stage_from_local " + POSE
    + r" \| securemr_local_from_cam_cv " + POSE
)


def quat_to_mat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def pose(values):
    p = np.array(values[:3], dtype=float)
    q = np.array(values[3:], dtype=float)
    q /= np.linalg.norm(q)
    T = np.eye(4)
    T[:3, :3] = quat_to_mat(*q)
    T[:3, 3] = p
    return T


def angle_deg(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


def mat_to_quat(R):
    # (w, x, y, z)
    w = np.sqrt(max(0.0, 1 + np.trace(R))) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return np.array([w, x, y, z])


def left(q):
    w, x, y, z = q
    return np.array([[w, -x, -y, -z], [x, w, -z, y], [y, z, w, -x], [z, -y, x, w]])


def right(q):
    w, x, y, z = q
    return np.array([[w, -x, -y, -z], [x, w, z, -y], [y, -z, w, x], [z, y, -x, w]])


def quat_to_mat_wxyz(q):
    w, x, y, z = q / np.linalg.norm(q)
    return quat_to_mat(x, y, z, w)


def hand_eye(As, Bs):
    """Solve A_i Y = X B_i: relative motions A_ij Y = Y B_ij with A_ij = inv(A_i) A_j."""
    rows, motions = [], []
    for i in range(len(As)):
        for j in range(i + 1, len(As)):
            Aij = np.linalg.inv(As[i]) @ As[j]
            Bij = np.linalg.inv(Bs[i]) @ Bs[j]
            if angle_deg(Aij[:3, :3]) < 3:
                continue
            qa, qb = mat_to_quat(Aij[:3, :3]), mat_to_quat(Bij[:3, :3])
            if qa[0] * qb[0] < 0:
                qb = -qb
            rows.append(left(qa) - right(qb))
            motions.append((Aij, Bij))
    if len(motions) < 2:
        return None
    _, _, vt = np.linalg.svd(np.vstack(rows))
    Ry = quat_to_mat_wxyz(vt[-1])
    M = np.vstack([Aij[:3, :3] - np.eye(3) for Aij, _ in motions])
    v = np.concatenate([Ry @ Bij[:3, 3] - Aij[:3, 3] for Aij, Bij in motions])
    ty = np.linalg.lstsq(M, v, rcond=None)[0]
    Y = np.eye(4)
    Y[:3, :3], Y[:3, 3] = Ry, ty
    return Y, len(motions)


def residual_angles(Y):
    """view_from_cam minus the pure OpenXR->OpenCV flip: (roll about optical axis, pitch, yaw) deg"""
    R = Y[:3, :3] @ np.diag([1.0, -1.0, -1.0])
    roll = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    pitch = np.degrees(np.arctan2(-R[1, 2], R[2, 2]))
    yaw = np.degrees(np.arcsin(np.clip(R[0, 2], -1, 1)))
    return roll, pitch, yaw


def main(path):
    all_samples = []
    for line in open(path, encoding="utf-8", errors="replace"):
        m = LINE.search(line)
        if m:
            eye = m.group(1) or "Left"
            g = [float(x) for x in m.groups()[1:]]
            all_samples.append((eye, int(g[0]), pose(g[1:8]), pose(g[8:15]), pose(g[15:22])))
    for eye in sorted({s[0] for s in all_samples}):
        samples = [s[1:] for s in all_samples if s[0] == eye]
        print(f"\n===== eye {eye}: {len(samples)} samples =====")
        analyze(samples)


def analyze(samples):
    As = [s[1] for s in samples]
    Ls = [s[2] for s in samples]
    Bs = [s[3] for s in samples]

    print("\nH1: C_i = inv(A_i) L_i B_i (should be constant, small)")
    Cs = [np.linalg.inv(A) @ L @ B for A, L, B in zip(As, Ls, Bs)]
    for C in Cs:
        print(f"  t={np.round(C[:3, 3], 3)}  rot={angle_deg(C[:3, :3]):6.2f} deg")
    print("  head yaw/pitch span (deg):",
          round(max(angle_deg(np.linalg.inv(As[0])[:3, :3] @ A[:3, :3]) for A in As), 1))
    print("  stage_from_local t:", np.round(Ls[0][:3, 3], 3), " head t:", np.round(As[0][:3, 3], 3))

    result = hand_eye(As, Bs)
    if result is None:
        print("\nH2: not enough head rotation for hand-eye")
        return
    Y, count = result
    Xs = [A @ Y @ np.linalg.inv(B) for A, B in zip(As, Bs)]
    X = Xs[0]
    print(f"\nH2 (hand-eye over {count} motions): view_from_cam Y t={np.round(Y[:3, 3], 4)} "
          f"rot={angle_deg(Y[:3, :3]):.2f} deg")
    print("  stage_from_securemr_local X per sample (constant if H2 holds):")
    for Xi in Xs:
        print(f"    t={np.round(Xi[:3, 3], 3)}  rot vs X0={angle_deg(X[:3, :3].T @ Xi[:3, :3]):5.2f} deg")
    errs = [np.linalg.norm((A @ Y)[:3, 3] - (X @ B)[:3, 3]) for A, B in zip(As, Bs)]
    print("  residual |A Y - X B| (m):", np.round(errs, 4))
    roll, pitch, yaw = residual_angles(Y)
    print(f"  camera vs VIEW (after axis flip): roll {roll:+.2f} deg, pitch {pitch:+.2f}, yaw {yaw:+.2f}")
    C0 = np.linalg.inv(As[0]) @ Ls[0] @ Bs[0]
    print("  H1 C roll/pitch/yaw:", np.round(residual_angles(C0), 2))
    print("\n  Y matrix (view_from_cam, OpenCV camera axes):")
    print(np.round(Y, 5))


if __name__ == "__main__":
    main(sys.argv[1])
