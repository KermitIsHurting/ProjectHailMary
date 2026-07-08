#!/usr/bin/env python3
"""Radar-camera extrinsic calibration solver (OVERHAUL P1.2).

Solves the SE(3) transform  p_cam = R * p_radar + t  from paired
observations of a single strong reflector (corner reflector) moved
through the shared field of view:

    radar 3D cluster centroid (x, y, z) [m]  <->  image pixel (u, v) [px]

Ground equipment, not flight code: plain Python + numpy, runs offline.

Input CSV (one pair per line, '#' comments allowed):

    x_m, y_m, z_m, u_px, v_px

Collect >= 8 pairs spread across the full FOV and depth range; each pair
contributes two reprojection equations against six unknowns, and spatial
spread is what separates rotation from translation.

Pixels must be undistorted (or the target kept near the image centre);
this solver models a pure pinhole. Intrinsics default to the AR0234
checkerboard calibration in constants.hpp (2026-04-09).

Output: a ready-to-paste config/extrinsics.yaml block plus per-point
residuals. The RMS residual IS the floor for any pixel-space association
gate downstream — record it next to the values.

Self-test (synthetic truth, no hardware):  ./calibrate_extrinsics.py --self-test
"""

import argparse
import sys

import numpy as np

# AR0234 intrinsics, constants.hpp (checkerboard calibration 2026-04-09).
DEFAULT_FX = 1862.7
DEFAULT_FY = 1877.7
DEFAULT_CX = 1032.8
DEFAULT_CY = 426.8

# Nominal mount: +90 deg about radar x = radar->camera axis convention.
R_NOMINAL = np.array([[1.0, 0.0, 0.0],
                      [0.0, 0.0, -1.0],
                      [0.0, 1.0, 0.0]])

MAX_ITERS = 50
STEP_TOL = 1e-10


def skew(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def exp_so3(theta):
    """Rodrigues: rotation matrix from an axis-angle vector."""
    angle = np.linalg.norm(theta)
    if angle < 1e-12:
        return np.eye(3) + skew(theta)
    axis = theta / angle
    k = skew(axis)
    return np.eye(3) + np.sin(angle) * k + (1.0 - np.cos(angle)) * (k @ k)


def rot_to_quat_wxyz(rot):
    """Quaternion (w,x,y,z) from a rotation matrix, w >= 0."""
    tr = np.trace(rot)
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        q = np.array([0.25 * s,
                      (rot[2, 1] - rot[1, 2]) / s,
                      (rot[0, 2] - rot[2, 0]) / s,
                      (rot[1, 0] - rot[0, 1]) / s])
    else:
        i = int(np.argmax(np.diag(rot)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(rot[i, i] - rot[j, j] - rot[k, k] + 1.0) * 2.0
        q = np.empty(4)
        q[0] = (rot[k, j] - rot[j, k]) / s
        q[1 + i] = 0.25 * s
        q[1 + j] = (rot[j, i] + rot[i, j]) / s
        q[1 + k] = (rot[k, i] + rot[i, k]) / s
    if q[0] < 0.0:
        q = -q
    return q / np.linalg.norm(q)


def project(points_cam, fx, fy, cx, cy):
    """Pinhole projection of Nx3 camera-frame points to Nx2 pixels."""
    z = points_cam[:, 2]
    return np.column_stack((fx * points_cam[:, 0] / z + cx,
                            fy * points_cam[:, 1] / z + cy))


def residuals(rot, t, radar_pts, pixels, fx, fy, cx, cy):
    """Stacked 2N reprojection residuals (projected - measured)."""
    return (project(radar_pts @ rot.T + t, fx, fy, cx, cy) - pixels).ravel()


def solve_extrinsics(radar_pts, pixels, fx, fy, cx, cy,
                     rot0=None, t0=None, verbose=True):
    """Levenberg-damped Gauss-Newton over SE(3), left perturbation.

    Update model:  R <- Exp(dtheta) R,  t <- Exp(dtheta) t + dt,
    so  d p_cam / d dtheta = -[p_cam]x  and  d p_cam / d dt = I.
    Returns (R, t, rms_px, per_point_residual_px).
    """
    rot = R_NOMINAL.copy() if rot0 is None else rot0.copy()
    t = np.zeros(3) if t0 is None else t0.copy()
    n = radar_pts.shape[0]
    lam = 1e-4

    def cost(rot_c, t_c):
        r = residuals(rot_c, t_c, radar_pts, pixels, fx, fy, cx, cy)
        return float(r @ r), r

    c_prev, r = cost(rot, t)
    for it in range(MAX_ITERS):
        p_cam = radar_pts @ rot.T + t
        z = p_cam[:, 2]
        if np.any(z <= 0.0):
            raise ValueError(
                "point behind camera during iteration — check pair data "
                "or initial guess")
        # Jacobian of pixel wrt p_cam, then wrt (dtheta, dt).
        jac = np.zeros((2 * n, 6))
        for i in range(n):
            x, y, zz = p_cam[i]
            dpix = np.array([[fx / zz, 0.0, -fx * x / (zz * zz)],
                             [0.0, fy / zz, -fy * y / (zz * zz)]])
            jac[2 * i:2 * i + 2, 0:3] = dpix @ (-skew(p_cam[i]))
            jac[2 * i:2 * i + 2, 3:6] = dpix
        g = jac.T @ r
        h = jac.T @ jac
        step_ok = False
        for _ in range(8):
            try:
                delta = np.linalg.solve(h + lam * np.eye(6), -g)
            except np.linalg.LinAlgError:
                lam *= 10.0
                continue
            rot_new = exp_so3(delta[0:3]) @ rot
            t_new = exp_so3(delta[0:3]) @ t + delta[3:6]
            c_new, r_new = cost(rot_new, t_new)
            if c_new < c_prev:
                rot, t, c_prev, r = rot_new, t_new, c_new, r_new
                lam = max(lam * 0.3, 1e-9)
                step_ok = True
                break
            lam *= 10.0
        if verbose:
            print(f"  iter {it:2d}  rms {np.sqrt(c_prev / (2 * n)):9.4f} px"
                  f"  lambda {lam:.1e}")
        if not step_ok or np.linalg.norm(delta) < STEP_TOL:
            break

    per_point = np.linalg.norm(r.reshape(-1, 2), axis=1)
    rms = float(np.sqrt(c_prev / (2 * n)))
    return rot, t, rms, per_point


def print_yaml(rot, t, rms, n_pairs):
    q = rot_to_quat_wxyz(rot)
    print("\n# ---- paste into config/extrinsics.yaml ----")
    print(f"# calibration: {n_pairs} pairs, RMS {rms:.2f} px")
    print("/**:")
    print("  ros__parameters:")
    print("    extrinsics:")
    print(f"      rotation_wxyz: [{q[0]:.8f}, {q[1]:.8f}, "
          f"{q[2]:.8f}, {q[3]:.8f}]")
    print(f"      translation_m: [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}]")


def self_test():
    """Synthetic truth: solver must recover a perturbed mount from noisy
    pixel observations. Fails loudly (exit 1) if tolerances are missed."""
    rng = np.random.default_rng(42)

    # Truth: nominal convention + ~2.3 deg composite mount error + lever arm.
    dtheta_true = np.deg2rad(np.array([1.5, -2.0, 1.0]))
    rot_true = exp_so3(dtheta_true) @ R_NOMINAL
    t_true = np.array([-0.02, 0.09, 0.03])

    # 40 reflector positions across the shared FOV, 3-14 m depth.
    n = 40
    radar_pts = np.column_stack((
        rng.uniform(-4.0, 4.0, n),
        rng.uniform(3.0, 14.0, n),
        rng.uniform(-1.0, 2.0, n)))

    pix_true = project(radar_pts @ rot_true.T + t_true,
                       DEFAULT_FX, DEFAULT_FY, DEFAULT_CX, DEFAULT_CY)
    pixels = pix_true + rng.normal(0.0, 0.5, pix_true.shape)  # 0.5 px noise

    rot, t, rms, _ = solve_extrinsics(
        radar_pts, pixels,
        DEFAULT_FX, DEFAULT_FY, DEFAULT_CX, DEFAULT_CY, verbose=False)

    # Rotation error as an angle, translation as euclidean distance.
    cos_err = (np.trace(rot_true.T @ rot) - 1.0) / 2.0
    rot_err_deg = float(np.rad2deg(np.arccos(np.clip(cos_err, -1.0, 1.0))))
    t_err_m = float(np.linalg.norm(t - t_true))

    print(f"self-test: rot err {rot_err_deg:.4f} deg, "
          f"t err {t_err_m * 1000.0:.2f} mm, rms {rms:.3f} px")
    ok = (rot_err_deg < 0.1) and (t_err_m < 0.01) and (rms < 1.0)
    if not ok:
        print("self-test FAILED (tolerances: <0.1 deg, <10 mm, <1 px rms)")
        return 1
    print("self-test PASSED")
    return 0


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="?", help="pairs file: x_m,y_m,z_m,u_px,v_px")
    ap.add_argument("--fx", type=float, default=DEFAULT_FX)
    ap.add_argument("--fy", type=float, default=DEFAULT_FY)
    ap.add_argument("--cx", type=float, default=DEFAULT_CX)
    ap.add_argument("--cy", type=float, default=DEFAULT_CY)
    ap.add_argument("--self-test", action="store_true",
                    help="run the synthetic recovery test and exit")
    args = ap.parse_args()

    if args.self_test:
        sys.exit(self_test())
    if args.csv is None:
        ap.error("a pairs CSV is required (or --self-test)")

    data = np.loadtxt(args.csv, delimiter=",", comments="#")
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] != 5:
        sys.exit(f"expected 5 columns (x,y,z,u,v), got {data.shape[1]}")
    if data.shape[0] < 4:
        sys.exit(f"need >= 4 pairs for a stable solve, got {data.shape[0]} "
                 "(8+ spread across the FOV recommended)")

    radar_pts, pixels = data[:, 0:3], data[:, 3:5]
    rot, t, rms, per_point = solve_extrinsics(
        radar_pts, pixels, args.fx, args.fy, args.cx, args.cy)

    print("\nper-point reprojection residual [px]:")
    for i, e in enumerate(per_point):
        flag = "  <-- check this pair" if e > 3.0 * max(rms, 1e-6) else ""
        print(f"  {i:3d}: {e:8.2f}{flag}")
    print(f"\nRMS residual: {rms:.3f} px over {data.shape[0]} pairs")
    print_yaml(rot, t, rms, data.shape[0])


if __name__ == "__main__":
    main()
