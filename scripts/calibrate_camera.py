#!/usr/bin/env python3
"""Guided camera calibration — tells you exactly where to stand and when to move."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import sys
import time

BOARD_ROWS = 7
BOARD_COLS = 10
SQUARE_MM = 23.0
CAPTURE_DIR = os.path.expanduser('~/ProjectHailMarry/calibration_frames')

# Each step: (description, target_spread_min%, target_spread_max%, target_x_region, tilt)
# spread = how much of the frame the board fills (bigger = closer)
# x_region: 'left', 'center', 'right' based on board centroid position
# tilt: 'flat', 'tilt' based on corner geometry
STEPS = [
    # Close (~2 ft) — board fills a lot of the frame
    {'dist': '2 ft',  'pos': 'CENTER', 'tilt': 'flat',    'spread_min': 8,  'spread_max': 100, 'x_min': 0.3, 'x_max': 0.7},
    {'dist': '2 ft',  'pos': 'LEFT',   'tilt': 'flat',    'spread_min': 8,  'spread_max': 100, 'x_min': 0.0, 'x_max': 0.4},
    {'dist': '2 ft',  'pos': 'RIGHT',  'tilt': 'flat',    'spread_min': 8,  'spread_max': 100, 'x_min': 0.6, 'x_max': 1.0},
    # Medium (~3 ft)
    {'dist': '3 ft',  'pos': 'CENTER', 'tilt': 'flat',    'spread_min': 3,  'spread_max': 12,  'x_min': 0.3, 'x_max': 0.7},
    {'dist': '3 ft',  'pos': 'LEFT',   'tilt': 'flat',    'spread_min': 3,  'spread_max': 12,  'x_min': 0.0, 'x_max': 0.4},
    {'dist': '3 ft',  'pos': 'RIGHT',  'tilt': 'flat',    'spread_min': 3,  'spread_max': 12,  'x_min': 0.6, 'x_max': 1.0},
    # Far (~6 ft)
    {'dist': '6 ft',  'pos': 'CENTER', 'tilt': 'flat',    'spread_min': 0.5, 'spread_max': 5,  'x_min': 0.3, 'x_max': 0.7},
    {'dist': '6 ft',  'pos': 'LEFT',   'tilt': 'flat',    'spread_min': 0.5, 'spread_max': 5,  'x_min': 0.0, 'x_max': 0.4},
    {'dist': '6 ft',  'pos': 'RIGHT',  'tilt': 'flat',    'spread_min': 0.5, 'spread_max': 5,  'x_min': 0.6, 'x_max': 1.0},
    # Tilted at ~3 ft
    {'dist': '3 ft',  'pos': 'CENTER', 'tilt': 'tilt-L',  'spread_min': 1,  'spread_max': 12,  'x_min': 0.2, 'x_max': 0.8},
    {'dist': '3 ft',  'pos': 'CENTER', 'tilt': 'tilt-R',  'spread_min': 1,  'spread_max': 12,  'x_min': 0.2, 'x_max': 0.8},
    {'dist': '3 ft',  'pos': 'CENTER', 'tilt': 'tilt-up', 'spread_min': 1,  'spread_max': 12,  'x_min': 0.2, 'x_max': 0.8},
    {'dist': '3 ft',  'pos': 'CENTER', 'tilt': 'tilt-dn', 'spread_min': 1,  'spread_max': 12,  'x_min': 0.2, 'x_max': 0.8},
    # Diagonal tilts
    {'dist': '2 ft',  'pos': 'CENTER', 'tilt': 'tilt-diag','spread_min': 2, 'spread_max': 100, 'x_min': 0.2, 'x_max': 0.8},
    {'dist': '4 ft',  'pos': 'CENTER', 'tilt': 'tilt-diag','spread_min': 1, 'spread_max': 10,  'x_min': 0.2, 'x_max': 0.8},
]

TILT_DESCRIPTIONS = {
    'flat':      'Hold board FLAT (facing camera straight)',
    'tilt-L':    'TILT board to the LEFT about 30 degrees',
    'tilt-R':    'TILT board to the RIGHT about 30 degrees',
    'tilt-up':   'TILT board UP (top away from you) about 30 degrees',
    'tilt-dn':   'TILT board DOWN (bottom away from you) about 30 degrees',
    'tilt-diag': 'TILT board DIAGONALLY (any angle, get creative)',
}


class GuidedCalibrationNode(Node):
    def __init__(self):
        super().__init__('guided_calibrator')
        self.bridge = CvBridge()
        self.obj_points = []
        self.img_points = []
        self.img_size = None
        self.step_idx = 0
        self.hold_start = 0
        self.hold_needed = 1.5  # seconds to hold still for capture

        os.makedirs(CAPTURE_DIR, exist_ok=True)

        objp = np.zeros((BOARD_ROWS * BOARD_COLS, 3), np.float32)
        objp[:, :2] = np.mgrid[0:BOARD_COLS, 0:BOARD_ROWS].T.reshape(-1, 2)
        objp *= SQUARE_MM / 1000.0
        self.objp_template = objp

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_cb, 1)

        print(f'\n{"="*60}')
        print(f'  GUIDED CAMERA CALIBRATION')
        print(f'  {BOARD_COLS}x{BOARD_ROWS} checkerboard, {SQUARE_MM}mm squares')
        print(f'  {len(STEPS)} steps — just follow the instructions')
        print(f'{"="*60}\n')
        self._print_step()

    def _print_step(self):
        if self.step_idx >= len(STEPS):
            return
        s = STEPS[self.step_idx]
        n = self.step_idx + 1
        print(f'\n  ---- Step {n}/{len(STEPS)} ----')
        print(f'  Distance : {s["dist"]}')
        print(f'  Position : {s["pos"]} of frame')
        print(f'  {TILT_DESCRIPTIONS[s["tilt"]]}')
        print(f'  Hold still when you see "HOLD STEADY..."')
        print()

    def image_cb(self, msg):
        if self.step_idx >= len(STEPS):
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        if self.img_size is None:
            self.img_size = (frame.shape[1], frame.shape[0])

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
        found, corners = cv2.findChessboardCorners(gray, (BOARD_COLS, BOARD_ROWS), flags)

        step = STEPS[self.step_idx]
        now = time.monotonic()

        if not found:
            self.hold_start = 0
            sys.stdout.write(f'\r  [{self.step_idx+1}/{len(STEPS)}] '
                           f'Looking for board... (hold it at {step["dist"]}, {step["pos"]})          ')
            sys.stdout.flush()
            return

        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        pts = corners_refined.reshape(-1, 2)

        # Compute board metrics
        spread = self._compute_spread(corners_refined)
        cx_norm = float(np.mean(pts[:, 0])) / self.img_size[0]

        # Check if position matches step requirements
        spread_ok = step['spread_min'] <= spread <= step['spread_max']
        x_ok = step['x_min'] <= cx_norm <= step['x_max']

        if not spread_ok or not x_ok:
            self.hold_start = 0
            hints = []
            if spread < step['spread_min']:
                hints.append('MOVE CLOSER')
            elif spread > step['spread_max']:
                hints.append('MOVE FARTHER')
            if cx_norm < step['x_min']:
                hints.append('MOVE board to the RIGHT -->')
            elif cx_norm > step['x_max']:
                hints.append('<-- MOVE board to the LEFT')
            hint_str = ' | '.join(hints)
            sys.stdout.write(f'\r  [{self.step_idx+1}/{len(STEPS)}] '
                           f'Board seen but: {hint_str}              ')
            sys.stdout.flush()
            return

        # Board is in the right position — start/continue hold timer
        if self.hold_start == 0:
            self.hold_start = now
            sys.stdout.write(f'\r  [{self.step_idx+1}/{len(STEPS)}] '
                           f'HOLD STEADY... capturing in {self.hold_needed:.1f}s          ')
            sys.stdout.flush()
            return

        elapsed = now - self.hold_start
        remaining = self.hold_needed - elapsed

        if remaining > 0:
            sys.stdout.write(f'\r  [{self.step_idx+1}/{len(STEPS)}] '
                           f'HOLD STEADY... capturing in {remaining:.1f}s          ')
            sys.stdout.flush()
            return

        # Capture!
        self.obj_points.append(self.objp_template)
        self.img_points.append(corners_refined)
        self.step_idx += 1

        fname = os.path.join(CAPTURE_DIR, f'frame_{self.step_idx:03d}.png')
        cv2.drawChessboardCorners(frame, (BOARD_COLS, BOARD_ROWS), corners_refined, found)
        cv2.imwrite(fname, frame)

        print(f'\r  CAPTURED step {self.step_idx}/{len(STEPS)}! '
              f'(spread={spread:.1f}%, x_pos={cx_norm:.2f})                ')

        self.hold_start = 0

        if self.step_idx >= len(STEPS):
            print(f'\n  All {len(STEPS)} steps complete! Running calibration...\n')
            self.run_calibration()
            rclpy.shutdown()
        else:
            self._print_step()

    def _compute_spread(self, corners):
        pts = corners.reshape(-1, 2)
        w = pts[:, 0].max() - pts[:, 0].min()
        h = pts[:, 1].max() - pts[:, 1].min()
        return 100.0 * (w * h) / (self.img_size[0] * self.img_size[1])

    def run_calibration(self):
        print('Running cv2.calibrateCamera...')
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points, self.img_points, self.img_size, None, None)

        fx, fy = mtx[0, 0], mtx[1, 1]
        cx, cy = mtx[0, 2], mtx[1, 2]
        k1, k2, p1, p2, k3 = dist[0]

        print(f'\n{"="*60}')
        print(f'  CALIBRATION RESULTS')
        print(f'  RMS reprojection error: {ret:.4f} px')
        print(f'{"="*60}')
        print(f'  FX = {fx:.1f}')
        print(f'  FY = {fy:.1f}')
        print(f'  CX = {cx:.1f}')
        print(f'  CY = {cy:.1f}')
        print(f'  k1 = {k1:.6f}')
        print(f'  k2 = {k2:.6f}')
        print(f'  p1 = {p1:.6f}')
        print(f'  p2 = {p2:.6f}')
        print(f'  k3 = {k3:.6f}')
        print(f'{"="*60}')

        cal_file = os.path.expanduser(
            '~/ProjectHailMarry/src/cuas_fusion/config/camera_calibration_result.yaml')
        with open(cal_file, 'w') as f:
            f.write(f'# Camera calibration — {time.strftime("%Y-%m-%d %H:%M")}\n')
            f.write(f'# {len(STEPS)} frames, RMS error: {ret:.4f} px\n')
            f.write(f'# Board: {BOARD_COLS}x{BOARD_ROWS}, square: {SQUARE_MM}mm\n\n')
            f.write(f'intrinsics:\n')
            f.write(f'  fx: {fx:.2f}\n')
            f.write(f'  fy: {fy:.2f}\n')
            f.write(f'  cx: {cx:.2f}\n')
            f.write(f'  cy: {cy:.2f}\n\n')
            f.write(f'distortion:\n')
            f.write(f'  k1: {k1:.8f}\n')
            f.write(f'  k2: {k2:.8f}\n')
            f.write(f'  p1: {p1:.8f}\n')
            f.write(f'  p2: {p2:.8f}\n')
            f.write(f'  k3: {k3:.8f}\n\n')
            f.write(f'reprojection_error_px: {ret:.4f}\n')

        print(f'\n  Results saved to: {cal_file}')
        print(f'  Frames saved to:  {CAPTURE_DIR}/')
        print(f'\n  Copy these into constants.hpp:')
        print(f'    CAMERA_FX = {fx:.1f}f;')
        print(f'    CAMERA_FY = {fy:.1f}f;')
        print(f'    CAMERA_CX = {cx:.1f}f;')
        print(f'    CAMERA_CY = {cy:.1f}f;')


def main():
    rclpy.init()
    node = GuidedCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        count = len(node.obj_points)
        if count >= 5:
            print(f'\n\n  Interrupted with {count} captures — running calibration...\n')
            node.run_calibration()
        else:
            print(f'\n  Only {count} captures — need at least 5. Exiting.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
