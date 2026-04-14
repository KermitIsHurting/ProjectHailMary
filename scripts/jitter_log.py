#!/usr/bin/env python3
"""Log fusion + inference jitter frame-by-frame for calibration diagnostics."""

import rclpy
from rclpy.node import Node
from cuas_msgs.msg import FusedDetectionArray
from vision_msgs.msg import Detection2DArray
import time, csv, sys, os

class JitterLogger(Node):
    def __init__(self):
        super().__init__('jitter_logger')
        self.prev_fu = None
        self.prev_fv = None
        self.prev_rx = None
        self.prev_ry = None
        self.prev_iu = None
        self.prev_iv = None
        self.frame = 0

        self.logfile = os.path.expanduser('~/ProjectHailMarry/logs/jitter_log.csv')
        os.makedirs(os.path.dirname(self.logfile), exist_ok=True)
        self.csvfile = open(self.logfile, 'w', newline='')
        self.writer = csv.writer(self.csvfile)
        self.writer.writerow([
            'frame', 'time_s',
            'fused_u', 'fused_v', 'du_fused', 'dv_fused',
            'radar_x', 'radar_y', 'radar_z', 'dx_radar', 'dy_radar',
            'yolo_cx', 'yolo_cy', 'yolo_w', 'yolo_h', 'du_yolo', 'dv_yolo',
            'range_m', 'vel_mps', 'confidence'
        ])

        self.latest_yolo = None
        self.create_subscription(
            FusedDetectionArray, '/fusion/detections', self.fused_cb, 10)
        self.create_subscription(
            Detection2DArray, '/inference/detections', self.yolo_cb, 10)

        self.t0 = time.monotonic()
        self.get_logger().info(f'Logging jitter to {self.logfile}')
        print(f'{"frm":>4} {"fu":>7} {"fv":>7} {"Δu":>6} {"Δv":>6} '
              f'{"rx":>6} {"ry":>6} {"rz":>6} {"Δrx":>6} {"Δry":>6} '
              f'{"yu":>7} {"yv":>7} {"Δyu":>6} {"Δyv":>6} '
              f'{"rng":>5} {"vel":>6} {"conf":>5}')
        print('-' * 120)

    def yolo_cb(self, msg):
        if msg.detections:
            d = msg.detections[0]
            self.latest_yolo = (
                d.bbox.center.position.x,
                d.bbox.center.position.y,
                d.bbox.size_x,
                d.bbox.size_y
            )

    def fused_cb(self, msg):
        if not msg.detections:
            return
        fd = msg.detections[0]
        self.frame += 1
        t = time.monotonic() - self.t0

        fu, fv = fd.pixel_u, fd.pixel_v
        rx, ry, rz = fd.position_x_m, fd.position_y_m, fd.position_z_m

        du = fu - self.prev_fu if self.prev_fu is not None else 0.0
        dv = fv - self.prev_fv if self.prev_fv is not None else 0.0
        drx = rx - self.prev_rx if self.prev_rx is not None else 0.0
        dry = ry - self.prev_ry if self.prev_ry is not None else 0.0

        yu, yv, yw, yh = 0, 0, 0, 0
        dyu, dyv = 0.0, 0.0
        if self.latest_yolo:
            yu, yv, yw, yh = self.latest_yolo
            if self.prev_iu is not None:
                dyu = yu - self.prev_iu
                dyv = yv - self.prev_iv
            self.prev_iu, self.prev_iv = yu, yv

        self.writer.writerow([
            self.frame, f'{t:.3f}',
            f'{fu:.1f}', f'{fv:.1f}', f'{du:.1f}', f'{dv:.1f}',
            f'{rx:.3f}', f'{ry:.3f}', f'{rz:.3f}', f'{drx:.3f}', f'{dry:.3f}',
            f'{yu:.1f}', f'{yv:.1f}', f'{yw:.1f}', f'{yh:.1f}',
            f'{dyu:.1f}', f'{dyv:.1f}',
            f'{fd.range_m:.2f}', f'{fd.velocity_mps:.2f}', f'{fd.confidence:.2f}'
        ])
        self.csvfile.flush()

        print(f'{self.frame:4d} {fu:7.1f} {fv:7.1f} {du:+6.1f} {dv:+6.1f} '
              f'{rx:6.2f} {ry:6.2f} {rz:6.2f} {drx:+6.3f} {dry:+6.3f} '
              f'{yu:7.1f} {yv:7.1f} {dyu:+6.1f} {dyv:+6.1f} '
              f'{fd.range_m:5.2f} {fd.velocity_mps:+6.2f} {fd.confidence:5.2f}')

        self.prev_fu, self.prev_fv = fu, fv
        self.prev_rx, self.prev_ry = rx, ry

def main():
    rclpy.init()
    node = JitterLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.csvfile.close()
    node.get_logger().info(f'Saved {node.frame} frames to {node.logfile}')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
