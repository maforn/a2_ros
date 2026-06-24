#!/usr/bin/env python3
"""
Publish a saved map PCD and detection CSV into ROS 2 for visualization in RViz/Foxglove.

  /challenge/pointcloud   PointCloud2   — map
  /challenge/detections   MarkerArray   — detection spheres + labels

Usage:
  ./plot_challenge_results.py map.pcd detections.csv
  ./plot_challenge_results.py map.pcd                   # map only
  ./plot_challenge_results.py - detections.csv          # detections only
"""

import argparse
import csv
import sys
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
import std_msgs.msg

_PALETTE = [
    (0.96, 0.26, 0.21),
    (0.13, 0.59, 0.95),
    (0.30, 0.69, 0.31),
    (1.00, 0.76, 0.03),
    (0.61, 0.15, 0.69),
    (0.00, 0.74, 0.83),
    (1.00, 0.34, 0.13),
    (0.38, 0.49, 0.55),
    (0.00, 0.59, 0.53),
    (0.91, 0.12, 0.39),
]


def _color(cls: str, class_map: dict) -> std_msgs.msg.ColorRGBA:
    if cls not in class_map:
        class_map[cls] = len(class_map) % len(_PALETTE)
    r, g, b = _PALETTE[class_map[cls]]
    c = std_msgs.msg.ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), 1.0
    return c


# ── PCD reader ────────────────────────────────────────────────────────────────

def read_pcd(path: str) -> np.ndarray:
    header: dict = {}
    data_offset = 0
    with open(path, 'rb') as f:
        while True:
            raw = f.readline()
            line = raw.decode('ascii', errors='ignore').strip()
            if not line or line.startswith('#'):
                continue
            key, *vals = line.split()
            if key == 'DATA':
                header['DATA'] = vals[0].lower()
                data_offset = f.tell()
                break
            header[key] = vals

    fields = header.get('FIELDS', [])
    sizes  = [int(s) for s in header.get('SIZE', [])]
    types  = header.get('TYPE', [])
    n_pts  = int(header.get('POINTS', ['0'])[0])
    fmt    = header.get('DATA', 'ascii')

    try:
        xi, yi, zi = fields.index('x'), fields.index('y'), fields.index('z')
    except ValueError:
        sys.exit(f'ERROR: PCD missing x/y/z — found: {fields}')

    if fmt == 'ascii':
        pts, in_data = [], False
        with open(path) as f:
            for line in f:
                if in_data:
                    v = line.strip().split()
                    if len(v) > max(xi, yi, zi):
                        pts.append((float(v[xi]), float(v[yi]), float(v[zi])))
                elif line.strip().lower().startswith('data'):
                    in_data = True
        return np.array(pts, dtype=np.float32)

    elif fmt == 'binary':
        _np_type = {'F': 'f', 'I': 'i', 'U': 'u'}
        dt = np.dtype([(f if f != '_' else f'_p{i}', _np_type.get(t, 'u') + str(s))
                       for i, (f, s, t) in enumerate(zip(fields, sizes, types))])
        with open(path, 'rb') as f:
            f.seek(data_offset)
            raw = np.frombuffer(f.read(n_pts * dt.itemsize), dtype=dt)
        return np.column_stack([raw['x'].astype(np.float32),
                                raw['y'].astype(np.float32),
                                raw['z'].astype(np.float32)])
    else:
        sys.exit(f'ERROR: unsupported PCD format "{fmt}". Convert with: '
                 'pcl_convert_pcd_ascii_binary in.pcd out.pcd 0')


# ── CSV reader ────────────────────────────────────────────────────────────────

def read_csv(path: str) -> list[dict]:
    with open(path, newline='') as f:
        return [{'id': int(r['id']), 'class': r['class'].strip(),
                 'x': float(r['x']), 'y': float(r['y']), 'z': float(r['z'])}
                for r in csv.DictReader(f)]


# ── message builders ──────────────────────────────────────────────────────────

def make_cloud(pts: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp    = stamp
    msg.height, msg.width = 1, len(pts)
    msg.is_bigendian = False
    msg.is_dense     = True
    msg.point_step   = 12
    msg.row_step     = 12 * len(pts)
    msg.fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    msg.data = pts.astype(np.float32).tobytes()
    return msg


def make_markers(detections: list[dict], frame_id: str, stamp) -> MarkerArray:
    arr = MarkerArray()
    class_map: dict = {}
    never = Duration(sec=0, nanosec=0)

    for det in detections:
        color = _color(det['class'], class_map)

        sphere = Marker()
        sphere.header.frame_id = frame_id
        sphere.header.stamp    = stamp
        sphere.ns, sphere.id   = 'detections', det['id'] * 2
        sphere.type            = Marker.SPHERE
        sphere.action          = Marker.ADD
        sphere.lifetime        = never
        sphere.pose.position.x = det['x']
        sphere.pose.position.y = det['y']
        sphere.pose.position.z = det['z']
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.4
        sphere.color   = color
        arr.markers.append(sphere)

        label = Marker()
        label.header.frame_id = frame_id
        label.header.stamp    = stamp
        label.ns, label.id    = 'detections', det['id'] * 2 + 1
        label.type            = Marker.TEXT_VIEW_FACING
        label.action          = Marker.ADD
        label.lifetime        = never
        label.pose.position.x = det['x']
        label.pose.position.y = det['y']
        label.pose.position.z = det['z'] + 0.35
        label.pose.orientation.w = 1.0
        label.scale.z  = 0.25
        label.color.r  = label.color.g = label.color.b = label.color.a = 1.0
        label.text     = f"{det['class']} #{det['id']}"
        arr.markers.append(label)

    return arr


# ── node ─────────────────────────────────────────────────────────────────────

class ChallengeResultsPublisher(Node):

    def __init__(self, pts, detections, frame_id):
        super().__init__('challenge_results')

        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)

        self._pts        = pts
        self._detections = detections
        self._frame_id   = frame_id

        self._pub_cloud   = self.create_publisher(PointCloud2,  '/challenge/pointcloud', qos) if pts is not None else None
        self._pub_markers = self.create_publisher(MarkerArray,  '/challenge/detections',  qos) if detections else None

        self.create_timer(1.0, self._publish)
        self.get_logger().info(
            'Publishing on /challenge/pointcloud and /challenge/detections '
            '(TRANSIENT_LOCAL) — open RViz to visualize')

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        if self._pub_cloud and self._pts is not None:
            self._pub_cloud.publish(make_cloud(self._pts, self._frame_id, stamp))
        if self._pub_markers and self._detections:
            self._pub_markers.publish(make_markers(self._detections, self._frame_id, stamp))


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('pcd', nargs='?', default=None,
                    help='Path to .pcd map file (pass - to skip)')
    ap.add_argument('csv', nargs='?', default=None,
                    help='Path to detections CSV')
    ap.add_argument('--frame-id', default='map')
    args = ap.parse_args()

    pcd_path = args.pcd if args.pcd and args.pcd != '-' else None
    csv_path = args.csv if args.csv and args.csv != '-' else None

    if not pcd_path and not csv_path:
        ap.print_help()
        sys.exit(1)

    pts        = None
    detections = []

    if pcd_path:
        print(f'Loading {pcd_path} …')
        pts = read_pcd(pcd_path)
        print(f'  {len(pts):,} points')

    if csv_path:
        print(f'Loading {csv_path} …')
        detections = read_csv(csv_path)
        print(f'  {len(detections)} detections')

    rclpy.init()
    try:
        rclpy.spin(ChallengeResultsPublisher(pts, detections, args.frame_id))
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
