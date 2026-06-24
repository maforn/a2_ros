#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy>=1.26", "matplotlib>=3.8", "rosbags>=0.9.4", "pillow>=10.0"]
# ///
"""Visualize a saved global map (PCD) together with object detections (CSV).

Both files come from the same run: `global_map.pcd` is published by RESPLE/DLIO in
the `world` frame, and `detections.csv` is written by detection_mapper in the `map`
frame. a2_ros publishes an identity static TF between `world` and `map` (see
src/meta_packages/a2_ros/launch/resple.launch.py), so the raw x/y/z columns of both
files already share one coordinate frame and can be plotted together with no
transform.

Usage:
    uv run scripts/post_process.py
    uv run scripts/post_process.py --mode 3d
    uv run scripts/post_process.py --map data/global_map.pcd --detections data/detections.csv
    uv run scripts/post_process.py --output map.png

Interactive editing mode (--edit) opens a local web app — the camera feed from the
rosbag recorded during the same run (auto-playing, looping) next to the 2D map — so
detections can be corrected by eye in the browser:
    uv run scripts/post_process.py --edit --bag data/bag_20260101_120000
    uv run scripts/post_process.py --edit --bag data/run1 --image-topic /camera/image/compressed
Requires `ffmpeg` on PATH (used once at startup to mux the bag's frames into an mp4).

If the bag carries the global-localization TF (a dynamic `world`/`map` -> robot
transform, i.e. it was recorded/replayed with RESPLE/DLIO running), the editor also
draws the camera's path on the map as a red arrow pinned to the current video frame,
with the travelled trace behind it — so an object spotted in the video can be placed
on the map by where the pointer is. Bags without that TF just omit the overlay.
"""

from __future__ import annotations

import argparse
import csv
import io
import json
import shutil
import subprocess
import tempfile
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent

_PCD_DTYPE_MAP = {
    ("F", 4): "f4", ("F", 8): "f8",
    ("U", 1): "u1", ("U", 2): "u2", ("U", 4): "u4", ("U", 8): "u8",
    ("I", 1): "i1", ("I", 2): "i2", ("I", 4): "i4", ("I", 8): "i8",
}


def read_pcd_xyz(path: Path) -> np.ndarray:
    """Read x,y,z columns out of an ASCII or binary PCD file as an (N, 3) array."""
    with open(path, "rb") as f:
        fields: list[str] = []
        sizes: list[int] = []
        types: list[str] = []
        counts: list[int] = []
        n_points = 0
        data_format = ""
        while True:
            raw = f.readline()
            if not raw:
                raise ValueError(f"{path}: PCD header ended without a DATA line")
            line = raw.decode("ascii", errors="strict").strip()
            if not line or line.startswith("#"):
                continue
            key, _, rest = line.partition(" ")
            if key == "FIELDS":
                fields = rest.split()
            elif key == "SIZE":
                sizes = [int(x) for x in rest.split()]
            elif key == "TYPE":
                types = rest.split()
            elif key == "COUNT":
                counts = [int(x) for x in rest.split()]
            elif key == "POINTS":
                n_points = int(rest)
            elif key == "DATA":
                data_format = rest.strip()
                break

        names, formats = [], []
        for name, size, ty, count in zip(fields, sizes, types, counts):
            dtype = _PCD_DTYPE_MAP[(ty, size)]
            if count == 1:
                names.append(name)
                formats.append(dtype)
            else:
                for i in range(count):
                    names.append(f"{name}_{i}")
                    formats.append(dtype)
        dtype = np.dtype({"names": names, "formats": formats})

        if data_format == "binary":
            buf = f.read(n_points * dtype.itemsize)
            points = np.frombuffer(buf, dtype=dtype, count=n_points)
        elif data_format == "ascii":
            points = np.loadtxt(f, dtype=dtype, max_rows=n_points)
        else:
            raise ValueError(
                f"{path}: unsupported PCD DATA format '{data_format}' "
                "(binary_compressed is not supported)"
            )

    xyz = np.column_stack([points["x"], points["y"], points["z"]])
    finite = np.isfinite(xyz).all(axis=1)
    if not finite.all():
        xyz = xyz[finite]
    return xyz


def read_detections(path: Path) -> list[dict]:
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        rows = []
        for row in reader:
            rows.append({
                "id": row["id"],
                "class": row["class"],
                "x": float(row["x"]),
                "y": float(row["y"]),
                "z": float(row["z"]),
                "confidence": float(row["confidence"]) if "confidence" in row and row["confidence"] != "" else None,
            })
        return rows


def detection_label(det: dict) -> str:
    if det["confidence"] is not None:
        return f"{det['class']} ({det['confidence']:.2f})"
    return det["class"]


def class_color_map(detections: list[dict]) -> dict:
    classes = sorted({d["class"] for d in detections})
    cmap = plt.get_cmap("tab10" if len(classes) <= 10 else "tab20")
    return {cls: cmap(i % cmap.N) for i, cls in enumerate(classes)}


def subsample(points: np.ndarray, max_points: int) -> np.ndarray:
    if len(points) <= max_points:
        return points
    idx = np.random.default_rng(0).choice(len(points), max_points, replace=False)
    return points[idx]


def height_color_limits(z: np.ndarray) -> tuple[float, float]:
    """Robust vmin/vmax for height coloring.

    The raw cloud has a few wild outliers (stray returns tens of metres above/below
    the scene) that, left in, stretch the colormap so the whole map collapses to one
    flat mid-colormap hue and floor and walls become indistinguishable. Clip to the
    2nd/98th percentile so the colormap spans the real floor-to-wall range instead."""
    lo, hi = np.percentile(z, [2, 98])
    if hi - lo < 1e-6:
        lo, hi = float(z.min()), float(z.max())
    return float(lo), float(hi)


def plot_2d(ax, points: np.ndarray, detections: list[dict], colors: dict, point_size: float, show_labels: bool):
    # Draw lowest points first so taller returns (walls) land on top of the floor at
    # the same x/y instead of being painted over by whatever point happened to be last.
    order = np.argsort(points[:, 2])
    points = points[order]
    vmin, vmax = height_color_limits(points[:, 2])
    sc = ax.scatter(points[:, 0], points[:, 1], c=points[:, 2], cmap="viridis",
                    vmin=vmin, vmax=vmax, s=point_size, alpha=0.5, linewidths=0)
    plt.colorbar(sc, ax=ax, label="height z [m]", shrink=0.8)
    for det in detections:
        color = colors[det["class"]]
        ax.scatter(det["x"], det["y"], color=color, edgecolors="black", s=90, marker="*", zorder=5)
        if show_labels:
            ax.annotate(
                detection_label(det), (det["x"], det["y"]),
                xytext=(6, 6), textcoords="offset points", fontsize=8,
                color=color, weight="bold",
                bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.7),
            )
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_title("Top-down map view")


def plot_3d(ax, points: np.ndarray, detections: list[dict], colors: dict, point_size: float, show_labels: bool):
    vmin, vmax = height_color_limits(points[:, 2])
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=points[:, 2], cmap="viridis",
               vmin=vmin, vmax=vmax, s=point_size, alpha=0.4, linewidths=0)
    for det in detections:
        color = colors[det["class"]]
        ax.scatter(det["x"], det["y"], det["z"], color=color, edgecolors="black", s=90, marker="*")
        if show_labels:
            ax.text(det["x"], det["y"], det["z"], "  " + detection_label(det), fontsize=7, color=color, weight="bold")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title("3D map view")


def build_legend(fig, colors: dict):
    handles = [
        plt.Line2D([0], [0], marker="*", color="w", markerfacecolor=c, markeredgecolor="black", markersize=12, label=cls)
        for cls, c in colors.items()
    ]
    return fig.legend(handles=handles, loc="upper right", title="Detections")


def save_detections(path: Path, detections: list[dict]) -> None:
    """Write detections back to CSV, taking a one-time .bak snapshot of the original."""
    bak = path.with_suffix(path.suffix + ".bak")
    if path.exists() and not bak.exists():
        shutil.copy(path, bak)
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["id", "class", "x", "y", "z", "confidence"])
        for d in detections:
            conf = "" if d["confidence"] is None else d["confidence"]
            writer.writerow([d["id"], d["class"], d["x"], d["y"], d["z"], conf])
    print(f"Saved {len(detections)} detections to {path}")


def load_bag_frames(bag_path: Path, image_topic: str) -> list[tuple[float, object, str]]:
    """Read every message on image_topic out of a rosbag2 (mcap) directory.

    Returns (timestamp_sec, deserialized_msg, msgtype) tuples sorted by time. Uses
    `rosbags` so no ROS install is required to read the bag.
    """
    from rosbags.highlevel import AnyReader

    frames: list[tuple[float, object, str]] = []
    with AnyReader([bag_path]) as reader:
        connections = [c for c in reader.connections if c.topic == image_topic]
        if not connections:
            available = sorted({c.topic for c in reader.connections})
            raise ValueError(
                f"Topic '{image_topic}' not found in {bag_path}. Available topics:\n  "
                + "\n  ".join(available)
            )
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            frames.append((timestamp / 1e9, msg, connection.msgtype))
    frames.sort(key=lambda item: item[0])
    return frames


def _quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Rotation matrix for a (x, y, z, w) quaternion."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def _tf_to_RT(tr) -> tuple[np.ndarray, np.ndarray]:
    t = tr.transform.translation
    q = tr.transform.rotation
    return _quat_to_matrix(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])


def _compose(a: tuple, b: tuple) -> tuple:
    """Compose two (R, t) rigid transforms: a applied to b (a @ b)."""
    Ra, ta = a
    Rb, tb = b
    return Ra @ Rb, Ra @ tb + ta


def _invert(rt: tuple) -> tuple:
    R, t = rt
    return R.T, -R.T @ t


def extract_camera_trajectory(bag_path: Path, video_t0: float, camera_frame: str = "camera_link") -> list[list[float]]:
    """Reconstruct the camera's path over the map from the bag's TF tree.

    The robot pose lives in `/tf`, not in any odometry topic: RESPLE/DLIO publishes the
    localization edge `world -> <body>` (the only dynamic transform whose parent is the
    map-fixed `world`/`map`/`odom` frame), and the camera hangs off the robot through a
    chain of static transforms. We pick that localization edge, walk the static tree from
    its moving child down to `camera_frame`, and compose the two at every timestamp to get
    world->camera. `world` is the same frame the global map is stored in (a2_ros publishes
    an identity world<->map TF), so these x/y plot directly on the map.

    Returns rows of [t, x, y, hx, hy]: t is seconds relative to `video_t0` (the first
    camera frame, so it lines up with video playback time), x/y the camera position in the
    map frame, and (hx, hy) the unit ground-projection of the camera's forward (+x) axis —
    i.e. which way it is looking. Returns [] if the bag has no global localization TF.
    """
    import collections

    from rosbags.highlevel import AnyReader

    static: dict[tuple[str, str], tuple] = {}
    dynamic: list[tuple[float, tuple[str, str], tuple]] = []
    counts: collections.Counter = collections.Counter()
    positions: dict[tuple[str, str], list] = collections.defaultdict(list)

    with AnyReader([bag_path]) as reader:
        cons = [c for c in reader.connections if c.topic in ("/tf", "/tf_static")]
        for connection, timestamp, rawdata in reader.messages(connections=cons):
            msg = reader.deserialize(rawdata, connection.msgtype)
            for tr in msg.transforms:
                key = (tr.header.frame_id, tr.child_frame_id)
                RT = _tf_to_RT(tr)
                if connection.topic == "/tf_static":
                    static[key] = RT
                else:
                    counts[key] += 1
                    t = tr.transform.translation
                    positions[key].append((t.x, t.y, t.z))
                    dynamic.append((timestamp / 1e9, key, RT))

    # The localization edge is the dynamic transform that actually moves and is anchored to
    # a map-fixed root. Prefer a world/map/odom parent; fall back to the most-moving edge.
    roots = {"world", "map", "odom"}
    def score(key):
        spread = float(np.std(np.array(positions[key]), axis=0).sum()) if positions[key] else 0.0
        return (key[0] in roots, spread > 1e-3, counts[key])
    candidates = [k for k in counts if score(k)[1]]
    if not candidates:
        return []
    loc_edge = max(candidates, key=score)
    if loc_edge[0] not in roots:
        return []  # no map-anchored pose in this bag — nothing to plot against the map
    _, moving = loc_edge

    # BFS the static tree (edges usable in both directions) from `moving` to camera_frame.
    adj: dict[str, list] = collections.defaultdict(list)
    for (parent, child), RT in static.items():
        adj[parent].append((child, RT))
        adj[child].append((parent, _invert(RT)))
    queue = collections.deque([(moving, (np.eye(3), np.zeros(3)))])
    seen = {moving}
    T_moving_cam = None
    while queue:
        frame, acc = queue.popleft()
        if frame == camera_frame:
            T_moving_cam = acc
            break
        for nb, RT in adj[frame]:
            if nb not in seen:
                seen.add(nb)
                queue.append((nb, _compose(acc, RT)))
    if T_moving_cam is None:
        T_moving_cam = (np.eye(3), np.zeros(3))  # no camera chain: fall back to the body path

    traj: list[list[float]] = []
    for stamp, key, RT in dynamic:
        if key != loc_edge:
            continue
        R, t = _compose(RT, T_moving_cam)
        fwd = R @ np.array([1.0, 0.0, 0.0])  # camera_link forward (REP-103: +x)
        norm = float(np.hypot(fwd[0], fwd[1])) or 1.0
        traj.append([stamp - video_t0, float(t[0]), float(t[1]), float(fwd[0] / norm), float(fwd[1] / norm)])
    traj.sort(key=lambda r: r[0])
    return traj


def decode_frame(msg, msgtype: str) -> np.ndarray:
    """Decode a sensor_msgs/CompressedImage or sensor_msgs/Image into an (H, W, C) array."""
    if "CompressedImage" in msgtype:
        from PIL import Image as PILImage

        return np.array(PILImage.open(io.BytesIO(bytes(msg.data))).convert("RGB"))
    if "Image" in msgtype:
        h, w, encoding = msg.height, msg.width, msg.encoding
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if encoding in ("rgb8", "bgr8"):
            arr = arr.reshape(h, w, 3)
            if encoding == "bgr8":
                arr = arr[:, :, ::-1]
            return arr
        if encoding == "mono8":
            return arr.reshape(h, w)
        raise ValueError(f"Unsupported image encoding: {encoding}")
    raise ValueError(f"Unsupported message type for image decoding: {msgtype}")


def extract_frames_to_dir(frames: list[tuple[float, object, str]], frame_dir: Path) -> float:
    """Write every frame out as numbered .jpg files for ffmpeg to mux. Returns the fps
    implied by the bag's own timestamps (so playback speed matches the recording)."""
    frame_dir.mkdir(parents=True, exist_ok=True)
    for idx, (_, msg, msgtype) in enumerate(frames):
        out_path = frame_dir / f"{idx:06d}.jpg"
        fmt = msg.format.lower() if "CompressedImage" in msgtype else ""
        if "jpeg" in fmt or "jpg" in fmt:
            out_path.write_bytes(bytes(msg.data))
        else:
            from PIL import Image as PILImage

            PILImage.fromarray(decode_frame(msg, msgtype)).convert("RGB").save(out_path, format="JPEG", quality=90)

    duration = frames[-1][0] - frames[0][0]
    if len(frames) > 1 and duration > 0:
        return (len(frames) - 1) / duration
    return 10.0


def mux_video(frame_dir: Path, out_path: Path, fps: float) -> None:
    if shutil.which("ffmpeg") is None:
        raise RuntimeError("ffmpeg not found on PATH — required to mux bag frames into a video for --edit")
    cmd = [
        "ffmpeg", "-y", "-framerate", f"{fps:.4f}", "-i", str(frame_dir / "%06d.jpg"),
        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(out_path),
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"ffmpeg failed:\n{result.stderr[-4000:]}")


_INDEX_HTML = r"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>a2_ros detection editor</title>
<style>
  html, body { margin: 0; height: 100%; overflow: hidden; user-select: none; background: #1e1e1e; color: #eee; font-family: system-ui, sans-serif; }
  #layout { display: flex; height: calc(100% - 42px); }
  #left, #right { flex: 1 1 50%; display: flex; flex-direction: column; align-items: center; justify-content: center; padding: 8px; box-sizing: border-box; min-width: 0; }
  video { max-width: 100%; max-height: 100%; background: #000; }
  canvas { background: #fff; cursor: crosshair; max-width: 100%; }
  #status { height: 42px; display: flex; align-items: center; gap: 16px; padding: 0 12px; background: #111; font-size: 13px; box-sizing: border-box; }
  #legend { display: flex; gap: 10px; flex-wrap: wrap; }
  .swatch { display: inline-block; width: 10px; height: 10px; border-radius: 50%; margin-right: 4px; vertical-align: middle; }
  button { background: #333; color: #eee; border: 1px solid #555; border-radius: 4px; padding: 4px 10px; cursor: pointer; }
  button:hover { background: #444; }
  #help { margin-left: auto; opacity: 0.7; }
</style>
</head>
<body>
  <div id="layout">
    <div id="left"><video id="cam" autoplay muted loop playsinline controls></video></div>
    <div id="right"><canvas id="map" width="900" height="800"></canvas></div>
  </div>
  <div id="status">
    <button id="undoBtn">Undo (z)</button>
    <button id="saveBtn">Save (s)</button>
    <span id="msg"></span>
    <span id="legend"></span>
    <span id="help">red arrow = camera pose synced to video (trace = path so far) · space: play/pause · ←/→: seek 2s · click empty space: add · drag star: move · hover+d: delete · hover+r: relabel · hover+h: set height · wheel: zoom · right-drag: pan</span>
  </div>
<script>
const canvas = document.getElementById('map');
const ctx = canvas.getContext('2d');
const PALETTE = ['#1f77b4','#ff7f0e','#2ca02c','#d62728','#9467bd','#8c564b','#e377c2','#7f7f7f','#bcbd22','#17becf'];
const classColor = new Map();
function colorFor(cls) {
  if (!classColor.has(cls)) classColor.set(cls, PALETTE[classColor.size % PALETTE.length]);
  return classColor.get(cls);
}

let points = null;        // Float32Array, [x,y,z, x,y,z, ...]
let meta = null;           // {min_x,max_x,min_y,max_y,min_z,max_z,count}
let detections = [];
let history = [];
let baseCanvas = null;     // pre-rendered point cloud raster at the fit-to-data transform
let baseScale, baseOffsetX, baseOffsetY;
let scale, offsetX, offsetY;   // current view transform (mutable: zoom/pan)
let hover = -1, dragging = -1, panning = false, lastPan = null, pressWorld = null, pressScreen = null;
let trajectory = [];       // [[t_rel, x, y, hx, hy], ...] camera pose over time, t synced to video
const cam = document.getElementById('cam');
const W = canvas.width, H = canvas.height, MARGIN = 24;

function viridisRGB(t) {
  const stops = [[68,1,84],[59,82,139],[33,144,140],[93,201,99],[253,231,37]];
  t = Math.min(1, Math.max(0, t));
  const seg = t * (stops.length - 1);
  const i = Math.min(stops.length - 2, Math.floor(seg));
  const f = seg - i;
  const a = stops[i], b = stops[i + 1];
  return [a[0]+(b[0]-a[0])*f, a[1]+(b[1]-a[1])*f, a[2]+(b[2]-a[2])*f];
}

function fitTransform() {
  const dx = Math.max(meta.max_x - meta.min_x, 1e-6);
  const dy = Math.max(meta.max_y - meta.min_y, 1e-6);
  const s = Math.min((W - 2 * MARGIN) / dx, (H - 2 * MARGIN) / dy);
  const ox = MARGIN - meta.min_x * s + ((W - 2 * MARGIN) - dx * s) / 2;
  const oy = MARGIN - meta.min_y * s + ((H - 2 * MARGIN) - dy * s) / 2;
  return [s, ox, oy];
}

function worldToScreen(x, y) {
  return [x * scale + offsetX, H - (y * scale + offsetY)];
}
function screenToWorld(sx, sy) {
  return [(sx - offsetX) / scale, (H - sy - offsetY) / scale];
}

function buildBaseRaster() {
  baseCanvas = document.createElement('canvas');
  baseCanvas.width = W; baseCanvas.height = H;
  const bctx = baseCanvas.getContext('2d');

  // Match the 2D matplotlib view exactly: alpha-blended points, height-colored, drawn
  // low-z first so taller returns (walls) composite on top of the floor — all over a
  // white background. The earlier max-z raster turned every isolated high sky/noise
  // return into a solid bright block that buried the scene; with 50% alpha over white,
  // a lone point is just a faint speck while dense floor/wall returns build up to solid
  // color. Normalize against the robust percentile range (vmin_z/vmax_z) so outliers
  // don't flatten the colormap.
  const ALPHA = 0.5;
  const rgb = new Float32Array(W * H * 3).fill(255); // white canvas
  const n = points.length / 3;
  const order = Array.from({ length: n }, (_, i) => i).sort((a, b) => points[a * 3 + 2] - points[b * 3 + 2]);
  const vmin = meta.vmin_z, vmax = meta.vmax_z;
  const zRange = Math.max(vmax - vmin, 1e-6);
  for (const i of order) {
    const x = points[i * 3], y = points[i * 3 + 1], z = points[i * 3 + 2];
    const sx = Math.round(x * baseScale + baseOffsetX);
    const sy = Math.round(H - (y * baseScale + baseOffsetY));
    if (sx < 0 || sx >= W || sy < 0 || sy >= H) continue;
    const [r, g, bl] = viridisRGB((z - vmin) / zRange);
    const p = (sy * W + sx) * 3;
    rgb[p]     = r  * ALPHA + rgb[p]     * (1 - ALPHA);
    rgb[p + 1] = g  * ALPHA + rgb[p + 1] * (1 - ALPHA);
    rgb[p + 2] = bl * ALPHA + rgb[p + 2] * (1 - ALPHA);
  }

  const imgData = bctx.createImageData(W, H);
  const data = imgData.data;
  for (let idx = 0; idx < W * H; idx++) {
    const s = idx * 3, p = idx * 4;
    data[p] = rgb[s]; data[p + 1] = rgb[s + 1]; data[p + 2] = rgb[s + 2]; data[p + 3] = 255;
  }
  bctx.putImageData(imgData, 0, 0);
}

function drawPointCloudTexture() {
  // Visible world rect under the current (possibly zoomed/panned) view.
  const corners = [screenToWorld(0,0), screenToWorld(W,0), screenToWorld(0,H), screenToWorld(W,H)];
  const xs = corners.map(c => c[0]), ys = corners.map(c => c[1]);
  const wx0 = Math.min(...xs), wx1 = Math.max(...xs), wy0 = Math.min(...ys), wy1 = Math.max(...ys);
  // Same rect expressed in the baseline raster's pixel space.
  let sx0 = wx0 * baseScale + baseOffsetX, sx1 = wx1 * baseScale + baseOffsetX;
  let sy0 = H - (wy1 * baseScale + baseOffsetY), sy1 = H - (wy0 * baseScale + baseOffsetY);
  sx0 = Math.max(0, Math.min(W, sx0)); sx1 = Math.max(0, Math.min(W, sx1));
  sy0 = Math.max(0, Math.min(H, sy0)); sy1 = Math.max(0, Math.min(H, sy1));
  const sw = sx1 - sx0, sh = sy1 - sy0;
  if (sw <= 0 || sh <= 0) return;
  ctx.drawImage(baseCanvas, sx0, sy0, sw, sh, 0, 0, W, H);
}

function render() {
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = '#fff'; ctx.fillRect(0, 0, W, H);
  if (baseCanvas) drawPointCloudTexture();
  drawTrajectory();
  detections.forEach((d, i) => {
    const [sx, sy] = worldToScreen(d.x, d.y);
    ctx.beginPath();
    ctx.arc(sx, sy, i === hover ? 9 : 7, 0, 2 * Math.PI);
    ctx.fillStyle = colorFor(d.class);
    ctx.fill();
    ctx.lineWidth = i === hover ? 3 : 1.5;
    ctx.strokeStyle = i === hover ? 'yellow' : 'black';
    ctx.stroke();
    ctx.font = 'bold 12px sans-serif';
    const conf = d.confidence != null ? ` (${d.confidence.toFixed(2)})` : '';
    const label = `${d.class}${conf} z=${d.z.toFixed(2)}`;
    // White halo + dark fill keeps the label readable over the white map raster.
    ctx.lineWidth = 3; ctx.strokeStyle = 'white'; ctx.strokeText(label, sx + 10, sy - 8);
    ctx.fillStyle = '#111'; ctx.fillText(label, sx + 10, sy - 8);
  });
  renderLegend();
}

// Index of the latest trajectory pose at or before the current video time, so the
// pointer tracks exactly what the camera is showing right now.
function currentTrajIndex() {
  if (!trajectory.length) return -1;
  const t = cam.currentTime || 0;
  if (t <= trajectory[0][0]) return 0;
  let lo = 0, hi = trajectory.length - 1, ans = 0;
  while (lo <= hi) {
    const mid = (lo + hi) >> 1;
    if (trajectory[mid][0] <= t) { ans = mid; lo = mid + 1; } else hi = mid - 1;
  }
  return ans;
}

function drawTrajectory() {
  if (trajectory.length < 2) return;
  // Whole route, faint — so you can see where the robot will go.
  ctx.lineWidth = 1; ctx.strokeStyle = 'rgba(90,90,90,0.4)';
  ctx.beginPath();
  for (let i = 0; i < trajectory.length; i++) {
    const [sx, sy] = worldToScreen(trajectory[i][1], trajectory[i][2]);
    i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy);
  }
  ctx.stroke();

  const idx = currentTrajIndex();
  if (idx < 0) return;
  // Trace already travelled up to the current video time, bold.
  ctx.lineWidth = 3; ctx.strokeStyle = '#d62728';
  ctx.beginPath();
  for (let i = 0; i <= idx; i++) {
    const [sx, sy] = worldToScreen(trajectory[i][1], trajectory[i][2]);
    i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy);
  }
  ctx.stroke();

  // Pointer: a triangle at the current camera position aimed where it is looking.
  const [, x, y, hx, hy] = trajectory[idx];
  const [sx, sy] = worldToScreen(x, y);
  let dx = hx, dy = -hy;                       // world->screen (screen y is flipped)
  const dn = Math.hypot(dx, dy) || 1; dx /= dn; dy /= dn;
  const L = 18, halfW = 8, px = -dy, py = dx;  // px,py: perpendicular to heading
  ctx.beginPath();
  ctx.moveTo(sx + dx * L, sy + dy * L);
  ctx.lineTo(sx + px * halfW, sy + py * halfW);
  ctx.lineTo(sx - px * halfW, sy - py * halfW);
  ctx.closePath();
  ctx.fillStyle = '#d62728'; ctx.fill();
  ctx.lineWidth = 1.5; ctx.strokeStyle = 'white'; ctx.stroke();
  ctx.beginPath(); ctx.arc(sx, sy, 4, 0, 2 * Math.PI);
  ctx.fillStyle = 'white'; ctx.fill(); ctx.strokeStyle = '#d62728'; ctx.stroke();
}

// Re-render in lockstep with video playback so the pointer follows the frame on screen.
function animate() {
  if (trajectory.length) render();
  requestAnimationFrame(animate);
}

function renderLegend() {
  const el = document.getElementById('legend');
  el.innerHTML = '';
  classColor.forEach((color, cls) => {
    const span = document.createElement('span');
    span.innerHTML = `<span class="swatch" style="background:${color}"></span>${cls}`;
    el.appendChild(span);
  });
}

function setMsg(text) { document.getElementById('msg').textContent = text; setTimeout(() => { if (document.getElementById('msg').textContent === text) document.getElementById('msg').textContent = ''; }, 3000); }

function pushHistory() { history.push(JSON.parse(JSON.stringify(detections))); }

function hitTest(sx, sy) {
  let best = -1, bestD = 16; // px radius
  detections.forEach((d, i) => {
    const [px, py] = worldToScreen(d.x, d.y);
    const dist = Math.hypot(px - sx, py - sy);
    if (dist < bestD) { bestD = dist; best = i; }
  });
  return best;
}

function nearestZ(x, y) {
  if (!points || points.length === 0) return 0;
  let bestD = Infinity, bestZ = 0;
  const n = points.length / 3;
  for (let i = 0; i < n; i++) {
    const dx = points[i * 3] - x, dy = points[i * 3 + 1] - y;
    const d2 = dx * dx + dy * dy;
    if (d2 < bestD) { bestD = d2; bestZ = points[i * 3 + 2]; }
  }
  return bestZ;
}

function nextId() {
  let maxId = -1;
  detections.forEach(d => { const n = parseInt(d.id, 10); if (!isNaN(n) && n > maxId) maxId = n; });
  return String(maxId + 1);
}

function canvasCoords(e) {
  const rect = canvas.getBoundingClientRect();
  return [(e.clientX - rect.left) * (W / rect.width), (e.clientY - rect.top) * (H / rect.height)];
}
function insideCanvas(e) {
  const rect = canvas.getBoundingClientRect();
  return e.clientX >= rect.left && e.clientX <= rect.right && e.clientY >= rect.top && e.clientY <= rect.bottom;
}

canvas.addEventListener('contextmenu', e => e.preventDefault());

canvas.addEventListener('mousedown', e => {
  const [sx, sy] = canvasCoords(e);
  if (e.button === 2) { panning = true; lastPan = [sx, sy]; return; }
  if (e.button !== 0) return;
  pressScreen = [sx, sy];
  pressWorld = screenToWorld(sx, sy);
  dragging = hitTest(sx, sy);
});

// Listening on window (not the canvas) is what lets a drag keep tracking once
// the cursor strays outside the canvas bounds mid-gesture.
window.addEventListener('mousemove', e => {
  if (panning && lastPan) {
    const [sx, sy] = canvasCoords(e);
    offsetX += sx - lastPan[0];
    offsetY += lastPan[1] - sy;
    lastPan = [sx, sy];
    render();
    return;
  }
  if (dragging >= 0) {
    const [sx, sy] = canvasCoords(e);
    const [wx, wy] = screenToWorld(sx, sy);
    detections[dragging].x = wx;
    detections[dragging].y = wy;
    render();
    return;
  }
  if (!insideCanvas(e)) {
    if (hover !== -1) { hover = -1; render(); }
    return;
  }
  const [sx, sy] = canvasCoords(e);
  const hit = hitTest(sx, sy);
  if (hit !== hover) { hover = hit; render(); }
});

window.addEventListener('mouseup', e => {
  if (panning) { panning = false; lastPan = null; return; }
  if (e.button !== 0) return;
  const [sx, sy] = canvasCoords(e);
  const moved = pressScreen && Math.hypot(sx - pressScreen[0], sy - pressScreen[1]) > 3;
  if (dragging >= 0) {
    if (moved) {
      pushHistory();
      const d = detections[dragging];
      d.z = nearestZ(d.x, d.y);
    }
    dragging = -1;
    render();
    pressScreen = null;
    return;
  }
  if (!moved && pressWorld && insideCanvas(e)) {
    const cls = window.prompt('Class name for new detection:', classColor.size ? Array.from(classColor.keys())[classColor.size - 1] : 'object');
    if (cls) {
      pushHistory();
      detections.push({ id: nextId(), class: cls, x: pressWorld[0], y: pressWorld[1], z: nearestZ(pressWorld[0], pressWorld[1]), confidence: null });
      render();
    }
  }
  pressScreen = null;
  pressWorld = null;
});

canvas.addEventListener('wheel', e => {
  e.preventDefault();
  const [sx, sy] = canvasCoords(e);
  const [wx, wy] = screenToWorld(sx, sy);
  const factor = e.deltaY < 0 ? 1.15 : 1 / 1.15;
  const newScale = Math.min(Math.max(scale * factor, baseScale * 0.05), baseScale * 50);
  offsetX = sx - wx * newScale;
  offsetY = (H - sy) - wy * newScale;
  scale = newScale;
  render();
}, { passive: false });

function doUndo() {
  if (!history.length) return;
  detections = history.pop();
  hover = -1;
  render();
}

function doSave() {
  fetch('/detections', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(detections) })
    .then(r => r.json()).then(j => setMsg(`Saved ${j.count} detections`))
    .catch(() => setMsg('Save failed'));
}

window.addEventListener('keydown', e => {
  if (e.target && (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA')) return;
  // Video transport: space toggles play/pause, left/right seek by 2s. The trajectory
  // overlay tracks cam.currentTime via animate(), so seeking moves the map pointer too.
  if (e.key === ' ' || e.code === 'Space') { e.preventDefault(); cam.paused ? cam.play() : cam.pause(); return; }
  if (e.key === 'ArrowLeft') { e.preventDefault(); cam.currentTime = Math.max(0, cam.currentTime - 2); render(); return; }
  if (e.key === 'ArrowRight') { e.preventDefault(); cam.currentTime = Math.min(cam.duration || Infinity, cam.currentTime + 2); render(); return; }
  if (e.key === 'd' && hover >= 0) { pushHistory(); detections.splice(hover, 1); hover = -1; render(); }
  else if (e.key === 'r' && hover >= 0) {
    const d = detections[hover];
    const cls = window.prompt('New class:', d.class);
    if (cls) { pushHistory(); d.class = cls; render(); }
  }
  else if (e.key === 'h' && hover >= 0) {
    const d = detections[hover];
    const val = window.prompt('New height (z, meters):', d.z.toFixed(3));
    if (val !== null && val.trim() !== '') {
      const z = parseFloat(val);
      if (!isNaN(z)) { pushHistory(); d.z = z; render(); }
    }
  }
  else if (e.key === 'z') doUndo();
  else if (e.key === 's') doSave();
});

document.getElementById('undoBtn').addEventListener('click', doUndo);
document.getElementById('saveBtn').addEventListener('click', doSave);

async function fetchJson(url) {
  const r = await fetch(url);
  if (!r.ok) throw new Error(`${url} -> HTTP ${r.status}`);
  return r.json();
}

async function init() {
  try {
    const [metaResp, ptsResp, detResp, trajResp] = await Promise.all([
      fetchJson('/map_meta'),
      fetch('/map_points.bin').then(r => { if (!r.ok) throw new Error(`/map_points.bin -> HTTP ${r.status}`); return r.arrayBuffer(); }),
      fetchJson('/detections'),
      fetchJson('/trajectory'),
    ]);
    meta = metaResp;
    points = new Float32Array(ptsResp);
    detections = detResp;
    trajectory = trajResp || [];
    [baseScale, baseOffsetX, baseOffsetY] = fitTransform();
    scale = baseScale; offsetX = baseOffsetX; offsetY = baseOffsetY;
    buildBaseRaster();
    render();
    cam.src = '/video.mp4';
    if (trajectory.length) { setMsg(`Trajectory: ${trajectory.length} poses`); requestAnimationFrame(animate); }
  } catch (err) {
    console.error(err);
    setMsg('Failed to load: ' + err.message);
    document.getElementById('msg').style.color = '#f55';
  }
}
init();
</script>
</body>
</html>
"""


class _EditorState:
    def __init__(self, points: np.ndarray, detections: list[dict], csv_path: Path, video_bytes: bytes,
                 trajectory: list[list[float]]):
        self.points = points
        self.detections = detections
        self.csv_path = csv_path
        self.video_bytes = video_bytes
        self.trajectory = trajectory


def _make_editor_handler(state: _EditorState):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _send_json(self, obj, status=200):
            body = json.dumps(obj).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/":
                body = _INDEX_HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/map_meta":
                pts = state.points
                if len(pts):
                    # vmin_z/vmax_z are the robust (percentile-clipped) height range used
                    # for the colormap, matching the 2D matplotlib view; min_z/max_z stay
                    # the true extent so dropped detections can still snap to real points.
                    vmin_z, vmax_z = height_color_limits(pts[:, 2])
                    self._send_json({
                        "min_x": float(pts[:, 0].min()), "max_x": float(pts[:, 0].max()),
                        "min_y": float(pts[:, 1].min()), "max_y": float(pts[:, 1].max()),
                        "min_z": float(pts[:, 2].min()), "max_z": float(pts[:, 2].max()),
                        "vmin_z": vmin_z, "vmax_z": vmax_z,
                        "count": len(pts),
                    })
                else:
                    self._send_json({"min_x": 0, "max_x": 1, "min_y": 0, "max_y": 1,
                                     "min_z": 0, "max_z": 1, "vmin_z": 0, "vmax_z": 1, "count": 0})
            elif self.path == "/map_points.bin":
                body = np.ascontiguousarray(state.points, dtype="<f4").tobytes()
                self.send_response(200)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/detections":
                self._send_json(state.detections)
            elif self.path == "/trajectory":
                self._send_json(state.trajectory)
            elif self.path == "/video.mp4":
                self._serve_video()
            else:
                self.send_error(404)

        def _serve_video(self):
            data = state.video_bytes
            total = len(data)
            range_header = self.headers.get("Range")
            if range_header and range_header.startswith("bytes="):
                start_s, _, end_s = range_header[6:].partition("-")
                start = int(start_s) if start_s else 0
                end = int(end_s) if end_s else total - 1
                end = min(end, total - 1)
                chunk = data[start:end + 1]
                self.send_response(206)
                self.send_header("Content-Range", f"bytes {start}-{end}/{total}")
                self.send_header("Content-Length", str(len(chunk)))
                self.send_header("Accept-Ranges", "bytes")
                self.send_header("Content-Type", "video/mp4")
                self.end_headers()
                self.wfile.write(chunk)
            else:
                self.send_response(200)
                self.send_header("Content-Length", str(total))
                self.send_header("Accept-Ranges", "bytes")
                self.send_header("Content-Type", "video/mp4")
                self.end_headers()
                self.wfile.write(data)

        def do_POST(self):
            if self.path != "/detections":
                self.send_error(404)
                return
            length = int(self.headers.get("Content-Length", 0))
            try:
                payload = json.loads(self.rfile.read(length))
                new_detections = []
                for d in payload:
                    new_detections.append({
                        "id": str(d["id"]), "class": str(d["class"]),
                        "x": float(d["x"]), "y": float(d["y"]), "z": float(d["z"]),
                        "confidence": None if d.get("confidence") is None else float(d["confidence"]),
                    })
            except (ValueError, KeyError, TypeError) as exc:
                self._send_json({"error": str(exc)}, status=400)
                return
            state.detections[:] = new_detections
            save_detections(state.csv_path, state.detections)
            self._send_json({"ok": True, "count": len(state.detections)})

    return Handler


def run_editor_web(points: np.ndarray, detections: list[dict], bag: Path, image_topic: str, csv_path: Path, port: int = 8000):
    print(f"Reading '{image_topic}' from {bag} ...")
    frames = load_bag_frames(bag, image_topic)
    if not frames:
        raise ValueError(f"No messages found on '{image_topic}' in {bag}")
    print(f"Loaded {len(frames)} frames spanning {frames[-1][0] - frames[0][0]:.1f}s")

    video_t0 = frames[0][0]
    trajectory = extract_camera_trajectory(bag, video_t0)
    if trajectory:
        print(f"Reconstructed camera trajectory: {len(trajectory)} poses from /tf")
    else:
        print("No global localization TF (world->...->camera) in this bag — "
              "skipping the trajectory overlay. Record/replay with RESPLE running to get it.")

    with tempfile.TemporaryDirectory(prefix="a2_post_process_") as tmp:
        tmp_path = Path(tmp)
        fps = extract_frames_to_dir(frames, tmp_path / "frames")
        video_path = tmp_path / "video.mp4"
        print(f"Encoding video at {fps:.1f} fps ...")
        mux_video(tmp_path / "frames", video_path, fps)
        video_bytes = video_path.read_bytes()

        state = _EditorState(points, detections, csv_path, video_bytes, trajectory)
        handler = _make_editor_handler(state)

        server = None
        for candidate_port in range(port, port + 20):
            try:
                server = ThreadingHTTPServer(("127.0.0.1", candidate_port), handler)
                break
            except OSError:
                continue
        if server is None:
            raise RuntimeError(f"Could not bind to any port in [{port}, {port + 19}]")

        url = f"http://127.0.0.1:{server.server_address[1]}/"
        print(f"Editor running at {url}")
        print(
            "Controls: click empty map space to add, drag a star to move,\n"
            "  hover + 'd' to delete, hover + 'r' to relabel, hover + 'h' to set height,\n"
            "  'z' to undo, 's'/Save to write the CSV. Wheel zooms, right-drag pans.\n"
            "  Press Ctrl+C here when done."
        )
        webbrowser.open(url)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            server.shutdown()
            print(f"Stopped. Last saved state is in {csv_path} (if you pressed Save).")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--map", type=Path, default=REPO_ROOT / "data" / "global_map.pcd", help="Path to the global map PCD file")
    parser.add_argument("--detections", type=Path, default=REPO_ROOT / "data" / "detections.csv", help="Path to the detections CSV file")
    parser.add_argument("--mode", choices=["2d", "3d", "both"], default="2d", help="View to render (default: 2d)")
    parser.add_argument("--output", type=Path, default=None, help="Save the figure to this path instead of showing it interactively")
    parser.add_argument("--max-points", type=int, default=300_000, help="Randomly subsample the map cloud above this many points")
    parser.add_argument("--point-size", type=float, default=1.0, help="Marker size for map points")
    parser.add_argument("--no-labels", action="store_true", help="Don't draw class/confidence text next to detections")
    parser.add_argument("--thresh", type=float, default=None, help="Only plot detections with confidence >= this value (drops detections with no confidence recorded)")
    parser.add_argument("--edit", action="store_true", help="Open the interactive web editor: looping camera feed + 2D map side by side in the browser, click/drag/d/r to add, move, delete, relabel detections, 's'/Save to write the CSV")
    parser.add_argument("--bag", type=Path, default=None, help="Path to the rosbag2 (mcap) directory recorded during the same run as --map/--detections (required with --edit)")
    parser.add_argument("--image-topic", type=str, default="/camera/image/compressed", help="Camera topic to stream in the editor (default: %(default)s)")
    parser.add_argument("--port", type=int, default=8000, help="Local port for the --edit web server (default: %(default)s)")
    args = parser.parse_args()

    points = subsample(read_pcd_xyz(args.map), args.max_points)
    if args.detections.exists():
        detections = read_detections(args.detections)
    elif args.edit:
        print(f"No detections file at {args.detections}; starting from an empty list.")
        detections = []
    else:
        raise FileNotFoundError(args.detections)

    if args.edit:
        if args.bag is None:
            parser.error("--edit requires --bag <rosbag2 directory>")
        if args.thresh is not None:
            print("--thresh is ignored in --edit mode (editing always operates on the full set)")
        run_editor_web(points, detections, args.bag, args.image_topic, args.detections, args.port)
        return

    if args.thresh is not None:
        before = len(detections)
        detections = [d for d in detections if d["confidence"] is not None and d["confidence"] >= args.thresh]
        print(f"--thresh {args.thresh}: kept {len(detections)}/{before} detections")
    colors = class_color_map(detections) if detections else {}
    show_labels = not args.no_labels

    if args.mode == "both":
        fig = plt.figure(figsize=(14, 6))
        ax2d = fig.add_subplot(1, 2, 1)
        ax3d = fig.add_subplot(1, 2, 2, projection="3d")
        plot_2d(ax2d, points, detections, colors, args.point_size, show_labels)
        plot_3d(ax3d, points, detections, colors, args.point_size, show_labels)
    else:
        fig = plt.figure(figsize=(9, 8))
        if args.mode == "2d":
            ax = fig.add_subplot(1, 1, 1)
            plot_2d(ax, points, detections, colors, args.point_size, show_labels)
        else:
            ax = fig.add_subplot(1, 1, 1, projection="3d")
            plot_3d(ax, points, detections, colors, args.point_size, show_labels)

    if colors:
        build_legend(fig, colors)
    fig.tight_layout()

    if args.output:
        fig.savefig(args.output, dpi=150)
        print(f"Saved figure to {args.output}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
