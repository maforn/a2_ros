#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy>=1.26", "matplotlib>=3.8"]
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
"""

from __future__ import annotations

import argparse
import csv
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

    return np.column_stack([points["x"], points["y"], points["z"]])


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


def plot_2d(ax, points: np.ndarray, detections: list[dict], colors: dict, point_size: float, show_labels: bool):
    sc = ax.scatter(points[:, 0], points[:, 1], c=points[:, 2], cmap="viridis", s=point_size, alpha=0.5, linewidths=0)
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
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=points[:, 2], cmap="viridis", s=point_size, alpha=0.4, linewidths=0)
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
    fig.legend(handles=handles, loc="upper right", title="Detections")


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
    args = parser.parse_args()

    points = subsample(read_pcd_xyz(args.map), args.max_points)
    detections = read_detections(args.detections)
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
