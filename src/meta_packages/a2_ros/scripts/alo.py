#!/usr/bin/env python3
"""
2D frontier-based exploration with Next-Best-View waypoint selection.

Policy
------
1. Ray-trace /registered_scan into a 2D occupancy grid.
2. Detect frontier cells (FREE adjacent to UNKNOWN).
3. If no frontiers remain → exploration complete.
4. Next-Best-View: sample FREE cells on a coarse grid, reject those too
   close to obstacles (nav_clearance), reject blacklisted positions, score
   by  info_gain^0.4 / distance  where info_gain = UNKNOWN cells within
   ray_max radius (computed via integral image — O(1) per candidate).
5. If the robot cannot reach the waypoint within wp_timeout seconds,
   blacklist it and replan.

Topics
  Sub  /registered_scan    PointCloud2    world-frame lidar
  Sub  /state_estimation   Odometry       robot pose
  Sub  /start_exploration  Bool           True=start, False=stop
  Pub  /way_point          PointStamped   next nav waypoint
  Pub  exploration_finish  Bool           fired on completion
  Pub  ~/map               OccupancyGrid  explored map (RViz)
  Pub  ~/frontiers         Marker         frontier cells, cyan dots
  Pub  ~/target            Marker         current NBV target, orange sphere
"""

import math
import struct
import time
import numpy as np
from scipy.ndimage import label as nd_label, binary_dilation
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker

UNKNOWN, FREE, OCC = 0, 1, 2


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('alo')

        # ── parameters ───────────────────────────────────────────────────
        self.res      = self.declare_parameter('resolution',         0.25).value  # m/cell
        half_w        = self.declare_parameter('grid_half_width',   80.0).value   # m each side
        self.z_lo     = self.declare_parameter('z_min_rel',         -0.3).value   # height filter rel. robot
        self.z_hi     = self.declare_parameter('z_max_rel',          1.5).value
        self.ray_max  = self.declare_parameter('max_ray_range',      5.0).value   # m
        fov_deg       = self.declare_parameter('scan_fov_deg',      360.0).value  # camera FOV filter (360 = disabled)
        self.fov_half = math.radians(fov_deg / 2.0) if fov_deg < 360.0 else None
        self.reach    = self.declare_parameter('reach_dist',         1.5).value   # m — waypoint "reached"
        self.min_wp_d = self.declare_parameter('min_wp_dist',        2.5).value   # m — min advance per wp
        self.clear_r  = self.declare_parameter('robot_clear_radius', 0.6).value   # m — footprint clearing
        self.nav_clr  = self.declare_parameter('nav_clearance',      0.5).value   # m — min wall clearance for wp
        self.wp_tmo   = self.declare_parameter('wp_timeout',        30.0).value   # s — blacklist if not reached
        self.done_tmo = self.declare_parameter('done_timeout',       8.0).value   # s — confirm no frontiers
        self.occ_thresh = self.declare_parameter('occ_hit_threshold', 2).value    # hits before cell → OCC
        self.hit_decay  = self.declare_parameter('occ_hit_decay',    0.95).value  # hit decay per plan tick
        plan_hz       = self.declare_parameter('planning_hz',        1.0).value
        wp_hz         = self.declare_parameter('wp_publish_hz',      2.0).value
        viz_hz        = self.declare_parameter('viz_hz',             1.0).value

        # ── grid ─────────────────────────────────────────────────────────
        n = int(2 * half_w / self.res) + 1
        self.n        = n
        self.grid     = np.zeros((n, n), dtype=np.uint8)   # UNKNOWN everywhere
        self._hit_grid = np.zeros((n, n), dtype=np.float32) # accumulated OCC hits
        self.origin   = np.zeros(2)  # world XY of cell (0,0)

        # ── state ────────────────────────────────────────────────────────
        self.robot_xy   = np.zeros(2)
        self.robot_z    = 0.0
        self.robot_yaw  = 0.0
        self.current_wp = None       # world-frame [x,y] target
        self.wp_set_t   = None       # monotonic time when current_wp was assigned
        self.exploring  = False
        self.last_scan  = None
        self.no_front_t = None
        self._blacklist: list       = []
        self._viz_frontiers         = np.zeros((0, 2), dtype=int)
        # Visit density: incremented under the robot each planning tick, slowly decays.
        # Used to penalise re-visiting already-covered areas (stops back-and-forth).
        self._visit_grid = np.zeros((n, n), dtype=np.float32)
        self._prev_xy    = np.zeros(2)   # robot position one plan tick ago
        self._heading    = np.zeros(2)   # unit vector of recent robot movement
        # Suppressed frontier cells: grid (r,c) pairs near failed (timed-out) waypoints.
        # The LiDAR sees over low barriers → marks cells beyond as FREE → creates frontiers
        # the robot can never reach.  Suppress those cells so they don't attract new waypoints.
        self._suppressed: np.ndarray = np.zeros((n, n), dtype=bool)
        # Noise-filtered OCC view computed each plan tick: isolated OCC cells
        # (no 4-connected OCC neighbor) are treated as passable for planning.
        self._occ_filtered: np.ndarray = np.zeros((n, n), dtype=bool)

        # ── ROS I/O ───────────────────────────────────────────────────────
        self.create_subscription(PointCloud2, '/registered_scan',   self._cb_scan,  10)
        self.create_subscription(Odometry,    '/state_estimation',  self._cb_odom,  10)
        self.create_subscription(Bool,        '/start_exploration', self._cb_start, 10)
        self.pub_wp     = self.create_publisher(PointStamped, '/goal_point',          1)
        self.pub_done   = self.create_publisher(Bool,          'exploration_finish',  1)
        self.pub_map    = self.create_publisher(OccupancyGrid, '~/map',               1)
        self.pub_fronts = self.create_publisher(Marker,        '~/frontiers',         1)
        self.pub_target = self.create_publisher(Marker,        '~/target',            1)

        self.create_timer(1.0 / plan_hz, self._plan)
        self.create_timer(1.0 / wp_hz,   self._publish_wp)
        self.create_timer(1.0 / viz_hz,  self._publish_viz)

        self.get_logger().info(
            f'FrontierExplorer ready — {n}×{n} grid @ {self.res} m/cell (±{half_w} m). '
            f'Waiting for /start_exploration …')

    # ── callbacks ──────────────────────────────────────────────────────────

    def _cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.robot_xy[:] = [p.x, p.y]
        self.robot_z = p.z
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _cb_scan(self, msg: PointCloud2):
        self.last_scan = msg

    def _cb_start(self, msg: Bool):
        if msg.data:
            if self.exploring:
                return
            hw = (self.n // 2) * self.res
            self.origin     = self.robot_xy - hw
            self.grid[:]     = UNKNOWN
            self._hit_grid[:] = 0.0
            self.current_wp  = None
            self.wp_set_t    = None
            self.no_front_t  = None
            self._blacklist  = []
            self._viz_frontiers = np.zeros((0, 2), dtype=int)
            self._visit_grid[:]  = 0.0
            self._suppressed[:]   = False
            self._occ_filtered[:] = False
            self._prev_xy    = self.robot_xy.copy()
            self._heading    = np.zeros(2)
            self.exploring   = True
            self.get_logger().info(
                f'Exploration started — origin ({self.origin[0]:.1f},{self.origin[1]:.1f})')
        else:
            if self.exploring:
                self.get_logger().info('Exploration stopped by BT')
            self.exploring  = False
            self.current_wp = None
            self.wp_set_t   = None

    # ── grid helpers ───────────────────────────────────────────────────────

    def _w2g(self, xy: np.ndarray) -> np.ndarray:
        return (xy - self.origin) / self.res

    def _g2w(self, rc: np.ndarray) -> np.ndarray:
        return self.origin + (rc + 0.5) * self.res

    # ── occupancy update ───────────────────────────────────────────────────

    def _update_grid(self):
        if self.last_scan is None:
            return
        msg  = self.last_scan
        foff = {f.name: f.offset for f in msg.fields}
        if not all(k in foff for k in ('x', 'y', 'z')):
            return
        xo, yo, zo = foff['x'], foff['y'], foff['z']
        ps    = msg.point_step
        n_pts = msg.width * msg.height
        raw   = bytes(msg.data)
        rx, ry, rz = float(self.robot_xy[0]), float(self.robot_xy[1]), float(self.robot_z)
        rob_gx = (rx - self.origin[0]) / self.res
        rob_gy = (ry - self.origin[1]) / self.res

        for i in range(0, n_pts, 16):
            base = i * ps
            px = struct.unpack_from('f', raw, base + xo)[0]
            py = struct.unpack_from('f', raw, base + yo)[0]
            pz = struct.unpack_from('f', raw, base + zo)[0]
            if not (np.isfinite(px) and np.isfinite(py) and np.isfinite(pz)):
                continue
            dz = pz - rz
            if not (self.z_lo <= dz <= self.z_hi):
                continue
            dx, dy = px - rx, py - ry
            dist   = (dx*dx + dy*dy) ** 0.5
            if dist < 0.05:
                continue
            if self.fov_half is not None:
                az = math.atan2(dy, dx)
                rel = (az - self.robot_yaw + math.pi) % (2.0 * math.pi) - math.pi
                if abs(rel) > self.fov_half:
                    continue
            is_occ = dist <= self.ray_max
            if not is_occ:
                s = self.ray_max / dist
                px, py = rx + dx*s, ry + dy*s
            tgx = (px - self.origin[0]) / self.res
            tgy = (py - self.origin[1]) / self.res
            self._mark_ray(rob_gx, rob_gy, tgx, tgy, is_occ)

        # Stamp robot footprint FREE so the LiDAR blind-spot never creates frontiers
        self._clear_footprint(rob_gx, rob_gy)

    def _mark_ray(self, x0, y0, x1, y1, occ_end: bool):
        r0, c0 = int(x0), int(y0)
        r1, c1 = int(x1), int(y1)
        n = self.n
        dr = abs(r1 - r0); dc = abs(c1 - c0)
        sr = 1 if r0 < r1 else -1
        sc = 1 if c0 < c1 else -1
        err = dr - dc
        r, c = r0, c0
        while True:
            in_b = 0 <= r < n and 0 <= c < n
            if r == r1 and c == c1:
                if in_b:
                    if occ_end:
                        # Accumulate hits; only commit to OCC once threshold is met.
                        # Single noisy returns never reach the threshold and decay away.
                        hits = self._hit_grid[r, c] + 1.0
                        self._hit_grid[r, c] = hits
                        if hits >= self.occ_thresh:
                            self.grid[r, c] = OCC
                    else:
                        self._hit_grid[r, c] = 0.0
                        self.grid[r, c] = FREE
                break
            if in_b and self.grid[r, c] != OCC:
                self.grid[r, c] = FREE
            e2 = 2 * err
            if e2 > -dc: err -= dc; r += sr
            if e2 <  dr: err += dr; c += sc

    def _clear_footprint(self, gx, gy):
        cr = int(self.clear_r / self.res) + 1
        r0 = max(0, int(gx) - cr);  r1 = min(self.n, int(gx) + cr + 1)
        c0 = max(0, int(gy) - cr);  c1 = min(self.n, int(gy) + cr + 1)
        rs = np.arange(r0, r1, dtype=float)
        cs = np.arange(c0, c1, dtype=float)
        rr, cc = np.meshgrid(rs, cs, indexing='ij')
        inside = ((rr - gx)**2 + (cc - gy)**2) <= (self.clear_r / self.res)**2
        self._hit_grid[r0:r1, c0:c1][inside] = 0.0
        self.grid[r0:r1, c0:c1][inside] = FREE

    # ── frontier detection ─────────────────────────────────────────────────

    def _find_frontiers(self) -> np.ndarray:
        free = self.grid == FREE
        unk  = self.grid == UNKNOWN
        unk_nb = (np.roll(unk, 1, 0) | np.roll(unk, -1, 0) |
                  np.roll(unk, 1, 1) | np.roll(unk, -1, 1))
        return np.argwhere(free & unk_nb)

    # ── Reachability via BFS ───────────────────────────────────────────────

    def _reachable_mask(self) -> np.ndarray:
        """Return a bool mask of cells reachable from the robot through non-OCC space.

        Uses scipy connected-component labeling (C-speed, ~3 ms on a 640×640 grid).
        Cells beyond barriers the LiDAR sees over are in a different component
        and are marked False, preventing the NBV from targeting them."""
        rob_g = self._w2g(self.robot_xy).astype(int)
        r0 = int(np.clip(rob_g[0], 0, self.n - 1))
        c0 = int(np.clip(rob_g[1], 0, self.n - 1))

        # Dilate FREE by 1 cell so single-cell UNKNOWN gaps inside doorways don't
        # break connectivity between two otherwise-reachable explored regions.
        # The OCC mask ensures we never route through walls.
        free_dilated    = binary_dilation(self.grid == FREE, iterations=1)
        passable        = (free_dilated & ~self._occ_filtered).astype(np.int8)
        labeled, _      = nd_label(passable)
        robot_lbl       = labeled[r0, c0]
        if robot_lbl == 0:
            return np.zeros((self.n, self.n), dtype=bool)
        return labeled == robot_lbl

    # ── Frontier clustering and waypoint selection ─────────────────────────

    def _frontier_clusters(self, frontiers: np.ndarray,
                           reach: np.ndarray) -> list[np.ndarray]:
        """Group frontier cells into connected clusters.

        Only reachable (through FREE space from the robot) and non-suppressed
        cells are included.  Returns a list of Nx2 int arrays (grid coords)."""
        if len(frontiers) == 0:
            return []
        front_mask = np.zeros((self.n, self.n), dtype=np.int8)
        front_mask[frontiers[:, 0], frontiers[:, 1]] = 1
        front_mask[~reach]           = 0   # unreachable → drop
        front_mask[self._suppressed] = 0   # previously timed-out → drop
        labeled, n_cl = nd_label(front_mask)
        return [np.argwhere(labeled == i) for i in range(1, n_cl + 1)]

    def _score_cluster(self, cluster: np.ndarray, rob_g: np.ndarray) -> float:
        """Score a frontier cluster: larger and farther clusters in the current
        heading direction that haven't been recently visited score higher."""
        cent   = cluster.mean(axis=0)
        dist_m = np.linalg.norm(cent - rob_g) * self.res + 0.1

        # Base: size^0.4 / distance
        score = len(cluster) ** 0.4 / dist_m

        # Visit penalty at cluster centroid
        ri = int(np.clip(cent[0], 0, self.n - 1))
        ci = int(np.clip(cent[1], 0, self.n - 1))
        novelty = 1.0 / (1.0 + self._visit_grid[ri, ci] * 0.05)

        # Directional bias
        if np.linalg.norm(self._heading) > 0.1:
            to  = (cent - rob_g) * self.res
            to /= np.linalg.norm(to) + 1e-6
            cos_a    = float(np.dot(to, self._heading))
            dir_bias = 1.0 + 0.5 * max(0.0, cos_a)
        else:
            dir_bias = 1.0

        return score * novelty * dir_bias

    def _approach_waypoint(self, cluster: np.ndarray,
                           rob_g: np.ndarray, reach: np.ndarray) -> np.ndarray | None:
        """Find the nearest reachable FREE cell to the cluster centroid that:
          - is reachable through FREE space
          - is at least min_wp_dist from the robot
          - is not blacklisted
        This is the actual navigation goal sent to the planner."""
        cent = cluster.mean(axis=0)

        # Search within ray_max of the cluster centroid
        R = int(self.ray_max / self.res)
        r0 = max(0, int(cent[0]) - R);  r1 = min(self.n, int(cent[0]) + R + 1)
        c0 = max(0, int(cent[1]) - R);  c1 = min(self.n, int(cent[1]) + R + 1)

        sub_ok = reach[r0:r1, c0:c1] & (self.grid[r0:r1, c0:c1] == FREE)
        cands  = np.argwhere(sub_ok)
        if len(cands) == 0:
            return None
        cands += np.array([r0, c0])

        # Min distance from robot
        dr = (cands[:, 0].astype(float) - rob_g[0]) * self.res
        dc = (cands[:, 1].astype(float) - rob_g[1]) * self.res
        dists_rob = np.sqrt(dr**2 + dc**2)
        cands = cands[dists_rob >= self.min_wp_d]
        if len(cands) == 0:
            return None

        # Blacklist
        if self._blacklist:
            wp_w = self.origin + (cands.astype(float) + 0.5) * self.res
            ok   = np.ones(len(cands), dtype=bool)
            for bl in self._blacklist:
                ok &= np.linalg.norm(wp_w - bl, axis=1) >= self.min_wp_d
            cands = cands[ok]
        if len(cands) == 0:
            return None

        # Pick the candidate closest to the cluster centroid
        dr_c = (cands[:, 0].astype(float) - cent[0]) * self.res
        dc_c = (cands[:, 1].astype(float) - cent[1]) * self.res
        i    = int(np.argmin(dr_c**2 + dc_c**2))
        return self.origin + (cands[i].astype(float) + 0.5) * self.res

    # ── planning loop ──────────────────────────────────────────────────────

    def _plan(self):
        if not self.exploring:
            return

        # Decay only unconfirmed hit counts (below threshold).
        # Once a cell is confirmed OCC its count is frozen — it stays OCC until a
        # FREE ray explicitly passes through it (which zeros the count in _mark_ray).
        # This means walls the robot isn't currently looking at are never erased.
        unconfirmed = self._hit_grid < self.occ_thresh
        self._hit_grid[unconfirmed] *= self.hit_decay

        self._update_grid()
        now = time.monotonic()

        # ── Noise-filtered OCC view ───────────────────────────────────────
        # Isolated OCC cells (no 4-connected OCC neighbor) are single-point
        # noise; real walls are always connected. Don't modify the stored grid
        # to avoid oscillation — just use this view for planning queries.
        _occ = self.grid == OCC
        _has_nb = (np.roll(_occ, 1, 0) | np.roll(_occ, -1, 0) |
                   np.roll(_occ, 1, 1) | np.roll(_occ, -1, 1))
        self._occ_filtered = _occ & _has_nb

        # ── Visit density stamp ───────────────────────────────────────────
        # Decay all cells slightly, then mark current robot position.
        # Candidates near high-density areas get penalised in NBV scoring.
        self._visit_grid *= 0.97   # ~50 % decay after ~23 plan ticks at 1 Hz
        g = self._w2g(self.robot_xy).astype(int)
        if 0 <= g[0] < self.n and 0 <= g[1] < self.n:
            self._visit_grid[g[0], g[1]] += 1.0

        # ── Heading update ────────────────────────────────────────────────
        move = self.robot_xy - self._prev_xy
        if np.linalg.norm(move) > 0.05:
            self._heading = move / np.linalg.norm(move)
        self._prev_xy = self.robot_xy.copy()

        # ── Reached or timed out? ─────────────────────────────────────────
        if self.current_wp is not None:
            dist = np.linalg.norm(self.robot_xy - self.current_wp)
            if dist < self.reach:
                self.get_logger().info(
                    f'Waypoint ({self.current_wp[0]:.1f},{self.current_wp[1]:.1f}) reached')
                self.current_wp = None
                self.wp_set_t   = None
            elif self.wp_set_t is not None and now - self.wp_set_t > self.wp_tmo:
                self.get_logger().warn(
                    f'Timeout on ({self.current_wp[0]:.1f},{self.current_wp[1]:.1f}) '
                    f'after {self.wp_tmo:.0f} s — blacklisting + suppressing nearby frontiers')
                self._blacklist.append(self.current_wp.copy())
                # Suppress frontier cells near the failed waypoint.
                # These are likely beyond a low barrier the LiDAR sees over but
                # the robot cannot traverse.
                wp_g = self._w2g(self.current_wp).astype(int)
                R_sup = max(1, int(1.5 / self.res))  # 1.5 m radius — tight, avoids over-suppression
                r0 = max(0, wp_g[0] - R_sup);  r1 = min(self.n, wp_g[0] + R_sup + 1)
                c0 = max(0, wp_g[1] - R_sup);  c1 = min(self.n, wp_g[1] + R_sup + 1)
                self._suppressed[r0:r1, c0:c1] = True
                self.current_wp = None
                self.wp_set_t   = None

        if self.current_wp is not None:
            return  # still heading to current wp

        # ── Find reachable frontier clusters ──────────────────────────────
        frontiers = self._find_frontiers()
        self._viz_frontiers = frontiers
        reach     = self._reachable_mask()
        clusters  = self._frontier_clusters(frontiers, reach)

        # Done = zero reachable unsuppressed clusters — NOT "planner returned None".
        # This is the only reliable signal; planner failures are transient.
        if not clusters:
            if self.no_front_t is None:
                self.no_front_t = now
                reason = 'no frontier cells' if len(frontiers) == 0 else \
                         'all frontiers unreachable or suppressed'
                self.get_logger().info(
                    f'Exploration may be complete ({reason}) — confirming in {self.done_tmo:.0f} s …')
            elif now - self.no_front_t > self.done_tmo:
                self.get_logger().info('Exploration COMPLETE')
                self.pub_done.publish(Bool(data=True))
                self.exploring = False
            return
        self.no_front_t = None

        # Score all clusters and pick the highest-scoring one we can navigate to.
        rob_g = self._w2g(self.robot_xy)
        clusters_by_score = sorted(clusters,
                                   key=lambda c: self._score_cluster(c, rob_g),
                                   reverse=True)
        wp = None
        for cl in clusters_by_score:
            wp = self._approach_waypoint(cl, rob_g, reach)
            if wp is not None:
                break

        if wp is None:
            self.get_logger().warn(
                f'All {len(clusters)} reachable clusters have no valid approach — waiting')
            return

        self.current_wp = wp
        self.wp_set_t   = now
        self.get_logger().info(
            f'→ ({wp[0]:.1f},{wp[1]:.1f})  '
            f'clusters={len(clusters)}  blacklist={len(self._blacklist)}')

    # ── waypoint publisher ─────────────────────────────────────────────────

    def _publish_wp(self):
        if not self.exploring or self.current_wp is None:
            return
        msg = PointStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.point.x = float(self.current_wp[0])
        msg.point.y = float(self.current_wp[1])
        msg.point.z = float(self.robot_z)
        self.pub_wp.publish(msg)

    # ── visualisation ──────────────────────────────────────────────────────

    def _publish_viz(self):
        if not self.exploring:
            return
        stamp = self.get_clock().now().to_msg()
        self._publish_map(stamp)
        self._publish_markers(stamp)

    def _publish_map(self, stamp):
        msg = OccupancyGrid()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'map'
        msg.info.resolution = self.res
        msg.info.width      = self.n
        msg.info.height     = self.n
        msg.info.origin.position.x    = float(self.origin[0])
        msg.info.origin.position.y    = float(self.origin[1])
        msg.info.origin.orientation.w = 1.0
        g    = self.grid.T
        occ  = self._occ_filtered.T
        data = np.where(g == FREE, np.int8(0),
               np.where(occ,       np.int8(100), np.int8(-1)))
        msg.data = data.flatten().tolist()
        self.pub_map.publish(msg)

    def _publish_markers(self, stamp):
        rz = float(self.robot_z)

        # Frontier cells — cyan POINTS
        mk = Marker()
        mk.header.stamp = stamp;  mk.header.frame_id = 'map'
        mk.ns = 'frontiers';  mk.id = 0
        mk.type = Marker.POINTS;  mk.action = Marker.ADD
        mk.scale.x = mk.scale.y = self.res
        mk.color.r = 0.0;  mk.color.g = 1.0;  mk.color.b = 1.0;  mk.color.a = 0.85
        mk.pose.orientation.w = 1.0
        for pt in self._viz_frontiers:
            xy = self._g2w(pt.astype(float))
            p = Point();  p.x, p.y, p.z = float(xy[0]), float(xy[1]), rz + 0.05
            mk.points.append(p)
        self.pub_fronts.publish(mk)

        # Current NBV target — orange sphere
        tgt = Marker()
        tgt.header.stamp = stamp;  tgt.header.frame_id = 'map'
        tgt.ns = 'target';  tgt.id = 0;  tgt.type = Marker.SPHERE
        if self.current_wp is not None:
            tgt.action = Marker.ADD
            tgt.pose.position.x    = float(self.current_wp[0])
            tgt.pose.position.y    = float(self.current_wp[1])
            tgt.pose.position.z    = rz + 0.5
            tgt.pose.orientation.w = 1.0
            tgt.scale.x = tgt.scale.y = tgt.scale.z = 0.6
            tgt.color.r = 1.0;  tgt.color.g = 0.4;  tgt.color.b = 0.0;  tgt.color.a = 1.0
        else:
            tgt.action = Marker.DELETE
        self.pub_target.publish(tgt)


def main():
    rclpy.init()
    rclpy.spin(FrontierExplorer())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
