/**
 * ALO — Autonomous LiDAR-based Occupancy frontier explorer (C++)
 *
 * Pipeline
 * --------
 *  1. /registered_scan (world-frame, Z-up)
 *       → Z-band filter relative to robot body Z:
 *           pz < rz + gs_z_min  → ignore (below-floor noise)
 *           pz < rz + gs_z_max  → ground (skip for map)
 *           pz < rz + z_max_rel → obstacle candidate
 *  2. obstacle cloud → angular-bucketed (0.5°) ray casting into 2-D occupancy grid
 *  3. occupancy grid → frontier detection → BFS reachability → cluster scoring
 *  4. best cluster   → approach waypoint → /goal_point
 *
 * Tune gs_z_min / gs_z_max as offsets from robot body centre Z.
 * Publishes ~/ground and ~/nonground as PointCloud2 for tuning.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <limits>
#include <optional>
#include <unordered_map>
#include <vector>

using SteadyClock = std::chrono::steady_clock;
using TimePoint   = SteadyClock::time_point;

static constexpr uint8_t UNKNOWN = 0;
static constexpr uint8_t FREE    = 1;
static constexpr uint8_t OCC     = 2;

struct Pt2 { float x, y; };
struct Pt3 { float x, y, z; };
struct GC  { int   r, c; };


// Build a minimal PointCloud2 (xyz float32) for publishing.
static sensor_msgs::msg::PointCloud2 makeCloud2(
  const std::vector<Pt3> &pts, const std_msgs::msg::Header &hdr)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header = hdr;  msg.height = 1;
  msg.width  = static_cast<uint32_t>(pts.size());
  msg.is_bigendian = false;  msg.is_dense = true;
  msg.point_step = 12;  msg.row_step = 12 * msg.width;
  for (auto [name, off] : std::vector<std::pair<std::string,uint32_t>>{{"x",0},{"y",4},{"z",8}}) {
    sensor_msgs::msg::PointField f;
    f.name = name; f.offset = off;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32; f.count = 1;
    msg.fields.push_back(f);
  }
  msg.data.resize(msg.row_step);
  auto *p = reinterpret_cast<float*>(msg.data.data());
  for (auto &v : pts) { *p++=v.x; *p++=v.y; *p++=v.z; }
  return msg;
}

class AloNode : public rclcpp::Node
{
public:
  explicit AloNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions{})
  : Node("alo", opts)
  {
    // ── parameters ────────────────────────────────────────────────────────
    res_       = declare_parameter("resolution",          0.15);
    half_w_    = declare_parameter("grid_half_width",    40.0);
    z_max_rel_ = declare_parameter("z_max_rel",           1.8);
    ray_max_   = declare_parameter("max_ray_range",        7.0);
    reach_d_   = declare_parameter("reach_dist",           1.0);
    min_wp_d_  = declare_parameter("min_wp_dist",          1.2);
    clear_r_     = declare_parameter("robot_clear_radius",      0.55);
    nav_clr_     = declare_parameter("nav_clearance",            0.2);
    min_nav_clr_ = declare_parameter("min_nav_clearance",        0.1);
    wp_tmo_    = declare_parameter("wp_timeout",          30.0);
    done_tmo_  = declare_parameter("done_timeout",         8.0);
    hit_decay_   = declare_parameter("occ_hit_decay",        0.95);
    occ_thresh_      = declare_parameter("occ_hit_threshold",    3);
    occ_min_nb_      = declare_parameter("occ_min_neighbors",    1);
    min_cluster_cells_ = declare_parameter("min_cluster_cells",  4);

    const double fov     = declare_parameter("scan_fov_deg",    360.0);
    const double plan_hz = declare_parameter("planning_hz",       1.0);
    const double wp_hz   = declare_parameter("wp_publish_hz",     2.0);
    const double viz_hz  = declare_parameter("viz_hz",            1.0);

    fov_half_ = (fov < 360.0) ? (fov * M_PI / 360.0) : -1.0;

    // ── Ground segmentation (Z-band relative to robot body centre) ────────
    // gs_z_min: ignore points below rz+gs_z_min (sub-floor noise).
    // gs_z_max: classify as ground if below rz+gs_z_max (skipped for map).
    // Everything from rz+gs_z_max up to rz+z_max_rel is an obstacle candidate.
    gs_z_min_ = declare_parameter("gs_z_min", -0.6);
    gs_z_max_ = declare_parameter("gs_z_max", -0.1);

    // ── Lidar input & voxel downsample ────────────────────────────────────
    // /registered_scan is already in world frame (published by registered_scan_pub).
    lidar_topic_ = declare_parameter("lidar_topic", std::string("/registered_scan"));
    voxel_size_  = declare_parameter("voxel_size",  0.10);

    // ── grid ──────────────────────────────────────────────────────────────
    n_ = static_cast<int>(2.0 * half_w_ / res_) + 1;
    grid_      .assign(n_ * n_, UNKNOWN);
    hit_grid_  .assign(n_ * n_, 0.0f);
    visit_grid_.assign(n_ * n_, 0.0f);
    occ_filt_  .assign(n_ * n_, false);

    // ── subscribers ───────────────────────────────────────────────────────
    // Grid update runs at full lidar rate so walls confirm quickly.
    // Planning (BFS/clustering) still runs at planning_hz via plan_timer_.
    scan_sub_  = create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_, 10,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr m){
        last_scan_ = m;
        updateGrid();
        morphCleanup();
      });
    odom_sub_  = create_subscription<nav_msgs::msg::Odometry>(
      "/state_estimation", 10,
      std::bind(&AloNode::odomCb, this, std::placeholders::_1));
    start_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/start_exploration", 10,
      std::bind(&AloNode::startCb, this, std::placeholders::_1));

    // ── publishers ────────────────────────────────────────────────────────
    wp_pub_   = create_publisher<geometry_msgs::msg::PointStamped>("/goal_point", 1);
    done_pub_ = create_publisher<std_msgs::msg::Bool>("exploration_finish", 1);
    map_pub_  = create_publisher<nav_msgs::msg::OccupancyGrid>("~/map", 1);
    frt_pub_   = create_publisher<visualization_msgs::msg::Marker>("~/frontiers", 1);
    clust_pub_ = create_publisher<visualization_msgs::msg::Marker>("~/best_cluster", 1);
    tgt_pub_   = create_publisher<visualization_msgs::msg::Marker>("~/target", 1);
    gnd_pub_  = create_publisher<sensor_msgs::msg::PointCloud2>("~/ground", 1);
    ngnd_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/nonground", 1);

    // ── timers ────────────────────────────────────────────────────────────
    plan_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / plan_hz), std::bind(&AloNode::plan,   this));
    wp_timer_   = create_wall_timer(
      std::chrono::duration<double>(1.0 / wp_hz),   std::bind(&AloNode::pubWp,  this));
    viz_timer_  = create_wall_timer(
      std::chrono::duration<double>(1.0 / viz_hz),  std::bind(&AloNode::pubViz, this));

    RCLCPP_INFO(get_logger(), "ALO C++ ready — %d×%d grid @ %.2f m/cell (±%.0f m). "
      "Waiting for /start_exploration …", n_, n_, res_, half_w_);
  }

private:
  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  grid helpers  ━━━━━━━━━━━━━

  int  idx(int r, int c) const { return r * n_ + c; }
  bool inBounds(int r, int c) const { return r >= 0 && r < n_ && c >= 0 && c < n_; }
  GC   w2g(float x, float y) const
       { return { int((x - ox_) / res_), int((y - oy_) / res_) }; }
  Pt2  g2w(int r, int c) const
       { return { float(ox_ + (r+0.5)*res_), float(oy_ + (c+0.5)*res_) }; }

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  Bresenham ray  ━━━━━━━━━━━━

  void markRay(float x0, float y0, float x1, float y1, bool occ_end)
  {
    int r = int(x0), c = int(y0), r1 = int(x1), c1 = int(y1);
    const int dr = std::abs(r1-r), dc = std::abs(c1-c);
    const int sr = (r < r1) ? 1 : -1, sc = (c < c1) ? 1 : -1;
    int err = dr - dc;
    for (;;) {
      const bool in = inBounds(r, c);
      if (r == r1 && c == c1) {
        if (in) {
          if (occ_end) {
            float &h = hit_grid_[idx(r,c)];
            if (++h >= occ_thresh_) grid_[idx(r,c)] = OCC;
          } else {
            hit_grid_[idx(r,c)] = 0.0f;
            grid_[idx(r,c)] = FREE;
          }
        }
        return;
      }
      if (in) {
        const int i = idx(r,c);
        if (grid_[i] != OCC) { hit_grid_[i] = 0.0f; grid_[i] = FREE; }
        // OCC cells are never ray-cleared — morphCleanup() handles noise removal.
      }
      const int e2 = 2 * err;
      if (e2 > -dc) { err -= dc; r += sr; }
      if (e2 <  dr) { err += dr; c += sc; }
    }
  }

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  footprint clear  ━━━━━━━━━━

  void clearFootprint(float gx, float gy)
  {
    const int   cr  = int(clear_r_ / res_) + 1;
    const float cr2 = float(clear_r_ / res_) * float(clear_r_ / res_);
    for (int r = std::max(0, int(gx)-cr); r < std::min(n_, int(gx)+cr+1); ++r)
      for (int c = std::max(0, int(gy)-cr); c < std::min(n_, int(gy)+cr+1); ++c) {
        const float dr = r-gx, dc = c-gy;
        if (dr*dr + dc*dc <= cr2) {
          hit_grid_[idx(r,c)] = 0.0f;
          grid_[idx(r,c)] = FREE;
        }
      }
  }

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  OCC noise filter  ━━━━━━━━━

  void computeOccFiltered()
  {
    std::fill(occ_filt_.begin(), occ_filt_.end(), false);
    for (int r = 0; r < n_; ++r)
      for (int c = 0; c < n_; ++c) {
        if (grid_[idx(r,c)] != OCC) continue;
        if ((r > 0    && grid_[idx(r-1,c)] == OCC) ||
            (r < n_-1 && grid_[idx(r+1,c)] == OCC) ||
            (c > 0    && grid_[idx(r,c-1)] == OCC) ||
            (c < n_-1 && grid_[idx(r,c+1)] == OCC))
          occ_filt_[idx(r,c)] = true;
      }
  }

  // Morphological erosion: remove OCC cells with fewer than occ_min_nb_ 4-connected
  // OCC neighbours. Isolated noise cells (0 neighbours) are cleared immediately;
  // connected walls (each cell has ≥1 neighbour along the wall) are preserved.
  // Call AFTER updateGrid() and BEFORE computeOccFiltered().
  void morphCleanup()
  {
    for (int r = 1; r < n_-1; ++r) {
      for (int c = 1; c < n_-1; ++c) {
        const int i = idx(r, c);
        if (grid_[i] != OCC) continue;
        int nb = 0;
        if (grid_[idx(r-1,c)] == OCC) ++nb;
        if (grid_[idx(r+1,c)] == OCC) ++nb;
        if (grid_[idx(r,c-1)] == OCC) ++nb;
        if (grid_[idx(r,c+1)] == OCC) ++nb;
        if (nb < occ_min_nb_) {
          hit_grid_[i] = 0.0f;
          grid_[i] = UNKNOWN;
        }
      }
    }
  }

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  BFS reachability  ━━━━━━━━━

  std::vector<bool> reachMask(int r0, int c0) const
  {
    std::vector<bool> reach(n_ * n_, false);
    if (!inBounds(r0, c0)) return reach;

    auto passable = [&](int r, int c) -> bool {
      if (!inBounds(r, c) || occ_filt_[idx(r,c)]) return false;
      return grid_[idx(r,c)] != OCC;  // FREE and UNKNOWN are passable; only confirmed OCC blocks
    };

    if (!passable(r0, c0)) return reach;
    std::deque<GC> q;
    reach[idx(r0,c0)] = true;  q.push_back({r0, c0});
    static constexpr int DR[] = {-1, 1, 0, 0};
    static constexpr int DC[] = { 0, 0,-1, 1};
    while (!q.empty()) {
      auto [r, c] = q.front(); q.pop_front();
      for (int d = 0; d < 4; ++d) {
        int nr = r+DR[d], nc = c+DC[d];
        if (!inBounds(nr,nc) || reach[idx(nr,nc)] || !passable(nr,nc)) continue;
        reach[idx(nr,nc)] = true;  q.push_back({nr, nc});
      }
    }
    return reach;
  }

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  frontier detection  ━━━━━━━

  std::vector<GC> findFrontiers() const
  {
    std::vector<GC> out;
    static constexpr int DR[] = {-1, 1, 0, 0};
    static constexpr int DC[] = { 0, 0,-1, 1};
    for (int r = 0; r < n_; ++r)
      for (int c = 0; c < n_; ++c) {
        if (grid_[idx(r,c)] != FREE) continue;
        for (int d = 0; d < 4; ++d) {
          int nr = r+DR[d], nc = c+DC[d];
          if (inBounds(nr,nc) && grid_[idx(nr,nc)] == UNKNOWN) {
            out.push_back({r,c}); break;
          }
        }
      }
    return out;
  }

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  clustering  ━━━━━━━━━━━━━━━

  struct Cluster { std::vector<GC> cells; float cent_r, cent_c; };

  std::vector<Cluster> clusterFrontiers(const std::vector<GC> &fronts,
                                        const std::vector<bool> &reach) const
  {
    std::vector<bool> fmask(n_ * n_, false);
    for (auto &f : fronts)
      if (reach[idx(f.r,f.c)])
        fmask[idx(f.r,f.c)] = true;

    std::vector<bool> visited(n_ * n_, false);
    std::vector<Cluster> clusters;
    static constexpr int DR[] = {-1, 1, 0, 0};
    static constexpr int DC[] = { 0, 0,-1, 1};
    for (int r = 0; r < n_; ++r)
      for (int c = 0; c < n_; ++c) {
        if (!fmask[idx(r,c)] || visited[idx(r,c)]) continue;
        Cluster cl;
        std::deque<GC> q;
        q.push_back({r,c});  visited[idx(r,c)] = true;
        float sr = 0, sc = 0;
        while (!q.empty()) {
          auto [cr, cc] = q.front(); q.pop_front();
          cl.cells.push_back({cr,cc}); sr += cr; sc += cc;
          for (int d = 0; d < 4; ++d) {
            int nr = cr+DR[d], nc = cc+DC[d];
            if (!inBounds(nr,nc) || visited[idx(nr,nc)] || !fmask[idx(nr,nc)]) continue;
            visited[idx(nr,nc)] = true;  q.push_back({nr,nc});
          }
        }
        if (int(cl.cells.size()) < min_cluster_cells_) continue;
        const float n = float(cl.cells.size());
        cl.cent_r = sr/n;  cl.cent_c = sc/n;
        clusters.push_back(std::move(cl));
      }
    return clusters;
  }

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  scoring  ━━━━━━━━━━━━━━━━━━

  double scoreCluster(const Cluster &cl, float rob_r, float rob_c) const
  {
    const float dr = (cl.cent_r - rob_r) * res_, dc = (cl.cent_c - rob_c) * res_;
    double score = std::pow(double(cl.cells.size()), 0.4) / (std::hypot(dr,dc) + 0.1);
    const int ri = int(std::clamp(cl.cent_r, 0.f, float(n_-1)));
    const int ci = int(std::clamp(cl.cent_c, 0.f, float(n_-1)));
    score *= 1.0 / (1.0 + visit_grid_[idx(ri,ci)] * 0.05);
    if (head_x_*head_x_ + head_y_*head_y_ > 0.01) {
      const double len = std::hypot(double(dr), double(dc)) + 1e-6;
      score *= 1.0 + 0.5 * std::max(0.0, (dr*head_x_ + dc*head_y_) / len);
    }
    // Penalise clusters hemmed against confirmed walls: count OCC cells within
    // 1 cell of the centroid. Wall-boundary clusters score lower so the robot
    // prefers frontiers that open into genuinely unexplored space.
    {
      int occ_nb = 0;
      const int cr = std::clamp(ri, 1, n_-2), cc = std::clamp(ci, 1, n_-2);
      static constexpr int DR[] = {-1,1,0,0,-1,-1,1,1};
      static constexpr int DC[] = {0,0,-1,1,-1,1,-1,1};
      for (int d = 0; d < 8; ++d)
        if (grid_[idx(cr+DR[d], cc+DC[d])] == OCC) ++occ_nb;
      score *= 1.0 / (1.0 + occ_nb * 0.5);
    }
    return score;
  }

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  approach waypoint  ━━━━━━━━

  std::optional<Pt2> approachWpWithClr(const Cluster &cl, float rob_r, float rob_c,
                                        const std::vector<bool> &reach, float clr) const
  {
    const int R    = int(ray_max_ / res_) + 1;
    const int r0   = std::max(0,  int(cl.cent_r)-R), r1 = std::min(n_, int(cl.cent_r)+R+1);
    const int c0   = std::max(0,  int(cl.cent_c)-R), c1 = std::min(n_, int(cl.cent_c)+R+1);
    const float min_d2 = float(min_wp_d_/res_) * float(min_wp_d_/res_);
    const int   nav_cr = int(clr/res_) + 1;
    const float nav_r2 = float(clr/res_) * float(clr/res_);
    float best_d2 = std::numeric_limits<float>::max();
    std::optional<Pt2> best;

    for (int r = r0; r < r1; ++r) {
      for (int c = c0; c < c1; ++c) {
        if (!reach[idx(r,c)] || grid_[idx(r,c)] != FREE) continue;
        const float drob = float(r-rob_r), dcob = float(c-rob_c);
        if (drob*drob + dcob*dcob < min_d2) continue;
        bool clear = true;
        for (int nr = r-nav_cr; nr <= r+nav_cr && clear; ++nr)
          for (int nc = c-nav_cr; nc <= c+nav_cr && clear; ++nc)
            if (inBounds(nr,nc) && grid_[idx(nr,nc)] == OCC) {
              const float dd = float(nr-r)*float(nr-r) + float(nc-c)*float(nc-c);
              if (dd < nav_r2) clear = false;
            }
        if (!clear) continue;
        Pt2 wp = g2w(r,c);
        const float drc = float(r-cl.cent_r), dcc = float(c-cl.cent_c);
        const float d2c = drc*drc + dcc*dcc;
        if (d2c < best_d2) { best_d2 = d2c; best = wp; }
      }
    }
    return best;
  }

  // Try full nav_clearance first; fall back to min_nav_clr_ for narrow passages.
  std::optional<Pt2> approachWp(const Cluster &cl, float rob_r, float rob_c,
                                 const std::vector<bool> &reach) const
  {
    auto wp = approachWpWithClr(cl, rob_r, rob_c, reach, nav_clr_);
    if (!wp && min_nav_clr_ < nav_clr_)
      wp = approachWpWithClr(cl, rob_r, rob_c, reach, min_nav_clr_);
    return wp;
  }

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  callbacks  ━━━━━━━━━━━━━━━━

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    rx_ = msg->pose.pose.position.x;
    ry_ = msg->pose.pose.position.y;
    rz_ = msg->pose.pose.position.z;
    const auto &q = msg->pose.pose.orientation;
    ryaw_ = std::atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
  }

  void startCb(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      if (exploring_) return;
      const float hw = float(n_/2) * res_;
      ox_ = float(rx_) - hw;  oy_ = float(ry_) - hw;
      std::fill(grid_.begin(),       grid_.end(),       UNKNOWN);
      std::fill(hit_grid_.begin(),   hit_grid_.end(),   0.0f);
      std::fill(visit_grid_.begin(), visit_grid_.end(), 0.0f);
      std::fill(occ_filt_.begin(),   occ_filt_.end(),   false);
      current_wp_.reset(); wp_set_t_.reset(); no_front_t_.reset();
      viz_fronts_.clear(); viz_cluster_.reset();
      head_x_ = head_y_ = 0.0;
      prev_rx_ = float(rx_); prev_ry_ = float(ry_);
      exploring_ = true;
      RCLCPP_INFO(get_logger(), "Exploration started — origin (%.1f, %.1f)", ox_, oy_);
    } else {
      if (exploring_) RCLCPP_INFO(get_logger(), "Exploration stopped");
      exploring_ = false;  current_wp_.reset();
    }
  }

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  grid update  ━━━━━━━━━━━━━━

  void updateGrid()
  {
    if (!last_scan_) return;
    const auto &msg = *last_scan_;

    // ── Parse PointCloud2 field offsets ───────────────────────────────────
    uint32_t xo = 0, yo = 4, zo = 8;
    for (auto &f : msg.fields) {
      if      (f.name == "x") xo = f.offset;
      else if (f.name == "y") yo = f.offset;
      else if (f.name == "z") zo = f.offset;
    }
    const uint32_t ps  = msg.point_step;
    const uint32_t np  = msg.width * msg.height;
    const uint8_t *raw = msg.data.data();

    const float rx = float(rx_), ry = float(ry_);
    const float ray_max_f = float(ray_max_);

    // Z thresholds in world frame (gs_z_min/max are offsets from robot body Z)
    const float z_bot = float(rz_) + float(gs_z_min_);  // below → ignore
    const float z_gnd = float(rz_) + float(gs_z_max_);  // below → ground
    const float z_top = float(rz_) + float(z_max_rel_); // above → ceiling

    // ── Pass 1: range + FOV filter → voxel downsample ─────────────────────
    // Filter first so the voxel grid only contains points that are within the
    // useful work area. Out-of-range / out-of-FOV points are dropped immediately.
    const float inv_vox = 1.0f / float(voxel_size_);
    std::unordered_map<uint64_t, Pt3> voxels;
    voxels.reserve(2048);
    for (uint32_t i = 0; i < np; ++i) {
      const uint8_t *p = raw + i * ps;
      float x, y, z;
      std::memcpy(&x, p+xo, 4); std::memcpy(&y, p+yo, 4); std::memcpy(&z, p+zo, 4);
      if (!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)) continue;
      const float dx = x - rx, dy = y - ry;
      const float d  = std::hypot(dx, dy);
      if (d < 0.05f || d > ray_max_f) continue;          // range gate
      if (fov_half_ > 0.0) {
        const double az  = std::atan2(dy, dx);
        const double rel = std::fmod(az - ryaw_ + M_PI, 2*M_PI) - M_PI;
        if (std::abs(rel) > fov_half_) continue;          // FOV gate
      }
      const int ix = int(std::floor(x * inv_vox));
      const int iy = int(std::floor(y * inv_vox));
      const int iz = int(std::floor(z * inv_vox));
      const uint64_t key = (uint64_t(uint32_t(ix+512000)) << 40) |
                           (uint64_t(uint32_t(iy+512000)) << 20) |
                            uint64_t(uint32_t(iz+512000));
      voxels.try_emplace(key, Pt3{x, y, z});
    }

    // ── Pass 2: debug publish (lazy) ──────────────────────────────────────
    const bool pub_gnd  = gnd_pub_ ->get_subscription_count() > 0;
    const bool pub_ngnd = ngnd_pub_->get_subscription_count() > 0;
    std::vector<Pt3> gnd_pts, ngnd_pts;
    if (pub_gnd || pub_ngnd) {
      for (auto &[k, pt] : voxels) {
        if (pt.z < z_bot || pt.z > z_top) continue;
        if (pt.z < z_gnd) { if (pub_gnd)  gnd_pts .push_back(pt); }
        else              { if (pub_ngnd) ngnd_pts.push_back(pt); }
      }
      if (pub_gnd)  gnd_pub_ ->publish(makeCloud2(gnd_pts,  msg.header));
      if (pub_ngnd) ngnd_pub_->publish(makeCloud2(ngnd_pts, msg.header));
    }

    // ── Pass 3: Z-band filter + angular bucketing → ray casting ──────────
    // One ray per 0.5° azimuth bin: OCC beats FREE; closest OCC; farthest FREE.
    static constexpr double BIN_RAD = 0.5 * M_PI / 180.0;
    struct Bucket { float tx, ty; bool occ; float dist; };
    std::unordered_map<int, Bucket> best;
    best.reserve(800);

    for (auto &[k, pt] : voxels) {
      const float px = pt.x, py = pt.y, pz = pt.z;
      if (pz < z_gnd || pz > z_top) continue;  // keep only obstacle zone: [z_gnd, z_top]
      const float dx = px - rx, dy = py - ry;
      const float d  = std::hypot(dx, dy);
      // All points already passed range gate — mark as OCC (within ray_max).
      // (d > ray_max_ can't happen here, but clip endpoint just in case.)
      const bool  occ = d <= ray_max_f;
      const float tx  = occ ? px : rx + dx * ray_max_f / d;
      const float ty  = occ ? py : ry + dy * ray_max_f / d;
      const int bin = int(std::floor(std::atan2(dy, dx) / BIN_RAD));
      auto [it, ins] = best.try_emplace(bin, Bucket{tx, ty, occ, d});
      if (!ins) {
        auto &cur = it->second;
        if      ( occ && !cur.occ)                 cur = {tx, ty, occ, d};
        else if ( occ &&  cur.occ && d < cur.dist)  cur = {tx, ty, occ, d};
        else if (!occ && !cur.occ && d > cur.dist)  cur = {tx, ty, occ, d};
      }
    }

    const float rob_gx = (rx_ - ox_) / res_, rob_gy = (ry_ - oy_) / res_;
    for (auto &[bin, bk] : best) {
      const float tgx = (bk.tx - ox_) / res_, tgy = (bk.ty - oy_) / res_;
      markRay(rob_gx, rob_gy, tgx, tgy, bk.occ);
    }
    clearFootprint(rob_gx, rob_gy);
  }

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  planning loop  ━━━━━━━━━━━━

  void plan()
  {
    auto now = SteadyClock::now();

    // Decay unconfirmed hits at planning rate (keeps noise from accumulating between scans).
    for (int i = 0; i < n_*n_; ++i)
      if (grid_[i] != OCC) hit_grid_[i] *= float(hit_decay_);
    // updateGrid + morphCleanup run in the scan callback at full lidar rate.

    if (!exploring_) return;
    computeOccFiltered();

    for (auto &v : visit_grid_) v *= 0.97f;
    auto [rob_r, rob_c] = w2g(float(rx_), float(ry_));
    if (inBounds(rob_r, rob_c)) visit_grid_[idx(rob_r, rob_c)] += 1.0f;

    const float dmx = float(rx_)-prev_rx_, dmy = float(ry_)-prev_ry_;
    const float dm  = std::hypot(dmx, dmy);
    if (dm > 0.05f) { head_x_ = dmx/dm; head_y_ = dmy/dm; }
    prev_rx_ = float(rx_); prev_ry_ = float(ry_);

    if (current_wp_) {
      const float dist = std::hypot(float(rx_)-current_wp_->x, float(ry_)-current_wp_->y);
      if (dist < float(reach_d_)) {
        RCLCPP_INFO(get_logger(), "Waypoint (%.1f,%.1f) reached", current_wp_->x, current_wp_->y);
        current_wp_.reset(); wp_set_t_.reset();
      } else if (wp_set_t_) {
        const double elapsed = std::chrono::duration<double>(now - *wp_set_t_).count();
        if (elapsed > wp_tmo_) {
          RCLCPP_WARN(get_logger(), "Timeout %.0f s — skipping (%.1f,%.1f), replanning",
                      wp_tmo_, current_wp_->x, current_wp_->y);
          current_wp_.reset(); wp_set_t_.reset();
        }
      }
    }
    if (current_wp_) return;

    auto fronts   = findFrontiers();
    viz_fronts_   = fronts;
    auto reach    = reachMask(rob_r, rob_c);
    auto clusters = clusterFrontiers(fronts, reach);

    if (clusters.empty()) {
      if (!no_front_t_) {
        no_front_t_ = now;
        RCLCPP_INFO(get_logger(), "No reachable frontiers — confirming in %.0f s", done_tmo_);
      } else if (std::chrono::duration<double>(now - *no_front_t_).count() > done_tmo_) {
        RCLCPP_INFO(get_logger(), "Exploration COMPLETE");
        std_msgs::msg::Bool done_msg;  done_msg.data = true;
        done_pub_->publish(done_msg);
        exploring_ = false;
      }
      return;
    }
    no_front_t_.reset();

    std::sort(clusters.begin(), clusters.end(), [&](const Cluster &a, const Cluster &b) {
      return scoreCluster(a, rob_r, rob_c) > scoreCluster(b, rob_r, rob_c);
    });

    for (auto &cl : clusters) {
      auto wp = approachWp(cl, float(rob_r), float(rob_c), reach);
      if (!wp) continue;
      current_wp_ = wp;  wp_set_t_ = now;
      // Store cluster centroid for ring visualisation
      auto cw = g2w(int(cl.cent_r), int(cl.cent_c));
      const float r = std::sqrt(float(cl.cells.size())) * float(res_) * 0.5f;
      viz_cluster_ = ClusterViz{cw.x, cw.y, std::max(r, 0.5f)};
      RCLCPP_INFO(get_logger(), "→ (%.1f,%.1f)  clusters=%zu",
                  wp->x, wp->y, clusters.size());
      break;
    }
  }

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  waypoint publisher  ━━━━━━━

  void pubWp()
  {
    if (!exploring_ || !current_wp_) return;
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp    = get_clock()->now();
    msg.header.frame_id = "map";
    msg.point.x = current_wp_->x;
    msg.point.y = current_wp_->y;
    msg.point.z = rz_;
    wp_pub_->publish(msg);
  }

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  visualisation  ━━━━━━━━━━━━

  void pubViz() { if (exploring_) { pubMap(); pubMarkers(); } }

  void pubMap()
  {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp    = get_clock()->now();
    msg.header.frame_id = "map";
    msg.info.resolution = float(res_);
    msg.info.width      = n_;
    msg.info.height     = n_;
    msg.info.origin.position.x    = ox_;
    msg.info.origin.position.y    = oy_;
    msg.info.origin.orientation.w = 1.0;
    msg.data.resize(n_ * n_);
    for (int r = 0; r < n_; ++r)
      for (int c = 0; c < n_; ++c) {
        const int8_t val = (grid_[idx(r,c)] == FREE)  ? int8_t(0)
                         : occ_filt_[idx(r,c)]        ? int8_t(100)
                         :                              int8_t(-1);
        msg.data[c * n_ + r] = val;
      }
    map_pub_->publish(msg);
  }

  void pubMarkers()
  {
    const auto stamp = get_clock()->now();
    const std::string frame = "map";

    // ── 1. All frontier cells — small cyan squares ─────────────────────────
    visualization_msgs::msg::Marker frt;
    frt.header.stamp = stamp; frt.header.frame_id = frame;
    frt.ns = "frontiers"; frt.id = 0;
    frt.type   = visualization_msgs::msg::Marker::POINTS;
    frt.action = visualization_msgs::msg::Marker::ADD;
    frt.scale.x = frt.scale.y = float(res_);
    frt.color.r = 0.f; frt.color.g = 1.f; frt.color.b = 1.f; frt.color.a = 0.7f;
    frt.pose.orientation.w = 1.0;
    for (auto &f : viz_fronts_) {
      auto [wx, wy] = g2w(f.r, f.c);
      geometry_msgs::msg::Point p; p.x = wx; p.y = wy; p.z = rz_ + 0.05;
      frt.points.push_back(p);
    }
    frt_pub_->publish(frt);

    // ── 2. Best cluster — yellow ring centred on chosen cluster ────────────
    visualization_msgs::msg::Marker ring;
    ring.header.stamp = stamp; ring.header.frame_id = frame;
    ring.ns = "best_cluster"; ring.id = 0;
    ring.type = visualization_msgs::msg::Marker::LINE_STRIP;
    ring.pose.orientation.w = 1.0;
    ring.scale.x = 0.08f;  // line width
    if (viz_cluster_) {
      ring.action = visualization_msgs::msg::Marker::ADD;
      ring.color.r = 1.f; ring.color.g = 0.9f; ring.color.b = 0.f; ring.color.a = 1.f;
      constexpr int N = 36;
      for (int i = 0; i <= N; ++i) {
        const float a = float(i) * 2.f * float(M_PI) / float(N);
        geometry_msgs::msg::Point p;
        p.x = viz_cluster_->cx + viz_cluster_->radius * std::cos(a);
        p.y = viz_cluster_->cy + viz_cluster_->radius * std::sin(a);
        p.z = rz_ + 0.1f;
        ring.points.push_back(p);
      }
    } else {
      ring.action = visualization_msgs::msg::Marker::DELETE;
    }
    clust_pub_->publish(ring);

    // ── 3. Approach waypoint — orange sphere (the actual nav goal) ─────────
    visualization_msgs::msg::Marker tgt;
    tgt.header.stamp = stamp; tgt.header.frame_id = frame;
    tgt.ns = "target"; tgt.id = 0;
    tgt.type = visualization_msgs::msg::Marker::SPHERE;
    tgt.pose.orientation.w = 1.0;
    if (current_wp_) {
      tgt.action = visualization_msgs::msg::Marker::ADD;
      tgt.pose.position.x = current_wp_->x;
      tgt.pose.position.y = current_wp_->y;
      tgt.pose.position.z = rz_ + 0.5;
      tgt.scale.x = tgt.scale.y = tgt.scale.z = 0.45f;
      tgt.color.r = 1.f; tgt.color.g = 0.35f; tgt.color.b = 0.f; tgt.color.a = 1.f;
    } else {
      tgt.action = visualization_msgs::msg::Marker::DELETE;
    }
    tgt_pub_->publish(tgt);
  }

  // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━  members  ━━━━━━━━━━━━━━━━━━

  double res_, half_w_, z_max_rel_, ray_max_, fov_half_;
  double reach_d_, min_wp_d_, clear_r_, nav_clr_, min_nav_clr_, wp_tmo_, done_tmo_, hit_decay_;
  double gs_z_min_, gs_z_max_, voxel_size_;
  std::string lidar_topic_;
  int    occ_thresh_, occ_min_nb_, min_cluster_cells_, n_;

  float ox_ = 0.f, oy_ = 0.f;
  std::vector<uint8_t> grid_;
  std::vector<float>   hit_grid_, visit_grid_;
  std::vector<bool>    occ_filt_;

  double rx_ = 0., ry_ = 0., rz_ = 0., ryaw_ = 0.;
  float  prev_rx_ = 0.f, prev_ry_ = 0.f;
  float  head_x_ = 0.f, head_y_ = 0.f;

  struct ClusterViz { float cx, cy, radius; };

  bool                        exploring_ = false;
  std::optional<Pt2>          current_wp_;
  std::optional<ClusterViz>   viz_cluster_;
  std::optional<TimePoint>    wp_set_t_, no_front_t_;
  std::vector<GC>             viz_fronts_;

  sensor_msgs::msg::PointCloud2::SharedPtr last_scan_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr           start_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr  wp_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr               done_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr      map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr   frt_pub_, clust_pub_, tgt_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr     gnd_pub_, ngnd_pub_;
  rclcpp::TimerBase::SharedPtr plan_timer_, wp_timer_, viz_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AloNode>());
  rclcpp::shutdown();
  return 0;
}
