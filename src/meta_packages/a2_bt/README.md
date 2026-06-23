# a2_bt — Behavior Tree executor and plugins

BT executor node (`bt_action_server`) plus custom BT plugins for the A2 quadruped.
Trees live in `behavior_trees/` and are loaded at startup.

---

## Running

```bash
# Terminal 1 — simulation
a2 sim

# Terminal 2 — object detection (required for ApproachAndPhoto)
a2 detect

# Terminal 3 — exploration / FAR Planner (required for ApproachAndPhoto)
a2 explore

# Terminal 4 — BT executor
a2 bt

# Terminal 5 — trigger a tree
ros2 action send_goal /bt_action_server btcpp_ros2_interfaces/action/ExecuteTree \
  "{target_tree: <TreeID>, payload: ''}"
```

---

## Available trees

### TwistLoop (`behavior_trees/twist_loop.xml`)

Boots the FSM, walks a square (4 sides × straight + turn), then stops and sits.

```
SetMode STAND_UP → SetMode BALANCE_STAND → SetMode VELOCITY_MOVE
→ Repeat 4×: [ PublishTwist forward | PublishTwist turn ]
→ SetMode BALANCE_STAND → SetMode STAND_DOWN
```

**Tune in the XML:**

| Port | Default | Description |
|---|---|---|
| `linear_x` | 0.5 | Forward speed [m/s] |
| `angular_z` | 0.5 | Turn rate [rad/s] |
| `duration_sec` | 3.0 / 7.3 | Straight / turn duration [s] |
| `wait_after_sec` | 0.5 | Settling pause after each command [s] |

---

### ApproachAndPhoto (`behavior_trees/approach_and_photo.xml`)

Boots the FSM, locks onto a detected object, navigates to a safe distance from it via FAR Planner, takes a photo, then sits.

```
SetMode STAND_UP → SetMode BALANCE_STAND → SetMode VELOCITY_MOVE
→ GetObjectPose   (locks onto detection, writes safe goal to blackboard)
→ NavigateToPose  (FAR Planner drives to {target_pose})
→ SetMode BALANCE_STAND
→ SaveImage
→ SetMode STAND_DOWN
```

**Requires:** `a2 detect` and `a2 explore` running before triggering the tree.

**Tune in the XML:**

| Node | Port | Default | Description |
|---|---|---|---|
| `GetObjectPose` | `class_id` | `""` | YOLO class to find (empty = any first detection) |
| `GetObjectPose` | `safe_distance` | 1.0 | Metres to stop from the object |
| `GetObjectPose` | `output_frame` | `map` | TF frame for the goal pose |
| `GetObjectPose` | `timeout_sec` | 15.0 | Abort if no detection in this time [s] |
| `NavigateToPose` | `timeout_sec` | 60.0 | Navigation timeout [s] |
| `SaveImage` | `output_path` | `/tmp/detection.jpg` | Where to save the photo |

---

## Plugins

| Plugin | Node name | Description |
|---|---|---|
| `SetMode` | `SetMode` | Calls `/a2/set_mode` service; skips gracefully if already at or past the requested mode |
| `PublishTwist` | `PublishTwist` | Publishes `TwistStamped` to a topic for `duration_sec`, then waits `wait_after_sec` |
| `GetObjectPose` | `GetObjectPose` | Locks onto first matching YOLO detection, TF-transforms it to the output frame, computes a safe approach point, writes `PoseStamped` to the blackboard |
| `NavigateToPose` | `NavigateToPose` | Sends a goal to FAR Planner via `/goal_point`, waits for `/far_reach_goal_status` |
| `SaveImage` | `SaveImage` | Saves the next frame from an image topic to disk |
| `CreatePose` | `CreatePose` | Utility: constructs a `PoseStamped` from x/y/yaw scalars |
| `CallTriggerService` | `CallTriggerService` | Calls any `std_srvs/Trigger` service; SUCCESS if it reports `success=true` |
| `CallEmptyService` | `CallEmptyService` | Calls any `std_srvs/Empty` service (e.g. resple's `save_map`); SUCCESS once it responds |

## FSM modes

| Value | Name | `a2` CLI |
|---|---|---|
| 1 | `STAND_DOWN` | `a2 sit` |
| 2 | `STAND_UP` | `a2 stand` |
| 3 | `BALANCE_STAND` | `a2 unlock` / `a2 stop` |
| 4 | `VELOCITY_MOVE` | `a2 walk` |
