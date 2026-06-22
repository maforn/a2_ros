# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

ROS 2 (Jazzy) workspace for the Unitree A2 quadruped: MuJoCo simulation, an RL
locomotion policy, LiDAR-inertial odometry, CMU navigation/exploration, object
detection, and behavior-tree autonomy. The same stack runs in sim and on the real
robot, selected by the `A2_MODE` env var (`sim` | `robot`).

## Working environment: everything runs inside Docker

Development happens inside the `a2_ros_dev` compose service — not on the host. The
host has no ROS, no MuJoCo, no colcon. Build/edit flow:

```bash
docker compose build a2_ros_dev && docker compose up -d a2_ros_dev
docker compose exec a2_ros_dev bash   # ROS + workspace auto-sourced via scripts/setup.sh
```

`a2_ros_dev` builds on the prebuilt multi-arch `a2_base` image pulled from GHCR — the
base is **not** built locally unless you set `A2_BASE_IMAGE=a2_ros:base`. See `README.md`
for the full setup, Zenoh router, and `.env` variables.

Source-vs-artifacts split is important: the repo is bind-mounted at `/a2_ros`, but colcon
`build`/`install`/`log` live in named volumes under `$A2_WS_ROOT` (`/a2_ros_ws`), outside
the source tree. That's why those dirs can't simply be `rm`'d — use `a2 clean`.

## The `a2` CLI is the command surface

`scripts/a2` (run as `a2 <cmd>` inside the container) wraps every common operation —
building, launching subsystems, commanding the robot, recording/playing bags. Read it
before adding ad-hoc `ros2 launch`/`colcon` invocations; prefer extending it. Notable:

- `a2 build [pkg] [-j N]` — colcon build (defaults to `--packages-up-to`-style scoping by
  package; parallel workers default to ¼ of cores, override with `-j`/`A2_BUILD_JOBS`).
- `a2 clean [--yes]`, `a2 log`, `a2 env`, `a2 ps`, `a2 down [stack]`.
- Launchers: `a2 sim [--rviz --dlio --headless --scene X]`, `a2 nav`, `a2 explore`,
  `a2 dlio`, `a2 resple`, `a2 detect`, `a2 bt [--tree X]`, `a2 nuc`, `a2 view`.
- Robot mode commands: `a2 stand` / `a2 unlock` / `a2 walk` / `a2 stop` / `a2 sit` map to
  FSM modes via the `/a2/set_mode` service (see FSM below).
- `a2 bag record|play`, `a2 keyboard`, `a2 foxglove`, `a2 plotjuggler`.

`a2`-launched stacks (`_run`) kill any prior instance first and forward SIGINT for
graceful teardown — relevant because Docker Desktop does not kill exec'd processes on
terminal close. `A2_MODE` auto-injects `use_sim_time:=true` and picks `_real` launch
variants where needed.

## Package layout & meta packages

First-party code lives in `src/`; `external/` holds vendored upstreams. **Most packages in
both trees are git submodules** (see `.gitmodules`) — including `src/core/a2_*`,
`src/control/a2_locomotion_controller`, and `src/object_detection`. Editing one means the
submodule workflow in `README.md` (branch inside the submodule, push it first, then bump
the gitlink in this repo). Don't commit changes assuming a single repo.

Non-submodule first-party packages (edit directly here): `src/meta_packages/*`
(`a2_ros`, `a2_bt`, `a2_pc2`, and the dependency-only meta packages `a2_sim`,
`a2_sim_full`, `a2_robot`, `a2_state_estimation`, `a2_object_detection`).

**Meta packages select what to build.** Each declares only `exec_depend`s for a deployment
scenario; you build the one for your target to pull in its deps:

```bash
a2 build a2_sim_full   # simulation + perception
a2 build a2_robot      # real robot (adds hesai_ros_driver, only depended on here)
```

`a2_ros` owns essentially all launch files and config (`src/meta_packages/a2_ros/launch`,
`.../config`) regardless of which meta package is built. `a2_pc2` is the exception: it runs
on a separate compute unit with its own launch files (see `docs/pc2.md`).

## Key architecture

- **Sim/robot abstraction** — `a2_unitree_bridge` (`src/core/a2_unitree_bridge`) is the
  seam between the locomotion stack and either MuJoCo or real Unitree hardware. The rest of
  the stack is written against ROS topics/TF and is largely mode-agnostic.

- **Locomotion FSM** — `a2_utils/mode_fsm` (`src/core/a2_utils`) gates operating-mode
  transitions; `a2_locomotion_controller` runs the ONNX RL policy. Modes:
  `ESTOP(0)`, `STAND_DOWN(1)`, `STAND_UP(2)`, `BALANCE_STAND(3)`, `VELOCITY_MOVE(4)`,
  `FREE(5)`. Transitions are validated — only legal steps are accepted (e.g. you must reach
  `BALANCE_STAND` before `VELOCITY_MOVE`). Acceptance means the FSM took the transition, not
  that the motion finished. Interface types are in `a2_interfaces`
  (`OperatingMode.msg`, `SetOperatingMode.srv`).

- **Behavior trees** — `src/meta_packages/a2_bt` is a BehaviorTree.CPP/ROS2 executor
  (`bt_action_server`). Trees in `behavior_trees/*.xml`, custom nodes paired as
  `include/a2_bt/*.hpp` + `plugins/*.cpp`. BT nodes drive the system through the same public
  surface (`/a2/set_mode`, `/cmd_vel`, FAR Planner `/goal_point`, detection topics). See
  `src/meta_packages/a2_bt/README.md` for trees, ports, and how to trigger via the
  `/bt_action_server` action.

- **Middleware** — Zenoh (`rmw_zenoh_cpp`) by default, selectable via `RMW_IMPLEMENTATION`;
  config rendered per-shell by `scripts/setup.sh` → `a2_deployment_config` scripts. A Zenoh
  router singleton must run (auto-started as a compose service; `a2 router` for a manual
  foreground one).

## Notes

- There is no first-party unit-test suite; verification is by running the stack
  (`a2 sim`, then drive/observe) and `a2 verify` (checks the MuJoCo install).
- `compose.yaml` runs privileged with host networking/IPC; `scripts/setup.sh` raises
  `net.core.rmem_max` in sim so the large MuJoCo lidar PointCloud2 isn't dropped over DDS.
