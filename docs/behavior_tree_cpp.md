# Behavior trees with BehaviorTree.CPP (`tardigrade_bt`)

`tardigrade_bt` runs missions written as BehaviorTree.CPP 4 XML files. A
mission is a text file in `src/tardigrade_bt/trees/`, so you can change what
the robot does without recompiling. The trees open in Groot2 on Windows or
Linux, and you can write and inspect them without Groot2 (for example on a Mac).

```text
trees/*.xml  ->  mission_runner (ticks the tree 10x per second)
   -> /tardigrade/control/velocity_setpoint/mission (TwistStamped, base_link)
   -> velocity_setpoint_mux -> controller -> allocator -> Unity or ESP
   <- /tardigrade/state/odometry/filtered, /tardigrade/status
```

The tree only sends velocity setpoints. The controller, allocator and ESP
bridge stay unchanged, and the mux still decides which source is in control.

## Safety rules

- `dry_run` is **true** by default: the tree is loaded and printed, nothing moves.
- Trees **never arm** the robot. `IsArmed` only checks it. A person arms it.
- Every example tree re-checks `OdometryFresh` and `IsArmed` on every tick
  (`ReactiveSequence`), puts a `Timeout` on every move, and falls back to
  `Surface` if anything fails.
- Every move sends zero when it ends for any reason. If the runner is killed
  with Ctrl+C, the mux zeroes the stale mission command within 0.5 s.

## Setup

The Docker image builds BehaviorTree.CPP 4.6.2 from source, so rebuild the
image once:

```bash
./docker-build.sh --build --detached
docker exec -it $(docker ps -q | head -1) bash
./build.sh --pkg tardigrade_interfaces
./build.sh --pkg tardigrade_bt
source install/setup.bash
```

`build.sh` uses `--symlink-install`, so after that first build you can edit an
existing tree XML and run it again without rebuilding. A brand-new tree file
needs one `./build.sh --pkg tardigrade_bt` (or pass its full path as `tree`).

## Run a tree

Check a tree loads (nothing moves):

```bash
ros2 launch tardigrade_bt bt_mission.launch.py tree:=playground
```

Run it in Unity: start `unity-op`, press Play, reset, enable external
control, arm, set the mux to `mission` (see `docs/unity_sil.md`), then:

```bash
ros2 launch tardigrade_bt bt_mission.launch.py tree:=dive_test dry_run:=false
```

`tree` is a name from `trees/` or a full path to any XML file. The runner
prints each step as it starts and ends:

```text
[mission_runner]: Dive 0.5 m: RUNNING
[mission_runner]: Dive 0.5 m: SUCCESS
[mission_runner]: Hold: RUNNING
...
[mission_runner]: Mission finished: SUCCESS
```

The same lines go to `/tardigrade/mission/status` (std_msgs/String) for
Foxglove and rosbag.

| Tree | What it does |
|---|---|
| `dive_test` | Dive 0.5 m, hold 5 s, return to the start depth. No driving or turning. |
| `playground` | Dive, forward 2 m, backward 2 m, turn left 90°, forward 2 m. |
| `square` | Dive, then a 2 m square, ending at the start facing the start heading. |

## See a tree without Groot2 (Mac)

```bash
python3 src/tardigrade_bt/scripts/bt_view.py src/tardigrade_bt/trees/playground.xml
python3 src/tardigrade_bt/scripts/bt_view.py src/tardigrade_bt/trees/playground.xml --mermaid playground.md
```

The first prints the tree in the terminal. The second writes a Mermaid
diagram that GitHub shows as a picture, and VS Code shows in the Markdown
preview with the "Markdown Preview Mermaid Support" extension.

## Groot2 (Windows or Linux)

Open any file in `trees/` in Groot2 to view or edit it. To watch a running
mission live, connect Groot2's monitor to the computer running the container
on port 1667 (the container publishes ports 1667 and 1668).

## Nodes you can use

Distances are metres, speeds m/s, headings degrees **left** of the heading the
robot had at `RecordStart`. Depth is metres below the start depth.

| Node | Kind | Ports (XML attributes) | Succeeds when |
|---|---|---|---|
| `WaitForOdometry` | action | `timeout_sec`=30, `max_age_sec`=0.5 | odometry arrives (fails after timeout) |
| `RecordStart` | action | | start pose saved (fails with no odometry) |
| `OdometryFresh` | condition | `max_age_sec`=0.5 | odometry is recent |
| `IsArmed` | condition | | `/tardigrade/status` says armed |
| `GoToDepth` | action | `depth_m`, `tolerance_m`=0.05, `max_speed`=0.2, `heading_deg`=0 | within tolerance of the depth |
| `Forward` / `Backward` | action | `distance_m`, `speed`=0.2, `heading_deg`=0 | travelled the distance |
| `StrafeLeft` / `StrafeRight` | action | `distance_m`, `speed`=0.2, `heading_deg`=0 | travelled the distance sideways |
| `Turn` | action | `heading_deg`, `tolerance_deg`=4 | facing the heading |
| `Hold` | action | `seconds`=2, `heading_deg`=0 | time is up |
| `Surface` | action | `seconds`=10, `speed`=0.2 | time is up (uses only the clock) |

Built-in BehaviorTree.CPP nodes used in the examples: `Sequence`,
`ReactiveSequence`, `Fallback`, `Timeout` (`msec`, real time), `AlwaysFailure`.
All built-in nodes are listed at https://www.behaviortree.dev/.

## Add a node

1. Write a class in `src/tardigrade_bt/src/nodes.cpp`. For a move, subclass
   `MoveAction` and fill in `begin()`, `done()` and `step()`.
2. Register it at the bottom of that file in `registerNodes()`.
3. Add a test in `test/test_trees.cpp` with the fake vehicle, then run
   `colcon test --packages-select tardigrade_bt && colcon test-result --verbose`.

## Known limitation

While the Unity yaw-estimate sign issue (CTL-01 / CTL-02) is open, moves that
hold a heading can make the simulated vehicle spin. `dive_test` is the best
first check in Unity.
