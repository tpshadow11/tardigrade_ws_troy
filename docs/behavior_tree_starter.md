# Starter behaviour tree

`tardigrade_mission/starter_tree.py` is a small py_trees mission for the
Unity simulator: wait for odometry, dive 1 m, drive 3 m out, turn around,
drive 3 m back. It publishes to the same mission velocity topic as
`qual_test`, so nothing else in the stack changes.

```text
Starter mission (sequence)
├── Wait for odometry (max 30 s)
├── Record start pose
└── Safety: odometry still fresh? (checked every tick)
    └── Legs (sequence)
        ├── Descend 1 m -> Hold 2 s
        ├── Drive out   -> Hold 2 s
        ├── Turn around -> Hold 2 s
        └── Drive back  -> Hold 2 s
```

The leaves live in `tardigrade_mission/bt_behaviours.py`. They only talk to
the robot through a small vehicle object, so `test/test_starter_tree.py` runs
the whole tree against a fake robot without ROS.

## Run it in Unity

1. Start the operator stack and press Play in Unity, as in
   [the Unity operator workflow](unity_operator_workflow.md).
2. In a second container shell, install py_trees if the image predates it,
   then rebuild:

   ```bash
   pip3 install py_trees==2.1.6
   ./build.sh --pkg tardigrade_mission
   source install/setup.bash
   ```

3. Enable external control and arm the Unity plant, then hand the mux to the
   mission source:

   ```bash
   ros2 service call /tardigrade/set_external_control \
     tardigrade_interfaces/srv/SetExternalControl "{enabled: true}"
   ros2 service call /tardigrade/set_armed \
     tardigrade_interfaces/srv/SetArmed "{armed: true}"
   ros2 param set /velocity_setpoint_mux active_source mission
   ```

4. Run the tree:

   ```bash
   ros2 run tardigrade_mission starter_tree --ros-args -p use_sim_time:=true
   ```

   Change the legs with `-p depth_m:=0.5 -p distance_m:=2.0`. The node prints
   the tree with each node's status whenever a new step starts, and exits when
   the mission succeeds or fails.

5. Afterwards, disarm and return the mux to manual:

   ```bash
   ros2 service call /tardigrade/set_armed \
     tardigrade_interfaces/srv/SetArmed "{armed: false}"
   ros2 param set /velocity_setpoint_mux active_source manual
   ```
