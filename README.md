# USV Simulation Environment

This repository is a ROS2 package that represents a simulation environment for testing of USV (Unmanned Surface Vessel) systems.
The simulation is based on VRX (Virtual RobotX) Gazebo simulation and PX4 Autopilot.

## About

This repository was built to simulate USV/ASV (Unmanned/Autonomous Surface Vessel) operation so that autonomy can be
developed and tested in software before it is deployed on real hardware. It exists to support the work of
**Udruga primjenjenih tehničkih znanosti** (UPTZ, eng. Association of Applied Technical Sciences)
on their autonomous vessel **Karolina** and their test platform **Veli Jože**.

The goal is to give a full stand-in for the physical vessels: vessel dynamics, differential thruster actuation,
sensors, and the same ROS 2 node graph that runs on the boats. Behaviours (waypoint following, COLREG avoidance,
docking) can therefore be written, broken, and fixed in simulation, and only then moved to Karolina or Veli Jože
for on-water validation.

## Table of Contents

- [About](#about)
- [Requirements](#requirements)
- [Installation Guide](#installation-guide)
  - [PX4 Autopilot Setup](#px4-autopilot-setup)
  - [ROS2 environment setup guide](#ros2-environment-setup-guide)
- [Start the simulation environment](#start-the-simulation-environment)
- [Run the simulation with Nav2](#run-the-simulation-with-nav2)
- [COLREG collision avoidance](#colreg-collision-avoidance)
  - [Background: Njord Autonomous Ship Challenge](#background-njord-autonomous-ship-challenge)
  - [Running the COLREG task](#running-the-colreg-task)
    - [Launch arguments](#launch-arguments)
    - [Otter starting position and launch timing](#otter-starting-position-and-launch-timing)
  - [The behavior tree](#the-behavior-tree)
  - [Reusing the behavior tree in another project](#reusing-the-behavior-tree-in-another-project)
  - [Vessel detection (LiDAR only)](#vessel-detection-lidar-only)
  - [The Otter script](#the-otter-script)
  - [Speed limits](#speed-limits)
- [USV package structure](#usv-package-structure)
- [Other notes](#other-notes)

## Requirements

- ROS2 Jazzy (LTS)
- Gazebo Harmonic (LTS)

Links to installation guides:
- ROS2 Jazzy: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- Gazebo Harmonic: https://gazebosim.org/docs/harmonic/ros_installation/

## Installation Guide

If not created already, create ROS 2 workspace directory.
```bash
mdkir ros2_ws/src
```

### PX4 Autopilot Setup

- **NOTE:** The required release of PX4 used in this repository is 1.15.4


- Run the following commands:

```bash
# Clone the forked repository in default/home directory
git clone https://github.com/Marko132001/PX4-Autopilot.git --recursive

# Change branch
git checkout px4_usv

# Installed required dependencies without Gazebo (--no-sim-tools)
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools

cd PX4-Autopilot/

make px4_sitl
```
**NOTE: if you clone PX4-Autopilot in another directory, make sure to change `px4_sitl` cmd inside
of `usv_sim.launch.py` so it matches the cloned directory.**

- Setup Micro XRCE-DDS Agent with following commands:

```bash
git clone -b v3.0.0 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git

cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..

make
sudo make install

sudo ldconfig /usr/local/lib/

# Run to see if the agent will start
MicroXRCEAgent udp4 -p 8888
```

If an error occurs regarding failure to fetch tag reference to fastdds, then edit in the CmakeLists.txt file found in the
Micro-XRCE-DDS-Agent directory the following line:

```bash
    # Old code around line 98 and 99
    set(_fastdds_version 2.12)
    set(_fastdds_tag 2.12.x)
    
    # Replace with following code
    set(_fastdds_version 2.14)
    set(_fastdds_tag 2.14.x)
```

If that doesn't work, then check out which tags are available for the Micro-XRCE-DDS-Agent GitHub repository at the link:
https://github.com/eProsima/Micro-XRCE-DDS-Agent/tags,
and replace the version and tag accordingly.


- Test  PX4-Autopilot and Micro-XRCE-DDS Agent with following commands:
```bash
# Start the agent in one terminal
MicroXRCEAgent udp4 -p 8888

# In second terminal
cd ~/PX4-Autopilot

make px4_sitl gz_x500

# As a correct result the Gazebo simulation should start 
# and you should see topics created in the MicroXRCEAgent terminal
```

- Setup px4_msgs ROS2 package:

```bash
# Navigate to your ROS2 workspace src directory
cd ros2_ws/src

git clone https://github.com/PX4/px4_msgs.git

git checkout release/1.15

# Synchronize messages with PX4-Autopilot repository:
rm -f ~/ros2_ws/src/px4_msgs/msg/*.msg
rm -f ~/ros2_ws/src/px4_msgs/srv/*.srv
cp ~/PX4-Autopilot/msg/*.msg ~/px4_msgs/msg/
cp ~/PX4-Autopilot/srv/*.srv ~/px4_msgs/srv/
```

- Install QGroundControl:
    - Install QGroundControl.AppImage from this link intead of one in the guide: https://github.com/mavlink/qgroundcontrol/releases/download/v4.4.1/QGroundControl.AppImage
    - Follow this guide: https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html#ubuntu

**Note: download the app image and inside the home directory, create a directory called QGroundControl, and put the AppImage inside it.
Make sure that the AppImage is executable.**

### ROS2 environment setup guide

- In the ~/.bashrc file you should have the following lines added:

```bash
# Both Cyclone DDS and eProsima Fast DDS are usuable. Prefer use of Cyclone DDS when possible
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/local_setup.sh
```

- VRX Gazebo simulation setup:

```bash
cd ~/ros2_ws/src

git clone https://github.com/osrf/vrx.git

# If using ROS2 Jazzy switch to jazzy branch
git checkout jazzy
```

Inside the cloned vrx repo, go inside directory `vrx_gz/models/coast_waves/model.sdf` and add `        <visibility_flags>2</visibility_flags>` in line 7,
so the code looks like following:
```xml
<link name="link">
  <visual name= "wave_visual">
    <visibility_flags>2</visibility_flags>
    <geometry>
    .....
```

- Clone main ROS2 simulation repository:

```bash
cd ~/ros2_ws/src

git clone https://github.com/Adria-Autonomous-Boat/riteh_usv_simulation.git
```

---

- At this point your ~/ros2_ws/src directory should minimally contain the following packages:

    - px4_msgs (for PX4-Autpilot topics)
    - riteh_usv_sim (the main ROS2 package)
    - vrx (simulation package)

    **NOTE on naming:** the git repository/directory is `riteh_usv_simulation`, but the ROS 2 package it declares is
    **`riteh_usv_sim`** — that is the name to use with `colcon build --packages-select`, `ros2 launch` and `ros2 run`.
    The Python module inside it keeps the name `usv_py` (imported as `usv_py.*`), and the generated service interface
    is `riteh_usv_sim/srv/SetGpsGoal`. Package names may not contain hyphens, which is why it is not `riteh-usv-sim`.

- In **CMakeLists.txt** file, you will find all the necessary dependencies required to build and run the environment (e.g. find_package(**rclcpp** REQUIRED)). Make sure you have all of them installed:
    - You can check if specific ROS2 package is installed by running: 

        ``` ros2 pkg list | grep {package-name} ```
    - Another approach is to use ```rosdep``` (https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html)
    - **NOTE:** *Eigen3* is a C++ library, not a ROS2 package (it should be preinstalled on an Ubuntu system)
    - **NOTE:** MAVSDK is an apt package that also needs to be installed (this is the version that was initially used: https://github.com/mavlink/MAVSDK/releases/tag/v2.4.1)
- In order to build the whole environment run the following commands in order:

```bash
# Build vrx package
cd ~/ros2_ws/src/vrx
colcon build --merge-install && . install/setup.bash

# Build px4_msgs package
cd ~/ros2_ws
colcon build --packages-select px4_msgs && . install/setup.bash

# Build the main package (riteh_usv_sim depends on vrx and px4_msgs)
cd ~/ros2_ws
colcon build --packages-select riteh_usv_sim && . install/setup.bash

# Build all packages in ros2_ws environment 
# (creates build, install and log folders in ros2_ws directory)
# (NOTE: lot of RAM usage)
cd ~/ros2_ws
colcon build && . install/setup.bash
```
**NOTE: This is RAM intensive, if you have 16GB RAM or less, you might need to increase swappiness.**


- For each change in the **riteh_usv_sim** package, before running anything you should run the following build command:

    ``` colcon build --packages-select riteh_usv_sim && . install/setup.bash ```

## Start the simulation environment

- In order to run the simulation, run the following launch file:
    
    ``` ros2 launch riteh_usv_sim usv_sim.launch.py ```

    - This command will start the VRX simulation, PX4 SITL (Software In The Loop) and QGroundControl
    - It will also run the nodes that are defined inside the launch file
    - You can also pass launch parameters for running specific world or model, for example:

        ``` ros2 launch riteh_usv_sim usv_sim.launch.py model:=usv_model01 world:=sydney_regatta```

    - **Note: This repository does NOT contain another USV model, it uses only Veli Jože model. If wanted, WAMV model can be found in the VRX repository.**

    - Current default world is sydney_regatta_colreg, as it was last used world for development of COLREG.

- After all processes start, you can use QGroundControl to plan a mission:
    - Define waypoints that the vessel will follow
    - ARM the vessel
    - Switch to MISSION mode
    - To cancel the mission execution, switch to HOLD mode or DISARM the vessel

- In order to run waypoint following functionality, the following nodes are required:

    - **differential_drive** - Handles conditions for switching between ARM/DISARM, MISSION/HOLD modes (sends lifecycle node status changes).

    - **differential_guidance** - The main waypoint following logic (lifecycle node)

    - **differential_kinematics** - Calculates actuator commands for left and right thruster 
    based on linear and angular velocity sent from **differential_guidance** node.

    - **pid_qgc_server** - Receives, updates and sends waypoints in cartesian format to **differential_guidance** node.

    - **engines_controller_sim** - Receives normalized actuator commands, converts them to PWM values and sends them to the actuators.

## Run the simulation with Nav2
- In order to run Nav2, in `usv_sim.launch.py` file uncomment the *return* block with Nav2 required nodes and comment the original *return* block. After that run `colcon build --packages-select riteh_usv_sim && . install/setup.bash` from your ROS2 environment directory.
- Open the first terminal and run the following launch file:
    - ``` ros2 launch riteh_usv_sim usv_sim.launch.py ```
- Wait for the PX4 SITL to fully initialize all the topics (two topics will appear last - *global_position* and *vehicle_attitude*).
- In the third terminal run the following launch file:
    - ``` ros2 launch riteh_usv_sim nav2_init.launch.py```
- Rviz2 should be opened and you should be able to use **Nav2 goal** button to give the vessel a target setpoint (within global costmap borders) which should automatically activate path following.

## COLREG collision avoidance

### Background: Njord Autonomous Ship Challenge

The COLREG world and the avoidance behaviour in this repository were developed for the **COLREG task of the
Njord Autonomous Ship Challenge**. In that task our vessel shares the water with an **Otter**, a Maritime Robotics
ASV, and has to avoid it in two encounters:

1. **Head-on** — the Otter approaches straight ahead. Under COLREG rule 14, both vessels alter course to
   starboard and pass port-to-port.
2. **Crossing, give-way** — the Otter approaches from our **starboard** side (our right). Under COLREG rule 15 we
   are the give-way vessel, so we must keep out of the way, passing astern of the Otter rather than cutting
   across its bow.

Everything below, the `sydney_regatta_colreg` world, the Otter model and its driving script, the LiDAR vessel
detector, and the behavior tree, exists to reproduce those two encounters repeatably.

### Running the COLREG task

The COLREG stack runs on top of the normal simulation, in three terminals. All three are required: Nav2 provides
the `navigate_to_pose` action server that both the channel navigator and the COLREG avoidance node use to command
the vessel, so without Terminal 2 the boat will not move at all.

- **Terminal 1 — simulation.** Start the environment with the COLREG world (it is the default):

    ```bash
    ros2 launch riteh_usv_sim usv_sim.launch.py
    ```

    Wait until PX4 SITL has fully initialized its topics (*global_position* and *vehicle_attitude* appear
    last) before continuing.

- **Terminal 2 — Nav2 stack.** After *global_position* and *vehicle_attitude* topics appear in PX4 SITL terminal, start the Nav2:
    ```bash
    ros2 launch riteh_usv_sim nav2_init.launch.py
    ```

- **Terminal 3 — COLREG autonomy:**

    ```bash
    ros2 launch riteh_usv_sim colreg.launch.py
    ```

    This starts the vessel detector, the channel navigator, the behavior tree runner (delayed 5 s so the
    navigator can get a GPS fix), the Gazebo <==> ROS bridge for the Otter, and the Otter driving script (delayed
    4.5 s — see [Otter starting position and launch timing](#otter-starting-position-and-launch-timing)).

- Alternatively, use the wrapper script, which does the same thing but tees all output to a timestamped log file
  under `logs/colreg_logs/`:

    ```bash
    ./scripts/colreg.sh
    ```

  Any launch arguments are passed straight through, e.g. `./scripts/colreg.sh goal_lat:=-33.722088 goal_lon:=150.674820`.

#### Launch arguments

All of these are passed to the `channel_navigator` node, so there is **no need to start it manually**,
the defaults below already encode a working run:

| Argument | Default | Meaning |
| --- | --- | --- |
| `goal_lat` | `-33.721814` | Final GPS goal latitude (decimal degrees). |
| `goal_lon` | `150.674820` | Final GPS goal longitude (decimal degrees). |
| `gate_commit_dist` | `20.0` | Distance (m) at which to commit to a detected gate. |
| `buoy_react_dist` | `20.0` | Distance (m) at which to react to a single buoy. |
| `max_buoy_depth` | `35.0` | Maximum depth (m) at which a buoy is still detected. |
| `send_to_nav2` | `true` | Send `NavigateToPose` goals to Nav2. Set `false` to only publish goals for inspection (the boat will not move). |

Examples:

```bash
# Default run — goal already set, nothing else needed
ros2 launch riteh_usv_sim colreg.launch.py

# Override the goal
ros2 launch riteh_usv_sim colreg.launch.py goal_lat:=-33.722088 goal_lon:=150.674820

# Let the behavior tree set the goal over the service instead of the launch file
ros2 launch riteh_usv_sim colreg.launch.py goal_lat:=nan goal_lon:=nan

# Dry run — see the goals being computed without commanding the vessel
ros2 launch riteh_usv_sim colreg.launch.py send_to_nav2:=false
```

**Do not also run `channel_navigator.py` manually with `ros2 run`.** The launch file already starts it. A second
instance takes the same node name (`/channel_navigator`), which ROS 2 does not prevent, and the resulting
duplicate-name collision breaks service discovery — `/channel_nav/set_gps_goal` disappears, the behavior tree
logs `set_gps_goal not available`, and the vessel never moves.

**On the two goal sources.** If `goal_lat`/`goal_lon` are set, the navigator converts them to a map-frame goal as
soon as the GPS anchor is established. The behavior tree *also* sets a goal via the `/channel_nav/set_gps_goal`
service a few seconds later, using the coordinates hardcoded in `usv_bt/trees/channel_gate_task.xml`. Whichever
arrives last wins, which in practice is the tree. Pass `goal_lat:=nan goal_lon:=nan` if you want the tree to be
the single source of truth, or edit the XML to match your launch defaults.

#### Otter starting position and launch timing

The Otter's starting pose and heading are set in `usv_sim.launch.py` in the `spawn_otter` action. Our USV spawns
at `-455, 172` heading `1.57` rad (due north, via `PX4_GZ_MODEL_POSE`), so every Otter pose is chosen relative to
that. A catalogue of tested encounter geometries is kept as comments directly above `spawn_otter` — head-on,
crossing from starboard, crossing from port, and several angled approaches that are expected to trigger the
emergency arc. **Swap one in to change the scenario**; the currently active pose is the one inside the
`arguments=[...]` list.

**Changing the spawn position usually means changing the launch delay too.** Two timers govern the encounter:

| Timer | Where | Default | Effect |
| --- | --- | --- | --- |
| `spawn_otter` period | `usv_sim.launch.py` | `15.0` s | When the Otter model appears in the world. |
| `otter_forward` period | `colreg.launch.py` | `4.5` s | When the Otter starts driving forward. |

Because `otter_forward.py` applies constant thrust in a straight line, the Otter's position at the moment of the
encounter is a product of *where it spawned* and *how long it has been driving*. Move the spawn further away and
the Otter arrives late — our USV may already have passed the crossing point, and no avoidance is triggered. Move
it closer, or start it driving too early, and the Otter is already on top of us before the LiDAR has a usable
detection, which forces the emergency arc instead of a clean COLREG manoeuvre.

So when you change the spawn pose, retune the `otter_forward` delay in `colreg.launch.py` until the two vessels
actually meet in the intended geometry. Expect a few iterations. Useful checks while tuning:

- `vessel_detector` logs `[VESSEL] dist=… bearing=… extent=…` — the encounter should begin with the Otter at
  roughly 30-40 m, comfortably inside `max_detect_range` (40 m).
- `VesselInRange` fires at 20 m; if you never see it switch from `clear`, the vessels are not meeting.
- If `ColregAvoid` reports the emergency range (12 m) straight away, the Otter started too early or too close.

### The behavior tree

The behaviour is a [BehaviorTree.CPP](https://www.behaviortree.dev/) v4 tree, defined in
`usv_bt/trees/channel_gate_task.xml` and ticked at 10 Hz by the `channel_gate_runner` node
(`usv_bt/src/channel_gate_runner.cpp`).

The structure is:

```
Sequence
├── WaitForAnchor        RetryUntilSuccessful( GpsAnchorReady )
└── NavLoop              RetryUntilSuccessful
    └── NavOrAvoid       ReactiveSequence
        ├── CheckVessel  Fallback
        │   ├── Inverter( VesselInRange )     → succeeds when no vessel is close
        │   └── ColregSequence  Sequence
        │       ├── ColregAvoid               → the COLREG manoeuvre
        │       ├── WaitForClear              RetryUntilSuccessful( VesselClear )
        │       └── Delay 15 s
        └── NavigateToGPS6  NavigateChannel   → drive toward the goal
```

Because the inner node is a `ReactiveSequence`, the vessel check is re-evaluated on every tick: navigation is
pre-empted the moment the Otter comes into range, and resumes once it is clear again.

The custom nodes, all registered in `channel_gate_runner.cpp` and implemented under `usv_bt/src/`:

| Node | Type | Role |
| --- | --- | --- |
| `GpsAnchorReady` | condition | Succeeds once the GPS/ENU anchor is set and the frame tree is valid. |
| `VesselInRange` | condition | True when a detected vessel is within `range_threshold` (20 m) and moving faster than `min_speed` (0.3 m/s). |
| `ColregAvoid` | action | Runs the avoidance manoeuvre: classifies head-on vs. crossing from relative bearing and heading, then offsets to starboard or yields astern. |
| `VesselClear` | condition | True once no vessel remains within `clear_range` (15 m). |
| `NorthingAbove` | condition | Northing threshold check, used for course/stage gating. |
| `NavigateChannel` | action | Drives toward the goal position via the waypoint follower. |

Key `ColregAvoid` parameters, tuned in the XML rather than in code: `starboard_offset` (6.0 m lateral offset for
head-on), `yield_offset` (7.0 m for the give-way crossing), `yield_clearance` (3.0 m), `emergency_range`
(12.0 m), `arc_radius` (2.75 m), and `replan_interval_s` (1.0 s). The head-on cone is ±22.5° about the bow, and
an encounter is only classified head-on if the Otter's heading is also roughly opposite to ours.

### Reusing the behavior tree in another project

The tree is deliberately kept modular and can be lifted out of this repository:

- **The tree is data, not code.** `channel_gate_task.xml` is loaded at runtime from the package share directory.
  Thresholds, offsets and the goal are all XML attributes, so tuning or re-sequencing the behaviour requires no
  rebuild — only a `colcon build` to re-install the file to `share/`.
- **The nodes are self-contained.** Each condition/action lives in its own header/source pair under
  `usv_bt/include/usv_bt/` and `usv_bt/src/`, and talks to the rest of the system only over ROS topics and TF.
  They are also exported as a BehaviorTree.CPP plugin (`usv_bt_nodes_plugin.cpp`), so they can be loaded by any
  BT executor — including Groot2 and Nav2's BT navigator — without linking this package's other nodes.
- **The waypoint follower is swappable.** `NavigateChannel` is the only node that issues motion commands toward
  a goal; the COLREG logic never talks to a controller directly. Whatever sits behind it — the in-house
  PID/differential guidance stack, or the Nav2 Regulated Pure Pursuit setup in `config/njord_usv/nav2_rpp_params.yaml` —
  can be switched without touching the avoidance nodes or the tree, because both expose the same
  "go to this position" interface. This is what lets the same COLREG behaviour run on Karolina and on Veli Jože,
  which use different controllers.

To reuse it elsewhere, copy the `usv_bt/` directory, keep `vessel_detector.py` (or substitute your own publisher
on `/vessel_detection/closest`), and reimplement `NavigateChannel` against your own waypoint follower.

### Vessel detection (LiDAR only)

Detection of the other vessel is currently **LiDAR-only** — no camera or radar is involved in the COLREG task.
`usv_py/image_processing/vessel_detector.py` subscribes to the Livox point cloud on `/sensors/livox_lidar/scan`
and:

1. Crops points to a forward arc (`forward_arc_deg`, 100° half-angle), a maximum range (`max_detect_range`, 40 m)
   and a maximum height (`max_point_height`, 2.5 m).
2. Clusters the remaining points with DBSCAN (`dbscan_eps` 0.6 m, `dbscan_min_samples` 5).
3. Classifies clusters by physical extent — anything below `min_vessel_extent` (0.8 m) is treated as a buoy and
   discarded; clusters up to `max_vessel_extent` (5.0 m) are accepted as vessels.
4. Transforms the closest vessel into the `map` frame and publishes it.

Outputs:

- `/vessel_detection/closest` (`geometry_msgs/PointStamped`) — closest detected vessel, consumed by
  `VesselInRange` and `VesselClear`.
- `/vessel_detection/markers` (`visualization_msgs/MarkerArray`) — RViz visualization.

The size-based buoy/vessel split is what makes this work in the Njord course, where buoys and the Otter share the
same water: the Otter is simply far larger than any buoy.

### The Otter script

The Otter is a passive Gazebo model — nothing in the simulator drives it — so the task needs a script to make it
move and create the encounter. `scripts/otter_forward.py` is that script: a small ROS 2 node that publishes a
constant thrust to both of the Otter's thrusters at 10 Hz:

- Topics: `/otter/thrusters/left/thrust` and `/otter/thrusters/right/thrust` (`std_msgs/Float64`)
- Thrust: `150.0` N per thruster, giving a steady straight-line transit

Equal thrust on both sides means the Otter holds a straight course, which is what the COLREG task assumes of the
stand-on/approaching vessel. The direction of that transit is set by the Otter's spawn heading in
`usv_sim.launch.py`. It is launched automatically by `colreg.launch.py` (4.5 s in, after the bridge is up), and
its topics reach Gazebo through the `otter_bridge` parameter bridge configured in
`usv_py/gz_utils/otter_bridge_config.py`. That delay is a tuning knob, not a constant — see
[Otter starting position and launch timing](#otter-starting-position-and-launch-timing).

To change the Otter's speed, edit the `THRUST` constant at the top of the script; the model's thrusters are
limited to `max_thrust_cmd` 158 N / `min_thrust_cmd` -103 N in `models/otter_usv/model.sdf`.

### Speed limits

The Njord COLREG task imposed a speed limit, and the simulation is tuned to match it:

| Vessel | Speed | Where it is set |
| --- | --- | --- |
| Otter (other vessel) | **2.5 knots** (~1.29 m/s) | `THRUST = 150.0` N in `scripts/otter_forward.py` |
| Our vessel | **2 knots** (~1.03 m/s) | `desired_linear_vel: 1.0` m/s in `config/njord_usv/nav2_rpp_params.yaml` |

Our vessel being the slower of the two is deliberate: it matches the competition constraint, and it means the
give-way manoeuvre has to be committed to early, since we cannot simply outrun the Otter across its bow.

Note that these are the *task* limits, not the platform limits — the underlying controller in
`config/njord_usv/usv_control_params.yaml` allows up to `max_forward_speed: 2.8` m/s. If you raise the Nav2
speed, retune the avoidance distances in the tree (`range_threshold`, `emergency_range`, the offsets) to match,
since they were chosen for these speeds.

## USV package structure

```bash
-usv
    - config  # contains yaml files with parameters required for waypoint follower node
    - hooks  # gz-sim resource path definitions
    - include  # header files of nodes and libraries
    - launch
    - models
    - src  # C++ files of nodes and libraries
    - srv  # definitions of service messages
    - scripts # bash and python scripts added to move Otter
    - usv_py  # python nodes and files
    - usv_bt  # C++ files and BT xml
    - worlds

```

## Other notes

In general for system development for Karolina and Veli Jože, Nav2 was used instead of a PID controller.
If you wish to use a PID controller instead of Nav2 stack, you will most likely have to tune
Kp, Ki and Kd parameters, since it was unused. Use of Nav2 is recommended tho.

Since a YOLO model for general boat detection, which was tested outside simulation and was able to recognize and follow
the Otter vessel, wasn't trained and implemented, the COLREG task relies only on LiDAR.
This causes some issues, since we are tracking just the cluster size and its speed, meaning that bigger
pointcloud clusters and those faster than a certain threshold are disregarded. 
Because of that, and because of previous issues with artefacts on costmap, inside the tree .xml file was added
 a delay of 15 seconds. There was an issue where the system would recognize artefacts as a vessel, so it would
immediately begin doing another COLREG manouver, even though Otter safely passed the vessel.