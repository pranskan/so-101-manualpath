# SO-101 Robotic Arm - Manual Path Planning Control

A unified control system for the SO-101 6-DOF robotic arm that **emphasizes manual path planning and execution** as its core feature. Record motion sequences by moving the arm to key positions, save them, and replay them on demand.

**Videos:**

[![Demo video](https://img.youtube.com/vi/MSCIpeYgwzI/0.jpg)](https://youtu.be/MSCIpeYgwzI) - Full workflow demonstration

[![Quick clip](https://img.youtube.com/vi/QxqIxc3luc4/0.jpg)](https://www.youtube.com/shorts/QxqIxc3luc4)

## Setup

```bash
pip install pyserial numpy scipy
# Optional for vision: pip install opencv-python pyrealsense2
```

## Usage

```bash
python master_control.py
```

```
[SO-101]> connect         # auto-detect arm
[SO-101]> scan            # verify servos 1-6
[SO-101]> calibrate       # first-time joint limit recording
[SO-101]> calload         # load existing calibration
[SO-101]> center          # move all joints to center
[SO-101]> help            # show all commands
```

## Commands

### 🎯 Manual Path Planning (Core Feature)

| Command | Description |
|---------|-------------|
| `start <name>` | Start recording a new motion path |
| `add` | Record current arm position as a waypoint |
| `undo` | Remove the last recorded waypoint |
| `wp` | List all waypoints in current path |
| `wp_del <n>` | Delete waypoint #n |
| `wp_replace <n>` | Replace waypoint #n with current position |
| `wp_insert <n>` | Insert current position before waypoint #n |
| `wp_goto <n>` | Move arm to waypoint #n |
| `wp_swap <a> <b>` | Swap two waypoints |
| `save` | Save recorded path to paths.json |
| `cancel` | Discard current recording |
| `list_paths` | List all saved paths |
| `exec <name>` | Execute a saved path (replay) |
| `smooth <name> [sec]` | Execute with smooth cubic spline interpolation |
| `del_path <name>` | Delete a saved path |
| `del_all_paths` | Delete all paths |

### Pillar Pick & Drop Task

| Command | Description |
|---------|-------------|
| `set_home` | Save current position as home for all pillar paths |
| `home` | Move arm to saved home position |
| `pick1` / `pick2` / `pick3` | Execute pick from pillar 1/2/3 |
| `drop1` / `drop2` / `drop3` | Execute drop on pillar 1/2/3 |
| `rec_pick1..3` | Start recording pick path (home auto-added) |
| `rec_drop1..3` | Start recording drop path (home auto-added) |
| `pillars` | Show which pillar paths are recorded |

### Setup & Calibration

| Command | Description |
|---------|-------------|
| `connect` | Auto-detect and connect to arm |
| `scan` | Scan for servos 1-6 |
| `disconnect` | Close connection |
| `status` | Show connection info |
| `calibrate` | Interactive min/max joint recording |
| `calload` | Load calibration.json |
| `calinfo` | Show joint limits and centers |

### Direct Motion Control

| Command | Description |
|---------|-------------|
| `center` | Move all joints to center |
| `move <id> <deg>` | Move servo (e.g. `move 1 90`) |
| `read <id>` | Read servo angle |
| `torque <on\|off>` | Enable/disable all servos |

### Kinematics & Gripper

| Command | Description |
|---------|-------------|
| `fk <a1,a2,a3,a4,a5,a6>` | Forward kinematics |
| `ik <x> <y> <z>` | Inverse kinematics |
| `grip_open` | Open gripper |
| `grip_close` | Close gripper |
| `demo <x> <y> <z>` | Pick & place demo |

### Vision (RealSense Camera)

| Command | Description |
|---------|-------------|
| `rs_init` | Initialize RealSense camera |
| `rs_detect` | Detect red objects |
| `cam_cal` | Interactive camera calibration (Kabsch algorithm) |
| `cam_cal_load` | Load camera calibration transform |
| `cam_cal_list` | List all camera calibration points |
| `cam_cal_del <n>` | Delete camera calibration point #n |
| `cam_cal_clear` | Delete all camera calibration points |
| `cam_cal_reset` | Delete camera calibration transform |
| `cam_test <x> <y> <z>` | Test camera→robot transform |

## Joints

| ID | Name | Description |
|----|------|-------------|
| 1 | shoulder_pan | Horizontal rotation |
| 2 | shoulder_lift | Vertical lift |
| 3 | elbow_flex | Elbow bend |
| 4 | wrist_flex | Wrist bend |
| 5 | wrist_roll | Wrist rotation |
| 6 | gripper | Open/close |

## Files

```
master_control.py         - main program (1300+ lines)
ik_solver.py              - FK/IK math (imported, 360 lines)
calibration.json          - joint limits (auto-created)
paths.json                - saved motion paths (auto-created)
camera_calibration.json   - camera-to-robot transform (auto-created)
calibration_points.json   - camera calibration points (auto-created)
dh-simulator.html         - DH parameter reference
so101_new_calib.urdf      - robot model (URDF)
```

## Advanced Features

### Manual Path Planning Workflow
The core of this project: record motion sequences by manually positioning the arm, save them, and replay on demand.

```
[SO-101]> start pick_task              # start recording
[SO-101]> move 1 45                    # move arm to position 1
[SO-101]> add                          # record waypoint
[SO-101]> move 1 90; move 2 120        # move arm to position 2
[SO-101]> add                          # record waypoint 2
[SO-101]> wp                           # list all waypoints
[SO-101]> wp_replace 2                 # re-record waypoint 2
[SO-101]> save                         # save path to disk
[SO-101]> exec pick_task               # replay path
[SO-101]> smooth pick_task 5           # replay with smooth curves over 5 seconds
```

### Pillar Pick & Drop Task
Pre-configured workflow for the ring-on-pillars demo:

```
[SO-101]> set_home                     # set arm's current position as home
[SO-101]> rec_pick1                    # start recording pick path (home auto-added)
[SO-101]> move 1 45; add               # add waypoint 2
[SO-101]> move 1 30; add               # add waypoint 3
[SO-101]> save                         # save pick1
[SO-101]> pick1                        # execute: go home → run pick1
[SO-101]> drop1                        # execute: go home → run drop1
```

### Smooth Trajectory Execution
Execute paths with cubic spline interpolation for smooth, continuous motion:
```
[SO-101]> smooth my_path 8.0           # execute over 8 seconds with smooth curves
```

### Camera Calibration
Interactive camera-to-robot coordinate frame calibration using the Kabsch algorithm:
```
[SO-101]> rs_init                      # initialize RealSense
[SO-101]> cam_cal                      # start calibration (click objects in video)
[SO-101]> cam_cal_list                 # review collected points
[SO-101]> cam_cal_del 3                # remove bad point
[SO-101]> cam_cal                      # recalibrate with remaining points
[SO-101]> cam_test 250 100 150         # verify transform
```

### Path Editing & Refinement
Fix recorded paths without re-recording everything:

```
[SO-101]> edit pick_task               # load path for editing
[SO-101]> wp                           # list waypoints
[SO-101]> wp_del 2                     # delete bad waypoint
[SO-101]> wp_goto 3                    # move arm to waypoint 3
[SO-101]> wp_insert 3                  # insert current position before waypoint 3
[SO-101]> wp_swap 1 2                  # swap waypoints 1 and 2
[SO-101]> save                         # save updated path
```

### Path Recording Workflow

## Recommended Libraries

These libraries can enhance the project further:

| Library | Use Case | Status |
|---------|----------|--------|
| **dynamixel-sdk** | Official Dynamixel protocol library with better error handling | Optional |
| **roboticstoolbox-python** | Pre-built kinematics for many robots, trajectory planning, visualization | Optional |
| **ikpy** | Load IK directly from URDF files | Works with so101_new_calib.urdf |
| **spatialmath-python** | SE(3) pose math, rotation representations | Optional |
| **modern-robotics** | DH parameter library (theory reference) | Optional |
| **keyboard** | Real-time keypress detection for non-blocking recording | Optional |
| **open3d** | Point cloud processing with RealSense depth data | Optional |
| **opencv-contrib-python** | Better vision with SIFT/ORB features (vs simple color) | For advanced vision |

## Programmatic Use

```python
from master_control import *

ser, port, baud = find_serial_port()
cal = load_calibration()
move_all_to_center(cal)
safe_move(1, 45, cal)
test_fk([90, 90, 90, 90, 90, 0])
test_ik(250, 0, 150)
gripper_open()

# Smooth path execution with scipy CubicSpline interpolation
execute_smooth_path("my_path", cal, total_time=5.0)
```
