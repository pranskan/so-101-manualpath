# SO-101 Robotic Arm Control

Unified control program for the SO-101 6-DOF arm with Dynamixel servos.

**Video:** https://www.youtube.com/shorts/QxqIxc3luc4

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

| Command | Description |
|---------|-------------|
| **Setup** | |
| `connect` | Auto-detect and connect to arm |
| `scan` | Scan for servos 1-6 |
| `disconnect` | Close connection |
| `status` | Show connection info |
| **Calibration** | |
| `calibrate` | Interactive min/max joint recording |
| `calload` | Load calibration.json |
| `calinfo` | Show joint limits and centers |
| **Motion** | |
| `center` | Move all joints to center |
| `move <id> <deg>` | Move servo (e.g. `move 1 90`) |
| `read <id>` | Read servo angle |
| `torque <on\|off>` | Enable/disable all servos |
| **Paths** | |
| `start <name>` | Start recording a path |
| `add` | Record current position as waypoint |
| `undo` | Remove last recorded waypoint |
| `save` | Save recorded path |
| `list_paths` | List all saved paths |
| `exec <name>` | Execute a saved path |
| `smooth <name> [sec]` | Execute with smooth cubic spline interpolation |
| `del_path <name>` | Delete a saved path |
| `del_all_paths` | Delete all saved paths |
| **Kinematics** | |
| `fk <a1,a2,a3,a4,a5,a6>` | Forward kinematics |
| `ik <x> <y> <z>` | Inverse kinematics |
| **Gripper / Pick & Place** | |
| `grip_open` | Open gripper |
| `grip_close` | Close gripper |
| `demo <x> <y> <z>` | Pick & place sequence |
| **Vision** | |
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

### Smooth Trajectory Execution
Use `smooth <path_name> [duration_sec]` to execute paths with cubic spline interpolation for smoother motion:
```
[SO-101]> smooth my_path 8.0    # execute over 8 seconds with smooth curves
```

### Camera Calibration
Interactive camera-to-robot coordinate frame calibration using the Kabsch algorithm:
```
[SO-101]> rs_init                 # initialize RealSense
[SO-101]> cam_cal                 # start calibration (click objects in video)
[SO-101]> cam_cal_list            # review collected points
[SO-101]> cam_cal_del 3           # remove bad point
[SO-101]> cam_cal                 # recalibrate with remaining points
[SO-101]> cam_test 250 100 150    # verify transform
```

### Path Recording Workflow
```
[SO-101]> start pick_task         # start recording
[SO-101]> move 1 45               # move arm
[SO-101]> add                     # record waypoint
[SO-101]> move 1 90
[SO-101]> add
[SO-101]> undo                    # remove bad waypoint
[SO-101]> save                    # save path
[SO-101]> exec pick_task          # replay later
[SO-101]> smooth pick_task 5      # smooth replay
```

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
