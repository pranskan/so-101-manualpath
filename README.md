# SO-100 Robotic Arm Control Suite

A comprehensive Python toolkit for calibrating, controlling, and programming motion sequences on an SO-100 6-DOF robotic arm with Dynamixel servos. This suite includes scripts for calibration, manual control, path recording/replay, and a web-based GUI.

## Features

- **Calibration**: Automated joint range detection and offset calculation for safe operation.
- **Manual Control**: Command-line interface for direct servo control with safety limits.
- **Path Recording & Replay**: Record, edit, and execute multi-waypoint motion sequences.
- **Web GUI**: Streamlit-based interface for interactive control and monitoring.
- **Safety Features**: Torque control, joint limits, and inversion handling for drive modes.

## Joint Configuration

The SO-100 arm has 6 joints mapped as follows:
- ID 1: shoulder_pan
- ID 2: shoulder_lift
- ID 3: elbow_flex
- ID 4: wrist_flex
- ID 5: wrist_roll
- ID 6: gripper

## Quick Start

### Prerequisites
- Python 3.7+
- USB serial connection to arm controller (default port: `/dev/tty.usbmodem5AE60848961`)
- Libraries: `pyserial`, `streamlit` (optional for GUI)

```bash
pip install pyserial streamlit
```

### Initial Setup
1. Connect the arm via USB and power it on.
2. Run calibration to determine joint limits and offsets.
3. Use manual control or GUI for testing.
4. Record and replay paths for automation.

## Scripts Overview

### Step1_calibrate.py
Interactive calibration script for recording joint ranges by manually moving the arm.

**Usage:**
```bash
python Step1_calibrate.py
```
- Select calibration mode: full (all joints), or individual joints (elbow, shoulder lift, shoulder pan).
- Disable torque, move joints through full range, and press Enter to record min/max.
- Outputs `calibration.json` with ticks-based data for safe operation.

**Commands:**
- `1`: Full calibration
- `2`: Elbow only
- `3`: Shoulder lift only
- `4`: Shoulder pan only
- `q`: Quit

### Step2_Test_calibration.py
Command-line interface for manual servo control, testing calibration, and monitoring angles.

**Usage:**
```bash
python Step2_Test_calibration.py
```
- Load calibration, move to center, perform safe moves, and monitor live angles.
- Supports logical-to-physical conversions and joint-space path planning.

**Key Commands:**
- `calload`: Load calibration
- `calcenter`: Move all joints to center
- `safe <id> <deg>`: Safe move within limits
- `safelogical <id> <logical_deg>`: Move using logical degrees (0-180)
- `monitor_angles`: Live angle monitoring (disable torque first)
- `path <angles> -> <angles>`: Plan and execute joint path
- `torque off all`: Disable torque on all servos
- `q`: Quit

### Step3_Path_Recording.py
Path recording and replay system for automating motion sequences.

**Usage:**
```bash
python Step3_Path_Recording.py
```
- Record waypoints by positioning the arm and adding points.
- Edit, rename, delete, and execute saved paths stored in `paths.json`.

**Commands:**
- `start <name>`: Start recording a new path
- `edit <name>`: Load existing path for editing
- `add`: Add current position as waypoint
- `change <index>`: Change a waypoint to current position
- `remove <index>`: Remove waypoint by index
- `save`: Save current path
- `execute <name>`: Execute a saved path
- `list`: List all saved paths
- `current`: Show current path waypoints
- `torque off all`: Disable torque
- `q`: Quit

### Step2_Streamlit.py
Web-based GUI for interactive arm control using Streamlit.

**Usage:**
```bash
streamlit run Step2_Streamlit.py
```
- Load calibration, move to center, perform safe moves, and monitor angles via browser.

**Features:**
- Buttons for loading calibration, moving to center, and safe moves.
- Input fields for servo ID and degrees.
- Real-time feedback in the Streamlit app.

## Configuration Files

### calibration.json
Stores joint calibration data in ticks format:
- `range_min`/`range_max`: Joint travel limits in servo ticks (0-4095).
- `homing_offset`: Center position in ticks.
- `drive_mode`: 0 (normal) or 1 (inverted).
- `logical_min`/`logical_max`: Logical range (typically 0-180).
- `physical_min`/`physical_max`: Physical degrees corresponding to logical range.

### paths.json
Stores recorded paths as lists of waypoints (each waypoint is a list of 6 angles in degrees).

## Calibration Process

1. Run `Step1_calibrate.py` and select mode.
2. Disable torque and manually move joints through full range.
3. Press Enter to record; script calculates min/max and center.
4. Verify in `Step2_Test_calibration.py` with `calcenter` and `safe` commands.
5. Adjust `calibration.json` if needed (e.g., add `logical_min`/`logical_max` for custom mappings).

## Safety Notes

- Always disable torque before manual movement to avoid damage.
- Use `safe` commands to respect joint limits.
- Monitor angles live with `monitor_angles` to ensure correct calibration.
- Power cycle servos after ID changes.

## Troubleshooting

- **No servos detected**: Check USB connection and port in scripts.
- **Angles not reading**: Ensure servos are powered and IDs match (1-6).
- **Calibration invalid**: Re-run calibration and move joints fully.
- **Streamlit issues**: Install dependencies and run in a virtual environment.

## Contributing

Feel free to extend scripts or add features. Ensure changes maintain safety protocols.

## License

This toolkit is provided as-is for educational and research purposes.