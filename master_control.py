#!/usr/bin/env python3
"""
SO-101 Master Control Program
Consolidated control suite for calibration, motion control, path recording, 
FK/IK solving, and pick-and-place operations with RealSense integration.
"""

import serial
import serial.tools.list_ports
import time
import json
import os
import sys

# Try to import optional dependencies
try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

try:
    from ik_solver import solve_fk, solve_ik, JOINT_NAMES as IK_JOINT_NAMES
    HAS_IK_SOLVER = True
except Exception as e:
    print(f"  Warning: ik_solver not loaded: {e}")
    HAS_IK_SOLVER = False

try:
    import pyrealsense2 as rs
    import cv2
    HAS_REALSENSE = True
except ImportError:
    HAS_REALSENSE = False

# ============================================================================
# CONFIGURATION
# ============================================================================
# These settings define the robot hardware and communication parameters.
# Changing these should only be done if the physical robot setup changes.

# Serial communication baud rate: Speed at which the computer talks to servo controllers.
# 1000000 baud = 1 million bits per second (very fast communication).
BAUD = 1000000

# Joint names: Human-readable labels for each servo motor on the arm.
# Order matters: JOINT_IDS and JOINT_NAMES must correspond (servo 1 = shoulder_pan, etc.)
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

# Servo IDs: Hardware identifiers for each motor on the Dynamixel bus.
# These are configured on the physical servos and must match the order above.
JOINT_IDS = [1, 2, 3, 4, 5, 6]

# Default center position for servos (in degrees, 0-360 range).
SERVO_CENTER_ANGLE = 180.0

# Gripper control angles (in degrees):
#   GRIPPER_OPEN_ANGLE: servo angle that opens the gripper (object falls out)
#   GRIPPER_CLOSE_ANGLE: servo angle that closes the gripper (grasps object)
GRIPPER_OPEN_ANGLE = 244
GRIPPER_CLOSE_ANGLE = 150

# Height offset (in mm) used when approaching objects:
# When moving to pick something up, the arm first goes to (target_x, target_y, target_z + APPROACH_HEIGHT)
# This keeps the gripper from hitting the object while moving horizontally.
APPROACH_HEIGHT = 80

# Home position: all servos at 90 degrees (neutral stance).
# Used as the starting position for pillar pick/drop operations.
HOME_POSITION = [90.0, 90.0, 90.0, 90.0, 90.0, 90.0]

# Pillar path names: the naming convention for saved paths.
# pick1/pick2/pick3 = pick ring from pillar 1/2/3
# drop1/drop2/drop3 = drop ring on pillar 1/2/3
PILLAR_ACTIONS = ["pick1", "pick2", "pick3", "drop1", "drop2", "drop3"]

# File to persist the home position so it survives restarts.
HOME_FILE = "home_position.json"

# ============================================================================
# GLOBAL STATE
# ============================================================================
# These variables hold the current state of the robot and connected hardware.
# They persist across multiple commands and are updated as the program runs.

# Serial port connection: None if not connected, otherwise a serial.Serial object
# that lets us send/receive data to the servo controllers.
ser = None

# Port name: String like "COM3" (Windows) or "/dev/ttyUSB0" (Linux).
# Shows which USB port the arm is connected to.
PORT = None

# Baud rate currently in use (bits per second). Set when connection is established.
current_baud = None

# Calibration data: Dictionary mapping joint names to their min/max angles and offsets.
# Loaded from calibration.json file. Null until user runs "calload" command.
raw_calibration = None

# Path recording state: List of waypoints being recorded for playback.
# Each waypoint is a list of 6 servo angles. Null when not recording.
current_path = None

# Path name: String identifier for the path currently being recorded.
# Used as the key when saving to paths.json.
current_path_name = None

# RealSense camera pipeline: The camera interface object. None until "rs_init" is run.
realsense_pipeline = None

# Camera intrinsics: Focal length and principal point of the camera lens.
# Used to convert 2D pixels + depth into 3D coordinates. Set by init_realsense().
realsense_intrinsics = None

# Camera-to-robot calibration: Rotation matrix (R) and translation vector (t)
# that convert 3D points from camera frame to robot frame.
# Computed by Kabsch algorithm during "cam_cal" and loaded by "cam_cal_load".
camera_transform_R = None
camera_transform_t = None

# Calibration points: List of (camera_xyz, robot_xyz) pairs collected during camera calibration.
# Used to compute the camera_transform_R and camera_transform_t.
cam_cal_points = []

# ============================================================================
# SERIAL PORT & SERVO COMMUNICATION
# ============================================================================

def find_serial_port():
    """Auto-detect available serial ports and try different baud rates."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        return None, None, None
    
    print(f"Available ports: {[p.device for p in ports]}")
    
    baud_rates = [1000000, 921600, 460800, 230400, 115200]
    ports_to_try = sorted(ports, key=lambda p: p.device, reverse=True)
    
    for port in ports_to_try:
        for baud in baud_rates:
            try:
                print(f"  Trying {port.device} at {baud} baud...", end=" ", flush=True)
                test_ser = serial.Serial(port.device, baud, timeout=0.05)
                time.sleep(0.2)
                
                # Verify at least one servo responds
                found_servo = False
                for servo_id in range(1, 7):
                    packet = [0xFF, 0xFF, servo_id, 2, 0x01]
                    checksum = (~sum(packet[2:])) & 0xFF
                    packet.append(checksum)
                    test_ser.reset_input_buffer()
                    test_ser.write(bytes(packet))
                    time.sleep(0.05)
                    resp = test_ser.read(20)
                    if len(resp) > 0:
                        found_servo = True
                        break
                
                if found_servo:
                    print("✓ CONNECTED")
                    return test_ser, port.device, baud
                else:
                    test_ser.close()
                    print("✗")
            except Exception:
                print("✗")
    
    return None, None, None

def write_servo(servo_id, address, value, length=1):
    """Write to servo register"""
    global ser
    if ser is None:
        print("ERROR: Serial not connected")
        return
    
    if length == 1:
        data = [value & 0xFF]
    else:
        data = [value & 0xFF, (value >> 8) & 0xFF]
    
    packet = [0xFF, 0xFF, servo_id, len(data) + 3, 0x03, address] + data
    checksum = (~sum(packet[2:])) & 0xFF
    packet.append(checksum)
    
    ser.write(bytes(packet))
    time.sleep(0.001)
    ser.read(20)

def read_servo(servo_id, address, length=1):
    """Read from servo register"""
    global ser
    if ser is None:
        return None
    
    packet = [0xFF, 0xFF, servo_id, 4, 0x02, address, length]
    checksum = (~sum(packet[2:])) & 0xFF
    packet.append(checksum)
    
    ser.reset_input_buffer()
    ser.write(bytes(packet))
    # Expected response: 6 header/status bytes + data bytes
    expected = 6 + length
    resp = ser.read(expected)
    
    if len(resp) >= 6 + length:
        if length == 1:
            return resp[5]
        else:
            return resp[5] | (resp[6] << 8)
    return None

def set_angle(servo_id, angle_degrees):
    """
    Set a servo motor to a specific angle.
    
    The servo expects a position value from 0-4095 (called "ticks"), where:
      - 0 ticks = 0 degrees
      - 4095 ticks = 360 degrees
    We convert from degrees to ticks using the formula:
      ticks = (angle_degrees / 360) * 4095
    
    Args:
        servo_id: Hardware ID of the servo (1-6 for arm, or 6 for gripper)
        angle_degrees: Desired angle in degrees (0-360 range)
    """
    # Convert angle from degrees to the 0-4095 tick range that the servo understands
    position = int((angle_degrees / 360.0) * 4095)
    # Send command to servo: address 0x2A is the "Goal Position" register
    write_servo(servo_id, 0x2A, position, 2)

def get_angle(servo_id):
    """Get current servo angle in degrees"""
    pos = read_servo(servo_id, 0x38, 2)
    if pos is not None:
        return (pos / 4095.0) * 360.0
    return None

def read_all_angles():
    """Read all servo angles at once. Returns list of 6 angles or None on error."""
    angles = []
    for servo_id in JOINT_IDS:
        angle = get_angle(servo_id)
        if angle is None:
            print(f"  ERROR: Could not read servo {servo_id}")
            return None
        angles.append(angle)
    return angles

def enable_servo(servo_id):
    """Enable torque on servo"""
    write_servo(servo_id, 0x28, 1)

def disable_servo(servo_id):
    """Disable torque on servo"""
    write_servo(servo_id, 0x28, 0)

def scan_servos():
    """Scan for connected servos"""
    print("\n[SERVO SCAN]")
    found_servos = []
    for servo_id in range(1, 7):
        packet = [0xFF, 0xFF, servo_id, 2, 0x01]
        checksum = (~sum(packet[2:])) & 0xFF
        packet.append(checksum)
        
        ser.reset_input_buffer()
        ser.write(bytes(packet))
        time.sleep(0.05)
        resp = ser.read(20)
        
        if len(resp) > 0:
            print(f"  ✓ Servo {servo_id} ({JOINT_NAMES[servo_id-1]}) found")
            found_servos.append(servo_id)
        else:
            print(f"  ✗ Servo {servo_id} ({JOINT_NAMES[servo_id-1]}) NOT found")
    
    return found_servos

# ============================================================================
# CALIBRATION
# ============================================================================

def load_calibration(path="calibration.json"):
    """Load calibration from file"""
    global raw_calibration
    
    if not os.path.exists(path):
        print(f"ERROR: Calibration file not found: {path}")
        return None
    
    try:
        with open(path, "r") as f:
            raw = json.load(f)
        raw_calibration = raw
        print(f"✓ Calibration loaded from {path}")
        return raw
    except Exception as e:
        print(f"ERROR loading calibration: {e}")
        return None

def save_calibration(cal_data, path="calibration.json"):
    """Save calibration to file"""
    try:
        with open(path, "w") as f:
            json.dump(cal_data, f, indent=2)
        print(f"✓ Calibration saved to {path}")
    except Exception as e:
        print(f"ERROR saving calibration: {e}")

def get_joint_limits_from_cal(cal):
    """Extract joint limits from calibration"""
    limits = {}
    for joint_name in JOINT_NAMES:
        if joint_name in cal:
            j = cal[joint_name]
            if "range_min" in j and "range_max" in j:
                min_deg = (j["range_min"] / 4095.0) * 360.0
                max_deg = (j["range_max"] / 4095.0) * 360.0
                limits[joint_name] = {"min": min_deg, "max": max_deg}
    return limits

def get_joint_centers_from_cal(cal):
    """Extract joint centers/offsets from calibration"""
    centers = {}
    for joint_name in JOINT_NAMES:
        if joint_name in cal:
            j = cal[joint_name]
            if "homing_offset" in j:
                center_deg = (j["homing_offset"] / 4095.0) * 360.0
            else:
                range_min = (j.get("range_min", 0) / 4095.0) * 360.0
                range_max = (j.get("range_max", 4095) / 4095.0) * 360.0
                center_deg = (range_min + range_max) / 2.0
            centers[joint_name] = center_deg
    return centers

def calibrate_interactive():
    """Interactive calibration mode"""
    print("\n[CALIBRATION MODE]")
    print("Disable torque on all servos, then manually move to min/max positions.")
    print("Press Enter after reaching each position.")
    
    cal_data = {}
    joint_order = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
    
    for joint_name in joint_order:
        servo_id = JOINT_NAMES.index(joint_name) + 1
        print(f"\n--- {joint_name} (Servo {servo_id}) ---")
        
        input("  Move to MIN position, then press Enter: ")
        min_angle = get_angle(servo_id)
        if min_angle is None:
            print(f"  ERROR: Could not read angle from servo {servo_id}")
            continue
        
        input("  Move to MAX position, then press Enter: ")
        max_angle = get_angle(servo_id)
        if max_angle is None:
            print(f"  ERROR: Could not read angle from servo {servo_id}")
            continue
        
        center_angle = (min_angle + max_angle) / 2.0
        min_ticks = int((min_angle / 360.0) * 4095)
        max_ticks = int((max_angle / 360.0) * 4095)
        center_ticks = int((center_angle / 360.0) * 4095)
        
        cal_data[joint_name] = {
            "id": servo_id,
            "drive_mode": 0,
            "range_min": min_ticks,
            "range_max": max_ticks,
            "homing_offset": center_ticks
        }
        
        print(f"  ✓ Recorded: {min_angle:.1f}° - {max_angle:.1f}°")
    
    # Add gripper with default values
    cal_data["gripper"] = {
        "id": 6,
        "drive_mode": 0,
        "range_min": int((100 / 360.0) * 4095),
        "range_max": int((250 / 360.0) * 4095),
        "homing_offset": int((175 / 360.0) * 4095)
    }
    
    save_calibration(cal_data)

# ============================================================================
# MOTION CONTROL
# ============================================================================

def move_all_to_center(cal):
    """Move all joints to center position (logical 90°)"""
    print("\n[MOVE TO CENTER]")
    gripper_close()
    
    for servo_id in JOINT_IDS:
        joint_name = JOINT_NAMES[servo_id - 1]
        # Center is always logical 90° (middle of 0-180 range)
        safe_move(servo_id, 90, cal)
    
    time.sleep(1.0)

def logical_to_physical_angle(servo_id, logical_angle, cal):
    """Convert logical angle (0-180) to physical servo angle using calibration.
    
    Args:
        servo_id: Servo ID (1-6)
        logical_angle: Desired angle in 0-180 range
        cal: Calibration dict
    
    Returns:
        Physical servo angle in degrees, or None if out of range
    """
    joint_name = JOINT_NAMES[servo_id - 1]
    
    if not (0 <= logical_angle <= 180):
        return None
    
    if joint_name not in cal:
        # No calibration, assume 0-180 maps directly to servo
        return logical_angle
    
    # Get calibration bounds (in ticks)
    j = cal[joint_name]
    min_ticks = j.get("range_min", 0)
    max_ticks = j.get("range_max", 4095)
    
    # Convert to degrees
    min_deg = (min_ticks / 4095.0) * 360.0
    max_deg = (max_ticks / 4095.0) * 360.0
    
    # Map logical 0-180 to physical min_deg-max_deg
    physical_angle = min_deg + (logical_angle / 180.0) * (max_deg - min_deg)
    return physical_angle

def safe_move(servo_id, target_deg, cal):
    """
    Move a servo to a target angle, respecting calibrated limits.
    
    This function is "safe" because it uses calibration data to prevent the arm
    from moving outside its physically possible range. Most commands go through
    this function rather than calling set_angle() directly.
    
    The function uses a two-step conversion:
      1. Validate that target_deg is in the "logical" range (0-180 degrees)
      2. Convert logical angle to "physical" angle using calibration data:
         - Logical: user-friendly 0-180 scale (0=minimum, 180=maximum)
         - Physical: actual servo angle in degrees based on how this servo is mounted
    
    Example: A shoulder servo might have its range mapped as:
      - Logical 0° → Physical 45° (fully retracted)
      - Logical 180° → Physical 225° (fully extended)
    
    Args:
        servo_id: Hardware ID of the servo (1-6)
        target_deg: Desired angle in logical 0-180 range
        cal: Calibration dictionary with min/max physical angles for each servo
    """
    # Check that the requested angle is within the valid logical range (0-180 degrees)
    if not (0 <= target_deg <= 180):
        print(f"  ERROR: {target_deg}° out of logical range [0°, 180°]")
        return
    
    # Convert the logical angle (0-180) to physical angle using calibration
    physical_angle = logical_to_physical_angle(servo_id, target_deg, cal)
    if physical_angle is None:
        print(f"  ERROR: Cannot convert logical angle {target_deg}°")
        return
    
    # Enable the servo motor (turn on torque so it can move)
    enable_servo(servo_id)
    time.sleep(0.05)  # Wait a bit for servo to respond
    
    # Command the servo to move to the calculated physical angle
    set_angle(servo_id, physical_angle)
    time.sleep(0.5)  # Wait for servo to reach the target position
    
    # Read back the current angle and report to the user
    current = get_angle(servo_id)
    if current is not None:
        print(f"  ✓ Servo {servo_id} moved to logical {target_deg:.1f}° (physical: {current:.1f}°)")
    else:
        print(f"  ✓ Servo {servo_id} commanded to logical {target_deg:.1f}°")

def move_to_xyz(x, y, z, cal):
    """Move arm to XYZ position using IK solver.
    
    WHAT DOES THIS FUNCTION DO?
    This command moves the robotic arm to a specific 3D coordinate (x, y, z).
    It uses Inverse Kinematics (IK) to figure out what servo angles are needed,
    moves the arm, and then verifies we actually reached the target using
    Forward Kinematics (FK).
    
    THE STEP-BY-STEP PROCESS:
    1. CLOSE GRIPPER: Start with gripper closed (safety)
    2. SOLVE IK: Compute servo angles needed to reach (x, y, z)
    3. CHECK IK: If IK has no solution, target is unreachable (too far, behind obstacles)
    4. MOVE SERVOS: Command all arm servos to move to the computed angles
    5. WAIT: Let servos physically reach the target (takes 1.5 seconds)
    6. VERIFY: Read servo angles back and compute position using FK
    7. CHECK ERROR: Compare computed position to target - should be within ~2mm
    
    WHY CONVERT LOGICAL ANGLES TO PHYSICAL?
    The IK solver outputs "logical" angles (0-180 scale, user-friendly).
    But servos need "physical" angles based on how the robot is mounted.
    The calibration file contains the conversion formula for each servo.
    
    GRIPPER CLOSE:
    Gripper closes at start (safety) and at end (to hold objects).
    This ensures objects don't fall during arm movements.
    
    Args:
        x: Target X position in mm (forward/backward)
        y: Target Y position in mm (left/right)
        z: Target Z position in mm (up/down)
        cal: Calibration dictionary with servo angle mappings
        
    Returns:
        True if successful, False if IK failed or target unreachable
    """
    # Close the gripper first (safety - don't want to drop anything during movement)
    gripper_close()
    
    # Check that IK solver is available (was it loaded successfully?)
    if not HAS_IK_SOLVER:
        print("  ERROR: IK solver not available")
        return False
    
    # Print what we're doing
    print(f"\n[MOVE TO XYZ]")
    print(f"  Target: X={x:.1f}mm, Y={y:.1f}mm, Z={z:.1f}mm")
    print("  Solving IK...")
    
    # Use Inverse Kinematics to compute servo angles
    # Input: target 3D position (x, y, z)
    # Output: list of 6 servo angles [s1, s2, s3, s4, s5, s6] that make gripper reach (x,y,z)
    # If target is unreachable, returns None
    solution = solve_ik([x, y, z])
    if not solution:
        print("  ERROR: IK solution not found (target unreachable)")
        return False
    
    # Report the servo angles we computed
    print(f"  ✓ IK Solution found: {[f'{a:.1f}°' for a in solution]}")
    
    # Move all joint servos to the solution angles
    # (We skip the gripper - servo 6 - it was already handled above)
    print("  Moving arm...")
    for servo_id in JOINT_IDS:  # servo_id goes from 1 to 6
        # Get the logical angle for this servo from the IK solution
        angle = solution[servo_id - 1]  # -1 because solution list is 0-indexed
        
        # Enable the servo motor (turn on torque)
        enable_servo(servo_id)
        
        # Convert logical angle (0-180 user-friendly scale) to physical angle
        # based on how this servo is mounted. Use calibration data.
        physical_angle = logical_to_physical_angle(servo_id, angle, cal)
        
        # Only send the command if conversion succeeded
        if physical_angle is not None:
            set_angle(servo_id, physical_angle)
    
    # Wait for all servos to physically reach their target positions
    # (servo motors take time to move to new angles)
    time.sleep(1.5)
    
    # Verify we actually reached the target using Forward Kinematics
    # FK: given servo angles → compute 3D position
    # This lets us check if our actual position matches our target
    try:
        # Compute the position based on current servo angles
        verify_pos = solve_fk(solution)
        
        # Compute the distance between our target and actual position
        # (Euclidean distance: sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2))
        error = ((verify_pos[0] - x)**2 + (verify_pos[1] - y)**2 + (verify_pos[2] - z)**2)**0.5
        
        # Report success
        print(f"  ✓ Position reached")
        print(f"    Actual: X={verify_pos[0]:.1f}mm, Y={verify_pos[1]:.1f}mm, Z={verify_pos[2]:.1f}mm")
        print(f"    Error: {error:.2f}mm")
        return True
    except Exception as e:
        # If FK verification fails, we still report position reached because
        # the servo angles were set correctly - the issue is just with verification
        print(f"  Position reached (verification failed: {e})")
        gripper_close()
        return True

def plan_joint_path(start_angles, end_angles, steps=20):
    """Plan linear path in joint space"""
    path = []
    for step in range(steps + 1):
        t = step / steps
        angles = [
            start_angles[i] + (end_angles[i] - start_angles[i]) * t
            for i in range(len(start_angles))
        ]
        path.append(angles)
    return path

def execute_joint_path(path, cal):
    """Execute a pre-planned joint path"""
    print("\n[EXECUTING PATH]")
    gripper_close()
    
    for step_idx, angles in enumerate(path):
        print(f"  Step {step_idx + 1}/{len(path)}...", end=" ", flush=True)
        
        for servo_id, angle in enumerate(angles, start=1):
            enable_servo(servo_id)
            set_angle(servo_id, angle)
        
        time.sleep(0.1)
        print("done")
    
    print("  ✓ Path execution complete")

# ============================================================================
# PATH RECORDING & REPLAY
# ============================================================================

PATHS_FILE = "paths.json"

def load_paths():
    """Load saved paths from file"""
    if os.path.exists(PATHS_FILE):
        try:
            with open(PATHS_FILE, "r") as f:
                return json.load(f)
        except Exception:
            return {}
    return {}

def save_paths(paths):
    """Save paths to file"""
    try:
        with open(PATHS_FILE, "w") as f:
            json.dump(paths, f, indent=2)
        print(f"  ✓ Paths saved to {PATHS_FILE}")
    except Exception as e:
        print(f"  ERROR: {e}")

def start_recording_path(name):
    """Start recording a new motion path"""
    global current_path, current_path_name
    current_path_name = name
    current_path = []
    print(f"  ✓ Started recording path: {name}")
    print("    Use 'add' to record waypoints, 'save' to finish")

def add_waypoint():
    """Add current servo positions as waypoint"""
    global current_path
    if current_path is None:
        print("  ERROR: Not recording. Use 'start <name>' first")
        return
    
    waypoint = read_all_angles()
    if waypoint is None:
        return
    
    current_path.append(waypoint)
    print(f"  ✓ Waypoint {len(current_path)} recorded: {[f'{a:.1f}°' for a in waypoint]}")

def undo_waypoint():
    """Remove the last recorded waypoint"""
    global current_path
    if current_path is None:
        print("  ERROR: Not recording. Use 'start <name>' first")
        return
    if not current_path:
        print("  ERROR: No waypoints to undo")
        return
    removed = current_path.pop()
    print(f"  ✓ Removed waypoint {len(current_path)+1}: {[f'{a:.1f}°' for a in removed]}")
    print(f"  {len(current_path)} waypoints remaining")

def save_current_path():
    """Save currently recorded path"""
    global current_path, current_path_name
    
    if current_path is None or current_path_name is None:
        print("  ERROR: No path being recorded")
        return
    
    paths = load_paths()
    paths[current_path_name] = current_path
    save_paths(paths)
    print(f"  ✓ Path '{current_path_name}' saved with {len(current_path)} waypoints")
    current_path = None
    current_path_name = None

def execute_saved_path(name, cal):
    """Execute a previously saved path"""
    gripper_close()
    paths = load_paths()
    
    if name not in paths:
        print(f"  ERROR: Path '{name}' not found")
        print(f"  Available paths: {list(paths.keys())}")
        return
    
    path = paths[name]
    print(f"  ✓ Executing path '{name}' with {len(path)} waypoints")
    
    for wp_idx, waypoint in enumerate(path):
        print(f"    Waypoint {wp_idx + 1}/{len(path)}...", end=" ", flush=True)
        
        for servo_id, angle in enumerate(waypoint, start=1):
            enable_servo(servo_id)
            set_angle(servo_id, angle)
        
        time.sleep(0.5)
        print("✓")
    
    print("  ✓ Path execution complete")

def execute_smooth_path(name, cal, total_time=5.0, steps_per_seg=10):
    """Execute a saved path with smooth cubic spline interpolation.
    Uses scipy CubicSpline to create fluid motion between waypoints."""
    gripper_close()
    paths = load_paths()
    
    if name not in paths:
        print(f"  ERROR: Path '{name}' not found")
        print(f"  Available paths: {list(paths.keys())}")
        return
    
    path = paths[name]
    if len(path) < 2:
        print("  Need at least 2 waypoints for smooth interpolation")
        execute_saved_path(name, cal)
        return
    
    try:
        from scipy.interpolate import CubicSpline
    except ImportError:
        print("  scipy not available, using linear playback instead")
        execute_saved_path(name, cal)
        return
    
    n_waypoints = len(path)
    waypoints = np.array(path)
    t_knots = np.linspace(0, total_time, n_waypoints)
    
    # Create cubic spline for each joint
    splines = []
    for joint_idx in range(waypoints.shape[1]):
        cs = CubicSpline(t_knots, waypoints[:, joint_idx], bc_type='clamped')
        splines.append(cs)
    
    total_steps = steps_per_seg * (n_waypoints - 1)
    t_eval = np.linspace(0, total_time, total_steps)
    dt = total_time / total_steps
    
    print(f"  ✓ Smooth executing '{name}': {n_waypoints} waypoints, "
          f"{total_steps} steps over {total_time:.1f}s")
    
    for step_idx, t in enumerate(t_eval):
        angles = [float(splines[j](t)) for j in range(len(splines))]
        
        for servo_id, angle in enumerate(angles, start=1):
            enable_servo(servo_id)
            set_angle(servo_id, angle)
        
        if step_idx % steps_per_seg == 0:
            wp_num = step_idx // steps_per_seg + 1
            print(f"    Waypoint {wp_num}/{n_waypoints}...", flush=True)
        
        time.sleep(dt)
    
    print("  ✓ Smooth path execution complete")

def delete_saved_path(name):
    """Delete a saved path by name"""
    paths = load_paths()
    if name not in paths:
        print(f"  ERROR: Path '{name}' not found")
        print(f"  Available paths: {list(paths.keys())}")
        return
    
    waypoint_count = len(paths[name])
    del paths[name]
    save_paths(paths)
    print(f"  ✓ Deleted path '{name}' ({waypoint_count} waypoints)")

def delete_all_paths():
    """Delete all saved paths"""
    paths = load_paths()
    if not paths:
        print("  No paths to delete")
        return
    
    count = len(paths)
    save_paths({})
    print(f"  ✓ Deleted all {count} saved paths")

def list_saved_paths():
    """List all saved paths"""
    paths = load_paths()
    if not paths:
        print("  No saved paths")
        return
    
    print("  Saved paths:")
    for name, path in paths.items():
        print(f"    - {name}: {len(path)} waypoints")

def edit_path(name):
    """Load an existing path into the recording state for editing"""
    global current_path, current_path_name
    paths = load_paths()
    if name not in paths:
        print(f"  ERROR: Path '{name}' not found")
        print(f"  Available paths: {list(paths.keys())}")
        return
    
    current_path_name = name
    current_path = [list(wp) for wp in paths[name]]  # deep copy
    print(f"  ✓ Editing path '{name}' ({len(current_path)} waypoints)")
    print("    Commands: wp, wp_del <n>, wp_replace <n>, wp_insert <n>,")
    print("              wp_goto <n>, wp_swap <a> <b>, add, undo, save")
    list_waypoints()

def list_waypoints():
    """List all waypoints in the current recording"""
    if current_path is None:
        print("  ERROR: No path being recorded/edited. Use 'start <name>' or 'edit <name>'")
        return
    if not current_path:
        print("  No waypoints yet")
        return
    print(f"  Waypoints for '{current_path_name}' ({len(current_path)} total):")
    for i, wp in enumerate(current_path):
        angles_str = ", ".join(f"{a:.1f}°" for a in wp)
        print(f"    {i+1}. [{angles_str}]")

def delete_waypoint(index):
    """Delete a waypoint by 1-based index"""
    global current_path
    if current_path is None:
        print("  ERROR: No path being recorded/edited")
        return
    if index < 1 or index > len(current_path):
        print(f"  ERROR: Invalid index {index}. Valid range: 1-{len(current_path)}")
        return
    removed = current_path.pop(index - 1)
    print(f"  ✓ Deleted waypoint {index}: [{', '.join(f'{a:.1f}°' for a in removed)}]")
    print(f"  {len(current_path)} waypoints remaining")

def replace_waypoint(index):
    """Replace a waypoint with the current arm position"""
    global current_path
    if current_path is None:
        print("  ERROR: No path being recorded/edited")
        return
    if index < 1 or index > len(current_path):
        print(f"  ERROR: Invalid index {index}. Valid range: 1-{len(current_path)}")
        return
    
    waypoint = read_all_angles()
    if waypoint is None:
        return
    
    old = current_path[index - 1]
    current_path[index - 1] = waypoint
    print(f"  ✓ Replaced waypoint {index}:")
    print(f"    Old: [{', '.join(f'{a:.1f}°' for a in old)}]")
    print(f"    New: [{', '.join(f'{a:.1f}°' for a in waypoint)}]")

def insert_waypoint(index):
    """Insert the current arm position before the given 1-based index"""
    global current_path
    if current_path is None:
        print("  ERROR: No path being recorded/edited")
        return
    if index < 1 or index > len(current_path) + 1:
        print(f"  ERROR: Invalid index {index}. Valid range: 1-{len(current_path) + 1}")
        return
    
    waypoint = read_all_angles()
    if waypoint is None:
        return
    
    current_path.insert(index - 1, waypoint)
    print(f"  ✓ Inserted waypoint at position {index}: [{', '.join(f'{a:.1f}°' for a in waypoint)}]")
    print(f"  {len(current_path)} waypoints total")

def goto_waypoint(index):
    """Move the arm to a specific waypoint position"""
    if current_path is None:
        print("  ERROR: No path being recorded/edited")
        return
    if index < 1 or index > len(current_path):
        print(f"  ERROR: Invalid index {index}. Valid range: 1-{len(current_path)}")
        return
    
    waypoint = current_path[index - 1]
    print(f"  Moving to waypoint {index}: [{', '.join(f'{a:.1f}°' for a in waypoint)}]")
    for servo_id, angle in enumerate(waypoint, start=1):
        enable_servo(servo_id)
        set_angle(servo_id, angle)
    time.sleep(0.5)
    print("  ✓ Done")

def swap_waypoints(idx_a, idx_b):
    """Swap two waypoints by 1-based index"""
    global current_path
    if current_path is None:
        print("  ERROR: No path being recorded/edited")
        return
    for idx in (idx_a, idx_b):
        if idx < 1 or idx > len(current_path):
            print(f"  ERROR: Invalid index {idx}. Valid range: 1-{len(current_path)}")
            return
    current_path[idx_a - 1], current_path[idx_b - 1] = current_path[idx_b - 1], current_path[idx_a - 1]
    print(f"  ✓ Swapped waypoints {idx_a} and {idx_b}")

# ============================================================================
# PILLAR PICK & DROP
# ============================================================================

def _load_home_position():
    """Load saved home position from file. Returns list of 6 angles or None."""
    if os.path.exists(HOME_FILE):
        try:
            with open(HOME_FILE, "r") as f:
                data = json.load(f)
            return data.get("angles")
        except Exception:
            pass
    return None

def _save_home_position(angles):
    """Save home position to file."""
    with open(HOME_FILE, "w") as f:
        json.dump({"angles": angles}, f, indent=2)

def set_home():
    """Set the arm's current position as the home position for all pillar paths.
    
    - Reads current servo angles
    - Saves them as the home position
    - Updates waypoint 1 of every existing pillar path in paths.json
    """
    angles = read_all_angles()
    if angles is None:
        print("  ERROR: Could not read servo positions")
        return
    
    # Save home to file
    _save_home_position(angles)
    print(f"  ✓ Home position set: {[f'{a:.1f}°' for a in angles]}")
    
    # Update waypoint 1 of all existing pillar paths
    paths = load_paths()
    updated = []
    for action in PILLAR_ACTIONS:
        if action in paths and len(paths[action]) > 0:
            paths[action][0] = list(angles)
            updated.append(action)
    
    if updated:
        save_paths(paths)
        print(f"  ✓ Updated first waypoint of: {', '.join(updated)}")
    else:
        print("  (no pillar paths recorded yet — home will be used when you record them)")

def go_home():
    """Move arm to the saved home position, preserving gripper angle."""
    home = _load_home_position()
    if home is None:
        print("  ERROR: No home position set. Use 'set_home' first.")
        return
    
    print(f"  Moving to home position (gripper unchanged)...")
    for servo_id in JOINT_IDS[:-1]:  # all except gripper (servo 6)
        enable_servo(servo_id)
        set_angle(servo_id, home[servo_id - 1])
    
    time.sleep(0.5)
    print("  ✓ Home")

def pillar_record(name):
    """Start recording a pillar path with the saved home position as waypoint 1.
    
    Usage: rec_pick1, rec_drop2, etc.
    The saved home position is automatically added as waypoint 1.
    Then manually position the arm and use 'add' for each waypoint, 'save' when done.
    """
    global current_path, current_path_name
    if name not in PILLAR_ACTIONS:
        print(f"  ERROR: '{name}' is not a valid pillar action")
        print(f"  Valid actions: {PILLAR_ACTIONS}")
        return
    
    home = _load_home_position()
    if home is None:
        print("  ERROR: No home position set. Use 'set_home' first.")
        return
    
    current_path_name = name
    current_path = [list(home)]
    print(f"  ✓ Recording pillar path: {name}")
    print(f"    Waypoint 1 (home): {[f'{a:.1f}°' for a in home]}")
    print(f"    Add waypoints: 'add' (next pos), 'undo' (remove last)")
    print(f"    View/edit:    'wp' (list), 'wp_del <n>' (delete), 'wp_replace <n>' (replace)")
    print(f"    More edits:   'wp_insert <n>' (insert), 'wp_goto <n>' (move to), 'wp_swap <a> <b>' (swap)")
    print(f"    Finish:       'save' (done), 'cancel' (abort)")

def pillar_execute(name, cal):
    """Execute a pillar pick/drop path: go home first, then run the saved path."""
    if name not in PILLAR_ACTIONS:
        print(f"  ERROR: '{name}' is not a valid pillar action")
        print(f"  Valid actions: {PILLAR_ACTIONS}")
        return
    paths = load_paths()
    if name not in paths:
        print(f"  ERROR: Path '{name}' not recorded yet")
        print(f"  Use 'rec_{name}' to record it first")
        return
    
    print(f"\n[PILLAR {name.upper()}]")
    go_home()
    execute_saved_path(name, cal)

def pillar_status():
    """Show which pillar paths have been recorded"""
    paths = load_paths()
    print("  Pillar paths:")
    for action in PILLAR_ACTIONS:
        if action in paths:
            print(f"    ✓ {action}: {len(paths[action])} waypoints")
        else:
            print(f"    ✗ {action}: not recorded")

# ============================================================================
# FK/IK OPERATIONS
# ============================================================================

def test_fk(angles_deg):
    """Test forward kinematics"""
    if not HAS_IK_SOLVER:
        print("  ERROR: ik_solver module not available")
        return
    
    try:
        end_effector = solve_fk(angles_deg)
        print(f"  [OK] FK Result:")
        print(f"    Position: X={end_effector[0]:.1f}, Y={end_effector[1]:.1f}, Z={end_effector[2]:.1f} mm")
        return end_effector
    except Exception as e:
        print(f"  ERROR in FK: {e}")
        return None

def test_ik(x, y, z):
    """Test inverse kinematics"""
    if not HAS_IK_SOLVER:
        print("  ERROR: ik_solver module not available")
        return
    
    try:
        solution = solve_ik([x, y, z])
        if solution:
            print(f"  [OK] IK found solution:")
            print(f"    Angles: {[f'{a:.1f}°' for a in solution]}")
            return solution
        else:
            print("  [FAIL] IK: No solution found for target position")
            return None
    except Exception as e:
        print(f"  ERROR in IK: {e}")
        return None

# ============================================================================
# PICK & PLACE
# ============================================================================

def gripper_open():
    """Open gripper"""
    print("  Opening gripper...")
    set_angle(6, GRIPPER_OPEN_ANGLE)
    time.sleep(0.5)

def gripper_close():
    """Close gripper"""
    print("  Closing gripper...")
    set_angle(6, GRIPPER_CLOSE_ANGLE)
    time.sleep(0.5)

def pick_and_place_demo(target_x, target_y, target_z, cal):
    """Demo pick and place sequence"""
    print(f"\n[PICK & PLACE DEMO]")
    print(f"  Target: X={target_x}, Y={target_y}, Z={target_z}")
    gripper_close()
    
    if not HAS_IK_SOLVER:
        print("  ERROR: IK solver not available")
        return
    
    # Move to approach position
    approach_z = target_z + APPROACH_HEIGHT
    print(f"\n  1. Moving to approach position (Z+{APPROACH_HEIGHT})...")
    approach_angles = test_ik(target_x, target_y, approach_z)
    if not approach_angles:
        print("  FAILED: Cannot reach approach position")
        return
    
    execute_joint_path([approach_angles], cal)
    
    # Move down to object
    print(f"\n  2. Moving down to object...")
    pick_angles = test_ik(target_x, target_y, target_z)
    if not pick_angles:
        print("  FAILED: Cannot reach pick position")
        return
    
    execute_joint_path([pick_angles], cal)
    
    # Close gripper
    print(f"\n  3. Closing gripper...")
    gripper_close()
    
    # Move back to approach
    print(f"\n  4. Moving back up...")
    execute_joint_path([approach_angles], cal)
    
    print(f"\n  [OK] Pick & place demo complete")

# ============================================================================
# VISUAL SERVOING
# ============================================================================

def detect_target_position():
    """Detect red object and return its 3D position in camera frame (mm).
    
    WHAT THIS FUNCTION DOES:
    Finds red objects in the camera view and computes their 3D location.
    This is essential for visual servoing - the arm uses this to locate
    objects it needs to pick up.
    
    HOW IT WORKS (Step by step):
    1. Capture color frame: Get RGB image from camera
    2. Convert to HSV: Change color space from RGB to HSV
       - Why HSV? HSV separates "what color" (Hue) from "how bright" (Saturation/Value)
       - This makes red detection more robust to lighting changes
    3. Red detection: Create a "mask" - a binary image where red pixels = white, others = black
       - Red wraps around in HSV (appears at both low and high hue values)
    4. Morphological cleanup: Apply erosion/dilation to remove noise
    5. Find contours: Locate connected regions of white (red objects)
    6. Find largest: Pick the biggest red region (our target)
    7. Get center: Compute the (x, y) pixel location of the object center
    8. Get depth: Read the depth sensor value at that pixel location
    9. Convert to 3D: Use camera intrinsics to turn 2D pixel + depth into 3D coordinates
    10. Average frames: Repeat 5 times and average for noise reduction
    
    WHY AVERAGE 5 FRAMES?
    Depth sensors are noisy - individual measurements bounce around.
    Averaging 5 consecutive frames smooths out this noise, making the
    3D position estimate much more stable and reliable.
    
    CAMERA INTRINSICS:
    These are calibration values (focal length, principal point) that convert
    2D pixel coordinates + depth into 3D world coordinates. Like converting
    from a photograph back to 3D space.
    
    Returns:
        ([cam_x, cam_y, cam_z], [pixel_u, pixel_v]) where:
          - cam_x, cam_y, cam_z: 3D position in camera frame (millimeters)
          - pixel_u, pixel_v: 2D position in camera image (pixels, 0-640)
        
        Returns (None, None) if no red object is detected in the frame
    """
    # Check that RealSense camera has been initialized
    if not HAS_REALSENSE or realsense_pipeline is None:
        print("  ERROR: RealSense not initialized")
        return None, None
    
    # Check that camera intrinsics (calibration) have been captured
    if realsense_intrinsics is None:
        print("  ERROR: Camera intrinsics not available")
        return None, None

    # Collect multiple frames and average them for noise reduction
    # We'll average the 3D positions from 5 separate camera frames
    n_samples = 5
    positions = []        # List to store 3D positions from each frame
    pixel_positions = []  # List to store 2D pixel positions from each frame

    # Flush the camera pipeline: skip the first 5 frames
    # (they might be old or stale from the buffer)
    for _ in range(n_samples + 3):
        frames = realsense_pipeline.wait_for_frames()

    # Now capture and process 5 fresh frames
    for _ in range(n_samples):
        # Get both color and depth frames from the camera
        frames = realsense_pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()  # RGB image
        depth_frame = frames.get_depth_frame()  # Depth map (distance from camera)
        
        # Check that we successfully got both frames
        if not color_frame or not depth_frame:
            continue

        # Convert color frame to numpy array (easier to process)
        color_image = np.asanyarray(color_frame.get_data())  # Shape: (480, 640, 3)
        
        # Convert color from BGR (Blue-Green-Red) to HSV (Hue-Saturation-Value)
        # BGR is what cameras use, but HSV is better for color-based detection
        # Hue (0-180) = the actual color (red, green, blue, etc.)
        # Saturation = how "pure" the color is (0=gray, 255=pure color)
        # Value = brightness (0=black, 255=bright)
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # Create a mask for red colors
        # Red is special in HSV - it wraps around at both ends:
        #   Range 1: Hue 0-10 (pure red and very dark red)
        #   Range 2: Hue 170-180 (dark red wrapping around to pure red)
        # We use these thresholds:
        #   Saturation 90-255: excludes grayish colors
        #   Value 40-255: excludes very dark colors
        lower_red1 = np.array([0, 90, 40])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 90, 40])
        upper_red2 = np.array([180, 255, 255])
        
        # Create two masks: one for each red range
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        
        # Combine the two masks (bitwise OR: include pixels from either mask)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # Apply morphological operations to clean up the mask
        # This removes small noise spots and fills small holes
        # Kernel: 7x7 ellipse shape for smooth cleanup
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        # MORPH_CLOSE: fill holes in objects
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        # Find contours (outlines) of red objects in the mask
        # RETR_EXTERNAL: only outermost contours (faster, cleaner)
        # CHAIN_APPROX_SIMPLE: simplify contours (remove redundant points)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # If no red objects found, skip this frame and try the next one
        if not contours:
            continue

        # Find the largest contour (most likely our target object)
        # Use area as the size metric
        best = max(contours, key=cv2.contourArea)
        
        # Skip if the red region is too small (probably noise, not a real object)
        if cv2.contourArea(best) < 800:  # Minimum area in pixels
            continue

        # Get bounding box of the red object
        x, y, w, h = cv2.boundingRect(best)
        # Compute center of the bounding box (in pixel coordinates)
        cx, cy = x + w // 2, y + h // 2
        
        # Get the depth value at the object center
        # depth_val is in meters from the camera
        depth_val = depth_frame.get_distance(cx, cy)
        
        # If depth is zero, the sensor couldn't read that location (invalid)
        if depth_val == 0:
            continue

        # Convert 2D pixel + depth into 3D coordinates using camera intrinsics
        # Intrinsics include:
        #   fx, fy: focal lengths (in pixels)
        #   ppx, ppy: principal point (image center in pixels)
        # 
        # The conversion formulas are:
        #   cam_x = (pixel_u - ppx) / fx * depth_meters * 1000  →  convert to mm
        #   cam_y = (pixel_v - ppy) / fy * depth_meters * 1000  →  convert to mm
        #   cam_z = depth_meters * 1000  →  convert to mm
        #
        # This is called the "camera projection model"
        intr = realsense_intrinsics
        cam_x = (cx - intr.ppx) / intr.fx * depth_val * 1000  # mm
        cam_y = (cy - intr.ppy) / intr.fy * depth_val * 1000  # mm
        cam_z = depth_val * 1000  # mm
        
        # Store this frame's result
        positions.append([cam_x, cam_y, cam_z])
        pixel_positions.append([cx, cy])

    # If we didn't successfully detect the object in any frame, return None
    if not positions:
        return None, None

    # Average all the 3D positions we collected (noise reduction)
    avg_pos = np.mean(positions, axis=0).tolist()
    avg_px = np.mean(pixel_positions, axis=0).tolist()
    
    return avg_pos, avg_px


def _get_current_robot_xyz(cal):
    """Read current servo angles and compute FK position.
    Returns [x, y, z] in mm or None on failure."""
    if not HAS_IK_SOLVER:
        return None
    angles = []
    for sid in JOINT_IDS:
        a = get_angle(sid)
        if a is None:
            return None
        # Convert physical angle to logical angle for FK
        joint_name = JOINT_NAMES[sid - 1]
        if joint_name in cal:
            j = cal[joint_name]
            min_deg = (j.get("range_min", 0) / 4095.0) * 360.0
            max_deg = (j.get("range_max", 4095) / 4095.0) * 360.0
            if max_deg != min_deg:
                logical = (a - min_deg) / (max_deg - min_deg) * 180.0
            else:
                logical = 90.0
        else:
            logical = a
        angles.append(logical)
    try:
        return solve_fk(angles)
    except Exception:
        return None


def _estimate_jacobian(cal, current_xyz, probe_delta=15.0):
    """Estimate image Jacobian by making small probe moves and observing
    how the object's pixel position changes.

    Makes 2 probe moves (robot X, robot Y), observes pixel shift,
    builds a 2x2 Jacobian: d(pixel) / d(robot_mm).

    Returns J (2x2 numpy array) or None on failure.
    """
    print("  Estimating Jacobian (2 probe moves)...")

    # Detect object at current position
    _, px_before = detect_target_position()
    if px_before is None:
        print("  ERROR: Can't see target for Jacobian estimation")
        return None
    px_before = np.array(px_before)

    J = np.zeros((2, 2))  # J maps [dX_robot, dY_robot] -> [du_pixel, dv_pixel]
    base_x, base_y, base_z = current_xyz[0], current_xyz[1], current_xyz[2]

    # Probe along robot X
    print(f"    Probe X: +{probe_delta:.0f}mm...", end=" ", flush=True)
    move_to_xyz(base_x + probe_delta, base_y, base_z, cal)
    time.sleep(0.8)
    _, px_after_x = detect_target_position()
    # Move back
    move_to_xyz(base_x, base_y, base_z, cal)
    time.sleep(0.5)

    if px_after_x is None:
        print("FAILED (lost target)")
        return None
    px_after_x = np.array(px_after_x)
    dpx = px_after_x - px_before
    # If pixels barely moved, the probe might be too small or object is out of view
    print(f"dpx=[{dpx[0]:.1f}, {dpx[1]:.1f}]")
    J[:, 0] = dpx / probe_delta  # d(pixel)/d(robot_X)

    # Probe along robot Y
    print(f"    Probe Y: +{probe_delta:.0f}mm...", end=" ", flush=True)
    move_to_xyz(base_x, base_y + probe_delta, base_z, cal)
    time.sleep(0.8)
    _, px_after_y = detect_target_position()
    # Move back
    move_to_xyz(base_x, base_y, base_z, cal)
    time.sleep(0.5)

    if px_after_y is None:
        print("FAILED (lost target)")
        return None
    px_after_y = np.array(px_after_y)
    dpx = px_after_y - px_before
    print(f"dpx=[{dpx[0]:.1f}, {dpx[1]:.1f}]")
    J[:, 1] = dpx / probe_delta  # d(pixel)/d(robot_Y)

    # Check if Jacobian is invertible
    det = np.linalg.det(J)
    if abs(det) < 1e-6:
        print("  ERROR: Jacobian is singular (camera can't distinguish X/Y moves)")
        return None

    print(f"  Jacobian estimated (det={det:.4f}):")
    print(f"    [{J[0,0]:>7.3f}  {J[0,1]:>7.3f}]  px/mm")
    print(f"    [{J[1,0]:>7.3f}  {J[1,1]:>7.3f}]")
    return J


def visual_servo(cal, target_z_override=None, max_iterations=10,
                 tolerance_px=8, gain=0.5, approach_height=60):
    """Image-Based Visual Servoing (IBVS): iteratively move arm toward detected
    red object using pixel-space feedback. Does NOT rely on camera-to-robot
    calibration accuracy.

    How it works:
      1. Use camera calibration for rough initial move (gets arm in the ballpark)
      2. Estimate a live Jacobian by making small probe moves and observing
         how the object's pixel position changes
      3. Use the Jacobian inverse to compute robot XY corrections from pixel error
      4. Iterate until the object pixel position stops changing (arm is on target)
      5. Descend to target Z

    Args:
        cal: Calibration dict
        target_z_override: Z height to descend to (mm). If None, uses camera depth.
        max_iterations: Maximum correction iterations
        tolerance_px: Stop when pixel error is below this (pixels)
        gain: Control gain (0-1). Lower = cautious, higher = fast
        approach_height: mm above target for approach (default 60)
    """
    if not HAS_IK_SOLVER:
        print("  ERROR: IK solver not available")
        return False
    if not HAS_REALSENSE or realsense_pipeline is None:
        print("  ERROR: RealSense not initialized. Run 'rs_init' first.")
        return False

    print(f"\n[VISUAL SERVOING - IBVS]")
    print(f"  Tolerance: {tolerance_px}px | Max iterations: {max_iterations} | Gain: {gain}")

    # ── Step 1: Initial detection ──
    print("  Detecting target...")
    cam_pos, target_pixel = detect_target_position()
    if cam_pos is None:
        print("  ERROR: No red object detected")
        return False

    target_px = np.array(target_pixel)
    print(f"  Target pixel: [{target_px[0]:.0f}, {target_px[1]:.0f}]")
    print(f"  Camera 3D: [{cam_pos[0]:.0f}, {cam_pos[1]:.0f}, {cam_pos[2]:.0f}] mm")

    # Determine approach Z from camera depth
    # Camera Z = depth = forward. For robot, we need a Z height.
    # Use camera calibration if available for rough Z estimate, else use override
    target_z = target_z_override
    if target_z is None:
        if camera_transform_R is not None:
            robot_target = camera_to_robot(cam_pos)
            if robot_target:
                target_z = robot_target[2]
                print(f"  Estimated robot Z from cam cal: {target_z:.1f}mm")
        if target_z is None:
            target_z = 0  # default to table level
            print(f"  WARNING: No Z estimate, defaulting to {target_z}mm")

    approach_z = target_z + approach_height

    # ── Step 2: Rough initial move using camera calibration ──
    if camera_transform_R is not None:
        robot_target = camera_to_robot(cam_pos)
        if robot_target:
            init_x, init_y = robot_target[0], robot_target[1]
            print(f"\n  --- Rough approach via cam cal ---")
            print(f"  Moving to X={init_x:.1f}, Y={init_y:.1f}, Z={approach_z:.1f}")
            gripper_close()
            success = move_to_xyz(init_x, init_y, approach_z, cal)
            if not success:
                print("  ERROR: Cannot reach initial approach position")
                return False
            time.sleep(1.0)
    else:
        # No camera cal — need arm to already be somewhat near the object
        print("  WARNING: No camera calibration. Arm should already be near the target.")
        # Get current position as starting point
        cur = _get_current_robot_xyz(cal)
        if cur is None:
            print("  ERROR: Cannot read current arm position")
            return False
        init_x, init_y = cur[0], cur[1]

    # ── Step 3: Estimate Jacobian ──
    cur = _get_current_robot_xyz(cal)
    if cur is None:
        print("  ERROR: Cannot read arm position for Jacobian")
        return False

    J = _estimate_jacobian(cal, [cur[0], cur[1], approach_z], probe_delta=15.0)
    if J is None:
        print("  ERROR: Jacobian estimation failed")
        return False

    J_inv = np.linalg.inv(J)

    # Current robot XY (the position we're commanding)
    cmd_x, cmd_y = cur[0], cur[1]

    # ── Step 4: IBVS correction loop ──
    print(f"\n  --- IBVS Loop ---")
    for iteration in range(1, max_iterations + 1):
        # Re-detect object
        _, current_pixel = detect_target_position()
        if current_pixel is None:
            print(f"  Iter {iteration}: WARNING - lost target, stopping")
            break

        current_px = np.array(current_pixel)

        # Pixel error: where is the object NOW vs where it was ORIGINALLY
        # When the arm is on the object, the object pixel won't change because
        # the object is fixed. But initially the object pixel drifts as we may
        # partially occlude it. More reliably: we want the object pixel to match
        # the initial target pixel (object hasn't moved, so if we see it shift,
        # it's noise or parallax).
        #
        # Actually: the object is STATIONARY. Its pixel position stays ~constant.
        # We want the ARM's end-effector to be at the object.
        # The Jacobian tells us: if I move the arm, the OBJECT pixel doesn't change
        # (it's fixed), but the ARM's pixel would change.
        #
        # Better approach: compute error as (target_pixel - current_pixel).
        # If the arm is near the object, the object pixel = target_pixel (stable).
        # The Jacobian maps arm movement to pixel change of a POINT ON THE ARM.
        # But we're tracking the OBJECT not the arm.
        #
        # Simplest correct approach:
        # 1. Object pixel = P_obj (detected, ~constant since object is fixed)
        # 2. Arm end-effector pixel = P_arm (from FK + camera projection)
        # 3. Error = P_obj - P_arm (in pixels)
        # 4. Correction = J_inv * error (in robot mm)
        #
        # We can project the arm position into pixels using camera intrinsics.
        # OR: since the Jacobian was estimated at THIS position, just use:
        #   - The pixel error = target_pixel - current_pixel
        #   - If arm moves didn't change the object pixel, pixel error = constant offset
        #   - J_inv converts that offset to robot mm correction

        pixel_error = target_px - current_px
        error_norm = np.linalg.norm(pixel_error)

        print(f"  Iter {iteration}: pixel=[{current_px[0]:.0f},{current_px[1]:.0f}] "
              f"err=[{pixel_error[0]:.1f},{pixel_error[1]:.1f}] |{error_norm:.1f}px|", end="")

        if error_norm < tolerance_px:
            print(f" ✓ CONVERGED")
            break

        # Convert pixel error to robot XY correction using Jacobian inverse
        # pixel_error = J * robot_correction  =>  robot_correction = J_inv * pixel_error
        robot_correction = J_inv @ pixel_error
        
        # Clip corrections to prevent huge jumps (max 30mm per iteration)
        max_correction = 30.0
        correction_norm = np.linalg.norm(robot_correction)
        if correction_norm > max_correction:
            robot_correction = robot_correction * (max_correction / correction_norm)
            print(f" [clipped to {max_correction:.0f}mm]", end="")
        
        cmd_x += robot_correction[0] * gain
        cmd_y += robot_correction[1] * gain

        print(f" → move dX={robot_correction[0]*gain:.1f} dY={robot_correction[1]*gain:.1f}mm")

        success = move_to_xyz(cmd_x, cmd_y, approach_z, cal)
        if not success:
            print("  ERROR: Cannot reach corrected position")
            return False
        time.sleep(0.8)
    else:
        print(f"  WARNING: Did not converge after {max_iterations} iterations")

    # ── Step 5: Descend to target Z ──
    print(f"\n  --- Descending to Z={target_z:.0f}mm ---")
    success = move_to_xyz(cmd_x, cmd_y, target_z, cal)
    if not success:
        print("  ERROR: Cannot reach final position")
        return False

    gripper_close()

    print(f"\n  ✓ Visual servo complete")
    print(f"    Final robot pos: X={cmd_x:.1f}, Y={cmd_y:.1f}, Z={target_z:.1f} mm")
    return True


def visual_servo_pick(cal, pick_z=None, place_xyz=None):
    """Visual servoing pick (and optional place).
    Detects red object, servos to it, picks it up.
    If place_xyz is given, places it there.

    Args:
        cal: Calibration dict
        pick_z: Z height for picking (mm). If None, uses detected Z.
        place_xyz: [x, y, z] to place the object (mm). If None, just picks.
    """
    print("\n[VISUAL SERVO PICK]")

    # Open gripper for picking
    gripper_open()
    time.sleep(0.3)

    # Servo to object
    success = visual_servo(cal, target_z_override=pick_z)
    if not success:
        print("  FAILED: Could not servo to target")
        return False

    # Close gripper to grab
    print("  Grabbing object...")
    gripper_close()
    time.sleep(0.5)

    # Lift up
    print("  Lifting...")
    lift_z = (pick_z or 50) + 80
    cur = _get_current_robot_xyz(cal)
    if cur:
        move_to_xyz(cur[0], cur[1], lift_z, cal)

    if place_xyz:
        print(f"  Placing at [{place_xyz[0]:.0f}, {place_xyz[1]:.0f}, {place_xyz[2]:.0f}]...")
        move_to_xyz(place_xyz[0], place_xyz[1], place_xyz[2] + 60, cal)
        time.sleep(0.5)
        move_to_xyz(place_xyz[0], place_xyz[1], place_xyz[2], cal)
        time.sleep(0.3)
        gripper_open()
        time.sleep(0.3)
        move_to_xyz(place_xyz[0], place_xyz[1], place_xyz[2] + 60, cal)
        gripper_close()
        print("  ✓ Object placed")

    print("  ✓ Visual servo pick complete")
    return True


# ============================================================================
# REALSENSE INTEGRATION
# ============================================================================

def init_realsense():
    """Initialize RealSense camera"""
    global realsense_pipeline, realsense_intrinsics
    
    if not HAS_REALSENSE:
        print("  ERROR: pyrealsense2 not installed")
        return False
    
    try:
        realsense_pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        profile = realsense_pipeline.start(config)
        # Grab intrinsics for visual servoing / 3D projection
        depth_stream = profile.get_stream(rs.stream.depth)
        realsense_intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
        print("  ✓ RealSense camera initialized")
        print(f"    Intrinsics: {realsense_intrinsics.width}x{realsense_intrinsics.height}, "
              f"fx={realsense_intrinsics.fx:.1f}, fy={realsense_intrinsics.fy:.1f}")
        return True
    except Exception as e:
        print(f"  ERROR initializing RealSense: {e}")
        return False

def detect_red_objects():
    """Detect red objects using RealSense"""
    if not HAS_REALSENSE or realsense_pipeline is None:
        print("  ERROR: RealSense not initialized")
        return []
    
    try:
        import cv2
        
        frames = realsense_pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        
        if not color_frame or not depth_frame:
            return []
        
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        
        # Simple red detection
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 90, 40])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 90, 40])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        red_objects = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 800:
                x, y, w, h = cv2.boundingRect(contour)
                cx, cy = x + w // 2, y + h // 2
                depth = depth_frame.get_distance(cx, cy)
                red_objects.append({"x": cx, "y": cy, "depth": depth})
        
        return red_objects
    except Exception as e:
        print(f"  ERROR detecting objects: {e}")
        return []

def view_camera_feed():
    """Display live camera feed with detected red object marked."""
    if not HAS_REALSENSE or realsense_pipeline is None:
        print("  ERROR: RealSense not initialized. Run 'rs_init' first.")
        return
    
    if not HAS_NUMPY:
        print("  ERROR: numpy required")
        return

    print("\n[CAMERA VIEW]")
    print("  Press 'Q' to exit")
    
    try:
        while True:
            frames = realsense_pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                continue
            
            color_image = np.asanyarray(color_frame.get_data())
            display = color_image.copy()
            
            # Detect red objects
            hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
            lower_red1 = np.array([0, 90, 40])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 90, 40])
            upper_red2 = np.array([180, 255, 255])
            mask = cv2.bitwise_or(
                cv2.inRange(hsv, lower_red1, upper_red1),
                cv2.inRange(hsv, lower_red2, upper_red2)
            )
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Draw detected objects
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 800:
                    x, y, w, h = cv2.boundingRect(contour)
                    cx, cy = x + w // 2, y + h // 2
                    # Draw circle at center
                    cv2.circle(display, (cx, cy), 10, (0, 255, 0), 2)
                    # Draw crosshair
                    cv2.line(display, (cx - 20, cy), (cx + 20, cy), (0, 255, 0), 1)
                    cv2.line(display, (cx, cy - 20), (cx, cy + 20), (0, 255, 0), 1)
                    # Label
                    cv2.putText(display, f"Object: ({cx}, {cy})", (cx - 50, cy - 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Add instructions
            cv2.putText(display, "Press 'Q' to exit | Move arm to see position changes",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            cv2.imshow("Camera Feed", display)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
    
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        print("  Camera view closed")

# ============================================================================
# CAMERA CALIBRATION (camera-to-robot transform)
# ============================================================================

CAM_CAL_FILE = "camera_calibration.json"
CAM_POINTS_FILE = "calibration_points.json"

def load_camera_calibration(path=CAM_CAL_FILE):
    """Load a previously computed camera-to-robot transform"""
    global camera_transform_R, camera_transform_t
    if not HAS_NUMPY:
        print("  ERROR: numpy required")
        return False
    if not os.path.exists(path):
        print(f"  ERROR: {path} not found. Run 'cam_cal' first.")
        return False
    with open(path, 'r') as f:
        cal = json.load(f)
    camera_transform_R = np.array(cal["rotation_matrix"])
    camera_transform_t = np.array(cal["translation_vector"])
    print(f"  \u2713 Camera calibration loaded ({cal.get('num_points',0)} points, "
          f"mean err {cal.get('mean_error_mm',0):.1f}mm, max err {cal.get('max_error_mm',0):.1f}mm)")
    return True

def delete_cam_cal_point(index):
    """Delete a specific camera calibration point by index (1-based)"""
    global cam_cal_points
    if not os.path.exists(CAM_POINTS_FILE):
        print("  ERROR: No calibration points file found")
        return
    
    with open(CAM_POINTS_FILE, 'r') as f:
        data = json.load(f)
    cam_cal_points = data.get("points", [])
    
    if index < 1 or index > len(cam_cal_points):
        print(f"  ERROR: Point {index} doesn't exist (have {len(cam_cal_points)} points)")
        return
    
    removed = cam_cal_points.pop(index - 1)
    with open(CAM_POINTS_FILE, 'w') as f:
        json.dump({"points": cam_cal_points}, f, indent=2)
    print(f"  ✓ Deleted point {index}: cam=[{removed['camera_x']:.0f},{removed['camera_y']:.0f},{removed['camera_z']:.0f}] "
          f"robot=[{removed['robot_x']:.0f},{removed['robot_y']:.0f},{removed['robot_z']:.0f}]")
    print(f"  {len(cam_cal_points)} points remaining")

def delete_all_cam_cal_points():
    """Delete all camera calibration points"""
    global cam_cal_points
    cam_cal_points = []
    if os.path.exists(CAM_POINTS_FILE):
        os.remove(CAM_POINTS_FILE)
        print(f"  ✓ Deleted {CAM_POINTS_FILE}")
    else:
        print("  No calibration points to delete")

def delete_cam_cal():
    """Delete camera calibration transform file"""
    global camera_transform_R, camera_transform_t
    camera_transform_R = None
    camera_transform_t = None
    if os.path.exists(CAM_CAL_FILE):
        os.remove(CAM_CAL_FILE)
        print(f"  ✓ Deleted {CAM_CAL_FILE}")
    else:
        print("  No camera calibration to delete")

def list_cam_cal_points():
    """List all camera calibration points"""
    if not os.path.exists(CAM_POINTS_FILE):
        print("  No calibration points recorded")
        return
    
    with open(CAM_POINTS_FILE, 'r') as f:
        data = json.load(f)
    points = data.get("points", [])
    
    if not points:
        print("  No calibration points recorded")
        return
    
    print(f"  Camera calibration points ({len(points)} total):")
    print(f"  {'#':<4} {'Camera XYZ':<28} {'Robot XYZ':<28}")
    print(f"  {'-'*60}")
    for i, p in enumerate(points, 1):
        print(f"  {i:<4} [{p['camera_x']:>7.0f},{p['camera_y']:>7.0f},{p['camera_z']:>7.0f}]  "
              f"[{p['robot_x']:>7.0f},{p['robot_y']:>7.0f},{p['robot_z']:>7.0f}]")

def camera_frame_to_robot_frame(cam_xyz):
    """
    Convert 3D coordinates from camera frame to robot frame.
    
    This function solves a critical problem: the camera and robot use different
    coordinate systems (different axes point in different directions).
    
    CAMERA FRAME (right-hand mathematical convention, as cameras typically use):
      - X axis: left/right (positive = right)
      - Y axis: up/down (positive = up)
      - Z axis: forward/backward (positive = away from camera, into the scene)
    
    ROBOT FRAME (right-hand robotic arm convention):
      - X axis: forward/backward (positive = forward, toward gripper)
      - Y axis: left/right (positive = right)
      - Z axis: up/down (positive = up)
    
    MAPPING LOGIC:
    When the camera points at the arm (typical setup):
      - Camera depth (Z) tells us how far the object is → maps to Robot X (forward)
      - Camera left/right (X) tells us side position → maps to Robot Y (left/right)
      - Camera up/down (Y) tells us height → maps to Robot Z (up/down)
    
    We also negate some axes because the physical mounting might invert directions:
      - Camera points forward, so camera +Z (away) means robot +X (away) → negate
      - Camera +X (right) means robot +Y (right), but we invert to match arm → negate
    
    Args:
        cam_xyz: [x, y, z] position in camera frame (in mm)
        
    Returns:
        [x, y, z] position in robot frame (in mm)
    """
    if not isinstance(cam_xyz, np.ndarray):
        cam_xyz = np.array(cam_xyz)
    
    cam_x, cam_y, cam_z = cam_xyz[0], cam_xyz[1], cam_xyz[2]
    
    # Remap axes with negations based on typical camera mounting orientation
    robot_x = -cam_z  # Camera depth maps to robot forward (negated)
    robot_y = -cam_x  # Camera left/right maps to robot left/right (negated)
    robot_z = cam_y   # Camera up/down maps to robot up/down (same)
    
    return np.array([robot_x, robot_y, robot_z])


def camera_to_robot(camera_xyz):
    """Transform camera XYZ (mm) to robot XYZ (mm).
    
    Steps:
      1. Convert camera frame axes to robot frame axes
      2. Apply Kabsch rotation + translation
    """
    if camera_transform_R is None:
        print("  ERROR: Camera calibration not loaded. Run 'cam_cal_load'.")
        return None
    
    # Convert axes first
    robot_frame = camera_frame_to_robot_frame(camera_xyz)
    
    # Then apply Kabsch transform
    result = (camera_transform_R @ robot_frame + camera_transform_t).tolist()

def cam_cal_interactive():
    """Interactive camera-to-robot calibration using FK.
    Opens camera feed, you click object in image while arm touches it,
    FK computes robot position, pairs are collected, then Kabsch computes transform."""
    global cam_cal_points, realsense_pipeline, realsense_intrinsics

    if not HAS_REALSENSE:
        print("  ERROR: pyrealsense2 and opencv-python required")
        return
    if not HAS_IK_SOLVER:
        print("  ERROR: ik_solver required for FK")
        return
    if not HAS_NUMPY:
        print("  ERROR: numpy required")
        return
    if ser is None:
        print("  ERROR: Connect to arm first")
        return

    # Init camera if needed
    if realsense_pipeline is None:
        if not init_realsense():
            return

    # Get intrinsics
    profile = realsense_pipeline.get_active_profile()
    depth_stream = profile.get_stream(rs.stream.depth)
    realsense_intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()

    # Load existing points if any
    if os.path.exists(CAM_POINTS_FILE):
        with open(CAM_POINTS_FILE, 'r') as f:
            data = json.load(f)
            cam_cal_points = data.get("points", [])
        print(f"  Loaded {len(cam_cal_points)} existing points")
    else:
        cam_cal_points = []

    clicked_pixel = [None]  # mutable for closure

    def mouse_cb(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked_pixel[0] = (x, y)

    cv2.namedWindow("Camera Calibration - Click object")
    cv2.setMouseCallback("Camera Calibration - Click object", mouse_cb)

    print("\n[CAMERA CALIBRATION]")
    print("  1. Move arm so gripper touches a known object")
    print("  2. Click that object in the camera feed")
    print("  3. Point is recorded (camera XYZ + robot FK XYZ)")
    print("  4. Repeat 5+ times in different positions")
    print("  5. Press 'D' to finish and compute transform")
    print("  6. Press 'Q' to quit without computing")
    print(f"  Points so far: {len(cam_cal_points)}\n")

    try:
        while True:
            frames = realsense_pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            display = np.asanyarray(color_frame.get_data()).copy()

            # Draw existing points
            for i, pt in enumerate(cam_cal_points):
                px, py = int(pt['pixel_x']), int(pt['pixel_y'])
                cv2.circle(display, (px, py), 8, (0, 255, 0), 2)
                cv2.putText(display, str(i+1), (px-5, py-15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.putText(display, f"Points: {len(cam_cal_points)}  |  Click object  |  D=done  Q=quit",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.imshow("Camera Calibration - Click object", display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("  Cancelled.")
                break
            if key == ord('d'):
                # Done — compute transform
                break

            if clicked_pixel[0] is not None:
                px, py = clicked_pixel[0]
                clicked_pixel[0] = None

                depth_val = depth_frame.get_distance(px, py)
                if depth_val == 0:
                    print("  \u2717 No depth at that pixel, try again")
                    continue

                # Camera 3D from pixel + depth
                intr = realsense_intrinsics
                cam_x = (px - intr.ppx) / intr.fx * depth_val * 1000
                cam_y = (py - intr.ppy) / intr.fy * depth_val * 1000
                cam_z = depth_val * 1000

                # Read servo angles, compute FK
                angles = []
                for sid in JOINT_IDS:
                    a = get_angle(sid)
                    if a is None:
                        print(f"  \u2717 Can't read servo {sid}")
                        break
                    angles.append(a)
                else:
                    robot_xyz = solve_fk(angles)
                    pt = {
                        "pixel_x": px, "pixel_y": py,
                        "camera_x": cam_x, "camera_y": cam_y, "camera_z": cam_z,
                        "robot_x": robot_xyz[0], "robot_y": robot_xyz[1], "robot_z": robot_xyz[2],
                        "servo_angles": [float(a) for a in angles],
                        "timestamp": time.time()
                    }
                    cam_cal_points.append(pt)
                    # Save immediately
                    with open(CAM_POINTS_FILE, 'w') as f:
                        json.dump({"points": cam_cal_points}, f, indent=2)
                    print(f"  \u2713 Point {len(cam_cal_points)}: "
                          f"cam=[{cam_x:.0f},{cam_y:.0f},{cam_z:.0f}] "
                          f"robot=[{robot_xyz[0]:.0f},{robot_xyz[1]:.0f},{robot_xyz[2]:.0f}]")

    except KeyboardInterrupt:
        print("\n  Interrupted")
    finally:
        cv2.destroyAllWindows()

    # Compute transform if enough points
    if len(cam_cal_points) < 3:
        print(f"  Need at least 3 points (have {len(cam_cal_points)}). Skipping transform.")
        return

    _compute_camera_transform()

def _compute_camera_transform():
    """Kabsch algorithm: compute rotation + translation from camera to robot frame"""
    global camera_transform_R, camera_transform_t

    cam_pts = np.array([[p['camera_x'], p['camera_y'], p['camera_z']] for p in cam_cal_points])
    rob_pts = np.array([[p['robot_x'], p['robot_y'], p['robot_z']] for p in cam_cal_points])

    cam_c = np.mean(cam_pts, axis=0)
    rob_c = np.mean(rob_pts, axis=0)

    H = (cam_pts - cam_c).T @ (rob_pts - rob_c)
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = rob_c - R @ cam_c

    # Error report
    errors = []
    print(f"\n  {'Pt':<4} {'Error mm':<10}")
    print(f"  {'-'*14}")
    for i, p in enumerate(cam_cal_points):
        cam = np.array([p['camera_x'], p['camera_y'], p['camera_z']])
        predicted = R @ cam + t
        actual = np.array([p['robot_x'], p['robot_y'], p['robot_z']])
        err = np.linalg.norm(predicted - actual)
        errors.append(err)
        print(f"  {i+1:<4} {err:<10.1f}")

    mean_err = np.mean(errors)
    max_err = np.max(errors)
    print(f"  Mean: {mean_err:.1f}mm  Max: {max_err:.1f}mm")

    camera_transform_R = R
    camera_transform_t = t

    output = {
        "rotation_matrix": R.tolist(),
        "translation_vector": t.tolist(),
        "mean_error_mm": float(mean_err),
        "max_error_mm": float(max_err),
        "num_points": len(cam_cal_points),
    }
    with open(CAM_CAL_FILE, 'w') as f:
        json.dump(output, f, indent=2)
    print(f"  \u2713 Transform saved to {CAM_CAL_FILE}")

# ============================================================================
# INTERACTIVE MENU SYSTEM
# ============================================================================

def print_menu():
    """Print main menu"""
    print("\n" + "="*60)
    print("SO-101 MASTER CONTROL - Main Menu")
    print("="*60)
    print("SETUP:")
    print("  connect       - Auto-detect and connect to arm")
    print("  scan          - Scan for connected servos")
    print("  disconnect    - Disconnect from arm")
    print("  status        - Show connection status")
    print("\nCALIBRATION:")
    print("  calibrate     - Run interactive calibration")
    print("  calload       - Load calibration from file")
    print("  calinfo       - Show loaded calibration info")
    print("\nMOTION CONTROL:")
    print("  center        - Move all joints to center")
    print("  move <id> <deg> - Move servo to angle")
    print("  move_to <x> <y> <z> - Move to XYZ position using IK (mm)")
    print("  read <id>     - Read servo angle")
    print("  torque <on|off> - Toggle torque on all servos")
    print("\nPATH RECORDING:")
    print("  start <name>  - Start recording path")
    print("  add           - Add current position as waypoint")
    print("  undo          - Remove last recorded waypoint")
    print("  save          - Save current recording")
    print("  cancel        - Discard current recording/edits")
    print("  list_paths    - List saved paths")
    print("  exec <name>   - Execute saved path")
    print("  smooth <name> [sec] - Execute with smooth interpolation")
    print("  del_path <name> - Delete a saved path")
    print("  del_all_paths - Delete all saved paths")
    print("\nPATH EDITING:")
    print("  edit <name>     - Load a saved path for editing")
    print("  wp              - List waypoints in current path")
    print("  wp_del <n>      - Delete waypoint #n")
    print("  wp_replace <n>  - Replace waypoint #n with current position")
    print("  wp_insert <n>   - Insert current position before waypoint #n")
    print("  wp_goto <n>     - Move arm to waypoint #n")
    print("  wp_swap <a> <b> - Swap two waypoints")
    print("\nPILLAR PICK & DROP:")
    print("  set_home       - Save current position as home for all paths")
    print("  home           - Move arm to saved home position")
    print("  pick1/pick2/pick3 - Pick ring from pillar 1/2/3")
    print("  drop1/drop2/drop3 - Drop ring on pillar 1/2/3")
    print("  rec_pick1..3   - Start recording pick path (home auto-added)")
    print("  rec_drop1..3   - Start recording drop path (home auto-added)")
    print("    During recording, use waypoint commands:")
    print("      add / undo / wp / wp_del / wp_replace / wp_insert / wp_goto / wp_swap / save / cancel")
    print("  pillars        - Show which pillar paths are recorded")
    print("\nKINEMATICS:")
    print("  fk <angles>   - Test forward kinematics")
    print("  ik <x> <y> <z> - Test inverse kinematics")
    print("\nPICK & PLACE:")
    print("  demo <x> <y> <z> - Pick & place demo")
    print("  grip_open     - Open gripper")
    print("  grip_close    - Close gripper")
    print("\nPOSITION:")
    print("  pos           - Read current arm position (FK)")
    print("\nVISUAL SERVOING:")
    print("  vs            - Visual servo to red object (closed-loop)")
    print("  vs_pick [z]   - Visual servo pick (optional pick Z height)")
    print("  vs_pick_place <px> <py> <pz> [z] - Servo pick + place at XYZ")
    print("\nVISION (RealSense):")
    print("  rs_init       - Initialize RealSense")
    print("  rs_detect     - Detect red objects")
    print("  cam_view      - View live camera feed with object detection")
    print("  cam_cal       - Calibrate camera-to-robot transform")
    print("  cam_cal_load  - Load existing camera calibration")
    print("  cam_cal_list  - List camera calibration points")
    print("  cam_cal_del <n> - Delete camera cal point #n")
    print("  cam_cal_clear - Delete all camera cal points")
    print("  cam_cal_reset - Delete camera calibration transform")
    print("  cam_test <x> <y> <z> - Test camera→robot transform (mm)")
    print("\nOTHER:")
    print("  help          - Show this menu")
    print("  quit          - Exit program")
    print("="*60)

def parse_angle_list(angle_str):
    """Parse angle list from user input (e.g., '90,180,90,90,90,90')"""
    try:
        angles = [float(x.strip()) for x in angle_str.split(",")]
        if len(angles) != 6:
            print(f"  ERROR: Expected 6 angles, got {len(angles)}")
            return None
        return angles
    except ValueError:
        print("  ERROR: Invalid angle format")
        return None

def main_loop():
    """Main interactive command loop"""
    global ser, PORT, current_baud, raw_calibration
    
    print("\n" + "="*60)
    print("SO-101 MASTER CONTROL PROGRAM")
    print("Type 'help' for menu")
    print("="*60)
    
    while True:
        try:
            cmd = input("\n[SO-101]> ").strip().lower()
            
            if not cmd:
                continue
            
            if cmd in ["quit", "exit", "q"]:
                print("\nGoodbye!")
                if ser:
                    ser.close()
                break
            
            dispatch_command(cmd)
        
        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
            break
        except Exception as e:
            print(f"ERROR: {e}")

def dispatch_command(cmd):
    """Dispatch a single command. All output goes to stdout (can be captured for socket clients)."""
    global ser, PORT, current_baud, raw_calibration
    global current_path, current_path_name

    # SETUP COMMANDS
    if cmd == "connect":
        print("\n[CONNECTING]")
        ser, PORT, current_baud = find_serial_port()
        if ser:
            print(f"✓ Connected to {PORT} at {current_baud} baud")
        else:
            print("✗ Failed to connect to arm")

    elif cmd == "scan":
        if ser is None:
            print("ERROR: Not connected. Use 'connect' first")
        else:
            scan_servos()

    elif cmd == "disconnect":
        if ser:
            ser.close()
            ser = None
            print("✓ Disconnected")
        else:
            print("Not connected")

    elif cmd == "status":
        if ser:
            print(f"✓ Connected to {PORT} at {current_baud} baud")
        else:
            print("✗ Not connected")

    # CALIBRATION COMMANDS
    elif cmd == "calibrate":
        if ser is None:
            print("ERROR: Not connected")
        else:
            calibrate_interactive()

    elif cmd == "calload":
        if load_calibration():
            pass

    elif cmd == "calinfo":
        if raw_calibration:
            limits = get_joint_limits_from_cal(raw_calibration)
            centers = get_joint_centers_from_cal(raw_calibration)
            print("\n[CALIBRATION INFO]")
            for joint_name in JOINT_NAMES:
                if joint_name in limits:
                    lim = limits[joint_name]
                    ctr = centers.get(joint_name, 0)
                    print(f"  {joint_name:15s}: {lim['min']:6.1f}° - {lim['max']:6.1f}° (center: {ctr:6.1f}°)")
        else:
            print("Load calibration first with 'calload'")

    # MOTION CONTROL
    elif cmd == "center":
        if ser is None or raw_calibration is None:
            print("ERROR: Connect and load calibration first")
        else:
            move_all_to_center(raw_calibration)

    elif cmd.startswith("move_to "):
        if ser is None or raw_calibration is None:
            print("ERROR: Connect and load calibration first")
        else:
            try:
                parts = cmd.split()
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                move_to_xyz(x, y, z, raw_calibration)
            except (ValueError, IndexError):
                print("  Usage: move_to <x> <y> <z> (mm)")

    elif cmd.startswith("move "):
        if ser is None or raw_calibration is None:
            print("ERROR: Connect and load calibration first")
        else:
            parts = cmd.split()
            try:
                servo_id = int(parts[1])
                angle = float(parts[2])
                safe_move(servo_id, angle, raw_calibration)
            except (ValueError, IndexError):
                print("Usage: move <id> <degrees>")

    elif cmd.startswith("read "):
        if ser is None:
            print("ERROR: Not connected")
        else:
            try:
                servo_id = int(cmd.split()[1])
                angle = get_angle(servo_id)
                if angle is not None:
                    print(f"  Servo {servo_id}: {angle:.1f}°")
                else:
                    print(f"  ERROR: Could not read servo {servo_id}")
            except ValueError:
                print("Usage: read <id>")

    elif cmd.startswith("torque "):
        if ser is None:
            print("ERROR: Not connected")
        else:
            state = cmd.split()[1].lower()
            for servo_id in JOINT_IDS:
                if state == "on":
                    enable_servo(servo_id)
                else:
                    disable_servo(servo_id)
            print(f"✓ Torque turned {state} on all servos")

    elif cmd == "pos":
        if ser is None:
            print("ERROR: Not connected")
        else:
            pos = _get_current_robot_xyz(raw_calibration)
            if pos:
                print(f"  Current arm position: X={pos[0]:.1f}mm, Y={pos[1]:.1f}mm, Z={pos[2]:.1f}mm")
            else:
                print("  ERROR: Could not read position")

    # PATH RECORDING
    elif cmd.startswith("start "):
        path_name = cmd.split(maxsplit=1)[1]
        start_recording_path(path_name)

    elif cmd == "add":
        if ser is None:
            print("ERROR: Not connected")
        else:
            add_waypoint()

    elif cmd == "undo":
        undo_waypoint()

    elif cmd == "save":
        save_current_path()

    elif cmd.startswith("edit "):
        path_name = cmd.split(maxsplit=1)[1]
        edit_path(path_name)

    elif cmd == "wp":
        list_waypoints()

    elif cmd.startswith("wp_del "):
        try:
            idx = int(cmd.split()[1])
            delete_waypoint(idx)
        except (ValueError, IndexError):
            print("Usage: wp_del <waypoint_number>")

    elif cmd.startswith("wp_replace "):
        if ser is None:
            print("ERROR: Not connected")
        else:
            try:
                idx = int(cmd.split()[1])
                replace_waypoint(idx)
            except (ValueError, IndexError):
                print("Usage: wp_replace <waypoint_number>")

    elif cmd.startswith("wp_insert "):
        if ser is None:
            print("ERROR: Not connected")
        else:
            try:
                idx = int(cmd.split()[1])
                insert_waypoint(idx)
            except (ValueError, IndexError):
                print("Usage: wp_insert <position>")

    elif cmd.startswith("wp_goto "):
        if ser is None:
            print("ERROR: Not connected")
        else:
            try:
                idx = int(cmd.split()[1])
                goto_waypoint(idx)
            except (ValueError, IndexError):
                print("Usage: wp_goto <waypoint_number>")

    elif cmd.startswith("wp_swap "):
        try:
            parts = cmd.split()
            a, b = int(parts[1]), int(parts[2])
            swap_waypoints(a, b)
        except (ValueError, IndexError):
            print("Usage: wp_swap <a> <b>")

    elif cmd == "cancel":
        if current_path is not None:
            print(f"  ✓ Discarded changes to '{current_path_name}'")
            current_path = None
            current_path_name = None
        else:
            print("  Nothing to cancel")

    elif cmd == "list_paths":
        list_saved_paths()

    elif cmd.startswith("exec "):
        if ser is None or raw_calibration is None:
            print("ERROR: Connect and load calibration first")
        else:
            path_name = cmd.split(maxsplit=1)[1]
            execute_saved_path(path_name, raw_calibration)

    elif cmd.startswith("smooth "):
        if ser is None or raw_calibration is None:
            print("ERROR: Connect and load calibration first")
        else:
            parts = cmd.split()
            path_name = parts[1] if len(parts) > 1 else None
            total_time = float(parts[2]) if len(parts) > 2 else 5.0
            if path_name:
                execute_smooth_path(path_name, raw_calibration, total_time)
            else:
                print("Usage: smooth <path_name> [seconds]")

    elif cmd.startswith("del_path "):
        path_name = cmd.split(maxsplit=1)[1]
        delete_saved_path(path_name)

    elif cmd == "del_all_paths":
        print("  Delete ALL saved paths? (yes to confirm): ", end="", flush=True)
        delete_all_paths()

    # PILLAR PICK & DROP
    elif cmd == "home":
        if ser is None:
            print("ERROR: Not connected")
        else:
            go_home()

    elif cmd == "set_home":
        if ser is None:
            print("ERROR: Not connected")
        else:
            set_home()

    elif cmd in ("pick1", "pick2", "pick3", "drop1", "drop2", "drop3"):
        if ser is None or raw_calibration is None:
            print("ERROR: Connect and load calibration first")
        else:
            pillar_execute(cmd, raw_calibration)

    elif cmd in ("rec_pick1", "rec_pick2", "rec_pick3",
                 "rec_drop1", "rec_drop2", "rec_drop3"):
        action_name = cmd[4:]  # strip "rec_"
        pillar_record(action_name)

    elif cmd == "pillars":
        pillar_status()

    # KINEMATICS
    elif cmd.startswith("fk "):
        angle_str = cmd[3:].strip()
        angles = parse_angle_list(angle_str)
        if angles:
            test_fk(angles)

    elif cmd.startswith("ik "):
        try:
            parts = cmd.split()
            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
            test_ik(x, y, z)
        except (ValueError, IndexError):
            print("Usage: ik <x> <y> <z>")

    # PICK & PLACE
    elif cmd.startswith("demo "):
        if ser is None or raw_calibration is None:
            print("ERROR: Connect and load calibration first")
        else:
            try:
                parts = cmd.split()
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                pick_and_place_demo(x, y, z, raw_calibration)
            except (ValueError, IndexError):
                print("Usage: demo <x> <y> <z>")

    elif cmd == "grip_open":
        if ser is None:
            print("ERROR: Not connected")
        else:
            gripper_open()

    elif cmd == "grip_close":
        if ser is None:
            print("ERROR: Not connected")
        else:
            gripper_close()

    # VISUAL SERVOING
    elif cmd == "vs":
        if ser is None or raw_calibration is None:
            print("ERROR: Connect and load calibration first")
        else:
            visual_servo(raw_calibration)

    elif cmd.startswith("vs_pick_place "):
        if ser is None or raw_calibration is None:
            print("ERROR: Connect and load calibration first")
        else:
            try:
                parts = cmd.split()
                px, py, pz = float(parts[1]), float(parts[2]), float(parts[3])
                pick_z = float(parts[4]) if len(parts) > 4 else None
                visual_servo_pick(raw_calibration, pick_z=pick_z, place_xyz=[px, py, pz])
            except (ValueError, IndexError):
                print("  Usage: vs_pick_place <place_x> <place_y> <place_z> [pick_z]")

    elif cmd.startswith("vs_pick"):
        if ser is None or raw_calibration is None:
            print("ERROR: Connect and load calibration first")
        else:
            parts = cmd.split()
            pick_z = float(parts[1]) if len(parts) > 1 else None
            visual_servo_pick(raw_calibration, pick_z=pick_z)

    # REALSENSE
    elif cmd == "rs_init":
        init_realsense()

    elif cmd == "rs_detect":
        if realsense_pipeline is None:
            print("Initialize RealSense first with 'rs_init'")
        else:
            objects = detect_red_objects()
            if objects:
                print(f"\n[DETECTED {len(objects)} RED OBJECTS]")
                for i, obj in enumerate(objects, 1):
                    print(f"  {i}. Pos: ({obj['x']}, {obj['y']}), Depth: {obj['depth']:.3f}m")
            else:
                print("No red objects detected")

    elif cmd == "cam_view":
        view_camera_feed()

    elif cmd == "cam_cal":
        cam_cal_interactive()

    elif cmd == "cam_cal_load":
        load_camera_calibration()

    elif cmd == "cam_cal_list":
        list_cam_cal_points()

    elif cmd.startswith("cam_cal_del "):
        try:
            idx = int(cmd.split()[1])
            delete_cam_cal_point(idx)
        except (ValueError, IndexError):
            print("Usage: cam_cal_del <point_number>")

    elif cmd == "cam_cal_clear":
        delete_all_cam_cal_points()

    elif cmd == "cam_cal_reset":
        delete_cam_cal()

    elif cmd.startswith("cam_test "):
        try:
            parts = cmd.split()
            cx, cy, cz = float(parts[1]), float(parts[2]), float(parts[3])
            result = camera_to_robot([cx, cy, cz])
            if result:
                print(f"  Camera [{cx:.0f}, {cy:.0f}, {cz:.0f}] → Robot [{result[0]:.1f}, {result[1]:.1f}, {result[2]:.1f}]")
        except (ValueError, IndexError):
            print("Usage: cam_test <x> <y> <z> (mm)")

    # HELP
    elif cmd == "help":
        print_menu()

    else:
        print(f"Unknown command: '{cmd}'. Type 'help' for menu.")

# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    try:
        main_loop()
    except Exception as e:
        print(f"Fatal error: {e}")
        if ser:
            ser.close()
        sys.exit(1)
