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

BAUD = 1000000
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
JOINT_IDS = [1, 2, 3, 4, 5, 6]
SERVO_CENTER_ANGLE = 180.0  # Default center position

GRIPPER_OPEN_ANGLE = 244
GRIPPER_CLOSE_ANGLE = 150
APPROACH_HEIGHT = 150

# ============================================================================
# GLOBAL STATE
# ============================================================================

ser = None
PORT = None
current_baud = None
raw_calibration = None
current_path = None
current_path_name = None
realsense_pipeline = None
realsense_intrinsics = None
camera_transform_R = None
camera_transform_t = None
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
                test_ser = serial.Serial(port.device, baud, timeout=0.5)
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
    time.sleep(0.005)
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
    time.sleep(0.02)
    
    resp = b''
    timeout_count = 0
    while len(resp) < 20 and timeout_count < 5:
        chunk = ser.read(1)
        if chunk:
            resp += chunk
        else:
            timeout_count += 1
        time.sleep(0.0005)
    
    if len(resp) > 5 + length - 1:
        if length == 1:
            return resp[5]
        else:
            return resp[5] | (resp[6] << 8)
    return None

def set_angle(servo_id, angle_degrees):
    """Set servo angle (0-360 degrees)"""
    position = int((angle_degrees / 360.0) * 4095)
    write_servo(servo_id, 0x2A, position, 2)

def get_angle(servo_id):
    """Get current servo angle in degrees"""
    pos = read_servo(servo_id, 0x38, 2)
    if pos is not None:
        return (pos / 4095.0) * 360.0
    return None

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
    """Move servo within calibrated limits using logical angles (0-180).
    
    Args:
        servo_id: Servo ID (1-6)
        target_deg: Desired angle in 0-180 logical range
        cal: Calibration dict
    """
    gripper_close()
    # Validate logical range
    if not (0 <= target_deg <= 180):
        print(f"  ERROR: {target_deg}° out of logical range [0°, 180°]")
        return
    
    # Convert to physical angle
    physical_angle = logical_to_physical_angle(servo_id, target_deg, cal)
    if physical_angle is None:
        print(f"  ERROR: Cannot convert logical angle {target_deg}°")
        return
    
    enable_servo(servo_id)
    time.sleep(0.05)
    set_angle(servo_id, physical_angle)
    time.sleep(0.5)
    
    current = get_angle(servo_id)
    if current is not None:
        print(f"  ✓ Servo {servo_id} moved to logical {target_deg:.1f}° (physical: {current:.1f}°)")
    else:
        print(f"  ✓ Servo {servo_id} commanded to logical {target_deg:.1f}°")

def move_to_xyz(x, y, z, cal):
    """Move arm to XYZ position using IK solver.s
    
    Args:
        x: Target X position in mm
        y: Target Y position in mm
        z: Target Z position in mm
        cal: Calibration dict
    """
    gripper_close()
    if not HAS_IK_SOLVER:
        print("  ERROR: IK solver not available")
        return False
    
    print(f"\n[MOVE TO XYZ]")
    print(f"  Target: X={x:.1f}mm, Y={y:.1f}mm, Z={z:.1f}mm")
    print("  Solving IK...")
    
    # Solve IK
    solution = solve_ik([x, y, z])
    if not solution:
        print("  ERROR: IK solution not found (target unreachable)")
        return False
    
    print(f"  ✓ IK Solution found: {[f'{a:.1f}°' for a in solution]}")
    
    # Move all joints to solution angles
    print("  Moving arm...")
    for servo_id in JOINT_IDS:
        angle = solution[servo_id - 1]
        enable_servo(servo_id)
        # Convert logical angle to physical and move
        physical_angle = logical_to_physical_angle(servo_id, angle, cal)
        if physical_angle is not None:
            set_angle(servo_id, physical_angle)
    
    time.sleep(1.5)  # Wait for arm to reach position
    
    # Verify with FK
    try:
        verify_pos = solve_fk(solution)
        error = ((verify_pos[0] - x)**2 + (verify_pos[1] - y)**2 + (verify_pos[2] - z)**2)**0.5
        print(f"  ✓ Position reached")
        print(f"    Actual: X={verify_pos[0]:.1f}mm, Y={verify_pos[1]:.1f}mm, Z={verify_pos[2]:.1f}mm")
        print(f"    Error: {error:.2f}mm")
        return True
    except Exception as e:
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
    
    waypoint = []
    for servo_id in JOINT_IDS:
        angle = get_angle(servo_id)
        if angle is None:
            print(f"  ERROR: Could not read servo {servo_id}")
            return
        waypoint.append(angle)
    
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
# REALSENSE INTEGRATION
# ============================================================================

def init_realsense():
    """Initialize RealSense camera"""
    global realsense_pipeline
    
    if not HAS_REALSENSE:
        print("  ERROR: pyrealsense2 not installed")
        return False
    
    try:
        realsense_pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        realsense_pipeline.start(config)
        print("  ✓ RealSense camera initialized")
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

def camera_to_robot(camera_xyz):
    """Transform camera XYZ (mm) to robot XYZ (mm)"""
    if camera_transform_R is None:
        print("  ERROR: Camera calibration not loaded. Run 'cam_cal_load'.")
        return None
    cam = np.array(camera_xyz)
    return (camera_transform_R @ cam + camera_transform_t).tolist()

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
    print("  list_paths    - List saved paths")
    print("  exec <name>   - Execute saved path")
    print("  smooth <name> [sec] - Execute with smooth interpolation")
    print("  del_path <name> - Delete a saved path")
    print("  del_all_paths - Delete all saved paths")
    print("\nKINEMATICS:")
    print("  fk <angles>   - Test forward kinematics")
    print("  ik <x> <y> <z> - Test inverse kinematics")
    print("\nPICK & PLACE:")
    print("  demo <x> <y> <z> - Pick & place demo")
    print("  grip_open     - Open gripper")
    print("  grip_close    - Close gripper")
    print("\nVISION (RealSense):")
    print("  rs_init       - Initialize RealSense")
    print("  rs_detect     - Detect red objects")
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
                confirm = input("  Delete ALL saved paths? (yes/no): ").strip().lower()
                if confirm == "yes":
                    delete_all_paths()
                else:
                    print("  Cancelled")
            
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
                confirm = input("  Delete ALL camera calibration points? (yes/no): ").strip().lower()
                if confirm == "yes":
                    delete_all_cam_cal_points()
                else:
                    print("  Cancelled")
            
            elif cmd == "cam_cal_reset":
                confirm = input("  Delete camera calibration transform? (yes/no): ").strip().lower()
                if confirm == "yes":
                    delete_cam_cal()
                else:
                    print("  Cancelled")
            
            elif cmd.startswith("cam_test "):
                try:
                    parts = cmd.split()
                    cx, cy, cz = float(parts[1]), float(parts[2]), float(parts[3])
                    result = camera_to_robot([cx, cy, cz])
                    if result:
                        print(f"  Camera [{cx:.0f}, {cy:.0f}, {cz:.0f}] → Robot [{result[0]:.1f}, {result[1]:.1f}, {result[2]:.1f}]")
                except (ValueError, IndexError):
                    print("Usage: cam_test <x> <y> <z> (mm)")
            
            # HELP & EXIT
            elif cmd == "help":
                print_menu()
            
            elif cmd in ["quit", "exit", "q"]:
                print("\nGoodbye!")
                if ser:
                    ser.close()
                break
            
            else:
                print(f"Unknown command: '{cmd}'. Type 'help' for menu.")
        
        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
            break
        except Exception as e:
            print(f"ERROR: {e}")

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
