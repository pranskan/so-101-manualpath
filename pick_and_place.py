import pyrealsense2 as rs
import numpy as np
import cv2
import serial
import serial.tools.list_ports
import json
import time
import math
from ik_solver import solve_ik

# ===== CONFIGURATION =====
BAUD = 1000000
GRIPPER_OPEN_ANGLE = 244  # degrees
GRIPPER_CLOSE_ANGLE = 150  # degrees
APPROACH_HEIGHT = 150  # mm above object before descending

# ===== SERIAL PORT SETUP =====
def find_serial_port():
    """Auto-detect available serial ports and connect to the robot."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        raise RuntimeError("No serial port found! Connect your robot.")
    
    print(f"Available ports: {[p.device for p in ports]}")
    
    baud_rates = [1000000, 921600, 460800, 230400, 115200]
    ports_to_try = sorted(ports, key=lambda p: p.device, reverse=True)
    
    for port in ports_to_try:
        for baud in baud_rates:
            try:
                print(f"Trying {port.device} at {baud} baud...", end=" ")
                test_ser = serial.Serial(port.device, baud, timeout=0.5)
                print("✓ Connected!")
                return test_ser, port.device, baud
            except Exception as e:
                print("✗")
                continue
    
    raise RuntimeError("Could not connect to any serial port!")

ser, PORT, BAUD = find_serial_port()
print(f"Connected to {PORT} at {BAUD} baud\n")
time.sleep(0.2)

# ===== SERVO CONTROL =====
def write_servo(servo_id, address, value, length=1):
    """Write to servo register"""
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
    """Set servo to angle in degrees"""
    position = int((angle_degrees / 360.0) * 4095)
    write_servo(servo_id, 0x2A, position, 2)

def get_angle(servo_id):
    """Read current servo angle in degrees"""
    pos = read_servo(servo_id, 0x38, 2)
    if pos is not None:
        angle = (pos / 4095.0) * 360.0
        return angle
    return None

def enable_servo(servo_id):
    """Enable torque on servo"""
    write_servo(servo_id, 0x28, 1)

def disable_servo(servo_id):
    """Disable torque on servo"""
    write_servo(servo_id, 0x28, 0)

def move_to_angles(angles, duration=1.0):
    """Move all servos to target angles"""
    steps = int(duration * 50)  # 50 steps per second
    
    # Get current angles
    current = []
    for i in range(1, 7):
        ang = get_angle(i)
        current.append(ang if ang is not None else 0)
    
    # Linear interpolation
    for step in range(steps):
        t = step / max(steps, 1)
        for i in range(6):
            target = angles[i]
            current_val = current[i]
            interp = current_val + (target - current_val) * t
            set_angle(i + 1, interp)
        time.sleep(0.02)

def open_gripper():
    """Open gripper"""
    set_angle(6, GRIPPER_OPEN_ANGLE)
    time.sleep(0.5)

def close_gripper():
    """Close gripper"""
    set_angle(6, GRIPPER_CLOSE_ANGLE)
    time.sleep(0.5)

# ===== CAMERA CALIBRATION =====
from camera_transform import transform as camera_transform

if camera_transform is None:
    print("Error: camera_calibration.json not found!")
    print("Run record_calibration_auto.py first.")
    exit(1)

def camera_to_robot(camera_xyz):
    """Transform camera coordinates to robot coordinates using calibration"""
    return camera_transform.transform(camera_xyz)

# ===== VISION SETUP =====
def detect_red_objects(frame, depth_frame=None, depth_intrinsics=None):
    """Detect red objects and calculate 3D position from camera."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Enhance contrast
    h, s, v = cv2.split(hsv)
    v = cv2.convertScaleAbs(v, alpha=1.2, beta=10)
    v = np.clip(v, 0, 255).astype(np.uint8)
    hsv = cv2.merge([h, s, v])
    
    # Red color ranges
    lower_red1 = np.array([0, 90, 40])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 90, 40])
    upper_red2 = np.array([180, 255, 255])
    
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    
    # Filter skin tones
    skin_lower = np.array([0, 5, 30])
    skin_upper = np.array([25, 50, 255])
    skin_mask = cv2.inRange(hsv, skin_lower, skin_upper)
    
    skin_lower2 = np.array([165, 5, 30])
    skin_upper2 = np.array([180, 50, 255])
    skin_mask2 = cv2.inRange(hsv, skin_lower2, skin_upper2)
    
    skin_combined = cv2.bitwise_or(skin_mask, skin_mask2)
    mask = cv2.subtract(mask, skin_combined)
    
    # Morphological operations
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    red_objects = []
    output = frame.copy()
    
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 800:
            x, y, w, h = cv2.boundingRect(contour)
            cx = x + w // 2
            cy = y + h // 2
            
            x_robot, y_robot, z_robot = None, None, None
            if depth_frame is not None and depth_intrinsics is not None:
                try:
                    depth_value = depth_frame.get_distance(cx, cy)
                    if 0.1 < depth_value < 1.0:  # Valid range: 100mm to 1000mm (1 meter max)
                        # Use exact same formula as record_calibration_auto.py
                        x_cam = (cx - depth_intrinsics.ppx) / depth_intrinsics.fx * depth_value
                        y_cam = (cy - depth_intrinsics.ppy) / depth_intrinsics.fy * depth_value
                        z_cam = depth_value
                        # Convert to mm
                        x_robot = x_cam * 1000
                        y_robot = y_cam * 1000
                        z_robot = z_cam * 1000
                except:
                    pass
            
            # Only add if we have valid depth
            if x_robot is not None:
                red_objects.append({
                    'x': cx,
                    'y': cy,
                    'width': w,
                    'height': h,
                    'x_cam': x_robot,
                    'y_cam': y_robot,
                    'z_cam': z_robot
                })
                
                cv2.rectangle(output, (x, y), (x + w, y + h), (255, 255, 0), 2)
                cv2.circle(output, (cx, cy), 5, (0, 255, 0), -1)
    
    return output, red_objects

# ===== MAIN PICK AND PLACE =====
print("Initializing RealSense camera...")
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

try:
    pipeline.start(config)
    print("Camera started successfully\n")
except Exception as e:
    print(f"Failed to start camera: {e}")
    exit(1)

# Get camera intrinsics
depth_intrinsics = None
try:
    profile = pipeline.get_active_profile()
    depth_stream = profile.get_stream(rs.stream.depth)
    depth_intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
except:
    print("Warning: Could not get camera intrinsics")

print("Camera calibration loaded\n")

# Enable servos
print("Enabling servos...")
for i in range(1, 7):
    enable_servo(i)
time.sleep(0.5)

# Open gripper
print("Opening gripper...")
open_gripper()

print("\n" + "="*60)
print("PICK AND PLACE READY")
print("="*60)
print("Commands:")
print("  detect  - Detect red objects in camera feed")
print("  pick #  - Pick object # (e.g., 'pick 1')")
print("  demo    - Automatic demo pick")
print("  quit    - Exit")
print("="*60 + "\n")

latest_red_objects = None

try:
    while True:
        cmd = input("Enter command: ").strip().lower()
        
        if cmd == "quit":
            print("Exiting...")
            break
        
        elif cmd == "detect":
            print("Detecting red objects...")
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if color_frame and depth_frame:
                color_image = np.asanyarray(color_frame.get_data())
                highlighted, red_objects = detect_red_objects(color_image, depth_frame, depth_intrinsics)
                latest_red_objects = red_objects
                
                if red_objects:
                    print(f"\nDetected {len(red_objects)} red object(s):")
                    for i, obj in enumerate(red_objects):
                        if obj['x_cam'] is not None:
                            print(f"  [{i+1}] Camera XYZ: X={obj['x_cam']:.1f}mm, Y={obj['y_cam']:.1f}mm, Z={obj['z_cam']:.1f}mm")
                        else:
                            print(f"  [{i+1}] No depth data")
                else:
                    print("No red objects detected")
                
                cv2.imshow("Detection", highlighted)
                cv2.waitKey(1)
        
        elif cmd.startswith("pick "):
            if latest_red_objects is None:
                print("Run 'detect' first to find objects")
                continue
            
            try:
                obj_idx = int(cmd.split()[1]) - 1
                if 0 <= obj_idx < len(latest_red_objects):
                    obj = latest_red_objects[obj_idx]
                    
                    if obj['x_cam'] is None:
                        print("No depth data for this object")
                        continue
                    
                    # Transform camera XYZ to robot XYZ (already in mm)
                    cam_xyz = [obj['x_cam'], obj['y_cam'], obj['z_cam']]
                    robot_xyz = camera_to_robot(cam_xyz)
                    
                    print(f"\nTargeting object {obj_idx+1}")
                    print(f"  Camera XYZ (mm): [{cam_xyz[0]:.1f}, {cam_xyz[1]:.1f}, {cam_xyz[2]:.1f}]")
                    print(f"  Robot XYZ (destination, mm): [{robot_xyz[0]:.1f}, {robot_xyz[1]:.1f}, {robot_xyz[2]:.1f}]")
                    
                    # Add approach height
                    approach_xyz = robot_xyz.copy()
                    approach_xyz[2] += APPROACH_HEIGHT
                    print(f"  Approach XYZ (mm): [{approach_xyz[0]:.1f}, {approach_xyz[1]:.1f}, {approach_xyz[2]:.1f}]")
                    
                    # Solve IK (with quick timeout)
                    print("  Solving IK for approach position...")
                    approach_angles = solve_ik(approach_xyz)
                    print("  Solving IK for grab position...")
                    grab_angles = solve_ik(robot_xyz)
                    
                    if approach_angles is None or grab_angles is None:
                        print("  ✗ IK failed - position unreachable")
                        continue
                    
                    # Move to approach position
                    print("  Moving to approach position...")
                    move_to_angles(approach_angles, duration=0.5)
                    time.sleep(0.5)
                    
                    # Move down to grab
                    print("  Moving down to grab...")
                    move_to_angles(grab_angles, duration=0.3)
                    time.sleep(0.5)
                    
                    # Close gripper
                    print("  Closing gripper...")
                    close_gripper()
                    
                    # Move back up
                    print("  Moving back up...")
                    move_to_angles(approach_angles, duration=0.5)
                    
                    print("  ✓ Object picked!")
                else:
                    print(f"Invalid object index (1-{len(latest_red_objects)})")
            except (ValueError, IndexError):
                print("Usage: pick <number>")
        
        elif cmd == "demo":
            print("Starting detection for demo...")
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if color_frame and depth_frame:
                color_image = np.asanyarray(color_frame.get_data())
                highlighted, red_objects = detect_red_objects(color_image, depth_frame, depth_intrinsics)
                
                if red_objects:
                    print(f"Found {len(red_objects)} red objects, picking first one...")
                    obj = red_objects[0]
                    
                    if obj['x_cam'] is not None:
                        cam_xyz = [obj['x_cam'], obj['y_cam'], obj['z_cam']]
                        robot_xyz = camera_to_robot(cam_xyz)
                        
                        print(f"\nAuto-picking first object:")
                        print(f"  Camera XYZ (mm): [{cam_xyz[0]:.1f}, {cam_xyz[1]:.1f}, {cam_xyz[2]:.1f}]")
                        print(f"  Robot XYZ (destination, mm): [{robot_xyz[0]:.1f}, {robot_xyz[1]:.1f}, {robot_xyz[2]:.1f}]")
                        
                        approach_xyz = robot_xyz.copy()
                        approach_xyz[2] += APPROACH_HEIGHT
                        print(f"  Approach XYZ (mm): [{approach_xyz[0]:.1f}, {approach_xyz[1]:.1f}, {approach_xyz[2]:.1f}]")
                        
                        approach_angles = solve_ik(approach_xyz)
                        grab_angles = solve_ik(robot_xyz)
                        
                        if approach_angles and grab_angles:
                            print("Moving to approach...")
                            move_to_angles(approach_angles, duration=0.5)
                            time.sleep(0.5)
                            
                            print("Grabbing...")
                            move_to_angles(grab_angles, duration=0.3)
                            time.sleep(0.5)
                            
                            close_gripper()
                            
                            print("Moving back up...")
                            move_to_angles(approach_angles, duration=0.5)
                            
                            print("✓ Demo complete!")
                        else:
                            print("✗ IK failed")
                    else:
                        print("No depth data")
                else:
                    print("No red objects detected")
        
        else:
            print("Invalid command")

except KeyboardInterrupt:
    print("\nInterrupted")
finally:
    print("Cleaning up...")
    open_gripper()
    pipeline.stop()
    cv2.destroyAllWindows()
    ser.close()
    print("Done")
