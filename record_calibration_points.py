"""
Interactive calibration point recorder.
Collects camera XYZ and robot XYZ pairs using FK to auto-compute robot position.
"""
import cv2
import pyrealsense2 as rs
import numpy as np
import json
import os
import time
import serial
import sys

# Import arm functions
from ik_solver import solve_fk

# Serial port setup
def find_serial_port():
    """Find and return the serial port for the arm"""
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    
    # Try COM ports in reverse order (COM4 first on Windows)
    com_ports = sorted([p.device for p in ports if 'COM' in p.device], reverse=True)
    
    if not com_ports:
        print("✗ No COM ports found")
        return None
    
    for port in com_ports:
        try:
            test_ser = serial.Serial(port, 1000000, timeout=0.1)
            print(f"✓ Found arm on {port}")
            test_ser.close()
            return port
        except:
            continue
    
    print(f"✗ Could not connect to arm on any COM port")
    return None

SERIAL_PORT = find_serial_port()
if not SERIAL_PORT:
    print("Calibration requires arm connection")
    exit(1)

ser = serial.Serial(SERIAL_PORT, 1000000, timeout=0.1)

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

# Camera setup
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

# Get intrinsics
profile = pipeline.get_active_profile()
depth_stream = profile.get_stream(rs.stream.depth)
depth_intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()

calibration_data = {
    "points": []
}

clicked_pixel = None

def mouse_callback(event, x, y, flags, param):
    """Click on image to select a calibration point"""
    global clicked_pixel
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_pixel = (x, y)
        print(f"\n✓ Pixel selected: ({x}, {y})")

# Load existing calibration or start fresh
CALIBRATION_FILE = "calibration_points.json"
if os.path.exists(CALIBRATION_FILE):
    with open(CALIBRATION_FILE, 'r') as f:
        calibration_data = json.load(f)
    print(f"\nLoaded {len(calibration_data['points'])} existing points from {CALIBRATION_FILE}")
else:
    print(f"Starting fresh calibration (will save to {CALIBRATION_FILE})")

print("\n" + "="*70)
print("CALIBRATION POINT RECORDER - AUTO FK")
print("="*70)
print("Instructions:")
print("1. A camera feed will open")
print("2. Move arm to a position")
print("3. Click on the object's position in the camera feed")
print("4. FK automatically reads servo angles and computes robot XYZ")
print("5. Point is recorded")
print("6. Repeat for multiple positions (5+ recommended for accuracy)")
print("7. Type 'done' to finish and compute transformation")
print("="*70 + "\n")

# Enable all servos
print("Enabling servos...")
for i in range(1, 7):
    enable_servo(i)
time.sleep(0.5)
print("✓ Servos enabled\n")

cv2.namedWindow("Camera Feed - Click to Select Point")
cv2.setMouseCallback("Camera Feed - Click to Select Point", mouse_callback)

point_count = len(calibration_data["points"])

try:
    while True:
        # Get frames
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        
        if not color_frame or not depth_frame:
            continue
        
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        
        # Draw existing points
        display = color_image.copy()
        for i, point in enumerate(calibration_data["points"]):
            if 'pixel_x' in point and 'pixel_y' in point:
                px, py = int(point['pixel_x']), int(point['pixel_y'])
                cv2.circle(display, (px, py), 8, (0, 255, 0), 2)
                cv2.putText(display, str(i+1), (px-5, py-15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        cv2.imshow("Camera Feed - Click to Select Point", display)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("\nQuitting without saving...")
            break
        
        if clicked_pixel:
            px, py = clicked_pixel
            
            # Get depth at clicked pixel
            depth_value = depth_frame.get_distance(px, py)
            if depth_value == 0:
                print("  ✗ No depth data at this pixel (too close or too far)")
                clicked_pixel = None
                continue
            
            # Convert pixel to 3D using intrinsics
            x = (px - depth_intrinsics.ppx) / depth_intrinsics.fx * depth_value
            y = (py - depth_intrinsics.ppy) / depth_intrinsics.fy * depth_value
            z = depth_value
            
            print(f"  Camera XYZ (meters): X={x:.3f}, Y={y:.3f}, Z={z:.3f}")
            print(f"  Camera XYZ (mm):     X={x*1000:.1f}, Y={y*1000:.1f}, Z={z*1000:.1f}")
            
            # Read servo angles and compute FK
            print("  Reading servo angles...")
            logical_angles = []
            for i in range(1, 7):
                angle = get_angle(i)
                if angle is None:
                    print(f"    ✗ Failed to read servo {i}")
                    clicked_pixel = None
                    break
                logical_angles.append(angle)
            else:
                # All angles read successfully - compute FK
                robot_xyz = solve_fk(logical_angles)
                print(f"  Servo angles: {[f'{a:.1f}°' for a in logical_angles]}")
                print(f"  Robot XYZ (FK): X={robot_xyz[0]:.1f}, Y={robot_xyz[1]:.1f}, Z={robot_xyz[2]:.1f}")
                
                # Store point
                point = {
                    "pixel_x": px,
                    "pixel_y": py,
                    "camera_x": x * 1000,  # Convert to mm
                    "camera_y": y * 1000,
                    "camera_z": z * 1000,
                    "robot_x": robot_xyz[0],
                    "robot_y": robot_xyz[1],
                    "robot_z": robot_xyz[2],
                    "servo_angles": [float(a) for a in logical_angles],
                    "timestamp": time.time()
                }
                
                calibration_data["points"].append(point)
                point_count += 1
                
                print(f"\n✓ Point {point_count} recorded")
                print(f"  Camera: [{point['camera_x']:.1f}, {point['camera_y']:.1f}, {point['camera_z']:.1f}]")
                print(f"  Robot:  [{robot_xyz[0]:.1f}, {robot_xyz[1]:.1f}, {robot_xyz[2]:.1f}]")
                
                # Save immediately
                with open(CALIBRATION_FILE, 'w') as f:
                    json.dump(calibration_data, f, indent=2)
                print(f"  Saved to {CALIBRATION_FILE}")
                
                clicked_pixel = None
        
        # Check for 'done' command
        if key == ord('d'):
            # Flush input and ask for confirmation
            user_input = input("\nType 'done' to finish and compute transformation (or 'c' to continue): ").strip().lower()
            if user_input == 'done':
                break

except KeyboardInterrupt:
    print("\nInterrupted by user")

cv2.destroyAllWindows()
pipeline.stop()

print(f"\n" + "="*70)
print(f"CALIBRATION COLLECTION COMPLETE")
print(f"Total points recorded: {len(calibration_data['points'])}")
print("="*70)

if len(calibration_data["points"]) < 3:
    print("⚠ Warning: Need at least 3 points to compute transformation. Exiting.")
    exit(1)

# Compute transformation matrix using Kabsch algorithm
print("\nComputing transformation matrix...")

camera_points = np.array([
    [p['camera_x'], p['camera_y'], p['camera_z']] 
    for p in calibration_data["points"]
])

robot_points = np.array([
    [p['robot_x'], p['robot_y'], p['robot_z']] 
    for p in calibration_data["points"]
])

# Center both point sets
camera_centroid = np.mean(camera_points, axis=0)
robot_centroid = np.mean(robot_points, axis=0)

camera_centered = camera_points - camera_centroid
robot_centered = robot_points - robot_centroid

# SVD to find rotation matrix
H = camera_centered.T @ robot_centered
U, S, Vt = np.linalg.svd(H)
R = Vt.T @ U.T

# Ensure proper rotation (det = 1)
if np.linalg.det(R) < 0:
    Vt[-1, :] *= -1
    R = Vt.T @ U.T

# Compute translation
t = robot_centroid - (R @ camera_centroid)

print(f"\nRotation matrix R:\n{R}")
print(f"\nTranslation vector t: {t}")

# Verify with all points
print(f"\n{'Point':<8} {'Camera XYZ':<35} {'Robot Expected':<35} {'Robot Actual':<35} {'Error (mm)':<12}")
print("-" * 130)

errors = []
for i, point in enumerate(calibration_data["points"]):
    cam = np.array([point['camera_x'], point['camera_y'], point['camera_z']])
    expected = R @ cam + t
    actual = np.array([point['robot_x'], point['robot_y'], point['robot_z']])
    error = np.linalg.norm(expected - actual)
    errors.append(error)
    
    print(f"{i+1:<8} [{cam[0]:7.1f}, {cam[1]:7.1f}, {cam[2]:7.1f}]      "
          f"[{expected[0]:7.1f}, {expected[1]:7.1f}, {expected[2]:7.1f}]      "
          f"[{actual[0]:7.1f}, {actual[1]:7.1f}, {actual[2]:7.1f}]      "
          f"{error:7.2f}")

mean_error = np.mean(errors)
max_error = np.max(errors)
print("-" * 130)
print(f"Average error: {mean_error:.2f}mm")
print(f"Max error:     {max_error:.2f}mm")

# Save transformation
output = {
    "rotation_matrix": R.tolist(),
    "translation_vector": t.tolist(),
    "mean_error_mm": float(mean_error),
    "max_error_mm": float(max_error),
    "num_points": len(calibration_data["points"]),
    "points": calibration_data["points"]
}

OUTPUT_FILE = "camera_calibration.json"
with open(OUTPUT_FILE, 'w') as f:
    json.dump(output, f, indent=2)

print(f"\n✓ Transformation saved to {OUTPUT_FILE}")
print("Ready to use in pick_and_place.py!")
