import serial
import time
import json
import os
import math
from ik_solver import solve_fk, solve_ik

PORT = "/dev/tty.usbmodem5AE60848961"
BAUD = 1000000

ser = serial.Serial(PORT, BAUD, timeout=0.5)
print("Connected to servo controller\n")

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
    time.sleep(0.01)
    ser.read(20)

def read_servo(servo_id, address, length=1):
    """Read from servo register"""
    packet = [0xFF, 0xFF, servo_id, 4, 0x02, address, length]
    checksum = (~sum(packet[2:])) & 0xFF
    packet.append(checksum)
    
    ser.reset_input_buffer()
    ser.write(bytes(packet))
    time.sleep(0.15)
    resp = ser.read(20)
    
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
    """Get current servo angle"""
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

# Load calibration
raw_calibration = None

def load_calibration(path="calibration.json"):
    global raw_calibration
    if not os.path.exists(path):
        print(f"Calibration file not found: {path}")
        return None
    print(f"Loading calibration from: {path}")
    with open(path, "r") as f:
        raw = json.load(f)
    raw_calibration = raw
    return raw

def get_drive_mode(joint_name):
    """Get drive_mode for a joint from raw calibration"""
    global raw_calibration
    if raw_calibration and joint_name in raw_calibration:
        return raw_calibration[joint_name].get("drive_mode", 0)
    return 0

def set_angle_with_inversion(servo_id, angle_degrees, joint_name):
    """Set servo angle, inverting if drive_mode is 1"""
    drive_mode = get_drive_mode(joint_name)
    
    if drive_mode == 1:
        inverted_angle = 360.0 - angle_degrees
        position = int((inverted_angle / 360.0) * 4095)
        print(f"Servo {servo_id} ({joint_name}): Target {angle_degrees}° -> Inverted to {inverted_angle}° (drive_mode=1)")
    else:
        position = int((angle_degrees / 360.0) * 4095)
    
    write_servo(servo_id, 0x2A, position, 2)

def physical_to_logical(physical_deg, joint_name):
    """Convert physical degrees to logical degrees using calibration"""
    if not raw_calibration or joint_name not in raw_calibration:
        return physical_deg
    
    mapping = raw_calibration[joint_name]
    logical_min = mapping.get("logical_min", 0)
    logical_max = mapping.get("logical_max", 180)
    physical_min = mapping.get("physical_min", 0)
    physical_max = mapping.get("physical_max", 360)
    
    if physical_max == physical_min:
        return logical_min
    
    ratio = (physical_deg - physical_min) / (physical_max - physical_min)
    logical = logical_min + ratio * (logical_max - logical_min)
    return max(logical_min, min(logical_max, logical))

def logical_to_physical(logical_deg, joint_name):
    """Convert logical degrees to physical degrees using calibration"""
    if not raw_calibration or joint_name not in raw_calibration:
        return logical_deg
    
    mapping = raw_calibration[joint_name]
    logical_min = mapping.get("logical_min", 0)
    logical_max = mapping.get("logical_max", 180)
    physical_min = mapping.get("physical_min", 0)
    physical_max = mapping.get("physical_max", 360)
    
    if logical_max == logical_min:
        return physical_min
    
    ratio = (logical_deg - logical_min) / (logical_max - logical_min)
    physical = physical_min + ratio * (physical_max - physical_min)
    return physical

def check_angles():
    """Read all servo angles and compute FK to get end effector position"""
    print("\n=== Check Current Angles ===")
    
    joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
    logical_angles = []
    
    print("\nCurrent servo angles:")
    print("-" * 80)
    print(f"{'Joint':<20} | {'Physical (°)':<15} | {'Logical (°)':<15}")
    print("-" * 80)
    
    for i, joint_name in enumerate(joint_names, start=1):
        physical = get_angle(i)
        if physical is not None:
            logical = physical_to_logical(physical, joint_name)
            logical_angles.append(logical)
            print(f"{joint_name:<20} | {physical:>13.1f}° | {logical:>13.1f}°")
        else:
            print(f"{joint_name:<20} | {'N/A':>13} | {'N/A':>13}")
            logical_angles.append(90)  # Default to home
    
    print("-" * 80)
    
    # Solve FK
    try:
        position = solve_fk(logical_angles)
        print(f"\n✓ Forward Kinematics Solution:")
        print(f"  X: {position[0]:.3f} mm")
        print(f"  Y: {position[1]:.3f} mm")
        print(f"  Z: {position[2]:.3f} mm")
        print(f"  Coordinates: {position}\n")
        return logical_angles, position
    except Exception as e:
        print(f"✗ FK failed: {e}\n")
        return None, None

def move_to_xyz(x=None, y=None, z=None):
    """Move arm to target XYZ position using IK"""
    print("\n=== Move to XYZ Position ===")
    
    # If coordinates not provided via command, prompt for them
    if x is None or y is None or z is None:
        try:
            x = float(input("Enter target X (mm): "))
            y = float(input("Enter target Y (mm): "))
            z = float(input("Enter target Z (mm): "))
        except ValueError:
            print("Error: Invalid input. Please enter numbers.")
            return
    
    target = [x, y, z]
    print(f"\nTarget position: X={x:.1f}, Y={y:.1f}, Z={z:.1f}")
    print("Solving IK... (this may take a moment)")
    
    # Solve IK
    angles = solve_ik(target)
    
    if angles is None:
        print("✗ IK solution not found. Target may be unreachable.\n")
        return
    
    print(f"\n✓ IK Solution found:")
    joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
    
    print("-" * 80)
    print(f"{'Joint':<20} | {'Logical (°)':<15} | {'Physical (°)':<15}")
    print("-" * 80)
    
    for i, (joint_name, logical) in enumerate(zip(joint_names, angles), start=1):
        physical = logical_to_physical(logical, joint_name)
        print(f"{joint_name:<20} | {logical:>13.1f}° | {physical:>13.1f}°")
    
    print("-" * 80)
    
    # Verify with FK
    try:
        verify_pos = solve_fk(angles)
        error = math.sqrt((verify_pos[0] - x)**2 + (verify_pos[1] - y)**2 + (verify_pos[2] - z)**2)
        print(f"\nVerification (FK): X={verify_pos[0]:.1f}, Y={verify_pos[1]:.1f}, Z={verify_pos[2]:.1f}")
        print(f"Position error: {error:.2f} mm\n")
    except Exception as e:
        print(f"Verification failed: {e}\n")
    
    # Ask if user wants to move
    response = input("Move arm to this position? (y/n): ").strip().lower()
    if response != 'y':
        print("Move cancelled.\n")
        return
    
    # Move arm
    print("\nMoving arm...")
    for i, (joint_name, logical) in enumerate(zip(joint_names, angles), start=1):
        physical = logical_to_physical(logical, joint_name)
        enable_servo(i)
        time.sleep(0.05)
        set_angle_with_inversion(i, physical, joint_name)
        time.sleep(0.1)
    
    print("Waiting for arm to move...")
    time.sleep(2)
    
    # Verify actual position
    final_logical, final_pos = check_angles()
    if final_pos:
        final_error = math.sqrt((final_pos[0] - x)**2 + (final_pos[1] - y)**2 + (final_pos[2] - z)**2)
        print(f"Final position error: {final_error:.2f} mm")

def safe_move(servo_id, target_deg, joint_name):
    """Safely move a single servo"""
    enable_servo(servo_id)
    time.sleep(0.05)
    
    # Convert logical to physical
    physical = logical_to_physical(target_deg, joint_name)
    set_angle_with_inversion(servo_id, physical, joint_name)
    print(f"{joint_name}: Moving to {target_deg:.1f}° (logical) / {physical:.1f}° (physical)")

# Main interactive loop
print("\n" + "="*80)
print("SO-100 ARM FK/IK TEST SUITE")
print("="*80)
print("\nThis tool allows you to:")
print("  1. Check current joint angles and compute FK")
print("  2. Move to target XYZ coordinates using IK")
print("  3. Manually move individual joints")
print("  4. Load calibration data\n")

cal = load_calibration()
if not cal:
    print("Warning: Could not load calibration. Some features may not work.\n")

print("\n=== FK/IK Test Commands ===")
print("  check_angles     - Read all joint angles and compute FK position")
print("  move_xyz         - Move arm to target X,Y,Z using IK")
print("  move <id> <deg>  - Manually move servo <id> to logical <deg> (0-180)")
print("  home             - Move all joints to home position (90°)")
print("  torque on <id>   - Enable torque on servo")
print("  torque off <id>  - Disable torque on servo")
print("  torque off all   - Disable torque on all servos")
print("  help             - Show this message")
print("  q                - Quit\n")

while True:
    try:
        cmd = input("Enter command: ").strip().lower()
        
        if cmd == 'q':
            print("Exiting...")
            break
        elif cmd == 'help':
            print("\n=== FK/IK Test Commands ===")
            print("  check_angles           - Read all joint angles and compute FK position")
            print("  move_xyz x y z         - Move arm to target X,Y,Z using IK (e.g., 'move_xyz 396.144 111.781 92.988')")
            print("  move <id> <deg>        - Manually move servo <id> to logical <deg> (0-180)")
            print("  home                   - Move all joints to home position (90°)")
            print("  torque on <id>         - Enable torque on servo")
            print("  torque off <id>        - Disable torque on servo")
            print("  torque off all         - Disable torque on all servos")
            print("  help                   - Show this message")
            print("  q                      - Quit\n")
        elif cmd == 'check_angles':
            check_angles()
        elif cmd.startswith('move_xyz '):
            try:
                parts = cmd.split()
                if len(parts) == 4:
                    x = float(parts[1])
                    y = float(parts[2])
                    z = float(parts[3])
                    move_to_xyz(x, y, z)
                else:
                    print("Usage: move_xyz x y z (e.g., 'move_xyz 396.144 111.781 92.988')")
            except ValueError:
                print("Error: Invalid numbers. Usage: move_xyz x y z")
        elif cmd == 'move_xyz':
            move_to_xyz()
        elif cmd == 'home':
            print("\nMoving all joints to home position (90°)...")
            joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
            for i, joint_name in enumerate(joint_names, start=1):
                safe_move(i, 90.0, joint_name)
                time.sleep(0.1)
            time.sleep(1)
            print("Home position reached.\n")
        elif cmd.startswith('move '):
            try:
                parts = cmd.split()
                servo_id = int(parts[1])
                logical_deg = float(parts[2])
                
                if 1 <= servo_id <= 6 and 0 <= logical_deg <= 180:
                    joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
                    joint_name = joint_names[servo_id - 1]
                    safe_move(servo_id, logical_deg, joint_name)
                    time.sleep(0.5)
                else:
                    print("Error: Servo ID must be 1-6, angle must be 0-180")
            except (ValueError, IndexError):
                print("Usage: move <servo_id> <logical_degrees> (e.g., 'move 1 90')")
        elif cmd.startswith('torque '):
            parts = cmd.split()
            if len(parts) < 3:
                print("Usage: torque on <id> | torque off <id> | torque off all")
            else:
                action = parts[1].lower()
                target = parts[2].lower()
                
                if action == 'on':
                    try:
                        servo_id = int(target)
                        enable_servo(servo_id)
                        print(f"Torque enabled on servo {servo_id}")
                    except ValueError:
                        print("Usage: torque on <id>")
                elif action == 'off':
                    if target == 'all':
                        for servo_id in range(1, 7):
                            disable_servo(servo_id)
                            time.sleep(0.05)
                        print("Torque disabled on all servos")
                    else:
                        try:
                            servo_id = int(target)
                            disable_servo(servo_id)
                            print(f"Torque disabled on servo {servo_id}")
                        except ValueError:
                            print("Usage: torque off <id> or torque off all")
        else:
            print("Unknown command. Type 'help' for available commands.")
    
    except KeyboardInterrupt:
        print("\nExiting...")
        break
