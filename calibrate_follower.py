"""
Manual calibration for SO-100 follower arm (lerobot-compatible format)
"""

import serial
import time
import json

PORT = "/dev/tty.usbmodem5AE60848961"
BAUD = 1000000

ser = serial.Serial(PORT, BAUD, timeout=0.5)
print("Connected to servo controller\n")

def write_servo(servo_id, address, value, length=1):
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
    position = int((angle_degrees / 360.0) * 4095)
    write_servo(servo_id, 0x2A, position, 2)
    print(f"Servo {servo_id}: Set to {angle_degrees}°")

def get_angle(servo_id):
    pos = read_servo(servo_id, 0x38, 2)
    if pos is not None:
        angle = (pos / 4095.0) * 360.0
        return angle
    return None

def enable_servo(servo_id):
    write_servo(servo_id, 0x28, 1)

def disable_servo(servo_id):
    """Disable torque on servo"""
    write_servo(servo_id, 0x28, 0)

# SO-100 follower arm joint names (lerobot format)
joint_names = [
    "shoulder_pan",    # ID 1
    "shoulder_lift",   # ID 2  
    "elbow_flex",      # ID 3
    "wrist_roll",      # ID 4
    "wrist_flex",      # ID 5
    "gripper"          # ID 6
]

# Lerobot-compatible calibration data structure
calibration_data = {
    "robot_type": "so100",
    "robot_variant": "follower",
    "joint_names": joint_names,
    "joint_limits": {},
    "joint_offsets": {},
    "port": "/dev/tty.usbmodem5AE60848961",
    "baudrate": 1000000
}

print("=== SO-100 Follower Arm Batch Calibration ===")
print("Move ALL joints through their full ranges. Script records min/max automatically.\n")

print("Calibration mode:")
print("  1. Full calibration (all joints)")
print("  2. Elbow only (re-calibrate just elbow_flex)")
mode = input("Enter mode (1 or 2): ").strip()

if mode == "2":
    # Elbow-only calibration
    print("\n=== Elbow-Only Calibration ===")
    disable_servo(3)
    time.sleep(0.1)
    print("Elbow torque disabled - move it freely.\n")
    
    print("Press ENTER to START recording elbow positions...")
    input()
    
    print("Recording elbow positions. Move elbow through full range. Press ENTER when done.\n")
    print("Live positions:")
    print("-" * 60)
    print(f"{'Elbow Position':<20} | {'Min':<15} | {'Max':<15} | {'Range':<15}")
    print("-" * 60)
    
    elbow_data = {"min": 360.0, "max": 0.0}
    start_time = time.time()
    sample_count = 0
    
    try:
        import sys
        import select
        
        while True:
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                input()
                break
            
            angle = get_angle(3)
            if angle is not None:
                elbow_data["min"] = min(elbow_data["min"], angle)
                elbow_data["max"] = max(elbow_data["max"], angle)
            
            current = angle or 0
            range_span = elbow_data["max"] - elbow_data["min"]
            print(f"\r{current:>8.1f}° | {elbow_data['min']:>13.1f}° | {elbow_data['max']:>13.1f}° | {range_span:>13.1f}°", end="", flush=True)
            
            sample_count += 1
            time.sleep(0.1)
    except Exception:
        input("Press ENTER when done:")
    
    print(f"\n" + "-" * 60)
    elapsed = time.time() - start_time
    print(f"Recording complete! ({sample_count} samples in {elapsed:.1f}s)")
    
    # Load existing calibration
    try:
        with open("so100_follower_calibration.json", "r") as f:
            ticks_cal = json.load(f)
    except FileNotFoundError:
        ticks_cal = {}
    
    # Update only elbow
    def deg_to_ticks(deg): return int(round((float(deg) / 360.0) * 4095))
    
    center_deg = (elbow_data["min"] + elbow_data["max"]) / 2
    ticks_cal["elbow_flex"] = {
        "id": 3,
        "drive_mode": 0,
        "range_min": deg_to_ticks(elbow_data["min"]),
        "range_max": deg_to_ticks(elbow_data["max"]),
        "homing_offset": deg_to_ticks(center_deg)
    }
    
    # Save
    with open("so100_follower_calibration.json", "w") as f:
        json.dump(ticks_cal, f, indent=2)
    
    print(f"\n✓ Elbow calibration saved:")
    print(f"  min={elbow_data['min']:.1f}°, max={elbow_data['max']:.1f}°, center={center_deg:.1f}°")
    print(f"  (ticks: range_min={deg_to_ticks(elbow_data['min'])}, range_max={deg_to_ticks(elbow_data['max'])}, homing_offset={deg_to_ticks(center_deg)})")

else:
    # Full calibration (existing code)
    # Disable torque on ALL servos
    print("\nDisabling torque on all servos...")
    for servo_id in range(1, 7):
        disable_servo(servo_id)
        time.sleep(0.05)
    print("All servos disabled - you can move them freely.\n")

    # Initialize tracking
    min_max_data = {name: {"min": 360.0, "max": 0.0} for name in joint_names}
    recording = False

    print("Press ENTER to START recording min/max positions...")
    input()
    recording = True
    print("Recording... Move all joints through their full ranges. Press ENTER when done.\n")

    start_time = time.time()
    sample_count = 0

    # Record continuously until user presses Enter
    try:
        # Start recording in background (blocking version: sample until Enter pressed)
        import sys
        import select
        
        print("Recording positions (this runs until you press ENTER)...")
        print("Live positions (update every 0.1s):")
        print("-" * 120)
        print(f"{'Joint Name':<15} | {'Current':<10} | {'Min':<10} | {'Max':<10} | {'Range':<10}")
        print("-" * 120)

        while True:
            # Check if input is available (non-blocking on Unix)
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                input()
                break
            
            # Sample all servos
            for i, joint_name in enumerate(joint_names):
                servo_id = i + 1
                angle = get_angle(servo_id)
                if angle is not None:
                    min_max_data[joint_name]["min"] = min(min_max_data[joint_name]["min"], angle)
                    min_max_data[joint_name]["max"] = max(min_max_data[joint_name]["max"], angle)
            
            # Print table (clear line and reprint)
            print("\r", end="")
            for joint_name in joint_names:
                data = min_max_data[joint_name]
                current = get_angle(joint_names.index(joint_name) + 1) or 0
                range_span = data["max"] - data["min"]
                print(f"{joint_name:<15} | {current:>8.1f}° | {data['min']:>8.1f}° | {data['max']:>8.1f}° | {range_span:>8.1f}°")
            
            sample_count += 1
            time.sleep(0.1)

        print("-" * 120)
            
    except Exception:
        # Fallback for Windows or if select not available
        print("Recording... Move all joints through their full ranges.")
        input("Press ENTER when done:")
        # Do final sample
        for i, joint_name in enumerate(joint_names):
            servo_id = i + 1
            angle = get_angle(servo_id)
            if angle is not None:
                min_max_data[joint_name]["min"] = min(min_max_data[joint_name]["min"], angle)
                min_max_data[joint_name]["max"] = max(min_max_data[joint_name]["max"], angle)

    recording = False
    elapsed = time.time() - start_time

    print(f"\nRecording complete! ({sample_count} samples in {elapsed:.1f}s)")
    print("\nRecorded ranges:")
    for joint_name in joint_names:
        data = min_max_data[joint_name]
        center = (data["min"] + data["max"]) / 2
        print(f"  {joint_name}: min={data['min']:.1f}°, max={data['max']:.1f}°, center={center:.1f}°")

    # Build calibration data
    def deg_to_ticks(deg): return int(round((float(deg) / 360.0) * 4095))

    calibration_data = {
        "robot_type": "so100",
        "robot_variant": "follower",
        "joint_names": joint_names,
        "joint_limits": {},
        "joint_offsets": {},
        "port": PORT,
        "baudrate": BAUD
    }

    ticks_cal = {}

    for joint_name in joint_names:
        data = min_max_data[joint_name]
        min_deg = data["min"]
        max_deg = data["max"]
        center_deg = (min_deg + max_deg) / 2

        # Lerobot format
        calibration_data["joint_limits"][joint_name] = {"min": min_deg, "max": max_deg}
        calibration_data["joint_offsets"][joint_name] = center_deg

        # Ticks format
        servo_id = joint_names.index(joint_name) + 1
        ticks_cal[joint_name] = {
            "id": servo_id,
            "drive_mode": 0,
            "range_min": deg_to_ticks(min_deg),
            "range_max": deg_to_ticks(max_deg),
            "homing_offset": deg_to_ticks(center_deg)
        }

    # Save both formats
    with open("so100_follower_calibration_lerobot.json", "w") as f:
        json.dump(calibration_data, f, indent=2)

    with open("so100_follower_calibration.json", "w") as f:
        json.dump(ticks_cal, f, indent=2)

    print("\n✓ Calibration saved to:")
    print("  - so100_follower_calibration_lerobot.json")
    print("  - so100_follower_calibration.json")