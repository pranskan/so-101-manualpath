# Manual calibration for SO-100 follower arm (lerobot-compatible format)

import serial
import serial.tools.list_ports
import time
import json

BAUD = 1000000

def find_serial_port():
    """Auto-detect available serial ports and connect to the robot."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        raise RuntimeError("No serial port found! Connect your robot.")
    
    print(f"Available ports: {[p.device for p in ports]}")
    
    # Try each port with different baud rates
    baud_rates = [1000000, 921600, 460800, 230400, 115200]
    
    # Try COM4 first if available
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
time.sleep(0.2)  # Brief initialization wait

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
    
    # Clear any garbage in the buffer first
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
    # Convert 0-360 input to 0-240 register range
    position = int((angle_degrees / 360.0) * 240.0 * 4095.0 / 240.0)
    write_servo(servo_id, 0x2A, position, 2)
    print(f"Servo {servo_id}: Set to {angle_degrees}°")

def get_angle(servo_id):
    """Get current servo angle"""
    pos = read_servo(servo_id, 0x38, 2)
    if pos is not None:
        # Convert 0-240 register range to 0-360 physical rotation
        angle = (pos / 4095.0) * 360.0
        return angle
    return None

def enable_servo(servo_id):
    """Enable torque on servo"""
    write_servo(servo_id, 0x28, 1)

def disable_servo(servo_id):
    """Disable torque on servo"""
    write_servo(servo_id, 0x28, 0)

# SO-100 follower arm joint names (lerobot format)
joint_names = [
    "shoulder_pan",    # ID 1
    "shoulder_lift",   # ID 2  
    "elbow_flex",      # ID 3
    "wrist_flex",      # ID 4  # Corrected: was wrist_roll
    "wrist_roll",      # ID 5  # Corrected: was wrist_flex
    "gripper"          # ID 6
]

# Lerobot-compatible calibration data structure
calibration_data = {
    "robot_type": "so100",
    "robot_variant": "follower",
    "joint_names": joint_names,
    "joint_limits": {},
    "joint_offsets": {},
    "port": PORT,
    "baudrate": BAUD
}

print("=== SO-100 Follower Arm Batch Calibration ===")
print("Move ALL joints through their full ranges. Script records min/max automatically.\n")

print("Calibration mode:")
print("  1. Full calibration (all joints)")
print("  2. Elbow only (re-calibrate just elbow_flex)")
print("  3. Shoulder lift only (re-calibrate just shoulder_lift)")
print("  4. Shoulder pan only (re-calibrate just shoulder_pan)")
mode = input("Enter mode (1, 2, 3, or 4): ").strip()

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
            
            current = angle if angle is not None else 0
            range_span = elbow_data["max"] - elbow_data["min"]
            print(f"\r{current:>8.1f}° | {elbow_data['min']:>13.1f}° | {elbow_data['max']:>13.1f}° | {range_span:>13.1f}°", end="", flush=True)
            
            sample_count += 1
            time.sleep(0.1)
    except Exception:
        input("Press ENTER when done:")
    
    print(f"\n" + "-" * 60)
    elapsed = time.time() - start_time
    print(f"Recording complete! ({sample_count} samples in {elapsed:.1f}s)")
    
    # Check if valid range was recorded
    if elbow_data["min"] >= elbow_data["max"] or elbow_data["min"] == 360.0:
        print("ERROR: No valid elbow angles recorded. Check servo connection and try again.")
        exit(1)
    
    # Load existing calibration
    try:
        with open("calibration.json", "r") as f:
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
    with open("calibration.json", "w") as f:
        json.dump(ticks_cal, f, indent=2)
    
    print(f"\n✓ Elbow calibration saved:")
    print(f"  min={elbow_data['min']:.1f}°, max={elbow_data['max']:.1f}°, center={center_deg:.1f}°")
    print(f"  (ticks: range_min={deg_to_ticks(elbow_data['min'])}, range_max={deg_to_ticks(elbow_data['max'])}, homing_offset={deg_to_ticks(center_deg)})")

elif mode == "3":
    # Shoulder lift-only calibration
    print("\n=== Shoulder Lift-Only Calibration ===")
    disable_servo(2)
    time.sleep(0.1)
    print("Shoulder lift torque disabled - move it freely.\n")
    
    print("Press ENTER to START recording shoulder lift positions...")
    input()
    
    print("Recording shoulder lift positions. Move shoulder lift through full range. Press ENTER when done.\n")
    print("Live positions:")
    print("-" * 60)
    print(f"{'Shoulder Lift Position':<25} | {'Min':<15} | {'Max':<15} | {'Range':<15}")
    print("-" * 60)
    
    shoulder_lift_data = {"min": 360.0, "max": 0.0}
    start_time = time.time()
    sample_count = 0
    
    try:
        import sys
        import select
        
        while True:
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                input()
                break
            
            angle = get_angle(2)
            if angle is not None:
                shoulder_lift_data["min"] = min(shoulder_lift_data["min"], angle)
                shoulder_lift_data["max"] = max(shoulder_lift_data["max"], angle)
            
            current = angle if angle is not None else 0
            range_span = shoulder_lift_data["max"] - shoulder_lift_data["min"]
            print(f"\r{current:>8.1f}° | {shoulder_lift_data['min']:>13.1f}° | {shoulder_lift_data['max']:>13.1f}° | {range_span:>13.1f}°", end="", flush=True)
            
            sample_count += 1
            time.sleep(0.1)
    except Exception:
        input("Press ENTER when done:")
    
    print(f"\n" + "-" * 60)
    elapsed = time.time() - start_time
    print(f"Recording complete! ({sample_count} samples in {elapsed:.1f}s)")
    
    # Check if valid range was recorded
    if shoulder_lift_data["min"] >= shoulder_lift_data["max"] or shoulder_lift_data["min"] == 360.0:
        print("ERROR: No valid shoulder lift angles recorded. Check servo connection and try again.")
        exit(1)
    
    # Load existing calibration
    try:
        with open("calibration.json", "r") as f:
            ticks_cal = json.load(f)
    except FileNotFoundError:
        ticks_cal = {}
    
    # Update only shoulder lift
    def deg_to_ticks(deg): return int(round((float(deg) / 360.0) * 4095))
    
    center_deg = (shoulder_lift_data["min"] + shoulder_lift_data["max"]) / 2
    ticks_cal["shoulder_lift"] = {
        "id": 2,
        "drive_mode": 0,
        "range_min": deg_to_ticks(shoulder_lift_data["min"]),
        "range_max": deg_to_ticks(shoulder_lift_data["max"]),
        "homing_offset": deg_to_ticks(center_deg)
    }
    
    # Save
    with open("calibration.json", "w") as f:
        json.dump(ticks_cal, f, indent=2)
    
    print(f"\n✓ Shoulder lift calibration saved:")
    print(f"  min={shoulder_lift_data['min']:.1f}°, max={shoulder_lift_data['max']:.1f}°, center={center_deg:.1f}°")
    print(f"  (ticks: range_min={deg_to_ticks(shoulder_lift_data['min'])}, range_max={deg_to_ticks(shoulder_lift_data['max'])}, homing_offset={deg_to_ticks(center_deg)})")
elif mode == "4":
    # Shoulder pan-only calibration
    print("\n=== Shoulder Pan-Only Calibration ===")
    disable_servo(1)
    time.sleep(0.1)
    print("Shoulder pan torque disabled - move it freely.\n")
    
    print("Press ENTER to START recording shoulder pan positions...")
    input()
    
    print("Recording shoulder pan positions. Move shoulder pan through full range. Press ENTER when done.\n")
    print("Live positions:")
    print("-" * 60)
    print(f"{'Shoulder Pan Position':<25} | {'Min':<15} | {'Max':<15} | {'Range':<15}")
    print("-" * 60)
    
    shoulder_pan_data = {"min": 360.0, "max": 0.0}
    start_time = time.time()
    sample_count = 0
    
    try:
        import sys
        import select
        
        while True:
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                input()
                break
            
            angle = get_angle(1)
            if angle is not None:
                shoulder_pan_data["min"] = min(shoulder_pan_data["min"], angle)
                shoulder_pan_data["max"] = max(shoulder_pan_data["max"], angle)
            
            current = angle if angle is not None else 0
            range_span = shoulder_pan_data["max"] - shoulder_pan_data["min"]
            print(f"\r{current:>8.1f}° | {shoulder_pan_data['min']:>13.1f}° | {shoulder_pan_data['max']:>13.1f}° | {range_span:>13.1f}°", end="", flush=True)
            
            sample_count += 1
            time.sleep(0.1)
    except Exception:
        input("Press ENTER when done:")
    
    print(f"\n" + "-" * 60)
    elapsed = time.time() - start_time
    print(f"Recording complete! ({sample_count} samples in {elapsed:.1f}s)")
    
    # Check if valid range was recorded
    if shoulder_pan_data["min"] >= shoulder_pan_data["max"] or shoulder_pan_data["min"] == 360.0:
        print("ERROR: No valid shoulder pan angles recorded. Check servo connection and try again.")
        exit(1)
    
    # Load existing calibration
    try:
        with open("calibration.json", "r") as f:
            ticks_cal = json.load(f)
    except FileNotFoundError:
        ticks_cal = {}
    
    # Update only shoulder pan
    def deg_to_ticks(deg): return int(round((float(deg) / 360.0) * 4095))
    
    center_deg = (shoulder_pan_data["min"] + shoulder_pan_data["max"]) / 2
    ticks_cal["shoulder_pan"] = {
        "id": 1,
        "drive_mode": 0,
        "range_min": deg_to_ticks(shoulder_pan_data["min"]),
        "range_max": deg_to_ticks(shoulder_pan_data["max"]),
        "homing_offset": deg_to_ticks(center_deg)
    }
    
    # Save
    with open("calibration.json", "w") as f:
        json.dump(ticks_cal, f, indent=2)
    
    print(f"\n✓ Shoulder pan calibration saved:")
    print(f"  min={shoulder_pan_data['min']:.1f}°, max={shoulder_pan_data['max']:.1f}°, center={center_deg:.1f}°")
    print(f"  (ticks: range_min={deg_to_ticks(shoulder_pan_data['min'])}, range_max={deg_to_ticks(shoulder_pan_data['max'])}, homing_offset={deg_to_ticks(center_deg)})")
else:
    # Full calibration (selective)
    print("\nEnter joints to calibrate (e.g., '1,2,3' for shoulder_pan, shoulder_lift, elbow_flex or 'all' for all joints):")
    joint_input = input("Joints: ").strip().lower()
    
    if joint_input == 'all':
        joints_to_calibrate = joint_names
    else:
        try:
            ids = [int(x.strip()) for x in joint_input.split(',')]
            joints_to_calibrate = [joint_names[i-1] for i in ids if 1 <= i <= len(joint_names)]
            if not joints_to_calibrate:
                print("No valid joints selected. Exiting.")
                exit(1)
        except ValueError:
            print("Invalid input. Use format like '1,2,3' or 'all'.")
            exit(1)
    
    print(f"Calibrating joints: {joints_to_calibrate}")
    
    # Disable torque on selected servos
    print("\nDisabling torque on selected servos...")
    for joint_name in joints_to_calibrate:
        servo_id = joint_names.index(joint_name) + 1
        disable_servo(servo_id)
        time.sleep(0.05)
    print("Selected servos disabled - you can move them freely.\n")

    # Initialize tracking for selected joints
    min_max_data = {name: {"min": 360.0, "max": 0.0} for name in joints_to_calibrate}
    recording = False

    print("Press ENTER to START recording min/max positions...")
    input()
    recording = True
    print("Recording... Move the selected joints through their full ranges. Press ENTER when done.\n")

    start_time = time.time()
    sample_count = 0

    # Record continuously until user presses Enter
    try:
        # Start recording in background (blocking version: sample until Enter pressed)
        import sys
        import select
        
        print("Recording positions (this runs until you press ENTER)...")
        print("Live positions (update every 0.1s):")
        print("-" * 140)
        print(f"{'Joint Name':<15} | {'Current':<10} | {'Min':<10} | {'Max':<10} | {'Range':<10} | {'Logical':<10}")
        print("-" * 140)

        while True:
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                input()
                break
            
            # Sample selected servos
            for joint_name in joints_to_calibrate:
                servo_id = joint_names.index(joint_name) + 1
                angle = get_angle(servo_id)
                if angle is not None:
                    min_max_data[joint_name]["min"] = min(min_max_data[joint_name]["min"], angle)
                    min_max_data[joint_name]["max"] = max(min_max_data[joint_name]["max"], angle)
            
            # Print table (clear line and reprint)
            print("\r", end="")
            for joint_name in joints_to_calibrate:
                data = min_max_data[joint_name]
                current = get_angle(joint_names.index(joint_name) + 1) or 0
                range_span = data["max"] - data["min"]
                logical = 0
                if range_span > 0:
                    logical = ((current - data["min"]) / range_span) * 180
                    logical = max(0, min(180, logical))
                print(f"{joint_name:<15} | {current:>8.1f}° | {data['min']:>8.1f}° | {data['max']:>8.1f}° | {range_span:>8.1f}° | {logical:>8.1f}°")
            
            sample_count += 1
            time.sleep(0.1)

        print("-" * 140)
            
    except Exception:
        # Fallback for Windows or if select not available
        print("Recording... Move the selected joints through their full ranges.")
        input("Press ENTER when done:")
        # Do final sample
        for joint_name in joints_to_calibrate:
            servo_id = joint_names.index(joint_name) + 1
            angle = get_angle(servo_id)
            if angle is not None:
                min_max_data[joint_name]["min"] = min(min_max_data[joint_name]["min"], angle)
                min_max_data[joint_name]["max"] = max(min_max_data[joint_name]["max"], angle)

    recording = False
    elapsed = time.time() - start_time

    print(f"\nRecording complete! ({sample_count} samples in {elapsed:.1f}s)")
    print("\nRecorded ranges:")
    for joint_name in joints_to_calibrate:
        data = min_max_data[joint_name]
        center = (data["min"] + data["max"]) / 2
        print(f"  {joint_name}: min={data['min']:.1f}°, max={data['max']:.1f}°, center={center:.1f}°")

    # Check for valid ranges
    invalid_joints = []
    for joint_name in joints_to_calibrate:
        data = min_max_data[joint_name]
        if data["min"] >= data["max"] or data["min"] == 360.0:
            invalid_joints.append(joint_name)
    
    if invalid_joints:
        print(f"\nERROR: No valid angles recorded for joints: {invalid_joints}")
        print("This usually means:")
        print("- Servos are not powered or connected")
        print("- Serial port is wrong")
        print("- You didn't move the joints during recording")
        print("Fix and try again.")
        exit(1)

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

    for joint_name in joints_to_calibrate:
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

    with open("calibration.json", "w") as f:
        json.dump(ticks_cal, f, indent=2)

    print("\n✓ Calibration saved to:")
    print("  - calibration.json")

def physical_to_logical(physical_deg, joint_name):
    """Convert physical degrees to logical degrees using calibration"""
    if not raw_calibration or joint_name not in raw_calibration:
        return physical_deg  # Fallback to physical if no calibration
    
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

# ...existing code...

while True:
    try:
        cmd = input("Enter command: ").strip()
        
        if cmd.lower() == 'q':
            print("Exiting...")
            break
        elif cmd.lower() == 'scan':
            connected_servos = scan_servos()
        elif cmd.lower().startswith('angle '):
            try:
                parts = cmd.split()
                servo_id = int(parts[1])
                angle = float(parts[2])
                if 0 <= angle <= 360:
                    enable_servo(servo_id)
                    time.sleep(0.1)
                    set_angle(servo_id, angle)
                    time.sleep(0.5)
                    current = get_angle(servo_id)
                    if current is not None:
                        print(f"Current angle: {current:.1f}°")
                    else:
                        print("Could not read angle")
                else:
                    print("Please enter 0-360")
            except (ValueError, IndexError):
                print("Usage: 'angle <id> <degrees>' (e.g., 'angle 1 90')")
        elif cmd.lower().startswith('pos '):
            try:
                servo_id = int(cmd.split()[1])
                enable_servo(servo_id)
                time.sleep(0.1)
                angle = get_angle(servo_id)
                if angle is not None:
                    print(f"Servo {servo_id}: Current angle: {angle:.1f}°")
                else:
                    print(f"Could not read servo {servo_id}")
            except (ValueError, IndexError):
                print("Usage: 'pos <id>' (e.g., 'pos 1')")
        elif cmd.lower().startswith('all '):
            try:
                angle = float(cmd.split()[1])
                if 0 <= angle <= 360:
                    print(f"Setting all servos to {angle}°...")
                    for servo_id in servo_ids:
                        enable_servo(servo_id)
                        time.sleep(0.05)
                        set_angle(servo_id, angle)
                        time.sleep(0.1)
                    time.sleep(1)
                    print("All servos set.")
                else:
                    print("Please enter 0-360")
            except (ValueError, IndexError):
                print("Usage: 'all <degrees>' (e.g., 'all 90')")
        elif cmd.lower().startswith('id '):
            try:
                parts = cmd.split()
                current_id = int(parts[1])
                new_id = int(parts[2])
                set_servo_id(current_id, new_id)
            except (ValueError, IndexError):
                print("Usage: 'id <old_id> <new_id>' (e.g., 'id 1 2')")
        elif cmd.lower() == 'broadcast':
            check_broadcast()
        elif cmd.lower() == 'readid':
            read_actual_id()
        elif cmd.lower() == 'calload':
            cal_cache = load_calibration(cal_path)
            if cal_cache:
                print("Calibration loaded.")
        elif cmd.lower().startswith('calpath '):
            try:
                cal_path = cmd.split(maxsplit=1)[1].strip()
                cal_cache = load_calibration(cal_path)
                if cal_cache:
                    print(f"Calibration loaded from: {cal_path}")
            except IndexError:
                print("Usage: calpath <file>")
        elif cmd.lower() == 'calinfo':
            if not cal_cache:
                print("Load calibration first with 'calload'")
            else:
                print("Calibration keys:", list(cal_cache.keys()))
                names = get_joint_names_from_cal(cal_cache)
                limits = get_joint_limits_from_cal(cal_cache)
                centers = get_joint_centers_from_cal(cal_cache)
                print(f"joint_names ({len(names)}): {names}")
                print(f"joint_limits keys: {list(limits.keys())}")
                print(f"joint_offsets keys: {list(centers.keys())}")
        elif cmd.lower() == 'callimits':
            if not cal_cache:
                print("Load calibration first with 'calload'")
            else:
                names = get_joint_names_from_cal(cal_cache)
                limits = get_joint_limits_from_cal(cal_cache)
                print("Joint limits:")
                for i, n in enumerate(names, start=1):
                    lim = limits.get(n)
                    if lim:
                        print(f"  {i}:{n}: min={lim['min']:.1f}°, max={lim['max']:.1f}°")
                    else:
                        print(f"  {i}:{n}: no limits recorded")
        elif cmd.lower() == 'calcenter':
            if not cal_cache:
                print("Load calibration first with 'calload'")
            else:
                move_all_to_center(cal_cache)
        elif cmd.lower().startswith('safe '):
            try:
                parts = cmd.split()
                sid = int(parts[1]); deg = float(parts[2])
                if not cal_cache:
                    print("Load calibration first with 'calload'")
                else:
                    safe_move(sid, deg, cal_cache)
            except (ValueError, IndexError):
                print("Usage: safe <id> <deg>")
        elif cmd.lower().startswith('path '):
            if not cal_cache:
                print("Load calibration first with 'calload'")
            else:
                try:
                    # Format: path a1 a2 a3 a4 a5 a6 -> b1 b2 b3 b4 b5 b6
                    raw = cmd[5:].strip()
                    left, right = raw.split("->")
                    a = [float(x) for x in left.strip().split()]
                    b = [float(x) for x in right.strip().split()]
                    if len(a) != 6 or len(b) != 6:
                        print("Provide 6 angles for start and goal")
                    else:
                        p = plan_joint_path(a, b, steps=20)
                        execute_joint_path(p, cal_cache)
                except Exception as e:
                    print(f"Path parse error: {e}")
        elif cmd.lower().startswith('torque '):
            parts = cmd.split()
            if len(parts) < 3:
                print("Usage: torque on <id> | torque off <id> | torque off all")
            else:
                action = parts[1].lower()
                target = parts[2].lower()
                if action == 'on':
                    try:
                        sid = int(target)
                        enable_servo(sid)
                        print(f"Torque enabled on ID {sid}")
                    except ValueError:
                        print("Usage: torque on <id>")
                elif action == 'off':
                    if target == 'all':
                        for sid in servo_ids:
                            disable_servo(sid)
                            time.sleep(0.05)
                        print("Torque disabled on all servos")
                    else:
                        try:
                            sid = int(target)
                            disable_servo(sid)
                            print(f"Torque disabled on ID {sid}")
                        except ValueError:
                            print("Usage: torque off <id> or torque off all")
                else:
                    print("Usage: torque on <id> | torque off <id> | torque off all")
        elif cmd.lower() == 'monitor_angles':
            if not cal_cache:
                print("Load calibration first with 'calload'")
            else:
                print("Disabling torque on all servos - move the arm manually.")
                for sid in servo_ids:
                    disable_servo(sid)
                    time.sleep(0.05)
                print("Torque disabled. Monitoring angles live (press ENTER to stop):")
                print("-" * 80)
                print(f"{'Joint':<15} | {'Physical (°)':<12} | {'Logical (°)':<12}")
                print("-" * 80)
                
                try:
                    import sys
                    import select
                    
                    while True:
                        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                            input()
                            break
                        
                        print("\r", end="")
                        for i, joint_name in enumerate(get_joint_names_from_cal(cal_cache), start=1):
                            physical = get_angle(i)
                            if physical is not None:
                                logical = physical_to_logical(physical, joint_name)
                                print(f"{joint_name:<15} | {physical:>11.1f} | {logical:>11.1f} | ", end="")
                            else:
                                print(f"{joint_name:<15} | {'N/A':>11} | {'N/A':>11} | ", end="")
                        time.sleep(0.1)
                except Exception:
                    input("Press ENTER to stop monitoring:")
                
                print("\nMonitoring stopped. Re-enabling torque...")
                for sid in servo_ids:
                    enable_servo(sid)
                    time.sleep(0.05)
                print("Torque re-enabled.")
        elif cmd.lower() == 'angles':
            if not cal_cache:
                print("Load calibration first with 'calload'")
            else:
                print("Current angles (joint|physical|logical):")
                for i, joint_name in enumerate(get_joint_names_from_cal(cal_cache), start=1):
                    physical = get_angle(i)
                    if physical is not None:
                        logical = physical_to_logical(physical, joint_name)
                        print(f"{joint_name}|{physical:.1f}|{logical:.1f}")
                    else:
                        print(f"{joint_name}|N/A|N/A")
        else:
            print("Invalid command. Type 'q' to quit.")
    except KeyboardInterrupt:
        print("\nExiting...")
        break

# ...existing code...