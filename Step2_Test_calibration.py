import serial
import serial.tools.list_ports
import time
import json
import os

BAUD = 1000000

def find_serial_port():
    """Auto-detect available serial ports and connect to the robot with verified baud rate."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        raise RuntimeError("No serial port found! Connect your robot.")
    
    print(f"Available ports: {[p.device for p in ports]}")
    
    # Try each port with different baud rates (reverse order to try higher speeds first)
    baud_rates = [1000000, 921600, 460800, 230400, 115200]
    
    # Try COM4 first if available
    ports_to_try = sorted(ports, key=lambda p: p.device, reverse=True)
    
    for port in ports_to_try:
        for baud in baud_rates:
            try:
                print(f"Trying {port.device} at {baud} baud...", end=" ", flush=True)
                test_ser = serial.Serial(port.device, baud, timeout=0.5)
                time.sleep(0.2)
                
                # Verify communication by scanning for servos
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
                    print("✓ Connected and verified!")
                    return test_ser, port.device, baud
                else:
                    test_ser.close()
                    print("✗ (no servos responded)")
                    continue
            except Exception as e:
                print("✗")
                continue
    
    raise RuntimeError("Could not connect to any serial port with responding servos!")

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
    time.sleep(0.005)  # Minimal delay for 1000000 baud
    ser.read(20)

def read_servo(servo_id, address, length=1):
    """Read from servo register"""
    packet = [0xFF, 0xFF, servo_id, 4, 0x02, address, length]
    checksum = (~sum(packet[2:])) & 0xFF
    packet.append(checksum)
    
    # Clear any garbage in the buffer first
    ser.reset_input_buffer()
    
    ser.write(bytes(packet))
    time.sleep(0.02)  # Minimal delay for 1000000 baud
    
    # Read response
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
    print(f"Servo {servo_id}: Set to {angle_degrees}°")

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

def set_servo_id(current_id, new_id):
    """Change servo ID with EEPROM unlock"""
    write_servo(current_id, 0x37, 0x00)
    time.sleep(0.1)
    write_servo(current_id, 0x05, new_id)
    time.sleep(0.1)
    write_servo(current_id, 0x37, 0x01)
    time.sleep(0.3)
    packet = [0xFF, 0xFF, new_id, 2, 0x01]
    checksum = (~sum(packet[2:])) & 0xFF
    packet.append(checksum)
    ser.reset_input_buffer()
    ser.write(bytes(packet))
    time.sleep(0.1)
    resp = ser.read(20)
    if len(resp) > 0:
        print(f"✓ Servo ID changed from {current_id} to {new_id}")
    else:
        print(f"✗ ID change failed. Try power cycling and running 'scan'")

def scan_servos():
    """Scan for all connected servos"""
    print("Scanning for servos (IDs 1-6)...")
    found_ids = []
    for servo_id in range(1, 7):
        packet = [0xFF, 0xFF, servo_id, 2, 0x01]
        checksum = (~sum(packet[2:])) & 0xFF
        packet.append(checksum)
        ser.write(bytes(packet))
        time.sleep(0.1)
        resp = ser.read(20)
        if len(resp) > 0:
            found_ids.append(servo_id)
            print(f"  ✓ Found servo with ID {servo_id}")
        else:
            print(f"  ✗ No response from ID {servo_id}")
    
    if found_ids:
        print(f"\n✓ Found {len(found_ids)} servo(s): {found_ids}\n")
        return found_ids
    else:
        print("✗ No servos found. Check connections and power.\n")
        return []

def check_broadcast():
    """Check if broadcast mode is enabled"""
    print("Checking broadcast settings...")
    
    # Try reading from each servo to see if they all respond the same
    for test_id in [1, 2, 3, 4]:
        packet = [0xFF, 0xFF, test_id, 2, 0x01]
        checksum = (~sum(packet[2:])) & 0xFF
        packet.append(checksum)
        ser.reset_input_buffer()
        ser.write(bytes(packet))
        time.sleep(0.1)
        resp = ser.read(20)
        
        if len(resp) > 0:
            print(f"ID {test_id}: Got response - {resp.hex()}")
        else:
            print(f"ID {test_id}: No response")

def read_actual_id():
    """Read the servo's actual ID from register 0x05"""
    print("Reading actual servo ID...")
    
    # First ping to make sure servo is there
    ping_packet = [0xFF, 0xFF, 1, 2, 0x01]
    checksum = (~sum(ping_packet[2:])) & 0xFF
    ping_packet.append(checksum)
    ser.reset_input_buffer()
    ser.write(bytes(ping_packet))
    time.sleep(0.1)
    ping_resp = ser.read(20)
    print(f"Ping response: {ping_resp.hex() if ping_resp else 'NONE'}")
    
    if len(ping_resp) > 2:
        responding_id = ping_resp[2]
        print(f"Servo responding as ID: {responding_id}")
    
    # Now try read
    ser.reset_input_buffer()
    packet = [0xFF, 0xFF, 1, 4, 0x02, 0x05, 1]
    checksum = (~sum(packet[2:])) & 0xFF
    packet.append(checksum)
    print(f"Sending read packet: {bytes(packet).hex()}")
    ser.write(bytes(packet))
    time.sleep(0.2)
    resp = ser.read(20)
    print(f"Read response: {resp.hex() if resp else 'NONE'}")
    
    if len(resp) > 5:
        actual_id = resp[5]
        print(f"Servo's stored ID (register 0x05): {actual_id}")
    else:
        print("Read failed - servo may not support register reads")

# --- Calibration helpers ---
raw_calibration = None

def load_calibration(path="calibration.json"):
    """Load calibration JSON. Converts from ticks format to internal format."""
    global raw_calibration
    if not os.path.exists(path):
        print(f"Calibration file not found: {path}")
        return None
    print(f"Loading calibration from: {path}")
    with open(path, "r") as f:
        raw = json.load(f)
    
    # Store raw calibration for drive_mode access
    raw_calibration = raw

    # Convert from ticks-based dict (the source of truth)
    def ticks_to_deg(t): return (float(t) / 4095.0) * 360.0

    joint_order = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
    joint_limits = {}
    joint_offsets = {}
    joint_names = []

    for name in joint_order:
        if name in raw:
            j = raw[name]
            joint_names.append(name)
            if "range_min" in j and "range_max" in j:
                min_deg = ticks_to_deg(j["range_min"])
                max_deg = ticks_to_deg(j["range_max"])
                joint_limits[name] = {"min": min_deg, "max": max_deg}
                # Use homing_offset as center if available, else use midpoint
                if "homing_offset" in j and j["homing_offset"] is not None:
                    center_deg = ticks_to_deg(j["homing_offset"])
                else:
                    center_deg = (min_deg + max_deg) / 2.0
                joint_offsets[name] = center_deg
            else:
                joint_limits[name] = {"min": 0.0, "max": 360.0}
                joint_offsets[name] = 180.0
        else:
            joint_names.append(name)
            joint_limits[name] = {"min": 0.0, "max": 360.0}
            joint_offsets[name] = 180.0

    converted = {
        "robot_type": "so100",
        "robot_variant": "follower",
        "joint_names": joint_names,
        "joint_limits": joint_limits,
        "joint_offsets": joint_offsets,
        "port": PORT,
        "baudrate": BAUD,
    }
    print("Converted ticks-based calibration to internal format.")
    return converted

def get_joint_names_from_cal(cal):
    return cal.get("joint_names", [])

def get_joint_limits_from_cal(cal):
    return cal.get("joint_limits", {})

def get_joint_centers_from_cal(cal):
    return cal.get("joint_offsets", {})

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
        # Invert: 360 - angle
        inverted_angle = 360.0 - angle_degrees
        position = int((inverted_angle / 360.0) * 4095)
        print(f"Servo {servo_id} ({joint_name}): Target {angle_degrees}° -> Inverted to {inverted_angle}° (drive_mode=1)")
    else:
        position = int((angle_degrees / 360.0) * 4095)
        print(f"Servo {servo_id} ({joint_name}): Set to {angle_degrees}°")
    
    write_servo(servo_id, 0x2A, position, 2)

def logical_to_physical(logical_deg, joint_name):
    """Convert logical degrees to physical degrees using calibration"""
    if not raw_calibration or joint_name not in raw_calibration:
        return logical_deg  # Fallback to logical if no calibration
    
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

def move_joint_to_center(servo_id, cal):
    joint_names = get_joint_names_from_cal(cal)
    centers = get_joint_centers_from_cal(cal)
    name = joint_names[servo_id - 1] if 0 < servo_id <= len(joint_names) else f"id_{servo_id}"
    center = centers.get(name)
    if center is None:
        print(f"No center offset for {name}")
        return
    enable_servo(servo_id)
    time.sleep(0.1)
    
    # Convert degrees to ticks for display
    target_ticks = int((center / 360.0) * 4095)
    current_angle = get_angle(servo_id)
    current_ticks = int((current_angle / 360.0) * 4095) if current_angle is not None else 0
    
    print(f"Moving {name} (ID {servo_id}):")
    print(f"  Target: {center:.1f}° ({target_ticks} ticks)")
    if current_angle is not None:
        print(f"  Current: {current_angle:.1f}° ({current_ticks} ticks)")
    else:
        print(f"  Current: N/A ({current_ticks} ticks)")
    
    # Use inversion-aware function
    set_angle_with_inversion(servo_id, center, name)
    
    time.sleep(0.5)
    final_angle = get_angle(servo_id)
    final_ticks = int((final_angle / 360.0) * 4095) if final_angle is not None else 0
    if final_angle is not None:
        print(f"  Final: {final_angle:.1f}° ({final_ticks} ticks)")
    else:
        print(f"  Final: N/A ({final_ticks} ticks)")

def move_all_to_center(cal):
    joint_names = get_joint_names_from_cal(cal)
    for i, name in enumerate(joint_names, start=1):
        move_joint_to_center(i, cal)
        time.sleep(0.2)

# --- Safe small moves within limits ---
def safe_move(servo_id, target_deg, cal):
    joint_names = get_joint_names_from_cal(cal)
    limits = get_joint_limits_from_cal(cal)
    name = joint_names[servo_id - 1] if 0 < servo_id <= len(joint_names) else f"id_{servo_id}"
    lim = limits.get(name)
    if not lim:
        print(f"No limits for {name}, sending as-is")
        enable_servo(servo_id)
        time.sleep(0.05)
        set_angle_with_inversion(servo_id, target_deg, name)
        return
    min_deg = lim["min"]
    max_deg = lim["max"]
    clamped = max(min_deg, min(max_deg, target_deg))
    enable_servo(servo_id)
    time.sleep(0.05)
    set_angle_with_inversion(servo_id, clamped, name)
    print(f"{name}: requested {target_deg:.1f}°, clamped [{min_deg:.1f}, {max_deg:.1f}] -> {clamped:.1f}°")

# --- Simple joint-space path generation ---
def plan_joint_path(start_angles, goal_angles, steps=20):
    """Linear interpolation in joint space."""
    if len(start_angles) != len(goal_angles):
        raise ValueError("start_angles and goal_angles must have same length")
    path = []
    for s in range(steps + 1):
        t = s / steps
        wp = [(1 - t) * sa + t * ga for sa, ga in zip(start_angles, goal_angles)]
        path.append(wp)
    return path

def execute_joint_path(path, cal):
    """Execute a joint-space path on hardware."""
    joint_names = get_joint_names_from_cal(cal)
    for wp in path:
        for i, angle in enumerate(wp, start=1):
            safe_move(i, angle, cal)
        time.sleep(0.2)
    print(f"Executed path with {len(path)} waypoints")

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

# Interactive control
print("=== SO ARM 101 Servo Control (6 Servos) ===")
print("Commands:")
print("  scan                  - Scan for connected servos")
print("  angle <id> <degrees>  - Set servo angle (e.g., 'angle 1 90')")
print("  pos <id>              - Read servo angle (e.g., 'pos 1')")
print("  all <degrees>         - Set all servos to same angle (e.g., 'all 90')")
print("  id <old> <new>        - Change servo ID (e.g., 'id 1 2')")
print("  broadcast             - Check if broadcast mode is enabled")
print("  readid                - Read servo's actual stored ID")
print("  q                     - Quit\n")

print("Extra commands:")
print("  calload             - Load calibration JSON")
print("  calpath <file>      - Set and load a specific calibration JSON path")
print("  calinfo             - Show calibration keys and summary")
print("  callimits           - Print joint names and limits")
print("  calcenter           - Move all joints to center")
print("  safe <id> <deg>     - Safe move clamped to limits")
print("  safelogical <id> <logical_deg> - Safe move using logical degrees")
print("  path <a..> -> <b..> - Plan & execute joint-space path (6 angles each)")
print("  torque on <id>       - Enable torque for a servo")
print("  torque off <id>      - Disable torque for a servo")
print("  torque off all       - Disable torque for all servos")

servo_ids = [1, 2, 3, 4, 5, 6]
connected_servos = []
cal_cache = None
cal_path = "calibration.json"

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
        elif cmd.lower().startswith('safelogical '):
            try:
                parts = cmd.split()
                sid = int(parts[1]); logical_deg = float(parts[2])
                if not cal_cache:
                    print("Load calibration first with 'calload'")
                else:
                    joint_names = get_joint_names_from_cal(cal_cache)
                    name = joint_names[sid - 1]
                    physical_deg = logical_to_physical(logical_deg, name)
                    print(f"Logical {logical_deg}° -> Physical {physical_deg:.1f}° for {name}")
                    safe_move(sid, physical_deg, cal_cache)
            except (ValueError, IndexError):
                print("Usage: safelogical <id> <logical_deg>")
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
