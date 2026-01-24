import streamlit as st
import serial
import time
import json
import os

# Constants
PORT = "/dev/tty.usbmodem5AE60848961"
BAUD = 1000000

@st.cache_resource
def get_serial():
    return serial.Serial(PORT, BAUD, timeout=0.5)

ser = get_serial()

# --- Servo functions (updated with ser parameter) ---
def write_servo(ser, servo_id, address, value, length=1):
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

def read_servo(ser, servo_id, address, length=1):
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

def set_angle(ser, servo_id, angle_degrees):
    position = int((angle_degrees / 360.0) * 4095)
    write_servo(ser, servo_id, 0x2A, position, 2)

def get_angle(ser, servo_id):
    pos = read_servo(ser, servo_id, 0x38, 2)
    if pos is not None:
        angle = (pos / 4095.0) * 360.0
        return angle
    return None

def enable_servo(ser, servo_id):
    write_servo(ser, servo_id, 0x28, 1)

def disable_servo(ser, servo_id):
    write_servo(ser, servo_id, 0x28, 0)

def set_servo_id(ser, current_id, new_id):
    write_servo(ser, current_id, 0x37, 0x00)
    time.sleep(0.1)
    write_servo(ser, current_id, 0x05, new_id)
    time.sleep(0.1)
    write_servo(ser, current_id, 0x37, 0x01)
    time.sleep(0.3)
    packet = [0xFF, 0xFF, new_id, 2, 0x01]
    checksum = (~sum(packet[2:])) & 0xFF
    packet.append(checksum)
    ser.reset_input_buffer()
    ser.write(bytes(packet))
    time.sleep(0.1)
    resp = ser.read(20)
    if len(resp) > 0:
        st.write(f"✓ Servo ID changed from {current_id} to {new_id}")
    else:
        st.write(f"✗ ID change failed")

def scan_servos(ser):
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
    return found_ids

def check_broadcast(ser):
    for test_id in [1, 2, 3, 4]:
        packet = [0xFF, 0xFF, test_id, 2, 0x01]
        checksum = (~sum(packet[2:])) & 0xFF
        packet.append(checksum)
        ser.reset_input_buffer()
        ser.write(bytes(packet))
        time.sleep(0.1)
        resp = ser.read(20)
        st.write(f"ID {test_id}: {resp.hex() if resp else 'NONE'}")

def read_actual_id(ser):
    ping_packet = [0xFF, 0xFF, 1, 2, 0x01]
    checksum = (~sum(ping_packet[2:])) & 0xFF
    ping_packet.append(checksum)
    ser.reset_input_buffer()
    ser.write(bytes(ping_packet))
    time.sleep(0.1)
    ping_resp = ser.read(20)
    st.write(f"Ping: {ping_resp.hex() if ping_resp else 'NONE'}")
    ser.reset_input_buffer()
    packet = [0xFF, 0xFF, 1, 4, 0x02, 0x05, 1]
    checksum = (~sum(packet[2:])) & 0xFF
    packet.append(checksum)
    ser.write(bytes(packet))
    time.sleep(0.2)
    resp = ser.read(20)
    if len(resp) > 5:
        actual_id = resp[5]
        st.write(f"Stored ID: {actual_id}")
    else:
        st.write("Read failed")

# --- Calibration helpers ---
raw_calibration = None

def load_calibration(path="calibration.json"):
    global raw_calibration
    if not os.path.exists(path):
        st.write(f"File not found: {path}")
        return None
    with open(path, "r") as f:
        raw = json.load(f)
    raw_calibration = raw
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
                joint_limits[name] = {"min": min(min_deg, max_deg), "max": max(min_deg, max_deg)}
                if "homing_offset" in j:
                    center_deg = ticks_to_deg(j["homing_offset"])
                else:
                    center_deg = (min_deg + max_deg) / 2.0
                joint_offsets[name] = center_deg
    converted = {
        "joint_names": joint_names,
        "joint_limits": joint_limits,
        "joint_offsets": joint_offsets,
    }
    return converted

def get_joint_names_from_cal(cal):
    return cal.get("joint_names", [])

def get_joint_limits_from_cal(cal):
    return cal.get("joint_limits", {})

def get_drive_mode(joint_name):
    global raw_calibration
    if raw_calibration and joint_name in raw_calibration:
        return raw_calibration[joint_name].get("drive_mode", 0)
    return 0

def set_angle_with_inversion(ser, servo_id, angle_degrees, joint_name):
    drive_mode = get_drive_mode(joint_name)
    if drive_mode == 1:
        inverted_angle = 360.0 - angle_degrees
        position = int((inverted_angle / 360.0) * 4095)
        st.write(f"Servo {servo_id} ({joint_name}): Target {angle_degrees}° -> Inverted to {inverted_angle}°")
    else:
        position = int((angle_degrees / 360.0) * 4095)
    write_servo(ser, servo_id, 0x2A, position, 2)

def logical_to_physical(logical_deg, joint_name):
    global raw_calibration
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

def move_joint_to_center(ser, servo_id, cal):
    joint_names = get_joint_names_from_cal(cal)
    centers = get_joint_centers_from_cal(cal)
    name = joint_names[servo_id - 1]
    center = centers.get(name)
    if center is None:
        st.write(f"No center for {name}")
        return
    enable_servo(ser, servo_id)
    time.sleep(0.1)
    set_angle_with_inversion(ser, servo_id, center, name)
    time.sleep(0.5)

def move_all_to_center(ser, cal):
    joint_names = get_joint_names_from_cal(cal)
    for i, name in enumerate(joint_names, start=1):
        move_joint_to_center(ser, i, cal)
        time.sleep(0.2)

def safe_move(ser, servo_id, target_deg, cal):
    joint_names = get_joint_names_from_cal(cal)
    limits = get_joint_limits_from_cal(cal)
    name = joint_names[servo_id - 1]
    lim = limits.get(name)
    if not lim:
        enable_servo(ser, servo_id)
        time.sleep(0.05)
        set_angle_with_inversion(ser, servo_id, target_deg, name)
        return
    min_deg = lim["min"]
    max_deg = lim["max"]
    clamped = max(min_deg, min(max_deg, target_deg))
    enable_servo(ser, servo_id)
    time.sleep(0.05)
    set_angle_with_inversion(ser, servo_id, clamped, name)
    st.write(f"{name}: requested {target_deg:.1f}°, clamped [{min_deg:.1f}, {max_deg:.1f}] -> {clamped:.1f}°")

def plan_joint_path(start_angles, goal_angles, steps=20):
    if len(start_angles) != len(goal_angles):
        raise ValueError("Lengths must match")
    path = []
    for s in range(steps + 1):
        t = s / steps
        wp = [(1 - t) * sa + t * ga for sa, ga in zip(start_angles, goal_angles)]
        path.append(wp)
    return path

def execute_joint_path(ser, path, cal):
    for wp in path:
        for i, angle in enumerate(wp, start=1):
            safe_move(ser, i, angle, cal)
        time.sleep(0.2)

def physical_to_logical(physical_deg, joint_name):
    global raw_calibration
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

def get_joint_centers_from_cal(cal):
    return cal.get("joint_offsets", {})

# --- Streamlit UI ---
st.title("SO-100 Arm Servo Control")
st.write("Connected to servo controller")

tab1, tab2, tab3 = st.tabs(["Basic Control", "Calibration", "Advanced"])

with tab1:
    st.header("Basic Control")
    if st.button("Scan Servos"):
        connected = scan_servos(ser)
        st.write("Connected servos:", connected)
    
    col1, col2 = st.columns(2)
    with col1:
        servo_id = st.number_input("Servo ID", 1, 6, 1, key="servo_id")
        angle = st.slider("Angle", 0.0, 360.0, 90.0, key="angle")
        if st.button("Set Angle"):
            enable_servo(ser, servo_id)
            time.sleep(0.1)
            set_angle(ser, servo_id, angle)
            time.sleep(0.5)
            current = get_angle(ser, servo_id)
            st.write(f"Set to {angle}°, current: {current}°")
    
    with col2:
        if st.button("Get Position"):
            enable_servo(ser, servo_id)
            time.sleep(0.1)
            angle = get_angle(ser, servo_id)
            st.write(f"Position: {angle}°")
    
    all_angle = st.slider("All Servos Angle", 0.0, 360.0, 90.0)
    if st.button("Set All"):
        for sid in range(1, 7):
            enable_servo(ser, sid)
            time.sleep(0.05)
            set_angle(ser, sid, all_angle)
        st.write("All set")
    
    current_id = st.number_input("Current ID", 1, 6, 1)
    new_id = st.number_input("New ID", 1, 6, 2)
    if st.button("Change ID"):
        set_servo_id(ser, current_id, new_id)
    
    if st.button("Broadcast Check"):
        check_broadcast(ser)
    
    if st.button("Read ID"):
        read_actual_id(ser)

with tab2:
    st.header("Calibration")
    if st.button("Load Calibration"):
        cal_cache = load_calibration("calibration.json")
        st.session_state.cal_cache = cal_cache
        st.write("Calibration loaded")
    
    cal_path = st.text_input("Calibration Path", "calibration.json")
    if st.button("Load Custom Path"):
        cal_cache = load_calibration(cal_path)
        st.session_state.cal_cache = cal_cache
        st.write(f"Loaded from {cal_path}")
    
    if st.button("Calibration Info"):
        if 'cal_cache' in st.session_state:
            cal = st.session_state.cal_cache
            st.write("Keys:", list(cal.keys()))
            names = get_joint_names_from_cal(cal)
            st.write("Names:", names)
        else:
            st.write("Load calibration first")
    
    if st.button("Limits"):
        if 'cal_cache' in st.session_state:
            names = get_joint_names_from_cal(st.session_state.cal_cache)
            limits = get_joint_limits_from_cal(st.session_state.cal_cache)
            for i, n in enumerate(names, start=1):
                lim = limits.get(n)
                if lim:
                    st.write(f"{i}:{n}: min={lim['min']:.1f}, max={lim['max']:.1f}")
        else:
            st.write("Load calibration first")
    
    if st.button("Move to Center"):
        if 'cal_cache' in st.session_state:
            move_all_to_center(ser, st.session_state.cal_cache)
            st.write("Moved to center")
        else:
            st.write("Load calibration first")
    
    safe_id = st.number_input("Safe ID", 1, 6, 1)
    safe_deg = st.number_input("Safe Degrees", 0.0, 360.0, 90.0)
    if st.button("Safe Move"):
        if 'cal_cache' in st.session_state:
            safe_move(ser, safe_id, safe_deg, st.session_state.cal_cache)
        else:
            st.write("Load calibration first")
    
    logical_deg = st.number_input("Logical Degrees", 0.0, 180.0, 90.0)
    if st.button("Safe Logical"):
        if 'cal_cache' in st.session_state:
            joint_names = get_joint_names_from_cal(st.session_state.cal_cache)
            name = joint_names[safe_id - 1]
            physical_deg = logical_to_physical(logical_deg, name)
            st.write(f"Logical {logical_deg}° -> Physical {physical_deg:.1f}°")
            safe_move(ser, safe_id, physical_deg, st.session_state.cal_cache)
        else:
            st.write("Load calibration first")
    
    if st.button("Monitor Angles"):
        if 'cal_cache' in st.session_state:
            monitor_placeholder = st.empty()
            for sid in range(1, 7):
                disable_servo(ser, sid)
            st.write("Torque disabled, monitoring...")
            for _ in range(100):  # ~10 seconds
                lines = []
                for i, joint_name in enumerate(get_joint_names_from_cal(st.session_state.cal_cache), start=1):
                    physical = get_angle(ser, i)
                    if physical is not None:
                        logical = physical_to_logical(physical, joint_name)
                        lines.append(f"{joint_name}: Physical {physical:.1f}°, Logical {logical:.1f}°")
                    else:
                        lines.append(f"{joint_name}: N/A")
                monitor_placeholder.text("\n".join(lines))
                time.sleep(0.1)
            for sid in range(1, 7):
                enable_servo(ser, sid)
            st.write("Monitoring stopped, torque re-enabled")
        else:
            st.write("Load calibration first")

with tab3:
    st.header("Advanced")
    path_input = st.text_input("Path", "90 90 90 90 90 90 -> 180 180 180 180 180 180")
    if st.button("Execute Path"):
        if 'cal_cache' in st.session_state:
            try:
                raw = path_input.strip()
                left, right = raw.split("->")
                a = [float(x) for x in left.strip().split()]
                b = [float(x) for x in right.strip().split()]
                if len(a) == 6 and len(b) == 6:
                    p = plan_joint_path(a, b, steps=20)
                    execute_joint_path(ser, p, st.session_state.cal_cache)
                    st.write("Path executed")
                else:
                    st.write("Provide 6 angles each")
            except Exception as e:
                st.write(f"Error: {e}")
        else:
            st.write("Load calibration first")
    
    torque_id = st.selectbox("Torque ID", ["all"] + [str(i) for i in range(1, 7)])
    torque_action = st.selectbox("Action", ["on", "off"])
    if st.button("Torque"):
        if torque_id == "all":
            for sid in range(1, 7):
                if torque_action == "on":
                    enable_servo(ser, sid)
                else:
                    disable_servo(ser, sid)
            st.write(f"Torque {torque_action} for all")
        else:
            sid = int(torque_id)
            if torque_action == "on":
                enable_servo(ser, sid)
            else:
                disable_servo(ser, sid)
            st.write(f"Torque {torque_action} for {sid}")
    
    if st.button("Current Angles"):
        if 'cal_cache' in st.session_state:
            lines = []
            for i, joint_name in enumerate(get_joint_names_from_cal(st.session_state.cal_cache), start=1):
                physical = get_angle(ser, i)
                if physical is not None:
                    logical = physical_to_logical(physical, joint_name)
                    lines.append(f"{joint_name}: {physical:.1f}° physical, {logical:.1f}° logical")
                else:
                    lines.append(f"{joint_name}: N/A")
            st.text("\n".join(lines))
        else:
            st.write("Load calibration first")
