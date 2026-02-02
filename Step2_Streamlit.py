import streamlit as st
import serial
import serial.tools.list_ports
import time
import json
import os

# Constants
BAUD = 1000000

def find_serial_port():
    """Auto-detect available serial ports and try different baud rates."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        return None, None, None
    
    # Try each port with different baud rates
    baud_rates = [1000000, 921600, 460800, 230400, 115200]
    for port in ports:
        for baud in baud_rates:
            try:
                print(f"Trying {port.device} at {baud} baud...", end=" ")
                test_ser = serial.Serial(port.device, baud, timeout=0.5)
                test_ser.close()
                print("✓")
                return port.device, baud, test_ser
            except:
                print("✗")
    return None, None, None

PORT, BAUD, _ = find_serial_port()

@st.cache_resource
def get_serial():
    if PORT is None:
        st.error("No serial port found! Please connect your robot.")
        st.stop()
    try:
        return serial.Serial(PORT, BAUD, timeout=0.5)
    except serial.SerialException as e:
        st.error(f"Failed to connect to serial port {PORT} at {BAUD} baud: {e}")
        st.stop()

ser = get_serial()

# --- Servo functions ---
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
                joint_limits[name] = {"min": min_deg, "max": max_deg}
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

def logical_to_physical(logical_deg, joint_name, cal):
    limits = cal.get("joint_limits", {}).get(joint_name, {})
    physical_min = limits.get("min", 0)
    physical_max = limits.get("max", 360)
    ratio = logical_deg / 180.0
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

def get_joint_centers_from_cal(cal):
    return cal.get("joint_offsets", {})

# --- Streamlit UI ---
st.title("SO-100 Arm Servo Control")
st.write("Connected to servo controller")

st.header("Calibration")

if st.button("Load Calibration"):
    cal_cache = load_calibration("calibration.json")
    st.session_state.cal_cache = cal_cache
    st.write("Calibration loaded")

if st.button("Move to Center"):
    if 'cal_cache' in st.session_state:
        move_all_to_center(ser, st.session_state.cal_cache)
        st.write("Moved to center")
    else:
        st.write("Load calibration first")

st.subheader("Safe Move")
safe_id = st.number_input("Safe ID", 1, 6, 1, key="safe_id")
safe_deg = st.number_input("Safe Degrees", 0.0, 360.0, 90.0, key="safe_deg")
if st.button("Safe Move"):
    if 'cal_cache' in st.session_state:
        safe_move(ser, safe_id, safe_deg, st.session_state.cal_cache)
    else:
        st.write("Load calibration first")

st.subheader("Safe Logical")
safe_logical_id = st.number_input("Safe Logical ID", 1, 6, 1, key="safe_logical_id")
safe_logical_deg = st.number_input("Logical Degrees", 0.0, 180.0, 90.0, key="safe_logical_deg")
if st.button("Safe Logical Move"):
    if 'cal_cache' in st.session_state:
        joint_names = get_joint_names_from_cal(st.session_state.cal_cache)
        name = joint_names[safe_logical_id - 1]
        physical_deg = logical_to_physical(safe_logical_deg, name, st.session_state.cal_cache)
        st.write(f"Logical {safe_logical_deg}° -> Physical {physical_deg:.1f}° for {name}")
        safe_move(ser, safe_logical_id, physical_deg, st.session_state.cal_cache)
    else:
        st.write("Load calibration first")

if st.button("Torque Off All"):
    for sid in range(1, 7):
        disable_servo(ser, sid)
    st.write("Torque disabled on all servos")