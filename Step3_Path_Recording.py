import serial
import time
import json
import os

PORT = "/dev/tty.usbmodem5AE60848961"
BAUD = 1000000

ser = serial.Serial(PORT, BAUD, timeout=0.5)
print("Connected to servo controller\n")

# Reuse existing functions from previous scripts
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

def get_angle(servo_id):
    pos = read_servo(servo_id, 0x38, 2)
    if pos is not None:
        angle = (pos / 4095.0) * 360.0
        return angle
    return None

def enable_servo(servo_id):
    write_servo(servo_id, 0x28, 1)

def disable_servo(servo_id):
    write_servo(servo_id, 0x28, 0)

def set_angle(servo_id, angle_degrees):
    position = int((angle_degrees / 360.0) * 4095)
    write_servo(servo_id, 0x2A, position, 2)

# Path management
PATHS_FILE = "paths.json"
current_path = None
current_path_name = None

def load_paths():
    if os.path.exists(PATHS_FILE):
        with open(PATHS_FILE, "r") as f:
            return json.load(f)
    return {}

def save_paths(paths):
    with open(PATHS_FILE, "w") as f:
        json.dump(paths, f, indent=2)

def start_path(name):
    global current_path, current_path_name
    current_path = []
    current_path_name = name
    print(f"Started recording path: {name}")

def edit_path(name):
    global current_path, current_path_name
    path = load_path(name)
    if path is None:
        return
    current_path = path.copy()  # Copy to avoid modifying original until saved
    current_path_name = name
    print(f"Loaded path '{name}' for editing. Current waypoints: {len(current_path)}")

def delete_path(name):
    paths = load_paths()
    if name in paths:
        del paths[name]
        save_paths(paths)
        print(f"Deleted path '{name}'")
    else:
        print(f"Path '{name}' not found.")

def add_waypoint():
    global current_path
    if current_path is None:
        print("No path started or loaded. Use 'start <name>' or 'edit <name>' first.")
        return
    angles = []
    for i in range(1, 7):  # 6 joints
        angle = get_angle(i)
        if angle is None:
            print(f"Failed to read angle for servo {i}")
            return
        angles.append(round(angle, 1))
    current_path.append(angles)
    print(f"Added waypoint: {angles} (total: {len(current_path)})")

def remove_waypoint(index):
    global current_path
    if current_path is None:
        print("No path loaded. Use 'edit <name>' first.")
        return
    try:
        idx = int(index) - 1  # 1-based to 0-based
        if 0 <= idx < len(current_path):
            removed = current_path.pop(idx)
            print(f"Removed waypoint {idx+1}: {removed} (remaining: {len(current_path)})")
        else:
            print("Invalid index.")
    except ValueError:
        print("Usage: remove <index> (e.g., remove 1)")

def save_path():
    global current_path, current_path_name
    if current_path is None or current_path_name is None:
        print("No path to save.")
        return
    paths = load_paths()
    paths[current_path_name] = current_path
    save_paths(paths)
    print(f"Saved path '{current_path_name}' with {len(current_path)} waypoints.")
    current_path = None
    current_path_name = None

def load_path(name):
    paths = load_paths()
    if name not in paths:
        print(f"Path '{name}' not found.")
        return None
    return paths[name]

# ...existing code...

def execute_path(name):
    path = load_path(name)
    if path is None:
        return
    print(f"Executing path '{name}' with {len(path)} waypoints...")
    
    # Pre-enable all servos
    for j in range(1, 7):
        enable_servo(j)
    time.sleep(0.1)
    
    prev_angles = None
    for i, waypoint in enumerate(path):
        print(f"Moving to waypoint {i+1}: {waypoint}")
        for j, angle in enumerate(waypoint, start=1):
            if prev_angles is None or angle != prev_angles[j-1]:
                set_angle(j, angle)
        prev_angles = waypoint.copy()
        time.sleep(0.5)  # Reduced from 1s, adjust as needed
    print("Path execution complete.")

# ...existing code...
def show_current_path():
    if current_path is None:
        print("No current path.")
    else:
        print(f"Current path '{current_path_name}': {len(current_path)} waypoints")
        for i, wp in enumerate(current_path, start=1):
            print(f"  {i}: {wp}")

# ...existing code...

def change_waypoint(index):
    global current_path
    if current_path is None:
        print("No path loaded. Use 'edit <name>' first.")
        return
    try:
        idx = int(index) - 1  # 1-based to 0-based
        if 0 <= idx < len(current_path):
            angles = []
            for i in range(1, 7):  # 6 joints
                angle = get_angle(i)
                if angle is None:
                    print(f"Failed to read angle for servo {i}")
                    return
                angles.append(round(angle, 1))
            current_path[idx] = angles
            print(f"Changed waypoint {idx+1} to current position: {angles}")
        else:
            print("Invalid index.")
    except ValueError:
        print("Usage: change <index> (e.g., change 1)")

# ...existing code...

# Interactive control
print("=== SO-100 Arm Path Recording & Replay ===")
print("Commands:")
print("  start <name>     - Start recording a new path")
print("  edit <name>      - Load an existing path for editing")
print("  add              - Add current position as waypoint to current path")
print("  change <index>   - Change a waypoint to current arm position")
print("  remove <index>   - Remove waypoint by index (1-based)")
print("  save             - Save the current path")
print("  rename <old> <new> - Rename a path")
print("  delete <name>    - Delete a saved path")
print("  load <name>      - Load a path (for info)")
print("  execute <name>   - Execute a saved path")
print("  list             - List all saved paths")
print("  current          - Show current path waypoints")
print("  torque off all   - Disable torque on all servos")
print("  q                - Quit")
print()

while True:
    try:
        cmd = input("Enter command: ").strip()
        
        if cmd.lower() == 'q':
            print("Exiting...")
            break
        elif cmd.lower().startswith('start '):
            name = cmd[6:].strip()
            if name:
                start_path(name)
            else:
                print("Usage: start <name>")
        elif cmd.lower().startswith('edit '):
            name = cmd[5:].strip()
            if name:
                edit_path(name)
            else:
                print("Usage: edit <name>")
        elif cmd.lower() == 'add':
            add_waypoint()
        elif cmd.lower().startswith('change '):
            index = cmd[7:].strip()
            if index:
                change_waypoint(index)
            else:
                print("Usage: change <index>")
        elif cmd.lower().startswith('remove '):
            index = cmd[7:].strip()
            if index:
                remove_waypoint(index)
            else:
                print("Usage: remove <index>")
        elif cmd.lower() == 'save':
            save_path()
        elif cmd.lower().startswith('rename '):
            parts = cmd.split()
            if len(parts) == 3:
                old_name = parts[1]
                new_name = parts[2]
                paths = load_paths()
                if old_name in paths:
                    paths[new_name] = paths.pop(old_name)
                    save_paths(paths)
                    print(f"Renamed path '{old_name}' to '{new_name}'")
                else:
                    print(f"Path '{old_name}' not found.")
            else:
                print("Usage: rename <old_name> <new_name>")
        elif cmd.lower().startswith('delete '):
            name = cmd[7:].strip()
            if name:
                delete_path(name)
            else:
                print("Usage: delete <name>")
        elif cmd.lower().startswith('load '):
            name = cmd[5:].strip()
            if name:
                path = load_path(name)
                if path:
                    print(f"Loaded path '{name}': {len(path)} waypoints")
            else:
                print("Usage: load <name>")
        elif cmd.lower().startswith('execute '):
            name = cmd[8:].strip()
            if name:
                execute_path(name)
            else:
                print("Usage: execute <name>")
        elif cmd.lower() == 'list':
            paths = load_paths()
            if paths:
                print("Saved paths:")
                for name, path in paths.items():
                    print(f"  {name}: {len(path)} waypoints")
            else:
                print("No saved paths.")
        elif cmd.lower() == 'current':
            show_current_path()
        elif cmd.lower() == 'torque off all':
            for sid in range(1, 7):
                disable_servo(sid)
            print("Torque disabled on all servos")
        else:
            print("Invalid command.")
    except KeyboardInterrupt:
        print("\nExiting...")
        break