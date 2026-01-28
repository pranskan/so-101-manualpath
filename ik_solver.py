import json
import math
import os
import numpy as np
from scipy.optimize import least_squares

# Load calibration data
CALIBRATION_PATH = "calibration.json"
if os.path.exists(CALIBRATION_PATH):
    with open(CALIBRATION_PATH, "r") as f:
        cal = json.load(f)
else:
    raise FileNotFoundError(f"Calibration file not found: {CALIBRATION_PATH}")

# DH Parameters from dh-simulator.html (fixed values)
DH_PARAMS = [
    {"theta": 0, "d": 119, "a": 0, "alpha": 90},    # Base
    {"theta": 0, "d": 0, "a": 111, "alpha": 0},     # Shoulder
    {"theta": 0, "d": 0, "a": 137, "alpha": 0},     # Elbow
    {"theta": 0, "d": 0, "a": 0, "alpha": 90},      # Wrist Flex
    {"theta": 0, "d": 100, "a": 0, "alpha": 0},     # Wrist Roll
    {"theta": 0, "d": 66, "a": 0, "alpha": 0}       # Gripper
]

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

def deg_to_rad(deg):
    return deg * math.pi / 180

def create_dh_matrix(theta, d, a, alpha):
    ct = math.cos(deg_to_rad(theta))
    st = math.sin(deg_to_rad(theta))
    ca = math.cos(deg_to_rad(alpha))
    sa = math.sin(deg_to_rad(alpha))
    
    return [
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ]

def multiply_matrices(a, b):
    result = [[0] * 4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            for k in range(4):
                result[i][j] += a[i][k] * b[k][j]
    return result

def logical_to_joint_angles(logical_angles):
    """Convert logical angles (0-180) to joint angles (0-180) for FK solver."""
    joint_angles = []
    for i, logical in enumerate(logical_angles):
        joint_name = JOINT_NAMES[i]
        phys_min = cal[joint_name]["physical_min"]
        phys_max = cal[joint_name]["physical_max"]
        
        # Convert logical to physical
        physical = phys_min + (logical / 180.0) * (phys_max - phys_min)
        
        # Convert physical to joint_angles scale (0-180)
        joint_angle = (physical - phys_min) / (phys_max - phys_min) * 180.0
        joint_angles.append(joint_angle)
    
    return joint_angles

def solve_fk(logical_angles):
    """
    Solve forward kinematics for SO-100 arm.
    
    Args:
        logical_angles (list): 6 logical angles (0-180°) for [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper]
    
    Returns:
        list: [x, y, z] end effector coordinates in mm
    """
    if len(logical_angles) != 6:
        raise ValueError("Must provide exactly 6 logical angles")
    
    # Convert logical to joint angles
    joint_angles = logical_to_joint_angles(logical_angles)
    
    # Initialize transformation matrix
    T = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    
    for i in range(len(DH_PARAMS)):
        # Calculate joint offset (matching simulator logic)
        if i == 0:  # Base
            joint_offset = -(joint_angles[i] - 90)
        elif i == 1:  # Shoulder
            joint_offset = -(joint_angles[i] - 90) + 90
        elif i == 2:  # Elbow
            joint_offset = -(joint_angles[i] - 90) - 90
        elif i == 3:  # Wrist Flex
            joint_offset = -(joint_angles[i] - 90) + 90
        elif i == 4:  # Wrist Roll
            joint_offset = -(joint_angles[i] - 90)
        elif i == 5:  # Gripper
            joint_offset = 0
        else:
            joint_offset = joint_angles[i] - 90
        
        theta = DH_PARAMS[i]["theta"] + joint_offset
        d = DH_PARAMS[i]["d"]
        a = DH_PARAMS[i]["a"]
        alpha = DH_PARAMS[i]["alpha"]
        
        Ti = create_dh_matrix(theta, d, a, alpha)
        T = multiply_matrices(T, Ti)
    
    # Extract end effector position [x, y, z]
    x = T[0][3]
    y = T[1][3]
    z = T[2][3]
    
    return [round(x, 3), round(y, 3), round(z, 3)]

def ik_cost(logical_angles, target):
    """Cost function for IK: error between FK position and target."""
    pos = solve_fk(logical_angles)
    return np.array(pos) - np.array(target)

def solve_ik_analytical(target):
    """
    Analytical inverse kinematics for SO-100 arm (position-only).
    
    Solves for joints 1-4 using geometric methods, accounting for wrist/gripper length.
    Joints 5-6 are set to 90° (wrist roll and gripper don't affect XYZ position).
    """
    x, y, z = target
    
    # Base rotation (joint 1)
    theta1_rad = math.atan2(y, x)
    theta1_deg = math.degrees(theta1_rad)
    
    # Map to logical angle for shoulder_pan
    phys_min = cal["shoulder_pan"]["physical_min"]
    phys_max = cal["shoulder_pan"]["physical_max"]
    logical1 = ((theta1_deg - phys_min) / (phys_max - phys_min)) * 180
    logical1 = max(0, min(180, logical1))  # Clamp to 0-180
    
    # Position in base frame after rotation
    x_prime = math.sqrt(x**2 + y**2)
    z_prime = z
    d1 = 119  # Base height
    
    # Link lengths
    link1 = 111  # Shoulder to elbow
    link2 = 137  # Elbow to wrist flex
    link_wrist = 100 + 66  # Wrist roll + gripper = 166mm total
    
    # Iterate over possible wrist flex angles to find a solution
    best_solution = None
    best_error = float('inf')
    
    for wrist_flex_logical in range(0, 181, 5):  # Try every 5 degrees
        # Convert wrist flex logical angle to physical
        wf_phys_min = cal["wrist_flex"]["physical_min"]
        wf_phys_max = cal["wrist_flex"]["physical_max"]
        wrist_flex_physical = wf_phys_min + (wrist_flex_logical / 180.0) * (wf_phys_max - wf_phys_min)
        
        # Convert to joint angle space
        wrist_flex_joint = (wrist_flex_physical - wf_phys_min) / (wf_phys_max - wf_phys_min) * 180.0
        
        # The wrist flex joint affects the direction of the wrist extension
        wrist_flex_offset = -(wrist_flex_joint - 90) + 90  # Match simulator logic
        wrist_flex_rad = math.radians(wrist_flex_offset)
        
        # Wrist flex position (working backwards from gripper tip)
        wx_prime = x_prime - link_wrist * math.cos(wrist_flex_rad)
        wz = z_prime - d1 - link_wrist * math.sin(wrist_flex_rad)
        d_to_wrist = math.sqrt(wx_prime**2 + wz**2)
        
        # Check if reachable
        if d_to_wrist > link1 + link2 or d_to_wrist < abs(link1 - link2):
            continue
        
        # Solve 2-link IK for shoulder and elbow
        cos_theta3 = (link1**2 + link2**2 - d_to_wrist**2) / (2 * link1 * link2)
        cos_theta3 = max(-1, min(1, cos_theta3))
        theta3_rad = math.acos(cos_theta3)
        theta3_deg = math.degrees(theta3_rad)
        
        # Shoulder angle
        theta2_rad = math.atan2(wz, wx_prime) - math.atan2(link2 * math.sin(theta3_rad), link1 + link2 * math.cos(theta3_rad))
        theta2_deg = math.degrees(theta2_rad)
        
        # Convert shoulder and elbow to logical angles
        sl_phys_min = cal["shoulder_lift"]["physical_min"]
        sl_phys_max = cal["shoulder_lift"]["physical_max"]
        shoulder_logical = ((theta2_deg - sl_phys_min) / (sl_phys_max - sl_phys_min)) * 180
        shoulder_logical = max(0, min(180, shoulder_logical))
        
        el_phys_min = cal["elbow_flex"]["physical_min"]
        el_phys_max = cal["elbow_flex"]["physical_max"]
        elbow_logical = ((theta3_deg - el_phys_min) / (el_phys_max - el_phys_min)) * 180
        elbow_logical = max(0, min(180, elbow_logical))
        
        # Test this solution
        test_angles = [logical1, shoulder_logical, elbow_logical, wrist_flex_logical, 90, 90]
        try:
            test_pos = solve_fk(test_angles)
            error = math.sqrt((test_pos[0] - x)**2 + (test_pos[1] - y)**2 + (test_pos[2] - z)**2)
            
            if error < best_error:
                best_error = error
                best_solution = test_angles
                
                # If good enough, return immediately
                if error < 10.0:  # Within 10mm
                    return [float(round(angle, 1)) for angle in test_angles]
        except:
            continue
    
    # Return best solution if found
    if best_solution and best_error < 50.0:
        return [float(round(angle, 1)) for angle in best_solution]
    
    return None

def solve_ik(target, initial_guess=None):
    """
    Solve inverse kinematics for SO-100 arm using analytical + numerical optimization + aggressive brute force.
    
    Prioritizes accuracy: aims for < 1mm error, uses extensive random sampling.
    
    Args:
        target (list): Target position [x, y, z] in mm
        initial_guess (list, optional): Initial logical angles (0-180°). Defaults to home position.
    
    Returns:
        list or None: 6 logical angles if solution found, else None
    """
    if len(target) != 3:
        raise ValueError("Target must be [x, y, z]")
    
    # First try analytical IK
    analytical_solution = solve_ik_analytical(target)
    if analytical_solution:
        pos = solve_fk(analytical_solution)
        error = np.linalg.norm(np.array(pos) - np.array(target))
        if error < 1.0:  # Very tight threshold
            return [float(round(angle, 1)) for angle in analytical_solution]
    
    # Fallback to numerical optimization with multiple intelligent initial guesses
    initial_guesses = [
        [90, 90, 90, 90, 90, 90],      # Home position
        [180, 0, 180, 90, 90, 90],     # Extended
        [0, 90, 90, 90, 90, 90],       # Rotated left
        [180, 90, 90, 90, 90, 90],     # Rotated right
        [90, 45, 135, 90, 90, 90],     # Mid-range 1
        [90, 135, 45, 90, 90, 90],     # Mid-range 2
        [45, 90, 90, 90, 90, 90],      # Angled base
        [135, 90, 90, 90, 90, 90],     # Other angle base
    ]
    
    if initial_guess is not None:
        initial_guesses.insert(0, initial_guess)
    
    lower_bounds = [0] * 6
    upper_bounds = [180] * 6
    bounds = (lower_bounds, upper_bounds)
    
    best_solution = None
    best_error = float('inf')
    solutions_found = []
    
    # Try standard optimization with stricter tolerances
    for guess in initial_guesses:
        try:
            result = least_squares(
                ik_cost,
                guess,
                bounds=bounds,
                args=(target,),
                ftol=1e-9,      # Very tight tolerance
                xtol=1e-9,
                gtol=1e-9,
                max_nfev=5000   # More iterations
            )
            
            final_position = solve_fk(result.x)
            position_error = np.linalg.norm(np.array(final_position) - np.array(target))
            
            if result.success and position_error < best_error:
                best_error = position_error
                best_solution = result.x
                solutions_found.append((position_error, result.x))
                
                # Success! Very accurate
                if position_error < 0.5:  # Sub-millimeter accuracy
                    return [float(round(angle, 1)) for angle in result.x]
                    
        except Exception:
            continue
    
    # If we got something decent, return it early
    if best_solution is not None and best_error < 2.0:
        return [float(round(angle, 1)) for angle in best_solution]
    
    # AGGRESSIVE BRUTE FORCE: Grid + random sampling for maximum accuracy
    print(f"Standard IK found best: {best_error:.2f}mm. Starting aggressive search for < 0.5mm...")
    
    import random
    max_attempts = 500000  # Much more aggressive
    attempt = 0
    last_improvement = 0
    
    # Start with grid sampling in regions likely to work
    grid_step = 30  # 30-degree grid
    grid_attempts = 0
    for b in range(0, 181, grid_step):
        for s in range(0, 181, grid_step):
            for e in range(0, 181, grid_step):
                for w in range(0, 181, grid_step):
                    grid_guess = [b, s, e, w, 90, 90]
                    
                    try:
                        result = least_squares(
                            ik_cost,
                            grid_guess,
                            bounds=bounds,
                            args=(target,),
                            ftol=1e-8,
                            xtol=1e-8,
                            max_nfev=3000
                        )
                        
                        final_position = solve_fk(result.x)
                        position_error = np.linalg.norm(np.array(final_position) - np.array(target))
                        
                        if position_error < best_error:
                            best_error = position_error
                            best_solution = result.x
                            last_improvement = grid_attempts
                            solutions_found.append((position_error, result.x))
                            
                            if position_error < 0.5:
                                print(f"Found excellent solution ({position_error:.3f}mm) via grid search!")
                                return [float(round(angle, 1)) for angle in result.x]
                            
                            if grid_attempts % 10 == 0:
                                print(f"  Grid {grid_attempts}: Best error = {best_error:.3f}mm")
                    except:
                        pass
                    
                    grid_attempts += 1
                    if grid_attempts > 100:  # Limit grid search
                        break
                if grid_attempts > 100:
                    break
            if grid_attempts > 100:
                break
        if grid_attempts > 100:
            break
    
    # Now do pure random sampling with adaptive convergence
    while attempt < max_attempts:
        # Adaptive sampling: focus on regions that work
        if len(solutions_found) > 0:
            # Sample around best solutions found
            best_angle_set = best_solution
            random_guess = [
                best_angle_set[i] + random.gauss(0, 15)  # Gaussian around best
                for i in range(6)
            ]
            # Clamp to bounds
            random_guess = [max(0, min(180, angle)) for angle in random_guess]
        else:
            # Pure random if no solutions yet
            random_guess = [random.uniform(0, 180) for _ in range(6)]
        
        try:
            result = least_squares(
                ik_cost,
                random_guess,
                bounds=bounds,
                args=(target,),
                ftol=1e-8,
                xtol=1e-8,
                max_nfev=3000
            )
            
            final_position = solve_fk(result.x)
            position_error = np.linalg.norm(np.array(final_position) - np.array(target))
            
            if position_error < best_error:
                best_error = position_error
                best_solution = result.x
                last_improvement = attempt
                solutions_found.append((position_error, result.x))
                
                # Success! Sub-millimeter
                if position_error < 0.5:
                    print(f"Found excellent solution ({position_error:.3f}mm) after {attempt} attempts!")
                    return [float(round(angle, 1)) for angle in result.x]
                
                # Very good
                if position_error < 1.0:
                    print(f"Found good solution ({position_error:.3f}mm) after {attempt} attempts!")
                    return [float(round(angle, 1)) for angle in result.x]
                
                # Log progress every 5000 attempts
                if attempt % 5000 == 0:
                    print(f"  Attempt {attempt}: Best error = {best_error:.3f}mm (solutions: {len(solutions_found)})")
        except Exception:
            pass
        
        # Early exit if no improvement for a while
        if attempt - last_improvement > 50000:
            if best_error < 2.0:
                print(f"No improvement for 50k attempts. Returning best: {best_error:.3f}mm")
                break
        
        attempt += 1
    
    # Return best solution found
    if best_solution is not None:
        final_position = solve_fk(best_solution)
        position_error = np.linalg.norm(np.array(final_position) - np.array(target))
        
        if position_error < 10.0:
            print(f"✓ Solution found: error={position_error:.3f}mm (after {attempt} attempts, {len(solutions_found)} valid solutions)")
            return [float(round(angle, 1)) for angle in best_solution]
        else:
            print(f"✗ Target unreachable: best error={position_error:.2f}mm (too far)")
            return None
    
    print(f"✗ IK failed completely: Could not find any valid solution")
    return None

# Interactive mode
if __name__ == "__main__":
    print("=== SO-100 Arm Kinematics Solver ===")
    print("Commands:")
    print("  Enter 6 angles: Solve FK (e.g., '90 90 90 90 90 90')")
    print("  ik x y z: Solve IK for target position (e.g., 'ik 0 0 285')")
    print("  q: Quit")
    print()
    
    while True:
        try:
            user_input = input("Enter command: ").strip()
            
            if user_input.lower() == 'q':
                print("Exiting...")
                break
            
            if user_input.lower().startswith('ik '):
                # IK mode
                parts = user_input.split()
                if len(parts) != 4:
                    print("Error: Use 'ik x y z'")
                    continue
                
                try:
                    target = [float(p) for p in parts[1:]]
                    angles = solve_ik(target)
                    if angles:
                        print(f"Target: {target}")
                        print(f"Solution: {angles}")
                        # Verify
                        pos = solve_fk(angles)
                        print(f"Verification (FK): {pos}")
                    else:
                        print("No solution found for target position.")
                    print()
                except ValueError:
                    print("Error: Invalid numbers for target position.")
                    continue
            else:
                # FK mode
                parts = user_input.split()
                if len(parts) != 6:
                    print("Error: Please enter exactly 6 angles for FK.")
                    continue
                
                try:
                    logical_angles = [float(p) for p in parts]
                    
                    # Validate range
                    if not all(0 <= angle <= 180 for angle in logical_angles):
                        print("Error: All angles must be between 0 and 180.")
                        continue
                    
                    # Solve FK
                    position = solve_fk(logical_angles)
                    print(f"Angles: {logical_angles}")
                    print(f"Position: X={position[0]}mm, Y={position[1]}mm, Z={position[2]}mm")
                    print()
                    
                except ValueError:
                    print("Error: Invalid number format. Please enter numbers only.")
                    continue
                
        except KeyboardInterrupt:
            print("\nExiting...")
            break
