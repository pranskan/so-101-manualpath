import json
import math
import os
import numpy as np
from scipy.optimize import least_squares, differential_evolution
from multiprocessing import Pool
import warnings
warnings.filterwarnings('ignore')

# Try to import numba for JIT compilation (optional)
HAS_NUMBA = False
try:
    from numba import jit
    HAS_NUMBA = True
except Exception:
    # Numba not available or broken - use stub decorator
    def jit(*args, **kwargs):
        def decorator(func):
            return func
        return decorator
CALIBRATION_PATH = "calibration.json"
if os.path.exists(CALIBRATION_PATH):
    with open(CALIBRATION_PATH, "r") as f:
        cal = json.load(f)
else:
    raise FileNotFoundError(f"Calibration file not found: {CALIBRATION_PATH}")

# DH Parameters from dh-simulator.html (fixed values)
# Calibrated from SOARM.png dimensions
# Base Z-axis pointing upward, typical vertical articulated arm configuration
DH_PARAMS = [
    {"theta": 0, "d": 119, "a": 0, "alpha": 90},          # Base
    {"theta": 0, "d": 0, "a": 111, "alpha": 0},           # Shoulder
    {"theta": 0, "d": 0, "a": 187, "alpha": 0},           # Elbow
    {"theta": 0, "d": 0, "a": 0, "alpha": 90},            # Wrist Flex
    {"theta": 0, "d": 100, "a": 0, "alpha": 0},           # Wrist Roll
    {"theta": 0, "d": 66, "a": 0, "alpha": 0}             # Gripper
]

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

def deg_to_rad(deg):
    return deg * math.pi / 180

@jit(nopython=False)
def create_dh_matrix_fast(theta_deg, d, a, alpha_deg):
    """Fast DH matrix creation using numpy"""
    theta = deg_to_rad(theta_deg)
    alpha = deg_to_rad(alpha_deg)
    ct = math.cos(theta)
    st = math.sin(theta)
    ca = math.cos(alpha)
    sa = math.sin(alpha)
    
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])

def multiply_matrices(a, b):
    """Fast matrix multiplication using numpy"""
    return np.dot(a, b)

def logical_to_joint_angles(logical_angles):
    """Convert logical angles (0-180) to DH solver angles with proper offsets.
    
    This matches the conversion used in dh-simulator.html:
    - Converts from UI range (0-180, where 90 is home) to DH angles
    - Applies joint-specific inversions and offsets for each servo
    
    Args:
        logical_angles: List of 6 angles in 0-180 range
    
    Returns:
        List of 6 DH theta angles with proper offsets applied
    """
    joint_angles = []
    for i, logical in enumerate(logical_angles):
        # Apply joint-specific transformation (matching dh-simulator.html)
        if i == 0:
            # Joint 1 (base): invert direction (0 and 180 swapped)
            joint_offset = -(logical - 90)
        elif i == 1:
            # Joint 2 (shoulder): invert direction and add 90 degree offset
            joint_offset = -(logical - 90) + 90
        elif i == 2:
            # Joint 3 (elbow): 90 becomes new 0, inverted and shifted
            joint_offset = -(logical - 90) - 90
        elif i == 3:
            # Joint 4 (wrist flex): inverted (0↔180) plus 90 degree rotation
            joint_offset = -(logical - 90) + 90
        elif i == 4:
            # Joint 5 (wrist roll): invert direction (rotates like base)
            joint_offset = -(logical - 90)
        elif i == 5:
            # Joint 6 (gripper): fixed, no rotation - always 0
            joint_offset = 0
        else:
            joint_offset = 0
        
        joint_angles.append(joint_offset)
    
    return joint_angles

def solve_fk(logical_angles):
    """
    Solve forward kinematics for SO-101 arm using DH parameters.
    
    This implementation matches dh-simulator.html exactly.
    
    Args:
        logical_angles (list): 6 logical angles (0-180°) for [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper]
    
    Returns:
        list: [x, y, z] end effector coordinates in mm
    """
    if len(logical_angles) != 6:
        raise ValueError("Must provide exactly 6 logical angles")
    
    # Get joint offsets based on servo input angles
    joint_offsets = logical_to_joint_angles(logical_angles)
    
    # Initialize transformation matrix
    T = np.eye(4)
    
    # Forward kinematics using DH parameters
    for i in range(len(DH_PARAMS)):
        theta = DH_PARAMS[i]["theta"] + joint_offsets[i]
        d = DH_PARAMS[i]["d"]
        a = DH_PARAMS[i]["a"]
        alpha = DH_PARAMS[i]["alpha"]
        
        Ti = create_dh_matrix_fast(theta, d, a, alpha)
        T = np.dot(T, Ti)
    
    # Extract end effector position
    x = T[0, 3]
    y = T[1, 3]
    z = T[2, 3]
    
    return [float(round(x, 3)), float(round(y, 3)), float(round(z, 3))]

def ik_cost(logical_angles, target):
    """Cost function for IK: error between FK position and target."""
    pos = solve_fk(logical_angles)
    return np.array(pos) - np.array(target)

def solve_ik_analytical(target):
    """
    Simple fallback IK: just try reasonable initial guesses.
    Returns one if it looks close enough.
    """
    x, y, z = target
    
    # Simple heuristic: base rotation
    import math
    base_angle = math.degrees(math.atan2(y, x))
    phys_min = cal["shoulder_pan"]["physical_min"]
    phys_max = cal["shoulder_pan"]["physical_max"]
    shoulder_pan_logical = ((base_angle - phys_min) / (phys_max - phys_min)) * 180
    shoulder_pan_logical = max(0, min(180, shoulder_pan_logical))
    
    # Try a few reasonable postures
    candidates = [
        [shoulder_pan_logical, 45, 90, 90, 90, 90],
        [shoulder_pan_logical, 90, 45, 90, 90, 90],
        [shoulder_pan_logical, 135, 45, 90, 90, 90],
        [shoulder_pan_logical, 45, 135, 90, 90, 90],
        [shoulder_pan_logical, 90, 90, 90, 90, 90],
    ]
    
    best_solution = None
    best_error = float('inf')
    
    for angles in candidates:
        try:
            pos = solve_fk(angles)
            error = math.sqrt((pos[0] - x)**2 + (pos[1] - y)**2 + (pos[2] - z)**2)
            if error < best_error:
                best_error = error
                best_solution = angles
                if error < 3.0:  # Good enough
                    return angles
        except:
            pass
    
    return best_solution if best_error < 50 else None

def solve_ik(target, initial_guess=None):
    """
    Solve inverse kinematics for SO-101 arm - optimized for speed and accuracy.
    
    Uses: analytical IK -> multi-start least_squares -> differential_evolution
    
    Args:
        target (list): Target position [x, y, z] in mm
        initial_guess (list, optional): Initial logical angles (0-180°).
    
    Returns:
        list or None: 6 logical angles if solution found (< 1mm error), else None
    """
    target = np.array(target, dtype=np.float64)
    
    # Try analytical IK first (fastest)
    analytical_solution = solve_ik_analytical(target)
    if analytical_solution:
        pos = np.array(solve_fk(analytical_solution))
        error = np.linalg.norm(pos - target)
        if error < 1.0:
            return [float(round(angle, 1)) for angle in analytical_solution]
    
    # Multi-start least_squares (fast and usually works)
    initial_guesses = [
        [90, 90, 90, 90, 90, 90],      # Home
        [180, 0, 180, 90, 90, 90],     # Extended
        [0, 90, 90, 90, 90, 90],       # Rotated left
        [180, 90, 90, 90, 90, 90],     # Rotated right
        [90, 45, 135, 90, 90, 90],     # Mid 1
        [90, 135, 45, 90, 90, 90],     # Mid 2
    ]
    
    if initial_guess is not None:
        initial_guesses.insert(0, initial_guess)
    
    bounds_ls = ([0]*6, [180]*6)  # For least_squares
    bounds_de = [(0, 180)] * 6     # For differential_evolution
    best_solution = None
    best_error = float('inf')
    
    for guess in initial_guesses:
        try:
            result = least_squares(
                ik_cost,
                guess,
                bounds=bounds_ls,
                args=(target,),
                ftol=1e-10,
                xtol=1e-10,
                max_nfev=2000
            )
            
            pos = np.array(solve_fk(result.x))
            error = np.linalg.norm(pos - target)
            
            if error < best_error:
                best_error = error
                best_solution = result.x
                
                if error < 1.0:
                    return [float(round(angle, 1)) for angle in result.x]
        except:
            pass
    
    # If least_squares didn't work, use global optimization
    if best_error > 1.0:
        try:
            def objective(angles):
                pos = np.array(solve_fk(angles))
                return np.linalg.norm(pos - target)
            
            # Use differential_evolution for global optimization (faster than brute force)
            result = differential_evolution(
                objective,
                bounds_de,
                seed=42,
                maxiter=500,
                atol=1e-6,
                tol=1e-6,
                polish=True,
                workers=1,
                updating='deferred'
            )
            
            if result.fun < 1.0:
                return [float(round(angle, 1)) for angle in result.x]
            
            if result.fun < best_error:
                best_error = result.fun
                best_solution = result.x
        except:
            pass
    
    # Return if we found something acceptable
    if best_solution is not None and best_error < 5.0:
        if best_error >= 1.0:
            print(f"Warning: IK solution has {best_error:.2f}mm error")
        return [float(round(angle, 1)) for angle in best_solution]
    
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
