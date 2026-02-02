"""
Camera to Robot coordinate transformation.
Uses calibration data (rotation matrix + translation vector) to map camera XYZ to robot XYZ.

The transformation handles the inherent mismatch between camera and robot coordinate frames:
- Camera frame: Computer vision perspective (X=left/right, Y=up/down, Z=depth)
- Robot frame: Arm manipulation perspective (X=forward/back, Y=left/right, Z=up/down)
"""

import json
import numpy as np
import os

class CameraTransform:
    def __init__(self, calibration_file="camera_calibration.json"):
        """Load calibration data from JSON file"""
        if not os.path.exists(calibration_file):
            raise FileNotFoundError(f"Calibration file not found: {calibration_file}")
        
        with open(calibration_file, 'r') as f:
            cal = json.load(f)
        
        self.R = np.array(cal["rotation_matrix"])
        self.t = np.array(cal["translation_vector"])
        self.mean_error = cal.get("mean_error_mm", 0)
        self.max_error = cal.get("max_error_mm", 0)
        self.num_points = cal.get("num_points", 0)
        
        print(f"✓ Calibration loaded from {calibration_file}")
        print(f"  Points used: {self.num_points}")
        print(f"  Mean error: {self.mean_error:.2f}mm")
        print(f"  Max error: {self.max_error:.2f}mm")
    
    def transform(self, camera_xyz):
        """
        Transform camera coordinates to robot coordinates.
        
        Args:
            camera_xyz (list or np.array): [x, y, z] in camera frame (mm)
        
        Returns:
            list: [x, y, z] in robot frame (mm)
        """
        cam = np.array(camera_xyz)
        robot = self.R @ cam + self.t
        return robot.tolist()
    
    def transform_batch(self, camera_points):
        """
        Transform multiple camera coordinates to robot coordinates.
        
        Args:
            camera_points (list of lists): [[x1, y1, z1], [x2, y2, z2], ...]
        
        Returns:
            list of lists: Robot coordinates for each point
        """
        return [self.transform(p) for p in camera_points]
    
    def get_rotation_matrix(self):
        """Get the rotation matrix (3x3)"""
        return self.R
    
    def get_translation_vector(self):
        """Get the translation vector (3x1)"""
        return self.t


# Create global instance for easy access
try:
    transform = CameraTransform()
except FileNotFoundError:
    print("⚠ Warning: camera_calibration.json not found. Run record_calibration_auto.py first.")
    transform = None


if __name__ == "__main__":
    # Test the transformation
    if transform is None:
        print("Cannot test without calibration file")
        exit(1)
    
    print("\n" + "="*70)
    print("CAMERA TRANSFORMATION TEST")
    print("="*70)
    
    # Test with some sample points
    test_points = [
        [0, 0, 100],
        [50, -50, 200],
        [-100, 100, 300]
    ]
    
    print("\nTest transformations:")
    for cam_xyz in test_points:
        robot_xyz = transform.transform(cam_xyz)
        print(f"Camera {cam_xyz} → Robot [{robot_xyz[0]:.1f}, {robot_xyz[1]:.1f}, {robot_xyz[2]:.1f}]")
    
    print("\nRotation Matrix R:")
    print(transform.get_rotation_matrix())
    
    print("\nTranslation Vector t:")
    print(transform.get_translation_vector())
