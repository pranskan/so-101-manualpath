import pyrealsense2 as rs
import numpy as np
import cv2

def detect_red_objects(frame, depth_frame=None, depth_intrinsics=None):
    """
    Detect red objects and calculate 3D position from camera.
    More strict color filtering to avoid dark objects.
    Returns: highlighted frame, list of red object info (x, y, z coordinates)
    """
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Enhance contrast slightly to help detect distant objects
    h, s, v = cv2.split(hsv)
    v = cv2.convertScaleAbs(v, alpha=1.2, beta=10)  # Increase brightness slightly
    v = np.clip(v, 0, 255).astype(np.uint8)
    hsv = cv2.merge([h, s, v])
    
    # Define strict red color range in HSV
    # Red: Hue 0-10 and 170-180 (wraps around), Saturation 90-255 (very saturated), Value 40-255
    lower_red1 = np.array([0, 90, 40])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 90, 40])
    upper_red2 = np.array([180, 255, 255])
    
    # Create mask for red objects (combine both ranges)
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    
    # Exclude skin tones (broader filter: low saturation reds)
    # Skin: Hue 0-25, Saturation 5-50 (very desaturated), Value 30-255
    skin_lower = np.array([0, 5, 30])
    skin_upper = np.array([25, 50, 255])
    skin_mask = cv2.inRange(hsv, skin_lower, skin_upper)
    
    # Also exclude desaturated reds from high hue range
    skin_lower2 = np.array([165, 5, 30])
    skin_upper2 = np.array([180, 50, 255])
    skin_mask2 = cv2.inRange(hsv, skin_lower2, skin_upper2)
    
    # Combine skin masks
    skin_combined = cv2.bitwise_or(skin_mask, skin_mask2)
    
    # Remove ALL skin tones from red mask
    mask = cv2.subtract(mask, skin_combined)
    
    # Morphological operations to clean up mask
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    red_objects = []
    output = frame.copy()
    
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 800:  # Increased minimum area to filter noise
            # Get bounding rect and center
            x, y, w, h = cv2.boundingRect(contour)
            cx = x + w // 2
            cy = y + h // 2
            
            # Calculate 3D coordinates if depth frame and intrinsics available
            x_robot, y_robot, z_robot = None, None, None
            if depth_frame is not None and depth_intrinsics is not None:
                try:
                    depth = depth_frame.get_distance(cx, cy)
                    if depth > 0:  # Valid depth
                        # Convert pixel coordinates to camera 3D coordinates
                        x_cam = (cx - depth_intrinsics.ppx) * depth / depth_intrinsics.fx
                        y_cam = (cy - depth_intrinsics.ppy) * depth / depth_intrinsics.fy
                        z_cam = depth
                        # Convert to robotic coordinates (X: forward/backward, Y: left/right, Z: up/down)
                        x_robot = z_cam  # forwards/backwards
                        y_robot = x_cam  # left/right
                        z_robot = -y_cam  # up/down (negative because camera Y is inverted)
                except:
                    pass
            
            red_objects.append({
                'x': cx,
                'y': cy,
                'width': w,
                'height': h,
                'x_robot': x_robot,
                'y_robot': y_robot,
                'z_robot': z_robot
            })
            
            # Draw bounding box (cyan for red objects)
            cv2.rectangle(output, (x, y), (x + w, y + h), (255, 255, 0), 2)
            
            # Draw center point
            cv2.circle(output, (cx, cy), 5, (0, 255, 0), -1)
            
            # Add XYZ coordinates text if available
            if x_robot is not None and y_robot is not None and z_robot is not None:
                text = f"X:{x_robot*1000:.0f} Y:{y_robot*1000:.0f} Z:{z_robot*1000:.0f}mm"
                cv2.putText(output, text, (x, y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 150), 2)
    
    return output, red_objects

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()

# List available devices
ctx = rs.context()
devices = ctx.query_devices()
if len(devices) == 0:
    print("No RealSense device connected!")
    exit(1)

print(f"Found {len(devices)} RealSense device(s)")
for dev in devices:
    print(f"  - {dev.get_info(rs.camera_info.name)}")

# Configure streams
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start pipeline
try:
    pipeline.start(config)
    print("Pipeline started successfully")
except Exception as e:
    print(f"Failed to start pipeline: {e}")
    exit(1)

# Get depth camera intrinsics
depth_intrinsics = None
try:
    profile = pipeline.get_active_profile()
    depth_stream = profile.get_stream(rs.stream.depth)
    depth_intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
except:
    print("Warning: Could not get camera intrinsics")

# Global variables for mouse callback
clicked_coordinates = None
latest_depth_frame = None

def mouse_callback(event, x, y, flags, param):
    """Mouse callback to get XYZ at clicked pixel."""
    global clicked_coordinates, latest_depth_frame
    
    if event == cv2.EVENT_LBUTTONDOWN and latest_depth_frame is not None and depth_intrinsics is not None:
        try:
            depth = latest_depth_frame.get_distance(x, y)
            if depth > 0:
                # Convert pixel to camera 3D coordinates
                x_cam = (x - depth_intrinsics.ppx) * depth / depth_intrinsics.fx
                y_cam = (y - depth_intrinsics.ppy) * depth / depth_intrinsics.fy
                z_cam = depth
                
                # Convert to robotic coordinates
                x_robot = z_cam
                y_robot = x_cam
                z_robot = -y_cam
                
                clicked_coordinates = {
                    'pixel_x': x,
                    'pixel_y': y,
                    'x_robot': x_robot,
                    'y_robot': y_robot,
                    'z_robot': z_robot
                }
                print(f"\n✓ Clicked at pixel ({x}, {y})")
                print(f"  Robotic XYZ: X={x_robot*1000:.1f}mm, Y={y_robot*1000:.1f}mm, Z={z_robot*1000:.1f}mm")
            else:
                print(f"✗ No depth data at pixel ({x}, {y})")
        except Exception as e:
            print(f"Error reading depth: {e}")

cv2.namedWindow("Red Object Detection")
cv2.setMouseCallback("Red Object Detection", mouse_callback)

try:
    while True:
        # Wait for frames
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        latest_depth_frame = depth_frame  # Store for mouse callback
        
        if not color_frame:
            continue
        
        # Convert to numpy array
        color_image = np.asanyarray(color_frame.get_data())
        
        # Detect red objects
        highlighted, red_objects = detect_red_objects(color_image, depth_frame, depth_intrinsics)
        
        # Display detected objects info (disabled spam)
        # if red_objects:
        #     print(f"\nDetected {len(red_objects)} red object(s):")
        #     for i, obj in enumerate(red_objects):
        #         if obj['x_robot'] is not None:
        #             print(f"  Object {i+1}: X={obj['x_robot']*1000:.0f}mm (forward/backward), Y={obj['y_robot']*1000:.0f}mm (left/right), Z={obj['z_robot']*1000:.0f}mm (up/down)")
        #         else:
        #             print(f"  Object {i+1}: Position ({obj['x']}, {obj['y']}) - No depth data")
        
        # Display
        cv2.imshow("Red Object Detection", highlighted)
        
        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    print("Pipeline stopped")