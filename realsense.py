import pyrealsense2 as rs
import numpy as np

# Initialize RealSense pipeline first, before importing cv2
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

# Start pipeline
try:
    pipeline.start(config)
    print("Pipeline started successfully")
except Exception as e:
    print(f"Failed to start pipeline: {e}")
    exit(1)

# Now import cv2 after RealSense is initialized
import cv2

try:
    while True:
        # Wait for frames
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        
        if not color_frame:
            continue
        
        # Convert to numpy array
        color_image = np.asanyarray(color_frame.get_data())
        
        # Display
        cv2.imshow("RealSense Color", color_image)
        
        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    print("Pipeline stopped")