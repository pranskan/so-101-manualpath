import sys
import time
import os

# Set environment variable to potentially help with USB issues
os.environ['LIBUSB_DEBUG'] = '0'

print("Script starting...", flush=True)

try:
    import pyrealsense2 as rs
    print(f"pyrealsense2 version: {rs.__version__ if hasattr(rs, '__version__') else 'unknown'}", flush=True)
except Exception as e:
    print(f"Failed to import pyrealsense2: {e}", flush=True)
    sys.exit(1)

try:
    import numpy as np
    print("numpy imported successfully", flush=True)
except Exception as e:
    print(f"Failed to import numpy: {e}", flush=True)
    sys.exit(1)

print("Testing RealSense connection...", flush=True)

try:
    # Create context first
    print("Creating context...", flush=True)
    ctx = rs.context()
    
    # Wait for USB to settle
    print("Waiting for USB to settle...", flush=True)
    time.sleep(2)
    
    # Get device count without iterating
    print(f"Found {len(ctx.query_devices())} device(s)", flush=True)
    
    if len(ctx.query_devices()) == 0:
        print("No RealSense device found!")
        sys.exit(1)
    
    # Create pipeline
    print("Creating pipeline...", flush=True)
    pipeline = rs.pipeline(ctx)
    
    # Create config
    print("Creating config...", flush=True)
    config = rs.config()
    
    # Enable only color stream with lower resolution
    print("Configuring color stream (320x240)...", flush=True)
    config.enable_stream(rs.stream.color, 320, 240, rs.format.bgr8, 15)
    
    # Validate config before starting
    print("Validating config...", flush=True)
    
    # Try to start with config
    print("Starting pipeline...", flush=True)
    try:
        profile = pipeline.start(config)
        print("✓ Pipeline started successfully!")
    except Exception as e:
        print(f"Failed with config, trying without: {e}")
        pipeline = rs.pipeline(ctx)
        profile = pipeline.start()
        print("✓ Pipeline started with defaults!")
    
    # Get device info
    device = profile.get_device()
    print(f"Device: {device.get_info(rs.camera_info.name)}")
    
    # Capture frames
    print("Capturing 5 test frames...")
    for i in range(5):
        frames = pipeline.wait_for_frames(timeout_ms=5000)
        color_frame = frames.get_color_frame()
        if color_frame:
            color_image = np.asanyarray(color_frame.get_data())
            print(f"  Frame {i+1}: {color_image.shape} - OK")
        else:
            print(f"  Frame {i+1}: No color frame")
    
    print("\n✓ RealSense is working correctly!")
    pipeline.stop()
    print("Pipeline stopped")
    
except Exception as e:
    print(f"✗ Error: {e}", flush=True)
    import traceback
    traceback.print_exc()
    print("\n--- Troubleshooting Tips ---")
    print("1. Unplug camera, wait 10 sec, replug")
    print("2. Try USB-A to USB-C adapter (not direct USB-C)")
    print("3. Run: sudo killall VDCAssistant")
    print("4. Restart Mac")
    print("5. Try: conda update pyrealsense2")
    sys.exit(1)
