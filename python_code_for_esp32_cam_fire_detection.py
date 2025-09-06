import cv2
from ultralytics import YOLO
import requests
import time
import numpy as np

# Load your YOLO model
model = YOLO("C:\\Users\\Hp\\Desktop\\best.pt")

# ESP32 endpoints
capture_url = 'http://192.168.4.1/capture'  # Single frame
pump_control_url = 'http://192.168.4.1/pump'
base_url = 'http://192.168.4.1'  # Base URL for testing

# Test connection to ESP32 first
print("Testing connection to ESP32-CAM...")
try:
    response = requests.get(base_url, timeout=5)
    print(f"✓ Connected to ESP32-CAM (Status: {response.status_code})")
    
    # Test the pump control endpoint
    response = requests.get(pump_control_url + "?state=off", timeout=2)
    print(f"✓ Pump control test: {response.text}")
    
    # Test the capture endpoint
    response = requests.get(capture_url, timeout=5)
    if response.status_code == 200:
        print("✓ Capture endpoint working")
    else:
        print("✗ Capture endpoint not working")
        
except requests.exceptions.RequestException as e:
    print(f"✗ Connection failed: {e}")
    print("Please check:")
    print("1. Your ESP32-CAM is powered on")
    print("2. You're connected to the 'Atom' WiFi network")
    print("3. The ESP32-CAM IP address is correct (192.168.4.1)")
    exit()

cv2.namedWindow("Fire Detection System", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Fire Detection System", 800, 600)

# Fire detection state
fire_detected = False
consecutive_fire_frames = 0
consecutive_no_fire_frames = 0

# Initialize FPS variables
frame_count = 0
start_time = time.time()
last_status_check = time.time()
fps = 0  # Initialize fps variable

def control_esp32_pump(state):
    """Send command to control ESP32-CAM pump relay (pin 13)"""
    try:
        response = requests.get(f"{pump_control_url}?state={'on' if state else 'off'}", timeout=2)
        print(f"Pump control: {response.text}")
        return True
    except requests.exceptions.RequestException as e:
        print(f"Error controlling pump: {e}")
        return False

def get_single_frame():
    """Capture a single frame from ESP32-CAM"""
    try:
        response = requests.get(capture_url, timeout=5)
        if response.status_code == 200:
            img_array = np.asarray(bytearray(response.content), dtype=np.uint8)
            frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            return True, frame
        return False, None
    except requests.exceptions.RequestException as e:
        print(f"Error getting frame: {e}")
        return False, None

while True:
    # Get frame from single frame capture
    success, frame = get_single_frame()
    if not success:
        print("Failed to get frame, retrying...")
        time.sleep(1)
        continue

    # Calculate FPS
    frame_count += 1
    current_time = time.time()
    elapsed_time = current_time - start_time
    
    if elapsed_time >= 1.0:
        fps = frame_count / elapsed_time
        frame_count = 0
        start_time = current_time
        
        # Periodically print status
        if current_time - last_status_check >= 5:
            print(f"FPS: {fps:.1f}")
            last_status_check = current_time

    # Run YOLO inference on the frame
    results = model.predict(frame, imgsz=640, verbose=False, conf=0.5)
    
    # Check if fire is detected
    current_fire_detected = False
    fire_confidence = 0
    
    for result in results:
        if len(result.boxes) > 0:
            for i, cls in enumerate(result.boxes.cls):
                # Try different possible class names for fire
                class_name = result.names[int(cls)].lower()
                if 'fire' in class_name or 'flame' in class_name:
                    current_fire_detected = True
                    fire_confidence = float(result.boxes.conf[i])
                    break
    
    # Add debouncing to prevent rapid toggling
    if current_fire_detected:
        consecutive_fire_frames += 1
        consecutive_no_fire_frames = 0
    else:
        consecutive_no_fire_frames += 1
        consecutive_fire_frames = 0
    
    # Control pump based on fire detection with debouncing
    if consecutive_fire_frames >= 3 and not fire_detected:
        print(f"FIRE DETECTED! (Confidence: {fire_confidence:.2f}) Activating water pump")
        if control_esp32_pump(True):
            fire_detected = True
            
    elif consecutive_no_fire_frames >= 5 and fire_detected:
        print("Fire cleared. Deactivating water pump")
        if control_esp32_pump(False):
            fire_detected = False

    # Draw bounding boxes and labels
    annotated_frame = results[0].plot()

    # Add status text to the frame
    status_text = f"Fire: {'DETECTED' if fire_detected else 'CLEAR'} | Pump: {'ON' if fire_detected else 'OFF'}"
    cv2.putText(annotated_frame, status_text, (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255) if fire_detected else (0, 255, 0), 2)
    
    # Add FPS info
    fps_text = f"FPS: {fps:.1f}"
    cv2.putText(annotated_frame, fps_text, (10, 70), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    # Display the frame
    cv2.imshow("Fire Detection System", annotated_frame)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

# Ensure pump is turned off when exiting
print("Exiting... Turning off pump")
control_esp32_pump(False)
cv2.destroyAllWindows()
