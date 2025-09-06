# Bluetooth-Controlled-RC-Car-Integrated-with-AI-Model-to-Initiate-Fire-Suppresion-System
A multi-functional robotic system designed for remote monitoring and automated response to fire hazards. This project integrates Bluetooth teleoperation, real-time video streaming, and an AI-powered computer vision system to create a proactive mobile safety solution.

🚀 Features
Bluetooth Teleoperation: Control the car remotely using a standard Bluetooth gamepad.

Live Video Streaming: View a real-time video feed from the onboard camera via a web interface.

AI-Powered Fire Detection: Utilizes a YOLOv8 object detection model running on a remote PC to analyze the video feed for fire.

Autonomous Suppression: Upon confirmed fire detection, the system autonomously activates a water pump to extinguish the flame.

Dual-Microcontroller Architecture: Distributes computational load between an ESP32 (driving & control) and an ESP32-CAM (vision & actuation).

Debounced Logic: Implements a confirmation system (3 consecutive detections) to prevent false positives.

🛠️ Hardware Architecture
The system is built on a 4WD RC car chassis and consists of two main subsystems:

1. Locomotion and Control Unit (ESP32 DevKit V1)
Controller: ESP32 DevKit V1

Function: Manages Bluetooth communication with the gamepad and controls movement.

Components:

L298N Motor Driver (controls 4x DC gear motors)

2x SG90 Servos (for pan-and-tilt camera movement)

Li-Po Battery (dedicated power for motors)

2. Vision and Suppression Unit (ESP32-CAM)
Controller: ESP32-CAM (AI-Thinker)

Function: Hosts a WiFi AP, streams video, and executes suppression commands.

Components:

OV2640 Camera

5V Relay Module (controls water pump)

Onboard LED (GPIO 4) for illumination

Separate Li-Po Battery (power for MCUs and pump)

📁 Software Architecture
The project is divided into three interconnected software components:

1. ESP32_DevKit_Firmware (Arduino C++)
Libraries: Bluepad32, ESP32Servo

Responsibility:

Pairs with a Bluetooth gamepad.

Maps joystick inputs to PWM signals for the L298N motor driver.

Controls the pan-and-tilt servos for camera aiming.

2. ESP32_CAM_Firmware (Arduino C++)
Libraries: ESPAsyncWebServer, AsyncTCP, esp_camera.h

Responsibility:

Hosts a WiFi Access Point (SSID: Atom).

Streams real-time video to a web client via WebSocket (/Camera).

Provides HTTP API endpoints:

GET /capture: Returns a single JPEG frame for AI processing.

GET /pump?state=on/off: Controls the water pump relay.

Serves a web interface for live viewing and status.

3. Python_AI_Client (Python)
Libraries: OpenCV, Ultralytics, Requests

Responsibility:

Connects to the ESP32-CAM's WiFi AP.

Periodically fetches images from the /capture endpoint.

Runs inference using a custom-trained YOLOv8 model (best.pt).

Implements debouncing logic (3 consecutive fire frames to activate, 5 to deactivate).

Sends HTTP commands to the /pump endpoint to trigger suppression.

Displays an annotated video feed with detection results and FPS.

📸 Media
Hardware Setup	Web Interface
<img src="media/image6.png" width="300">	<img src="media/image5.png" width="300">
4WD Chassis with Components	Live Stream with Fire Alert
🔧 Installation & Setup
1. Hardware Wiring
Detailed pin connections and Bill of Materials (BOM) can be found in the project report (ie project report.docx) in the Appendix section.

2. Flashing the ESP32s
ESP32 DevKit V1:

Open ESP32_DevKit_Firmware.ino in Arduino IDE.

Install the Bluepad32 library. (Ensure your Arduino IDE is configured for ESP32).

Select the correct board and port, then upload.

ESP32-CAM:

Open ESP32_CAM_Firmware.ino in Arduino IDE.

Install the ESPAsyncWebServer and AsyncTCP libraries.

Select AI Thinker ESP32-CAM as the board.

You will need a USB-to-UART adapter (like an FTDI programmer) to flash the ESP32-CAM.

3. Setting up the Python AI Client
Create a Python virtual environment (recommended):

bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
Install the required libraries:

bash
pip install opencv-python ultralytics requests
Place your custom-trained YOLOv8 model file in the project directory and update the path in the Python script:

python
model = YOLO("path/to/your/best.pt")
Run the script:

bash
python Python_AI_Client.py
🎮 Usage
Power on the RC car. The ESP32-CAM will create a WiFi network named Atom.

Connect your computer (which will run the Python AI client) to the Atom WiFi network.

Power on your Bluetooth gamepad and pair it with the ESP32 DevKit.

Run the Python_AI_Client.py script on your computer.

Open a web browser and go to http://192.168.4.1 to view the live camera feed.

Use the gamepad to drive the car to the area you want to monitor.

The AI client will now analyze the video feed. If a fire is detected consecutively, the pump will activate automatically.

📊 Performance
Video Stream: Stable VGA (640x480) resolution at 5-10 FPS.

Detection Accuracy: Successfully identifies fire with minimal false positives thanks to the debouncing logic.

Response Time: Pump activation occurs within ~1-2 seconds of a confirmed detection.

Control: Responsive and smooth motor control with effective joystick dead-zones.

🤝 Contributing
Contributions, issues, and feature requests are welcome! Feel free to check the issues page.

📜 License
This project is licensed for academic and personal use.
