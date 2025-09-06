#include "esp_camera.h"
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#define LIGHT_PIN 4
#define PUMP_RELAY_PIN 13  // Pin connected to the relay for the water pump

// Camera configuration
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// WiFi credentials
const char* ssid = "Atom";
const char* password = "12345678";

// Web server and WebSocket
AsyncWebServer server(80);
AsyncWebSocket wsCamera("/Camera");
AsyncWebSocket wsCarInput("/CarInput");
uint32_t cameraClientId = 0;
bool lightState = false;
bool pumpState = false;  // Track pump state

// HTML page
const char htmlHomePage[] PROGMEM = R"HTMLHOMEPAGE(
<!DOCTYPE html>
<html>
  <head>
    <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
    <style>
      body {
        background-color: #FFA500;
        text-align: center;
        font-family: Arial, sans-serif;
        margin: 0;
        padding: 20px 0;
      }
      .header {
        font-size: 36px;
        font-weight: bold;
        color: #333;
        text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        margin-bottom: 20px;
        letter-spacing: 2px;
        background: linear-gradient(to right, #FF4500, #FF8C00);
        -webkit-background-clip: text;
        -webkit-text-fill-color: transparent;
        padding: 10px;
      }
      .light-button {
        background-color: #4CAF50;
        border-radius: 50%;
        width: 120px;
        height: 120px;
        margin: 20px auto;
        display: flex;
        align-items: center;
        justify-content: center;
        font-size: 24px;
        color: white;
        box-shadow: 0 4px 8px rgba(0,0,0,0.2);
        cursor: pointer;
        user-select: none;
        transition: all 0.2s;
      }
      .light-button:active {
        transform: translateY(4px);
        box-shadow: 0 2px 4px rgba(0,0,0,0.2);
      }
      #cameraImage {
        border: 5px solid black;
        border-radius: 10px;
        box-shadow: 0 4px 8px rgba(0,0,0,0.2);
        margin-bottom: 20px;
      }
      .fire-warning {
        display: none;
        position: fixed;
        top: 50%;
        left: 50%;
        transform: translate(-50%, -50%);
        background-color: #FF4500;
        color: white;
        padding: 20px;
        border-radius: 10px;
        box-shadow: 0 0 20px rgba(255,0,0,0.5);
        z-index: 1000;
        font-size: 24px;
        font-weight: bold;
        text-align: center;
        animation: pulse 1s infinite;
      }
      @keyframes pulse {
        0% { transform: translate(-50%, -50%) scale(1); }
        50% { transform: translate(-50%, -50%) scale(1.05); }
        100% { transform: translate(-50%, -50%) scale(1); }
      }
    </style>
  </head>
  <body>
    <div class="header">ATOM's CAMERA</div>
    <div style="width:400px; margin:auto;">
      <img id="cameraImage" src="" style="width:400px, height:250px;">
      <div class="light-button" onclick="toggleLight()">
        TOGGLE LIGHT
      </div>
    </div>
    
    <div id="fireWarning" class="fire-warning">
       FIRE DETECTED! 
    </div>
  
    <script>
      var websocketCamera;
      var websocketCarInput;

      function initCameraWebSocket() {
        var webSocketCameraUrl = "ws://" + window.location.hostname + "/Camera";
        websocketCamera = new WebSocket(webSocketCameraUrl);
        websocketCamera.binaryType = 'blob';
        websocketCamera.onclose = function() {
          setTimeout(initCameraWebSocket, 2000);
        };
        websocketCamera.onmessage = function(event) {
          document.getElementById("cameraImage").src = URL.createObjectURL(event.data);
        };
      }
      
      function initCarInputWebSocket() {
        var webSocketCarInputUrl = "ws://" + window.location.hostname + "/CarInput";
        websocketCarInput = new WebSocket(webSocketCarInputUrl);
        websocketCarInput.onclose = function() {
          setTimeout(initCarInputWebSocket, 2000);
        };
        websocketCarInput.onmessage = function(event) {
          if (event.data === "FireDetected") {
            document.getElementById("fireWarning").style.display = "block";
          } else if (event.data === "FireCleared") {
            document.getElementById("fireWarning").style.display = "none";
          }
        };
      }
      
      function initWebSocket() {
        initCameraWebSocket();
        initCarInputWebSocket();
      }

      function toggleLight() {
        if (websocketCarInput && websocketCarInput.readyState === WebSocket.OPEN) {
          websocketCarInput.send("ToggleLight");
        }
      }
    
      window.onload = initWebSocket;
    </script>
  </body>    
</html>
)HTMLHOMEPAGE";

// Web server handlers
void handleRoot(AsyncWebServerRequest *request) {
  request->send_P(200, "text/html", htmlHomePage);
}

void handleNotFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "File Not Found");
}

// Endpoint for Python to control the pump
void handlePumpControl(AsyncWebServerRequest *request) {
  if (request->hasParam("state")) {
    String state = request->getParam("state")->value();
    if (state == "on") {
      digitalWrite(PUMP_RELAY_PIN, HIGH);
      pumpState = true;
      request->send(200, "text/plain", "PUMP turned ON");
      Serial.println("PUMP turned ON via HTTP");
      wsCarInput.textAll("FireDetected");  // Notify web clients
    } else if (state == "off") {
      digitalWrite(PUMP_RELAY_PIN, LOW);
      pumpState = false;
      request->send(200, "text/plain", "PUMP turned OFF");
      Serial.println("PUMP turned OFF via HTTP");
      wsCarInput.textAll("FireCleared");  // Notify web clients
    } else {
      request->send(400, "text/plain", "Invalid state parameter");
    }
  } else {
    request->send(400, "text/plain", "Missing state parameter");
  }
}

// Single frame capture endpoint
void handleCapture(AsyncWebServerRequest *request) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    request->send(500, "text/plain", "Camera capture failed");
    return;
  }
  
  request->send_P(200, "image/jpeg", (const uint8_t*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

// WebSocket event handlers
void onCarInputWebSocketEvent(AsyncWebSocket *server, 
                            AsyncWebSocketClient *client, 
                            AwsEventType type,
                            void *arg, 
                            uint8_t *data, 
                            size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected\n", client->id());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      digitalWrite(LIGHT_PIN, LOW);
      lightState = false;
      break;
    case WS_EVT_DATA:
      {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
          data[len] = 0;
          String message = String((char*)data);
          
          Serial.printf("Command: %s\n", message.c_str());
          
          if (message == "ToggleLight") {
            lightState = !lightState;
            digitalWrite(LIGHT_PIN, lightState ? HIGH : LOW);
            Serial.printf("Light set to: %s\n", lightState ? "ON" : "OFF");
          }
        }
      }
      break;
    default:
      break;
  }
}

void onCameraWebSocketEvent(AsyncWebSocket *server,
                          AsyncWebSocketClient *client,
                          AwsEventType type,
                          void *arg,
                          uint8_t *data,
                          size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("Camera client #%u connected\n", client->id());
      cameraClientId = client->id();
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("Camera client #%u disconnected\n", client->id());
      cameraClientId = 0;
      break;
    default:
      break;
  }
}

// Camera setup
void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if(psramFound()) {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    ESP.restart();
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_VGA);
  
  digitalWrite(LIGHT_PIN, HIGH);
  lightState = true;
  Serial.println("Camera initialized - Light turned ON");
}

// Camera picture sending for WebSocket
void sendCameraPicture() {
  if (cameraClientId == 0) return;

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Failed to get camera frame");
    return;
  }

  wsCamera.binary(cameraClientId, fb->buf, fb->len);
  esp_camera_fb_return(fb);

  while (wsCamera.client(cameraClientId)->queueIsFull()) {
    delay(1);
  }
}

// Pin initialization
void setUpPinModes() {
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(PUMP_RELAY_PIN, OUTPUT);
  
  // Ensure relay starts in OFF position
  digitalWrite(LIGHT_PIN, LOW);
  digitalWrite(PUMP_RELAY_PIN, LOW);
  
  // Double-check relay state with delay
  delay(100);
  digitalWrite(PUMP_RELAY_PIN, LOW);
  Serial.println("Pins initialized - Relay confirmed OFF");
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  delay(1000);

  setUpPinModes();

  // Start WiFi
  WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  // Setup web server with endpoints
  server.on("/", HTTP_GET, handleRoot);
  server.on("/pump", HTTP_GET, handlePumpControl);  // Pump control endpoint
  server.on("/capture", HTTP_GET, handleCapture);   // Single frame endpoint
  server.onNotFound(handleNotFound);

  // Setup WebSockets
  wsCamera.onEvent(onCameraWebSocketEvent);
  server.addHandler(&wsCamera);
  wsCarInput.onEvent(onCarInputWebSocketEvent);
  server.addHandler(&wsCarInput);

  server.begin();
  Serial.println("HTTP server started");
  Serial.println("Available endpoints:");
  Serial.println("  - /         : Web interface");
  Serial.println("  - /capture  : Single JPEG frame");
  Serial.println("  - /pump     : Control pump (state=on/off)");

  setupCamera();
}

void loop() {
  wsCamera.cleanupClients();
  wsCarInput.cleanupClients();
  sendCameraPicture();

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 5000) {
    lastPrint = millis();
    Serial.printf("System status - Heap: %d, Min Heap: %d, Light: %s, Pump: %s\n", 
                 ESP.getFreeHeap(), 
                 ESP.getMinFreeHeap(),
                 lightState ? "ON" : "OFF",
                 pumpState ? "ON" : "OFF");
  }
}