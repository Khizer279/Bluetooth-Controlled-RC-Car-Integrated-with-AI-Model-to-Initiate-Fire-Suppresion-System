#include <Bluepad32.h>
#include <driver/ledc.h>
#include <ESP32Servo.h>

// ================= PIN CONFIG =================
#define ENA 5      // Left motor PWM
#define IN1 18     // Left motor forward
#define IN2 19     // Left motor backward
#define ENB 17     // Right motor PWM
#define IN3 16     // Right motor forward
#define IN4 4      // Right motor backward

#define SERVO_PIN_1 25  // Right joystick Y-axis
#define SERVO_PIN_2 26  // Right joystick X-axis

// ================= PARAMETERS =================
const int DEADZONE = 60;        // Increased from 15 (proportional to raw range)
const int SERVO_DEADZONE = 80;  // Increased from 20
const int ANGLE_CHANGE_THRESHOLD = 2;
const int MAX_SPEED = 180;
const int MAX_JOYSTICK = 512;   // Maximum raw joystick value

// LEDC Configuration
const int LEDC_FREQUENCY = 1000;  // 1kHz
const ledc_timer_bit_t LEDC_RESOLUTION = LEDC_TIMER_8_BIT;
const ledc_timer_t LEDC_TIMER = LEDC_TIMER_0;
const ledc_mode_t LEDC_MODE = LEDC_LOW_SPEED_MODE;

// Serial output control
const unsigned long SERIAL_UPDATE_INTERVAL = 500;
unsigned long lastSerialOutputTime = 0;

// Servo objects
Servo servo1;
Servo servo2;

// PWM Channels
#define LEFT_MOTOR_CHANNEL LEDC_CHANNEL_0
#define RIGHT_MOTOR_CHANNEL LEDC_CHANNEL_1

// Bluepad32 controllers
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// ================= HELPER FUNCTIONS =================
void setupLEDC() {
  // (Keep the same as original)
  ledc_timer_config_t timer_conf;
  timer_conf.speed_mode = LEDC_MODE;
  timer_conf.duty_resolution = LEDC_RESOLUTION;
  timer_conf.timer_num = LEDC_TIMER;
  timer_conf.freq_hz = LEDC_FREQUENCY;
  timer_conf.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&timer_conf);

  ledc_channel_config_t left_motor_conf;
  left_motor_conf.gpio_num = ENA;
  left_motor_conf.speed_mode = LEDC_MODE;
  left_motor_conf.channel = LEFT_MOTOR_CHANNEL;
  left_motor_conf.intr_type = LEDC_INTR_DISABLE;
  left_motor_conf.timer_sel = LEDC_TIMER;
  left_motor_conf.duty = 0;
  left_motor_conf.hpoint = 0;
  ledc_channel_config(&left_motor_conf);

  ledc_channel_config_t right_motor_conf;
  right_motor_conf.gpio_num = ENB;
  right_motor_conf.speed_mode = LEDC_MODE;
  right_motor_conf.channel = RIGHT_MOTOR_CHANNEL;
  right_motor_conf.intr_type = LEDC_INTR_DISABLE;
  right_motor_conf.timer_sel = LEDC_TIMER;
  right_motor_conf.duty = 0;
  right_motor_conf.hpoint = 0;
  ledc_channel_config(&right_motor_conf);
}

int smoothAngle(int current, int target, int step = 2) {
  if (abs(current - target) <= step) return target;
  return (target > current) ? current + step : current - step;
}

void controlMotor(int in1, int in2, ledc_channel_t channel, int speed) {
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  ledc_set_duty(LEDC_MODE, channel, abs(speed));
  ledc_update_duty(LEDC_MODE, channel);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledc_set_duty(LEDC_MODE, LEFT_MOTOR_CHANNEL, 0);
  ledc_set_duty(LEDC_MODE, RIGHT_MOTOR_CHANNEL, 0);
  ledc_update_duty(LEDC_MODE, LEFT_MOTOR_CHANNEL);
  ledc_update_duty(LEDC_MODE, RIGHT_MOTOR_CHANNEL);
}


void processController(ControllerPtr ctl) {
  if (!ctl->isConnected() || !ctl->isGamepad()) return;

  // Read raw joystick values (-511 to 512)
  int stickY = ctl->axisY();
  int stickX = ctl->axisX();
  int rightX = ctl->axisRX();
  int rightY = ctl->axisRY();

  // Apply deadzones for motors only
  if (abs(stickY) <= DEADZONE) stickY = 0;
  if (abs(stickX) <= DEADZONE) stickX = 0;

  // ===== MOTOR CONTROL (unchanged) =====
  if (stickY == 0 && stickX == 0) {
    stopMotors();
  }
  else {
    int baseSpeed = map(abs(stickY), 0, MAX_JOYSTICK, 0, MAX_SPEED);
    int turnEffect = map(abs(stickX), 0, MAX_JOYSTICK, 0, MAX_SPEED);

    int leftSpeed = 0;
    int rightSpeed = 0;

    if (stickY != 0) {
      leftSpeed = (stickY > 0) ? baseSpeed : -baseSpeed;
      rightSpeed = leftSpeed;

      if (stickX != 0) {
        if (stickX > 0) {
          leftSpeed -= turnEffect;
          rightSpeed += turnEffect;
        } else {
          leftSpeed += turnEffect;
          rightSpeed -= turnEffect;
        }
      }
    } else {
      if (stickX > 0) {
        leftSpeed = -turnEffect;
        rightSpeed = turnEffect;
      } else {
        leftSpeed = turnEffect;
        rightSpeed = -turnEffect;
      }
    }

    leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

    controlMotor(IN1, IN2, LEFT_MOTOR_CHANNEL, leftSpeed);
    controlMotor(IN3, IN4, RIGHT_MOTOR_CHANNEL, rightSpeed);
  }

  // ===== SERVO CONTROL (from your second code) =====
  if (rightX) {
    int servoPosX = map(rightX, -508, 512, 0, 180);
    servo2.write(servoPosX);  // servo2 = horizontal
  }

  if (rightY) {
    int servoPosY = map(rightY, -508, 512, 0, 180);
    servo1.write(servoPosY);  // servo1 = vertical
  }

  // Debug output
  if (millis() - lastSerialOutputTime >= SERIAL_UPDATE_INTERVAL) {
    Serial.printf("LY:%d LX:%d | RY:%d RX:%d | Motors L:%d R:%d | Servos:%d,%d\n",
                stickY, stickX, rightY, rightX,
                ledc_get_duty(LEDC_MODE, LEFT_MOTOR_CHANNEL),
                ledc_get_duty(LEDC_MODE, RIGHT_MOTOR_CHANNEL),
                servo1.read(), servo2.read());
    lastSerialOutputTime = millis();
  }
}

// ================= BLUEPAD32 CALLBACKS =================
// (Keep the same as original)
void onConnectedController(ControllerPtr ctl) {
  Serial.print("Controller connected: ");
  Serial.println(ctl->getModelName().c_str());
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      myControllers[i] = ctl;
      return;
    }
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  Serial.println("Controller disconnected");
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
      stopMotors();
      servo1.write(90);
      servo2.write(90);
      return;
    }
  }
}

// ================= SETUP =================
// (Keep the same as original)
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Starting Robot Controller...");

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  setupLEDC();
  stopMotors();

  servo1.attach(SERVO_PIN_1);
  servo2.attach(SERVO_PIN_2);
  servo1.write(90);
  servo2.write(90);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);

  Serial.println("System ready - waiting for controller...");
}

// ================= LOOP =================
// (Keep the same as original)
void loop() {
  BP32.update();
  for (auto controller : myControllers) {
    if (controller && controller->isConnected()) {
      processController(controller);
    }
  }
}