/*
 * ============================================================================
 *  maze_solver.ino — SONAR Autonomous Maze Solver
 * ============================================================================
 *  Project : ESP32 Ultrasonic Maze Solver (SONAR)
 *  Author  : Moreshwar R
 *  Board   : ESP32 Dev Module
 *  Library : BluetoothSerial (built-in with ESP32 board package)
 *
 *  Description
 *  -----------
 *  Autonomous maze-navigating robot using three HC-SR04 ultrasonic
 *  rangefinders and a reactive finite-state controller.
 *
 *  Perception Layer
 *  ----------------
 *  Each loop iteration:
 *    1. Ping all three sensors (left / front / right)
 *    2. Threshold continuous distance into a binary "wall present" bit
 *    3. Combine into a 3-bit world state [L, F, R]
 *
 *  Decision Policy (Reactive FSM)
 *  ------------------------------
 *    [L F R] = 0 0 0  ->  Forward            (open corridor)
 *    [L F R] = 1 0 0  ->  Pivot Left         (until front clears)
 *    [L F R] = 0 0 1  ->  Pivot Right        (until front clears)
 *    [L F R] = 1 1 1  ->  Stop               (dead end)
 *    otherwise        ->  Forward (default conservative fallback)
 *
 *  Telemetry
 *  ---------
 *  Bluetooth Serial device name: "MazeBot_Logic_Update"
 *  Streams every state transition for live in-field debugging.
 *
 *  Safety / Robustness
 *  -------------------
 *  - pulseIn() timeout (25 ms) returns 100 cm = "no wall"
 *  - Sensors are read sequentially to avoid acoustic crosstalk
 *  - All motor commands clipped to valid PWM range
 *
 *  Baud rate: 115200
 * ============================================================================
 */

#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// ---------------------------------------------------------------------------
//  Pin Map — ESP32 → TB6612FNG Motor Driver
// ---------------------------------------------------------------------------
const int PWMA = 23;   // Motor A speed (PWM)
const int AIN1 = 21;   // Motor A direction 1
const int AIN2 = 22;   // Motor A direction 2

const int PWMB = 16;   // Motor B speed (PWM)
const int BIN1 = 18;   // Motor B direction 1
const int BIN2 = 4;    // Motor B direction 2

// ---------------------------------------------------------------------------
//  Pin Map — ESP32 → HC-SR04 Ultrasonic Sensors
// ---------------------------------------------------------------------------
const int TRIG_L = 27;  const int ECHO_L = 26;   // Left sensor
const int TRIG_F = 25;  const int ECHO_F = 33;   // Front sensor
const int TRIG_R = 32;  const int ECHO_R = 35;   // Right sensor  (GPIO 35 input-only — OK for ECHO)

// ---------------------------------------------------------------------------
//  Behavior Parameters
// ---------------------------------------------------------------------------
const int wallDist    = 15;   // cm — below this counts as "wall present"
const int cruiseSpeed = 40;   // PWM duty (0–255) when driving straight
const int turnSpeed   = 45;   // PWM duty when pivoting

// ===========================================================================
//  Setup
// ===========================================================================
void setup() {
  Serial.begin(115200);
  SerialBT.begin("MazeBot_Logic_Update");

  // --- Motor pins ---
  pinMode(PWMA, OUTPUT);  pinMode(AIN1, OUTPUT);  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);  pinMode(BIN1, OUTPUT);  pinMode(BIN2, OUTPUT);

  // --- Ultrasonic pins ---
  pinMode(TRIG_L, OUTPUT);  pinMode(ECHO_L, INPUT);
  pinMode(TRIG_F, OUTPUT);  pinMode(ECHO_F, INPUT);
  pinMode(TRIG_R, OUTPUT);  pinMode(ECHO_R, INPUT);

  // --- Hardware PWM (LEDC) — 5 kHz / 8-bit ---
  ledcAttach(PWMA, 5000, 8);
  ledcAttach(PWMB, 5000, 8);
}

// ===========================================================================
//  Main Loop — Perceive → Decide → Act
// ===========================================================================
void loop() {

  // -------- 1. PERCEIVE --------
  // Threshold continuous distance into a binary "wall present" bit
  bool left   = (getDist(TRIG_L, ECHO_L) < wallDist);
  bool center = (getDist(TRIG_F, ECHO_F) < wallDist);
  bool right  = (getDist(TRIG_R, ECHO_R) < wallDist);

  // -------- 2. DECIDE + ACT --------

  // [0 0 0] Open corridor — cruise forward
  if (!center && !left && !right) {
    SerialBT.println("BT: [ FORWARD ]");
    moveForward();
  }

  // [1 0 0] Wall on the left only — pivot left until front clears
  else if (left && !center && !right) {
    SerialBT.println("BT: [ ADJUST LEFT ]");
    while (getDist(TRIG_F, ECHO_F) < wallDist) {
      turnLeft();
    }
  }

  // [0 0 1] Wall on the right only — pivot right until front clears
  else if (right && !center && !left) {
    SerialBT.println("BT: [ ADJUST RIGHT ]");
    while (getDist(TRIG_F, ECHO_F) < wallDist) {
      turnRight();
    }
  }

  // [1 1 1] Dead end — full block, stop
  else if (left && center && right) {
    SerialBT.println("BT: [ FULL BLOCK - STOP ]");
    stopMotors();
  }

  // Any other state — conservative fallback
  else {
    moveForward();
  }

  delay(10);
}

// ===========================================================================
//  Movement Primitives
// ===========================================================================

void moveForward() {
  digitalWrite(AIN1, HIGH);  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);  digitalWrite(BIN2, LOW);
  ledcWrite(PWMA, cruiseSpeed);  ledcWrite(PWMB, cruiseSpeed);
}

void turnLeft() {
  // Pivot left — left wheel reverse, right wheel forward
  digitalWrite(AIN1, LOW);   digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);  digitalWrite(BIN2, LOW);
  ledcWrite(PWMA, turnSpeed);  ledcWrite(PWMB, turnSpeed);
}

void turnRight() {
  // Pivot right — left wheel forward, right wheel reverse
  digitalWrite(AIN1, HIGH);  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);   digitalWrite(BIN2, HIGH);
  ledcWrite(PWMA, turnSpeed);  ledcWrite(PWMB, turnSpeed);
}

void stopMotors() {
  ledcWrite(PWMA, 0);  ledcWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);  digitalWrite(BIN2, LOW);
}

// ===========================================================================
//  HC-SR04 Distance Measurement
//  Returns distance in cm. On echo timeout, returns 100 cm ("no wall").
// ===========================================================================
long getDist(int trig, int echo) {
  digitalWrite(trig, LOW);   delayMicroseconds(2);
  digitalWrite(trig, HIGH);  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 25000);   // 25 ms timeout
  if (duration <= 0) return 100;                // no echo → treat as open space

  // Convert µs to cm using the speed of sound (343 m/s)
  // distance = (duration × 0.0343) / 2  →  simplified to × 0.034 / 2
  return duration * 0.034 / 2;
}
