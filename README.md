# ESP32 Ultrasonic Maze Solver 🤖

An autonomous **obstacle-avoiding maze-solving robot** built on an **ESP32**, navigating using three **HC-SR04 ultrasonic rangefinders** (left / front / right). A reactive **state-based controller** converts continuous distance readings into discrete wall states and drives a **finite-state navigation policy** with real-time Bluetooth Serial telemetry.

<p align="center">
  <a href="https://youtu.be/XvG3TrTDW1Y" target="_blank">
    <img src="docs/images/obstacle_bot.jpg" width="520" alt="Watch this short demo"/>
  </a>
  <br/>
  <em>▶️ Click the image to watch the demo on YouTube</em>
</p>

---

## ✨ Highlights

- **Three-sensor ultrasonic perception** — left / front / right HC-SR04 array
- **Discretized world model** — continuous range → binary wall states via threshold (15 cm)
- **Reactive FSM** — Forward / Adjust-Left / Adjust-Right / Full-Block-Stop
- **Pivot-turn correction loop** — bot rotates until front sensor clears wall
- **Live telemetry over Bluetooth Serial** for in-field debugging without USB tether
- **Hardware PWM** via ESP32 LEDC for smooth speed control on the TB6612FNG driver
- Echo timeout handling (treats `pulseIn` timeouts as "no wall" instead of garbage data)

---

## 🛠️ Hardware

| Component              | Part                          |
| ---------------------- | ----------------------------- |
| MCU                    | ESP32 DevKit V1               |
| Range Sensors          | HC-SR04 (×3 — L / F / R)      |
| Motor Driver           | TB6612FNG dual H-bridge       |
| Motors                 | N20 geared DC motors (×2)     |
| Power                  | 2S Li-ion 7.4 V               |
| Regulator              | LM2596 buck (7.4 V → 5 V)     |

---

## 🧠 Navigation Logic

The continuous distance from each sensor is thresholded into a binary "wall present" state, giving a 3-bit world descriptor `[L, F, R]`. The controller is a reactive FSM:

| State `[L F R]` | Action            | Behavior                                    |
| --------------- | ----------------- | ------------------------------------------- |
| `0 0 0`         | **Forward**       | Open corridor — cruise straight             |
| `1 0 0`         | **Adjust Left**   | Pivot left until front clears               |
| `0 0 1`         | **Adjust Right**  | Pivot right until front clears              |
| `1 1 1`         | **Stop**          | Dead-end — full block                       |
| *otherwise*     | Forward (default) | Conservative fallback                       |


---

## 📡 Telemetry

Bluetooth Serial (device name: `MazeBot_Logic_Update`) streams state transitions in real time. Pair from a phone Bluetooth terminal app to watch decisions live.

---

## 🚀 Future Work

- Replace reactive FSM with **flood-fill** or **DFS-with-backtracking** for true shortest-path maze solving
- Add **wheel encoders** for closed-loop odometry
- Migrate sensor stack to **VL53L0X ToF** for tighter, faster ranging
- Custom **KiCad PCB** integrating ESP32, TB6612FNG, and sensor headers

---

## 🏷️ Topics

![esp32](https://img.shields.io/badge/-esp32-000000?style=flat-square)
![robotics](https://img.shields.io/badge/-robotics-FF6F00?style=flat-square)
![autonomous-navigation](https://img.shields.io/badge/-autonomous--navigation-2E7D32?style=flat-square)
![maze-solver](https://img.shields.io/badge/-maze--solver-D32F2F?style=flat-square)
![ultrasonic](https://img.shields.io/badge/-ultrasonic-7B1FA2?style=flat-square)
![hc-sr04](https://img.shields.io/badge/-hc--sr04-455A64?style=flat-square)
![finite-state-machine](https://img.shields.io/badge/-finite--state--machine-1E90FF?style=flat-square)
![embedded-systems](https://img.shields.io/badge/-embedded--systems-37474F?style=flat-square)
![tb6612fng](https://img.shields.io/badge/-tb6612fng-5D4037?style=flat-square)
![pwm](https://img.shields.io/badge/-pwm-F57C00?style=flat-square)
![arduino](https://img.shields.io/badge/-arduino-00979D?style=flat-square)
![cpp](https://img.shields.io/badge/-cpp-00599C?style=flat-square)
![bluetooth](https://img.shields.io/badge/-bluetooth-0082FC?style=flat-square)
![reactive-control](https://img.shields.io/badge/-reactive--control-6A1B9A?style=flat-square)
![hardware](https://img.shields.io/badge/-hardware-263238?style=flat-square)
