# Alex: Tele-operated search and rescue robot

CG2111A Engineering Principles and Practice II, Semester 2 2025/2026  
Team B04-G1, National University of Singapore

Alex is a tele-operated ground robot built to move through a maze-like arena, detect colour-coded medical supply zones, pick up medpaks with a 4-DoF robotic arm, and map the environment in real time using LIDAR. The robot is controlled by two operators at the same time, and the full mission has to be completed within an 8-minute window.

---

## Table of Contents

- [Team](#team)
- [System overview](#system-overview)
- [Hardware](#hardware)
- [Firmware (Arduino)](#firmware-arduino)
- [Software (Raspberry Pi)](#software-raspberry-pi)
- [Communication architecture](#communication-architecture)
- [Setup and running](#setup-and-running)
- [Dependencies](#dependencies)

---

## Team

| Name |
|---|
| Tan Yu Jun |
| Ong Ming Yu |
| Yew Kar Yik |
| Tripathi Anoushka |
| Zou Yuyang |

---

## System Overview

Alex uses a layered setup across two compute units:

- **Arduino Mega** handles the time-critical parts: motor PWM, servo PWM (Timer 5), colour sensing (Timer 2), E-stop ISR, and serial packet framing.
- **Raspberry Pi** acts as the coordination hub: it translates operator input into TPacket commands, relays sensor data to both operator terminals, hosts a TLS TCP server for the second terminal, and runs SLAM in a separate process.

Two operator laptops connect at the same time: Operator 1 through SSH for movement, camera, and colour control, and Operator 2 through a TLS TCP client (second_terminal.py) for arm control.



---

## Hardware

**Chassis:** Three-layer acrylic platform.

| Layer | Contents |
|---|---|
| Bottom | 4× DC gear motors, Arduino Mega + L293D motor shield, 8× AA battery pack, TCS3200 colour sensor, RPLIDAR |
| Middle | 4-DoF servo arm, 4× AA battery pack, power distribution breadboard, E-stop button |
| Top | Raspberry Pi, powerbank, elevated camera on angled riser |

**Key design decisions:**
- The heavy battery pack sits on the bottom layer to lower the centre of gravity and make driving more stable.
- The colour sensor is mounted at the very front underside of the chassis so it can detect zones before the robot fully enters them.
- The RPLIDAR is placed at the height of the first layer so it still has line-of-sight with short medpak bottles.
- We kept cable routing tight with zip ties to reduce the chance of loose wires interfering with movement or causing collision penalties.

---

## Firmware (Arduino)

All firmware runs in a fully non-blocking main loop, with no delay() calls. Five subsystems run together using ISRs and cooperative state machines:

### Motor control
Movement uses simple "game-like" WASD control: each directional command triggers one 300 ms burst, then stops automatically using `millis()`. This keeps the robot from overshooting in the narrow arena.

### Servo PWM- Timer 5 / PORTK
We implemented PWM in bare metal using Timer 5 in CTC mode at 2 MHz (prescaler 8, `OCR5A=39999`, which gives a 20 ms period). The `TIMER5_COMPB` ISR steps through 8 stages to drive all 4 arm joints at the same time with no noticeable jitter. We avoided Timer 1 because it is reserved by the L293D AFMotor library.

### Colour sensing- Timer 2
The TCS3200 outputs a frequency that changes with light intensity. Timer 2 in CTC mode fires every 1 ms, and `updateContinuousColor()` counts rising edges from `INT2` over a 100 ms window for each channel. A full R/G/B cycle takes about 300 ms, and it does this without blocking the rest of the firmware.

**Classifier:** k=1 Nearest Neighbour on a 5-colour training set (Red, Green, Blue, White, Brown) using squared Euclidean distance. This avoids floating-point square roots and is more robust than fixed thresholds when lighting changes.

### E-stop synchronisation
The hardware button and software `COMMAND_ESTOP` packets both use the same `volatile buttonState` and `buttonPhase` variables. A 4-phase press-release-press-release sequence keeps the software and hardware triggers aligned. A software E-stop fast-forwards `buttonPhase` to 2, and a software resume resets it to 0.

### Serial framing- TPacket
All communication uses a fixed 103-byte frame:

| Field | Size | Description |
|---|---|---|
| Magic number | 2 B | 0xDEAD-frame sync marker |
| Packet type | 1 B | COMMAND/RESPONSE / MESSAGE |
| Command/Response | 1 B | Specific action type |
| Padding | 2 B | Byte alignment |
| Data string | 32 B | Null-terminated ASCII label |
| Params | 64 B | 16 × uint32 (angles, speeds, RGB values) |
| XOR checksum | 1 B | XOR of all 100 TPacket bytes |

---

## Software (Raspberry Pi)

Entry point: `pi_sensor.py`

### Main loop- `runCommandInterface()`
The main event loop is non-blocking and checks four sources in sequence:
1. Incoming Arduino packets via `receiveFrame()` -> `printPacket()`
2. Forwarding of received packets to the second terminal via `relay.onPacketReceived()`
3. Keyboard input from `stdin` using `select.select()` with zero timeout
4. Incoming arm commands from the second terminal, which are forwarded to the Arduino over serial

A 20 ms sleep at the end keeps CPU usage under control without adding noticeable latency. SLAM runs as a separate process and does not block this loop.

### Arm control- "align and extend" strategy
The arm does not have a dedicated wrist joint, and the camera angle makes depth estimation unreliable. To work around that, we used a pre-calibrated pickup procedure:

1. **Align**- Operator 2 rotates the base servo until the arm is centred horizontally on the medpak.
2. **Measure**- Operator 1 captures a photo and measures the medpak's pixel distance from the bottom of the frame against a physical ruler.
3. **Extend and grasp**- Operator 2 looks up the pre-calibrated shoulder and elbow angles for that screen distance. The angles are locked to a fixed gripper height of 13.8 cm, so the gripper reaches the bottle at a consistent level.

During navigation, the arm stays in a "home" position so the robot takes up less space and can turn more easily.

### Colour detection
Press `c` to toggle continuous streaming mode. The Arduino cycles through R/G/B measurements and sends `RESP_COLOR` packets in real time. 

### SLAM visualisation- `slam_client.py`
We replaced the default ASCII terminal map with a Python GUI. Open space is shown in white, walls in dark purple, and the robot position as a blue box. It is much easier to read than the text map, especially during fast navigation.

---

## Communication Architecture

There are three active channels during a mission:

| Channel | Protocol | Details |
|---|---|---|
| Arduino <-> Pi | USART Serial, 9600 baud 8N1 | All TPacket frames, bare-metal circular-buffer USART driver |
| Pi <-> Second terminal | TLS TCP, port 65432 | Self-signed certificate, mutual verification, encrypts arm commands on the NUS network |
| Operator laptops <-> Pi | Tailscale | Stable peer-to-peer tunnel over the NUS network via Tailscale IP |

E-stop safety is enforced on both ends: `pi_sensor.py` refuses to forward any command when E-stop is active, and `second_terminal.py` mirrors the E-stop state from `RESP_STATUS` packets and also stops sending commands.

---


---

## Setup and Running

### Prerequisites
- Raspberry Pi running Raspberry Pi OS 
- Arduino Mega with L293D motor shield
- RPLIDAR connected to the Pi
- Python 3.8+ on the Pi and both operator laptops
- Tailscale installed on all three machines

### Pi setup
```bash
# Install Python dependencies
pip install pyserial rplidar-roboticia

# Generate TLS certificate (first time only)
openssl req -x509 -newkey rsa:4096 -keyout certs/server.key -out certs/server.crt -days 365 -nodes

# Flash arduino/alex.ino to the Arduino Mega via Arduino IDE

# Start the main control interface
python3 pi/pi_sensor.py

# In a separate terminal, start the SLAM visualiser
python3 pi/slam_client.py
```

### Operator 2 setup (second terminal)
```bash
# Copy certs/server.crt from the Pi to this machine
python3 pi/second_terminal.py --host <TAILSCALE_PI_IP> --cert server.crt
```

### Operator 1 controls (pi_sensor.py)
| Key | Action |
|---|---|
| `w` / `s` / `a` / `d` | Forward / back / left / right (300 ms burst) |
| `q` | Emergency stop |
| `c` | Toggle continuous colour streaming |
| `p` | Capture camera image |

### Operator 2 controls (second_terminal.py)
| Command | Action |
|---|---|
| `b<angle>` | Rotate base servo (e.g. `b090`) |
| `s<angle>` | Shoulder angle |
| `e<angle>` | Elbow angle |
| `g<angle>` | Gripper open/close |
| `home` | Return arm to retracted home position |

---

## Dependencies

| Component | Library / Tool |
|---|---|
| Arduino motor control | AFMotor (L293D shield) |
| Arduino serial | Bare-metal USART (no library) |
| Arduino servo | Bare-metal Timer 5 (no Servo.h) |
| Pi serial | `pyserial` |
| Pi SLAM | `rplidar-roboticia` |
| Pi TLS server | Python `ssl` (stdlib) |
| Networking | Tailscale |

---


