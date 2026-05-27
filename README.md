# Smart Traffic Light Controller using QNX RTOS

> Real-time intelligent traffic light controller built on QNX SDP 8 and Raspberry Pi 4 with GPIO memory mapping, multithreading, ultrasonic sensing, adaptive timing, and a built-in web dashboard.
---

## 📌 Overview
This project implements a **Smart Traffic Light Controller** running on **QNX SDP 8** using a **Raspberry Pi 4**.
The system combines:
- Real-time traffic signal control
- Adaptive traffic management
- Emergency override handling
- Peak-hour optimization
- Sensor-based vehicle detection
- Embedded HTTP web dashboard
- Multithreaded RTOS architecture

The controller directly accesses Raspberry Pi GPIO registers using **memory-mapped I/O (`mmap`)**, without external GPIO libraries.

---

# ✨ Features

## 🚦 Traffic Light FSM

- RED → GREEN → YELLOW cycle
- Tick-based finite state machine
- Deterministic timing behavior

## 🚑 Emergency Mode

- Green LED blinking
- Current FSM state preserved
- Seamless resume after emergency clear

## 📏 Distance-Based Smart Mode

Uses an HC-SR04 ultrasonic sensor to detect nearby vehicles.

| Distance | Traffic State |
|----------|----------------|
| < 50 cm | GREEN |
| 50–100 cm | YELLOW |
| > 100 cm | RED |

---

## 🕒 Peak-Hour Optimization

Automatically adjusts traffic timings based on time of day.

### Peak Hours
- 08:00 – 10:00
- 17:00 – 19:00

### Timing Adjustments

| Mode | RED | GREEN | YELLOW |
|------|------|--------|---------|
| Peak | 3s | 8s | 3s |
| Off-Peak | 7s | 5s | 3s |

---

## 🌐 Embedded Web Dashboard

Built-in HTTP server running on port `8080`.

### Endpoints

| Endpoint | Function |
|----------|-----------|
| `/` | Dashboard UI |
| `/state` | JSON system state |
| `/cmd` | Remote commands |

### Dashboard Features

- Live traffic signal visualization
- Current mode display
- Remaining timer
- Distance monitoring
- Peak/off-peak indication
- Remote control buttons

---

# System Architecture

```text
+--------------------------------------------------+
|              Smart Traffic Controller            |
+--------------------------------------------------+
|                                                  |
|   FSM Thread          -> Traffic light control   |
|   Sensor Thread       -> HC-SR04 distance read   |
|   HTTP Thread         -> Web dashboard           |
|   Peak Scheduler      -> Adaptive timing         |
|   Logger Thread       -> System monitoring       |
|   Command Thread      -> User commands           |
|                                                  |
+--------------------------------------------------+
                |
                v
+------------------------------------+
| GPIO Memory-Mapped Register Access |
+------------------------------------+
                |
                v
+------------------------------------+
| LEDs + HC-SR04 Sensor              |
+------------------------------------+
````

---

# Multithreading Design

| Thread         | Priority | Purpose               |
| -------------- | -------- | --------------------- |
| Command Thread | 25       | User commands         |
| FSM Thread     | 20       | Traffic light control |
| Sensor Thread  | 15       | Ultrasonic sensor     |
| HTTP Thread    | 12       | Web dashboard         |
| Peak Scheduler | 11       | Time-based control    |
| Logger Thread  | 10       | Status logging        |

Scheduling policy:

```c
SCHED_RR
```

---

# Hardware Requirements

## Components

* Raspberry Pi 4
* HC-SR04 Ultrasonic Sensor
* Red LED
* Yellow LED
* Green LED
* 330Ω Resistors
* Breadboard & Jumper Wires

---

# 🔧 GPIO Connections

| Component    | GPIO   | Physical Pin |
| ------------ | ------ | ------------ |
| Red LED      | GPIO17 | Pin 11       |
| Yellow LED   | GPIO27 | Pin 13       |
| Green LED    | GPIO22 | Pin 15       |
| HC-SR04 TRIG | GPIO23 | Pin 16       |
| HC-SR04 ECHO | GPIO24 | Pin 18       |

### Power Connections

* 3.3V → HC-SR04 VCC
* GND → Common Ground

---
# ⚙️ Software Requirements

* QNX SDP 8
* QNX Momentics Toolchain
* GCC Compiler
* Raspberry Pi 4 BSP
* SSH & SCP utilities
---

# 🛠️ Build Instructions

## Compile

```bash
make PLATFORM=aarch64le BUILD_PROFILE=debug
```
---

# 🚀 Deployment

## Copy executable to Raspberry Pi

```bash
scp build/aarch64le-debug/traffic_light root@10.0.0.1:/tmp/
```

## Run application

```bash
ssh root@10.0.0.1 "/tmp/traffic_light"```

> Must run as root because GPIO registers are accessed using physical memory mapping.
---

# 🌐 Web Dashboard

Open browser:

```text
http://10.0.0.1:8080
```

---

# 🎮 Console Commands

| Command | Action             |
| ------- | ------------------ |
| `e`     | Emergency mode     |
| `n`     | Clear emergency    |
| `h`     | Distance mode      |
| `g`     | Fixed cycle mode   |
| `p`     | Toggle peak timing |
| `q`     | Quit application   |

---

# 📂 Project Structure

```text
traffic_light/
│
├── traffic_light.c
├── Makefile
├── README.md
└── build/
```

---

# 🔍 Key Technical Concepts

* Real-Time Operating Systems (RTOS)
* Finite State Machines (FSM)
* POSIX Threads
* Mutex Synchronization
* GPIO Memory Mapping
* Embedded HTTP Server
* Adaptive Traffic Management
* Ultrasonic Distance Measurement
* Priority-Based Scheduling
---

# 📈 Future Improvements

* AI-based vehicle detection
* Camera integration
* Multi-junction synchronization
* CAN bus communication
* Interrupt-driven GPIO
* MQTT/IoT integration
* Cloud analytics dashboard

---

# 🎓 Learning Outcomes

This project demonstrates:
* Embedded systems programming
* QNX RTOS development
* Concurrent programming
* Low-level hardware interfacing
* Real-time scheduling
* IoT dashboard integration

---

# 👨‍💻 Author

## Jailakshmi Kangula
Branch : Electronics and Communication Engineering (ECE)

Interests :
* Embedded Systems
* RTOS
* FPGA Design
* AI-driven Automation
* IoT Systems

---

# 📜 License

This project is open-source and available under the MIT License.
