# ESP32 Micro-ROS Robot Controller

## Complete Code Analysis & Documentation

---

## 🎯 Project Overview

This is a **micro-ROS robot controller** running on an **ESP32** that communicates with a ROS 2 system over serial. It controls a differential/single-motor robot with steering servo, LEDs, ultrasonic distance sensing, and encoder-based precision movement.

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────┐
│                    ROS 2 Host PC                     │
│                                                     │
│  Publisher: /LEDs (Int8)                            │
│  Publisher: /servo (Int8)                           │
│  Publisher: /motor_cmd (String)                     │
│  Subscriber: /ultrasonic (Int16)                    │
└──────────────────┬──────────────────────────────────┘
                   │ Serial (micro-ROS agent)
┌──────────────────▼──────────────────────────────────┐
│                   ESP32 Node                         │
│              "ESP32_Robot_Node"                       │
│                                                     │
│  ┌─────────┐ ┌───────┐ ┌───────┐ ┌──────────────┐  │
│  │  LEDs   │ │ Servo │ │ Motor │ │  Ultrasonic   │  │
│  │Subscr.  │ │Subscr.│ │Subscr.│ │  Publisher    │  │
│  └────┬────┘ └───┬───┘ └───┬───┘ └──────┬───────┘  │
│       │          │         │             │          │
│  ┌────▼────┐ ┌───▼───┐ ┌───▼────────┐ ┌─▼────────┐│
│  │GPIO 4,  │ │Servo  │ │PID Motor   │ │HC-SR04   ││
│  │32, 33   │ │Pin 25 │ │+ Encoder   │ │Pins 27,26││
│  └─────────┘ └───────┘ └────────────┘ └──────────┘│
└─────────────────────────────────────────────────────┘
```

---

## 📌 Pin Mapping

```
Component          Pin(s)        Type
─────────────────────────────────────────
Buzzer             14            Output
Left LED           33            Output
Right LED          32            Output
Front LED          4             Output
Servo              25            PWM (50Hz)
Motor ENA          21            PWM (5kHz, Timer 0)
Motor IN1          19            Digital Output
Motor IN2          18            Digital Output
Encoder A          22            Interrupt Input
Encoder B          23            Interrupt Input
Ultrasonic TRIG    27            Digital Output
Ultrasonic ECHO    26            Interrupt Input
```

---

## 📡 ROS 2 Topics — Inputs & Outputs

### Subscribers (Inputs from ROS 2 → ESP32)

```
┌────────────┬────────┬─────────────────────────────────────┐
│ Topic      │ Type   │ Values & Meaning                    │
├────────────┼────────┼─────────────────────────────────────┤
│ /LEDs      │ Int8   │ 0: Both OFF                        │
│            │        │ 1: Left ON, Right OFF              │
│            │        │ 2: Left OFF, Right ON              │
│            │        │ 3: Both ON                         │
│            │        │ 4: Front LED ON                    │
│            │        │ 5: Front LED OFF                   │
├────────────┼────────┼─────────────────────────────────────┤
│ /servo     │ Int8   │ -6 to +6                           │
│            │        │ Multiplied by 10, added to 90°     │
│            │        │ So: 30° to 150° servo range        │
│            │        │ 0 = center (90°)                   │
├────────────┼────────┼─────────────────────────────────────┤
│ /motor_cmd │ String │ "s <rpm>"      → continuous speed  │
│            │        │ "m <deg> <rpm>" → move degrees     │
│            │        │ "c <cm> <rpm>"  → move centimeters │
│            │        │ "x"            → stop              │
│            │        │ "r"            → reset encoder     │
└────────────┴────────┴─────────────────────────────────────┘
```

### Publisher (Output from ESP32 → ROS 2)

```
┌──────────────┬───────┬──────────────────────────────────┐
│ Topic        │ Type  │ Meaning                          │
├──────────────┼───────┼──────────────────────────────────┤
│ /ultrasonic  │ Int16 │ Distance in cm (2-400)           │
│              │       │ -1 = invalid/timeout reading     │
│              │       │ Published every 100ms            │
└──────────────┴───────┴──────────────────────────────────┘
```

---

## 🔧 Key Technical Solutions

### 1. Micro-ROS Connection State Machine (The Big Fix)

The biggest problem with basic micro-ROS examples is they **crash and hang forever** when the agent disconnects. This code implements a **4-state auto-reconnect system**:

```
┌──────────────┐     ping OK     ┌─────────────────┐
│              │ ───────────────► │                 │
│ WAITING_AGENT│                  │ AGENT_AVAILABLE │
│              │ ◄─────────────── │                 │
└──────────────┘  create failed   └────────┬────────┘
       ▲                                    │
       │                             create success
       │                                    │
       │          ┌─────────────────┐       │
       │          │                 │ ◄─────┘
       │          │ AGENT_CONNECTED │
       │          │                 │
       │          └────────┬────────┘
       │                   │
       │            spin error or
       │            ping timeout
       │                   │
       │          ┌────────▼────────┐
       └───────── │    AGENT_       │
    after cleanup │ DISCONNECTED    │
                  └─────────────────┘
```

**What each state does:**

```c
WAITING_AGENT:     Pings agent every 100ms, no entities exist
AGENT_AVAILABLE:   Agent found, creates all nodes/topics/executor
AGENT_CONNECTED:   Normal operation, spins executor, pings every 1s
AGENT_DISCONNECTED: Destroys all entities, stops motors, resets servo
```

### 2. All Six Fixes Explained

```
┌─────────────────┬──────────────────────┬──────────────────────────────┐
│ Issue           │ Broken Default       │ This Code's Fix              │
├─────────────────┼──────────────────────┼──────────────────────────────┤
│ Connection      │ error_loop() hangs   │ State machine retries        │
│                 │ forever on failure   │ automatically, never blocks  │
│                 │                      │                              │
│ QoS (Quality    │ RELIABLE: waits for  │ BEST_EFFORT: fire-and-forget │
│ of Service)     │ acknowledgment,      │ No waiting, no blocking,     │
│                 │ causes lag/timeout   │ fast for real-time control   │
│                 │                      │                              │
│ Executor        │ spin_some(10ms)      │ spin_some(0) = instant       │
│ timeout         │ blocks loop 10ms     │ return if no messages,       │
│                 │ per cycle            │ loop runs at full speed      │
│                 │                      │                              │
│ Watchdog        │ Loop runs too fast,  │ delayMicroseconds(100)       │
│                 │ ESP32 watchdog       │ yields CPU briefly,          │
│                 │ can trigger reset    │ prevents watchdog reset      │
│                 │                      │                              │
│ Disconnect      │ No cleanup, leaked   │ destroyEntities() properly   │
│ cleanup         │ memory, can't        │ frees everything, allows     │
│                 │ reconnect            │ clean reconnection           │
│                 │                      │                              │
│ Timer conflict  │ ESP32 Timer 0 used   │ Timer 0 → motor PWM only    │
│                 │ by both servo and    │ Timers 1,2,3 → servo lib    │
│                 │ motor PWM            │ No conflict                  │
└─────────────────┴──────────────────────┴──────────────────────────────┘
```

**Timer allocation specifically:**
```c
// Servo gets timers 1, 2, 3
ESP32PWM::allocateTimer(1);
ESP32PWM::allocateTimer(2);
ESP32PWM::allocateTimer(3);

// Motor PWM uses timer 0 (via ledcSetup channel 0)
#define PWM_CHANNEL 0
ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
```

---

## 🔊 Ultrasonic Sensor — Non-Blocking Design & Calibration

### The Problem with Standard Ultrasonic Code

```c
// TYPICAL BLOCKING CODE (BAD):
digitalWrite(TRIG, HIGH);
delayMicroseconds(10);        // blocks
digitalWrite(TRIG, LOW);
duration = pulseIn(ECHO, HIGH); // blocks up to 30ms!
distance = duration * 0.034 / 2;
```

This **freezes the entire loop** for up to 30ms every reading. PID, servo, and ROS communication all stop.

### This Code's Non-Blocking Solution

```
Timeline:
────────────────────────────────────────────────────────►
│                                                       │
├─ triggerUltrasonic()    ├─ ISR captures echo_start     │
│  Send 10μs pulse       │  (rising edge)               │
│  Set trig_sent=true    │                               │
│                        ├─ ISR captures echo_end        │
│                        │  (falling edge)               │
│                        │  Set echo_done=true           │
│                        │                               │
│                        ├─ processUltrasonic()          │
│                        │  Calculates distance          │
│                        │  from echo_end - echo_start   │
│                        │                               │
│  Next trigger after 100ms ──────────────────────────► │
```

**Three separate functions, zero blocking:**

```c
// 1. TRIGGER — called every 100ms from loop()
void triggerUltrasonic() {
    // 12μs total, negligible
    echo_done = false;
    trig_sent = true;
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
}

// 2. ISR — hardware interrupt, runs automatically
void IRAM_ATTR echoISR() {
    if (digitalRead(ECHO_PIN) == HIGH)
        echo_start = micros();   // rising edge
    else {
        echo_end = micros();     // falling edge
        echo_done = true;
    }
}

// 3. PROCESS — called every loop(), checks if done
void processUltrasonic() {
    if (!trig_sent) return;      // nothing pending
    if (echo_done) {
        // calculate distance
        duration = echo_end - echo_start;
        distance = (duration * 0.0343) / 2.0;
        // apply calibration
        distance *= calibration_factor;
        // clamp to valid range
        // 2cm - 400cm
    }
    else if (timeout > 30ms) {
        distance = -1;  // no echo received
    }
}
```

### Calibration Parameters

```c
float calibration_factor = 1.0;    // Multiply result if sensor reads wrong
                                    // Measured 95cm when actual is 100cm?
                                    // Set to 100/95 = 1.053

float speed_of_sound = 0.0343;     // cm per microsecond at ~20°C
                                    // Adjust for temperature:
                                    // 0°C  → 0.0331
                                    // 20°C → 0.0343
                                    // 30°C → 0.0349
```

**How to calibrate:**
1. Place object at known distance (e.g., 100cm)
2. Read the published `/ultrasonic` value
3. If it reads 97: set `calibration_factor = 100.0 / 97.0 = 1.031`
4. If speed of sound differs by temperature, adjust `speed_of_sound`

---

## ⚙️ Motor Control System

### Hardware Setup

```
ESP32 ──► L298N Motor Driver ──► DC Motor with Encoder

Pin 21 (ENA) ──► PWM speed control (5kHz, 8-bit)
Pin 19 (IN1) ──► Direction pin 1
Pin 18 (IN2) ──► Direction pin 2
Pin 22 ──► Encoder Channel A (interrupt)
Pin 23 ──► Encoder Channel B (interrupt)
```

### Encoder — Quadrature Decoding

The encoder has two channels (A and B) offset by 90°. Both channels trigger interrupts on CHANGE:

```
Forward rotation:
Channel A: ─┐  ┌──┐  ┌──┐  ┌──
             └──┘  └──┘  └──┘
Channel B: ──┐  ┌──┐  ┌──┐  ┌─
              └──┘  └──┘  └──┘

When A changes: if A == B → decrement, else → increment
When B changes: if A == B → increment, else → decrement
```

```c
#define TICKS_PER_REV 1802  // encoder ticks per full wheel revolution
```

**Critical: ISR uses `portENTER_CRITICAL_ISR`** to prevent race conditions when reading the tick counter from the main loop.

### Three Control Modes

```
control_mode = 0:  STOPPED — no motor output
control_mode = 1:  SPEED MODE — PID maintains constant RPM
control_mode = 2:  POSITION MODE — move to target tick count
```

### PID Speed Controller

```
                    ┌─────────────┐
  speed_target ──►  │             │
  (RPM)             │  PID        │──► PWM output ──► Motor
                    │  Controller │
  current_rpm ──►   │             │
  (measured)        └─────────────┘

  Runs every 20ms (PID_INTERVAL)
  Speed measured every 50ms from encoder tick delta
```

```c
Kp = 0.7    // Proportional: immediate response to error
Ki = 4.0    // Integral: eliminates steady-state error
Kd = 0.0    // Derivative: unused (not needed for this motor)
```

**Speed measurement:**
```c
void measureSpeed() {
    // Every 50ms:
    delta_ticks = current_ticks - previous_ticks;
    dt = elapsed_time_seconds;
    current_rpm = (delta_ticks / TICKS_PER_REV) / (dt / 60.0);
    // Converts tick rate to RPM
}
```

**Anti-windup on integral term:**
```c
if (speed_integral > 200) speed_integral = 200;
if (speed_integral < -200) speed_integral = -200;
```

### Minimum PWM Handling

```c
#define MIN_PWM 140  // Below this, motor doesn't move (stall zone)

// When user requests PWM 1-255, map to 140-255
int actual = map(pwm, 1, 255, MIN_PWM, 255);
```

```
User PWM:    0   1  ─────────────────────── 255
                 │                            │
Actual PWM:  0  140 ─────────────────────── 255
                 │                            │
Motor:      OFF  Just barely moving          Full speed
```

### Position Mode — Move by Degrees or Centimeters

```
"m 360 50"  →  Move 360° at 50 RPM (one full wheel revolution)
"c 50 60"   →  Move 50cm at 60 RPM
```

**Centimeter to degree conversion:**
```c
float cm_per_degree = 0.015288;  // calibrated value
// This means: 1 degree of wheel rotation = 0.015288 cm travel
// Or equivalently: 360° = 5.5 cm (wheel circumference)

// To move 50cm:
float degrees = 50.0 / 0.015288 = 3270.4 degrees
// Then converted to ticks:
long ticks = (degrees / 360.0) * 1802 = 16,361 ticks
```

**How to calibrate `cm_per_degree`:**
1. Reset encoder: `ros2 topic pub /motor_cmd std_msgs/String "data: 'r'"`
2. Mark starting position on floor
3. Command: `"m 3600 40"` (10 full revolutions)
4. Measure actual distance traveled (e.g., 55cm)
5. Calculate: `cm_per_degree = 55.0 / 3600.0 = 0.01528`

### Deceleration Zone

```
Speed ▲
      │  ████████████████████
      │  █  move_speed      █
      │  █                  █████
      │  █                  █   ████
      │  █                  █       ███
      │  █                  █          ██
      ├──█──────────────────█────────────█──► Position
      │  Start        decel_zone(300)  Target
                         ticks
```

```c
int decel_zone = 300;       // Start slowing 300 ticks before target
int position_tolerance = 30; // Stop when within 30 ticks of target

if (abs_error < decel_zone) {
    factor = abs_error / decel_zone;  // 1.0 → 0.0
    if (factor < 0.25) factor = 0.25; // minimum 25% speed
    target_speed = move_speed * factor;
}
```

### Position Accumulation (Intended Position)

```c
// Each move command ADDS to intended_position
// This prevents drift from tolerance gaps

moveDistance(360, 50);  // intended_position = 1802
moveDistance(360, 50);  // intended_position = 3604 (not recalculated from current)
```

---

## 🎮 Motor Command Protocol

```
┌──────────┬───────────────────┬────────────────────────────────┐
│ Command  │ Example           │ Action                         │
├──────────┼───────────────────┼────────────────────────────────┤
│ s <rpm>  │ "s 60"            │ Spin continuously at 60 RPM   │
│          │ "s -40"           │ Spin reverse at 40 RPM        │
│          │ "s 0"             │ Stop                          │
├──────────┼───────────────────┼────────────────────────────────┤
│ m <d> <r>│ "m 360 50"        │ Move 360° at 50 RPM           │
│          │ "m -720 80"       │ Move 2 rev reverse at 80 RPM  │
│          │ "m 180"           │ Move 180° at default 60 RPM   │
├──────────┼───────────────────┼────────────────────────────────┤
│ c <cm> <r│ "c 100 60"        │ Move 100cm at 60 RPM          │
│          │ "c -50 40"        │ Move 50cm reverse at 40 RPM   │
├──────────┼───────────────────┼────────────────────────────────┤
│ x        │ "x"               │ Stop motor immediately        │
├──────────┼───────────────────┼────────────────────────────────┤
│ r        │ "r"               │ Reset encoder to zero          │
└──────────┴───────────────────┴────────────────────────────────┘
```

---

## 💡 LED Control Values

```
ros2 topic pub /LEDs std_msgs/Int8 "data: 3"

Value  Left_LED(33)  Right_LED(32)  Front_LED(4)
─────  ───────────   ────────────   ────────────
  0       OFF            OFF          (unchanged)
  1       ON             OFF          (unchanged)
  2       OFF            ON           (unchanged)
  3       ON             ON           (unchanged)
  4     (unchanged)   (unchanged)       ON
  5     (unchanged)   (unchanged)       OFF
```

---

## 🔄 Servo Control

```
ros2 topic pub /servo std_msgs/Int8 "data: 3"

Input:  -6  -5  -4  -3  -2  -1   0   1   2   3   4   5   6
         │                        │                        │
Servo:  30° 40° 50° 60° 70° 80° 90° 100°110°120°130°140°150°

Center = 90°
Each unit = 10° of servo movement
Range constrained to -6 to +6 for safety
```

---

## ⏱️ Loop Timing Breakdown

```
loop() runs continuously (~every 100μs + processing)

Every 100μs:   delayMicroseconds(100) — watchdog yield
Every ~0μs:    executor spin_some(0) — check for ROS messages
Every 20ms:    PID update (speed or position control)
Every 50ms:    Speed measurement from encoder
Every 100ms:   Ultrasonic trigger + publish distance
Every 1000ms:  Ping agent (connection health check)
```

```
Timeline per loop iteration:
┌─────────────────────────────────────────────────────┐
│ 1. State machine (connect/reconnect)                │
│ 2. Spin executor (process any incoming messages)    │
│ 3. Trigger ultrasonic (if 100ms elapsed)            │
│ 4. Process ultrasonic (check if echo received)      │
│ 5. Measure motor speed (if 50ms elapsed)            │
│ 6. Run PID controller (if 20ms elapsed)             │
│ 7. Yield CPU for 100μs                              │
└─────────────────────────────────────────────────────┘
```

---

## 🔌 String Message Memory Management

```c
#define MOTOR_MSG_BUF_SIZE 64
char motor_msg_buffer[MOTOR_MSG_BUF_SIZE];

// In setup():
motor_msg.data.data = motor_msg_buffer;     // point to our buffer
motor_msg.data.size = 0;                     // current length
motor_msg.data.capacity = MOTOR_MSG_BUF_SIZE; // max length
```

**Why this matters:** micro-ROS String messages don't allocate memory automatically. Without pre-allocating a buffer, receiving a String message **crashes the ESP32**. This manually assigns a 64-byte static buffer.

---

## 📋 Quick Reference — Testing Commands

```bash
# Start micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0

# Turn on both LEDs
ros2 topic pub --once /LEDs std_msgs/Int8 "data: 3"

# Center servo
ros2 topic pub --once /servo std_msgs/Int8 "data: 0"

# Turn servo full left
ros2 topic pub --once /servo std_msgs/Int8 "data: -6"

# Motor: spin at 60 RPM
ros2 topic pub --once /motor_cmd std_msgs/String "data: 's 60'"

# Motor: move 50cm at 60 RPM
ros2 topic pub --once /motor_cmd std_msgs/String "data: 'c 50 60'"

# Motor: stop
ros2 topic pub --once /motor_cmd std_msgs/String "data: 'x'"

# Read ultrasonic distance
ros2 topic echo /ultrasonic
```

---

## 📁 Suggested GitHub Repository Structure

```
ESP32-MicroROS-Robot/
├── README.md                    ← Put this analysis there
├── esp32_robot_controller/
│   └── esp32_robot_controller.ino   ← Your code
├── docs/
│   ├── wiring_diagram.png
│   └── pin_mapping.md
├── .gitignore
└── LICENSE
```

### `.gitignore` for Arduino/PlatformIO:

```
.pio/
.vscode/
build/
*.o
*.elf
*.bin
```

---

## 🚀 How to Put This on GitHub (First Time)

```bash
# 1. Install git if not installed
sudo apt install git

# 2. Create project folder
mkdir ESP32-MicroROS-Robot
cd ESP32-MicroROS-Robot

# 3. Initialize git
git init

# 4. Create your files (copy your .ino, create README.md)
# Place files as shown in structure above

# 5. Stage all files
git add .

# 6. First commit
git commit -m "Initial commit: ESP32 micro-ROS robot controller"

# 7. Go to github.com → New Repository → Name it ESP32-MicroROS-Robot
#    Do NOT initialize with README (you already have one)

# 8. Connect and push
git remote add origin https://github.com/YOUR_USERNAME/ESP32-MicroROS-Robot.git
git branch -M main
git push -u origin main
```

After this, for future changes:
```bash
git add .
git commit -m "Description of what changed"
git push
```
