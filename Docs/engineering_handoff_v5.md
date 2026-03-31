# Engineering Handoff v5 — Self-Balancing Robot Project

## 1. Project Objective
Develop a robust self-balancing two-wheel robot with:
- STM32 F401/F405 main controller
- ODrive torque control over CAN
- ICM20948 IMU
- Python GUI tuning + AI diagnostics

Primary milestone achieved:
- short-term balancing demonstrated

Current milestone:
- stable long-term balancing and recovery

---

## 2. Current Technical State

### Firmware
- STM32 control loop operational
- USB CDC debug stream stable
- EKF / fused pitch estimate available
- CATCH and IDLE logic implemented

### Control Modes
- IDLE
- CATCH
- BALANCE (transition still unstable)

### Measured Issues
1. pitch zero offset
2. drift after vertical hold
3. oscillation growth
4. saturation events
5. catch exit instability

---

## 3. Known Working Parameters

### Balance
- K_pitch = 0.8
- K_rate = 0.45–0.55
- K_wheel = 0.2
- U_limit = 0.4–0.45

### Catch
- K_pitch = 0.52–0.60
- K_rate = 0.45–0.50
- hold = 180–300 ms

---

## 4. Python GUI Handoff

## Purpose
Engineering tool for:
- live tuning
- debug capture
- preset storage
- AI analysis

## Stable Rules
After connect:
- NO auto commands
- passive listen only

Manual actions only:
- get all
- apply preset
- set parameter
- AI analyze

## Critical Lessons
Auto current.json apply after connect is forbidden.
It disturbs DBG flow.

---

## 5. AI Analysis Window
Send only:
- 1 DBG before attempt (en=0)
- all DBG during attempt (en=1)
- 1 DBG after stop (en=0)

This rule is mandatory.

---

## 6. Major Risks
### Highest risk
pitch zero calibration error

### Secondary
rate spikes / IMU noise

### Third
controller saturation

---

## 7. Immediate Next Engineering Tasks
1. IMU zero calibration procedure
2. rate low-pass filter
3. catch-to-balance hysteresis
4. wheel velocity damping
5. forward/backward motion layer

---

## 8. Recommended First Action In New Chat
Start from:
IMU pitch zero calibration and trim correction

This is the highest leverage task.
