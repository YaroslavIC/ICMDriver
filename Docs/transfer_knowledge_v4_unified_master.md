# 📦 Transfer Knowledge File (v4 — UNIFIED MASTER)

## 0. Назначение
Единый мастер-файл для полного переноса проекта балансирующего робота:
- firmware STM32
- control loop
- tuning history
- Python GUI history
- AI analysis workflow
- bugs / fixes / lessons learned

---

## 1. System Architecture
- STM32 F401/F405
- ODrive torque control
- ICM20948 IMU
- USB CDC CLI + DBG stream
- Python GUI v9.x

---

## 2. Control Logic
Основной закон:
u = Kp * pitch + Kd * rate + Kv * wheel_vel

with clamp:
u = clamp(u, ±U_LIMIT)

Modes:
- IDLE
- CATCH
- BALANCE

---

## 3. Key Coefficients
control_k_pitch ≈ 0.8
control_k_pitch_rate ≈ 0.45–0.55
control_k_wheel_vel ≈ 0.2

catch_k_pitch ≈ 0.52–0.60
catch_k_pitch_rate ≈ 0.45–0.50

trim ≈ 0–0.2

---

## 4. Main Problems
1. pitch zero offset (~0.8 rad)
2. unstable long-term balance
3. rate spikes
4. saturation
5. poor CATCH→BALANCE transition

---

## 5. Python GUI — Full Evolution

### Stage 1
simple serial terminal

### Stage 2
parameter panel

### Stage 3
preset + current.json

### Stage 4
auto reconnect

### Stage 5
graphs

### Stage 6
AI analysis

### Final concept
passive after connect
manual-only actions
transparent logging

---

## 6. Critical GUI Bugs
- hidden current.json auto apply
- hidden get all after connect
- RX loop crash
- missing attempt state attributes
- single DBG line issue
- thread/UI update issues

All fixed in later v9.x

---

## 7. AI Workflow
Input:
- one pre en=0 DBG line
- all en=1 lines
- one post en=0 line
- full parameter set

Output:
- summary
- risks
- recommendations
- set commands

---

## 8. Stable Workflow
1 connect
2 verify DBG
3 en 1
4 run attempt
5 AI analyze
6 save preset

---

## 9. Engineering Conclusions
System already balances.
Main issue = stability + calibration.

---

## 10. Next Step
1 IMU zero
2 rate filter
3 damping
4 hysteresis
5 movement control
