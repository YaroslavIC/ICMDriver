
# Transfer File – Balancing Robot Tuning (Full Knowledge Dump)

Date: 2026-03-28T08:14:00.758332

---

## 1. System Overview

- STM32 + ODrive (CAN)
- Control modes:
  - IDLE
  - CATCH
  - BALANCE
- Sensors:
  - IMU (pitch, pitch_rate)
  - Wheel encoders (wheel_vel_avg)

---

## 2. Key Problems Identified

### 2.1 Saturation in CATCH
- At high pitch (>600 mrad), `raw` → clamp limit
- Leads to:
  - loss of control authority
  - transition to IDLE
  - robot fall

### 2.2 Base Speed Accumulation
- wheel velocity grows:
  - wv up to 300–700
- insufficient damping → overshoot + fall

### 2.3 D-term Sensitivity
- Reducing `CATCH_K_PITCH_RATE` to 0.35:
  - caused oscillations
  - premature BAL entry
  - unstable behavior

### 2.4 Debug mismatch (fixed)
- V-term missing in debug
- Now fixed → debug matches real control

### 2.5 Transition Issues
- Early transition to BAL caused instability
- Need strong damping before switching

---

## 3. Tested Constants and Results

### Base (stable-ish baseline)
```
CATCH_U_LIMIT = 0.40 → too weak
```

---

### Test 1: Increase torque limit
```
CATCH_U_LIMIT = 0.45
```
Result:
- Less early saturation
- Improvement

---

### Test 2: Add wheel velocity damping
```
CATCH_K_WHEEL_VEL = 0.18
```
Result:
- Reduced base runaway
- Good improvement

---

### Test 3: Reduce D-term
```
CATCH_K_PITCH_RATE = 0.35
```
Result:
- BAD
- oscillations
- unstable transitions
- reverted

---

### Test 4: Increase pitch gain
```
CATCH_K_PITCH = 0.60
```
Result:
- Too aggressive
- saturation returns
- no improvement

---

### Test 5: Increase wheel damping more
```
CATCH_K_WHEEL_VEL = 0.22
```
Result:
- Better damping
- still insufficient at high pitch
- kept

---

### Test 6: Reduce pitch gain
```
CATCH_K_PITCH = 0.50
```
Result:
- No saturation
- BUT insufficient recovery torque
- fails at ~600 mrad

---

### Current Best Tradeoff Direction
```
CATCH_K_PITCH ≈ 0.52 (interpolation)
```

---

## 4. Current Recommended Set

```
CATCH_U_LIMIT          0.45f
CATCH_K_PITCH          0.52f
CATCH_K_PITCH_RATE     0.45f
CATCH_K_WHEEL_VEL      0.22f
```

---

## 5. Key Insights

1. Balance between:
   - torque limit
   - pitch gain
   - damping

2. Three competing effects:
   - High Kp → saturation
   - Low Kp → weak recovery
   - Low Kd → oscillation
   - Low Kv → runaway base

3. Optimal region is narrow.

4. Main failure mode now:
   - insufficient torque BEFORE saturation
   - not saturation itself

---

## 6. Debug Observations

- Correct structure:
  - raw ≈ P + D + V
- Important indicators:
  - sat flag
  - wheel velocity (wv)
  - pitch at failure (~600–700 mrad)

---

## 7. Next Steps (Future Work)

- Fine tuning:
  - CATCH_K_PITCH around 0.52–0.55
- Possibly:
  - adaptive gain vs pitch
  - torque ramping
- Improve transition logic:
  - stricter BAL entry

---

## 8. Critical Lessons (DO NOT REPEAT)

- Do NOT:
  - reduce Kd → instability
  - increase Kp blindly → saturation
- Always:
  - change ONE parameter
  - verify logs

---

END OF FILE
