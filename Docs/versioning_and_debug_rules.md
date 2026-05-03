# RULES: VERSIONING + DEBUG LOG CONTROL (MANDATORY)

## 1. Module version macros
Every runtime-critical module that participates in the debug tag **must** have a version macro in its `.h` file.

Current mandatory set:
```c
#define MAIN_MODULE_VERSION           "M04"
#define BALANCE_MODULE_VERSION        "B09"
#define HARDWAREINIT_MODULE_VERSION   "H05"
#define APP_SERIAL_MODULE_VERSION     "S05"
#define EKF_DRIVER_MODULE_VERSION     "E02"
```

Format:
```c
"<LETTER><2-digit number>"
```

Examples:
```c
"M04"
"B09"
"H05"
```

---

## 2. Mandatory version increment rule
**Any logic change in a module = mandatory version increment**

Examples:

If changed:
```c
balance.c
balance.h
```

Must increment:
```c
B09 -> B10
```

If changed:
```c
main.c
main.h
```

Must increment:
```c
M04 -> M05
```

If changed:
```c
app_serial.c
app_serial.h
```

Must increment:
```c
S05 -> S06
```

---

## 3. Strictly prohibited
It is forbidden to:
- modify `.c` logic without increasing module version
- analyze logs without checking the debug tag
- continue testing on stale firmware

---

## 4. Debug tag format
`main.c` must contain a unified revision tag:
```c
#define PROJECT_BALANCE_DEBUG_REVISION \
    MAIN_MODULE_VERSION "-" \
    BALANCE_MODULE_VERSION "-" \
    HARDWAREINIT_MODULE_VERSION "-" \
    APP_SERIAL_MODULE_VERSION "-" \
    EKF_DRIVER_MODULE_VERSION
```

Expected log example:
```text
DBG[M04-B09-H05-S05-E02]
```

---

## 5. Mandatory pre-log analysis check
Before **any** log analysis:

1. Check:
```text
DBG[Mxx-Bxx-Hxx-Sxx-Exx]
```

2. Confirm it matches current source files

If mismatch:
```text
Clean -> Build -> Flash -> Retest
```

Only then analyze the log.

---

## 6. Human-readable file version
Each `.h` should also contain a readable version line:
```c
// Версия: 01.03 05.04.26 07:10:00
```

This is secondary.
Primary control = `*_MODULE_VERSION`

---

## 7. Flash format version
If persistent structs change:
```c
flash_cfg_store.h
```

Mandatory:
```c
FLASH_CFG_STORE_FORMAT_VERSION++
```

Example:
```c
0x0005u -> 0x0006u
```

---

## 8. Mandatory workflow
```text
1. modify code
2. increment module version
3. increment flash format version if struct changed
4. build
5. verify DBG tag
6. perform tests
7. analyze logs
```

This is a **mandatory engineering project rule**.
