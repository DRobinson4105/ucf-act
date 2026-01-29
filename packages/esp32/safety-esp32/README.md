# safety-esp32

ESP-IDF C++ project for the safety esp32c6. Monitors hardware E-stop inputs, CAN fault/status, ultrasonic proximity, and emits a heartbeat LED/logger.

## Features
- NC mushroom and wireless remote GPIO inputs
- Safety CAN parsing (app E-stop + control fault) and E-stop publish
- Ultrasonic UART distance reader with too-close threshold
- Heartbeat LED/logging (shared component)
	- WS2812 onboard LED via RMT on GPIO8
	- Idle: green, Activity: blue, Error: red
- FreeRTOS safety task loop with periodic polling

## Build (Windows/Linux/macOS)
```bash
idf.py set-target esp32c6
idf.py build
```

## Flash (Windows PowerShell)
```powershell
idf.py -p COM3 flash
```

## Flash (Windows WSL)
```bash
idf.py -p /dev/ttyACM1 flash
```

## Monitor (Windows PowerShell)
```powershell
idf.py -p COM3 monitor
```

## Monitor (Windows WSL)
```bash
idf.py -p /dev/ttyACM1 monitor
```