# control-esp32

ESP-IDF C++ project for the control esp32c6. Uses TWAI for CAN motor control, a required MCP4728 I2C DAC for autonomous throttle, and a heartbeat LED/logger.

## Features
- TWAI (CAN) init at 1 Mbps with standard frames
- UIM342 motor protocol helpers for enable/disable and control commands
- MCP4728 DAC init on I2C (startup halts if DAC is not detected)
- Heartbeat LED/logging (shared component)
	- WS2812 onboard LED via RMT on GPIO8
	- Idle: green, Activity: blue, Error: red
- FreeRTOS tasks for CAN RX and main startup

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
idf.py -p /dev/ttyACM0 flash
```

## Monitor (Windows PowerShell)
```powershell
idf.py -p COM3 monitor
```

## Monitor (Windows WSL)
```bash
idf.py -p /dev/ttyACM0 monitor
```