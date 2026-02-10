# CAN Bus

CAN bus communication system for the autonomous golf cart, consisting of two ESP32-C6 microcontrollers and a Jetson AGX Orin.

## Documentation

| Topic | Location |
|-------|----------|
| Frame layouts, state codes, flags | [common/can_protocol/README.md](common/can_protocol/README.md) |
| Control ESP32 messages, pins, components | [control-esp32/README.md](control-esp32/README.md) |
| Safety ESP32 messages, autonomy allowed reasons, pins, components | [safety-esp32/README.md](safety-esp32/README.md) |
| Host-native unit tests | [tests/README.md](tests/README.md) |

## Testing

Host-native unit tests for the protocol libraries (no ESP-IDF required):

```bash
make -C tests
```

See [tests/README.md](tests/README.md) for details.

## Building

Requires ESP-IDF v5.5. Use the devcontainer for a pre-configured environment. First, choose what device to build:

```bash
cd control-esp32
cd safety-esp32
```

Then, in the directory of the device:

```bash
idf.py set-target esp32c6
idf.py build
```

## Flashing

The serial port varies by OS and connection order. Find your ports first:

### Finding Ports

| OS | Command | Common Ports |
|----|---------|--------------|
| **Linux** | `ls /dev/ttyACM*` or `ls /dev/ttyUSB*` | `/dev/ttyACM0`, `/dev/ttyACM1` |
| **macOS** | `ls /dev/cu.usb*` | `/dev/cu.usbmodem*`, `/dev/cu.usbserial*` |
| **Windows** | Device Manager → Ports | `COM3`, `COM4`, etc. |
| **WSL** | `ls /dev/ttyACM*` (with usbipd) | `/dev/ttyACM0`, `/dev/ttyACM1` |

### Flash Commands

```bash
idf.py -p PORT flash monitor
```

**Examples:**
```bash
# Linux
idf.py -p /dev/ttyACM0 flash monitor

# macOS
idf.py -p /dev/cu.usbmodem14101 flash monitor

# Windows (PowerShell)
idf.py -p COM3 flash monitor

# WSL (requires usbipd-win to forward USB, see below)
idf.py -p /dev/ttyACM0 flash monitor
```

### WSL USB Forwarding

If using WSL, you need [usbipd-win](https://github.com/dorssel/usbipd-win) to access USB devices:

```powershell
# In Windows PowerShell (Admin)
usbipd list
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```