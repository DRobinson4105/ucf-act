# CAN Bus

CAN bus communication system for the autonomous golf cart, consisting of a Jetson AGX Orin (Planner), two ESP32-C6 microcontrollers (Control/Safety), and two UIM2852CA stepper motors (Steering/Braking).

The Planner, Control, and Safety nodes share a unified heartbeat format, state enum, and fault code namespace over 1 Mbps CAN. Safety commands target states with NOT_READY/READY/ENABLE/ACTIVE, while Planner/Control report live local states (including OVERRIDE/FAULT):

| Node                           | Role                                                         | Heartbeat ID |
|--------------------------------|--------------------------------------------------------------|--------------|
| **Safety** (ESP32-C6-WROOM-1)  | System state authority, e-stop/error monitoring, power relay | 0x100        |
| **Planner** (Jetson AGX Orin)  | Path planning, sends throttle/steering/braking commands      | 0x110        |
| **Control** (ESP32-C6-WROOM-1) | Executes actuator commands (throttle mux, stepper motors)    | 0x120        |

Two UIM2852CA stepper motors (steering node 5, braking node 6) also share the bus using extended 29-bit CAN frames but do not participate in the heartbeat protocol.

## Documentation

| Topic                                                     | Location                                               |
|-----------------------------------------------------------|--------------------------------------------------------|
| Frame layouts, state codes, fault codes, flags            | [common/protocol/README.md](common/protocol/README.md) |
| Control messages, pins, wiring, components                | [control-esp32/README.md](control-esp32/README.md)     |
| Safety system state authority, e-stop logic, pins, wiring | [safety-esp32/README.md](safety-esp32/README.md)       |
| Host-native unit tests                                    | [tests/README.md](tests/README.md)                     |

## CAN Bus Wiring

Five nodes share a single CAN bus at 1 Mbps. The three compute nodes (Safety, Planner, Control) are physically co-located. The two stepper motors are mounted several feet away on the cart chassis. Both ESP32-C6 boards use Waveshare SN65HVD230 transceivers (TWAI TX=GPIO4, RX=GPIO5); Planner uses its own CAN interface; the stepper motors have built-in CAN transceivers.

### Physical Connections Per Node

Each CAN node requires three wires to the bus:

| Wire  | Description                               |
|-------|-------------------------------------------|
| CAN-H | CAN high signal                           |
| CAN-L | CAN low signal                            |
| GND   | Common ground reference between all nodes |

### Bus Termination

120 ohm termination resistors between CAN-H and CAN-L, enabled on the Waveshare transceiver boards at **Safety** and **Planner** (both have onboard solder-jumper options for termination). **Control's** Waveshare board has termination **disabled**. The stepper motors do not provide termination.

### Wiring Notes

- Twisted-pair wiring recommended for CAN-H/CAN-L runs to the stepper motors (several feet)
- Common GND between all five nodes (ESP32 boards and Planner share cart chassis ground)
- SN65HVD230 modules powered from their respective ESP32 3.3V rail
- Stepper motors powered from the 24V autonomous power rail (switched by Safety's power relay)

### Message Traffic

| ID       | Name              | Sender           | Frame Type      |
|----------|-------------------|------------------|-----------------|
| 0x100    | SAFETY_HEARTBEAT  | Safety           | Standard 11-bit |
| 0x110    | PLANNER_HEARTBEAT | Planner          | Standard 11-bit |
| 0x111    | PLANNER_COMMAND   | Planner          | Standard 11-bit |
| 0x120    | CONTROL_HEARTBEAT | Control          | Standard 11-bit |
| Extended | STEPPER\_\*       | Control / Motors | Extended 29-bit |

See [common/protocol/README.md](common/protocol/README.md) for frame layouts and byte-level detail.

## Development Setup

### 1. Install ESP-IDF v5.5

Follow the official [ESP-IDF installation guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html#installation) for your OS. This project requires **ESP-IDF v5.5** targeting **esp32c6**.

Windows: use [WSL 2](https://learn.microsoft.com/en-us/windows/wsl/install) with Ubuntu and follow the Linux instructions inside WSL. Also will need [usbipd-win](https://github.com/dorssel/usbipd-win) for USB device forwarding (see [WSL USB Forwarding](#wsl-usb-forwarding) below).

### 2. Install direnv

This project uses [direnv](https://direnv.net/) to automatically load the ESP-IDF toolchain environment when you `cd` into the project directory. No manual sourcing required.

```bash
# Linux
sudo apt install direnv

# macOS
brew install direnv
```

Then add the hook to your shell profile and reload:

```bash
# bash (~/.bashrc)
echo 'eval "$(direnv hook bash)"' >> ~/.bashrc && source ~/.bashrc

# zsh (~/.zshrc)
echo 'eval "$(direnv hook zsh)"' >> ~/.zshrc && source ~/.zshrc

# fish (~/.config/fish/config.fish)
echo 'direnv hook fish | source' >> ~/.config/fish/config.fish && source ~/.config/fish/config.fish
```

### 3. Configure the project

Set your local ESP-IDF path (one-time):

```bash
cp .env.example .env
```

Edit `.env` and set `IDF_PATH` to wherever you installed ESP-IDF (the default is `~/esp/esp-idf`). Then approve the direnv config:

```bash
direnv allow
```

From now on, every time you `cd` into `packages/can` (or any subdirectory), the ESP-IDF toolchain is automatically available. When you leave the directory, it's unloaded.

### 4. Host-native tests

Running `make -C tests` requires `gcc`, `g++`, and `make`:

- **Linux:** `sudo apt install build-essential`
- **macOS:** `xcode-select --install`
- **Windows/WSL:** `sudo apt install build-essential` (inside WSL)

## Building

Choose one firmware target and run commands in that directory:

- Control firmware: `cd control-esp32`
- Safety firmware: `cd safety-esp32`

Then:

```bash
idf.py set-target esp32c6
idf.py build
```

## Flashing

The serial port varies by OS and connection order. Find your ports first:

### Finding Ports

| OS                | Command                                | Common Ports                              |
|-------------------|----------------------------------------|-------------------------------------------|
| **Linux**         | `ls /dev/ttyACM*` or `ls /dev/ttyUSB*` | `/dev/ttyACM0`, `/dev/ttyACM1`            |
| **macOS**         | `ls /dev/cu.usb*`                      | `/dev/cu.usbmodem*`, `/dev/cu.usbserial*` |
| **Windows (WSL)** | `ls /dev/ttyACM*` (with usbipd)        | `/dev/ttyACM0`, `/dev/ttyACM1`            |

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

# WSL (requires usbipd-win to forward USB, see below)
idf.py -p /dev/ttyACM0 flash
idf.py -p /dev/ttyACM0 monitor
```

Note: under WSL, `usbipd-win` doesn't support `flash` and `monitor` in the same command -- run them separately.

### WSL USB Forwarding

If using WSL, you need [usbipd-win](https://github.com/dorssel/usbipd-win) to access USB devices:

```powershell
# In Windows PowerShell (Admin)
usbipd list
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```

After attaching, the device appears under `/dev/ttyACM*` inside WSL.

## Testing

Host-native unit tests for shared protocol, logic, and component code (no ESP-IDF required):

```bash
make -C tests
```

See [tests/README.md](tests/README.md) for details.
