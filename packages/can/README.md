# Vehicle Communications

Vehicle communications for the autonomous golf cart, consisting of a Jetson AGX Orin (Planner), two ESP32-C6-DevKitM-1 microcontrollers (Control/Safety), and two UIM2852CA motors (Steering/Braking).

The system uses two transports:

- **Orin ↔ ESP32 UART links** — one cable per ESP32, plugged from each board's "COM" USB-C port (routed through the DevKitM-1's CP2102N bridge to UART0) to the Orin. 1 Mbaud, framed by `usb_serial_link` + `orin_link_protocol`.
- **1 Mbps CAN bus** shared between Safety, Control, and the two UIM2852 motors.

Safety and Control share a unified heartbeat format, state enum, and cause channels. Safety commands target states with NOT_READY/READY/ENABLE/ACTIVE. Non-fault stop inputs are carried in `stop_flags`; issues/timeouts are carried in `fault_flags`.

| Node                                | Role                                                           | Heartbeat                                |
| ----------------------------------- | -------------------------------------------------------------- | ---------------------------------------- |
| **Safety** (ESP32-C6-DevKitM-1)     | System state authority, stop/fault monitoring, power relay     | CAN 0x100 + Orin UART (to Orin)          |
| **Planner** (Jetson AGX Orin)       | Path planning, sends commands to Control and heartbeat to Safety over its two UART cables | Orin UART (to Safety) |
| **Control** (ESP32-C6-DevKitM-1)    | Executes actuator commands (throttle, steering/braking motors) | CAN 0x120 + Orin UART (to Orin)          |

Two UIM2852CA motors (steering node 7, braking node 6) share the CAN bus using extended 29-bit frames but do not participate in the heartbeat protocol.

## Documentation

| Topic                                                         | Location                                               |
| ------------------------------------------------------------- | ------------------------------------------------------ |
| Frame layouts, state codes, fault flags, stop/status flags    | [common/protocol/README.md](common/protocol/README.md) |
| Control messages, pins, wiring, components                    | [control-esp32/README.md](control-esp32/README.md)     |
| Safety system state authority, stop/fault logic, pins, wiring | [safety-esp32/README.md](safety-esp32/README.md)       |
| Host-native unit tests                                        | [tests/README.md](tests/README.md)                     |

## CAN Bus Wiring

Four nodes share a single CAN bus at 1 Mbps: Safety, Control, and the two UIM2852 motors. The Orin does not participate in the CAN bus — it connects to each ESP32 directly over UART0 via the DevKitM-1 COM USB-C port. Both ESP32-C6 boards use Waveshare SN65HVD230 transceivers (TWAI 3V3, GND, TX=GPIO4, RX=GPIO5); the UIM2852 motors have built-in CAN transceivers (CAN-H, CAN-L).

Each CAN node connects to the bus with two signal wires:

| Wire  | Wire Color | Description     |
| ----- | ---------- | --------------- |
| CAN-H | Yellow     | CAN high signal |
| CAN-L | Green      | CAN low signal  |

### Bus Termination

Use exactly two 120 ohm termination resistors between CAN-H and CAN-L, one at **Safety** and one at the opposite end of the bus. **Control's** Waveshare transceiver board has termination disabled by desoldering and removing the resistor. The UIM2852 motors do not provide termination.

### Wiring Notes

- Twisted-pair wiring recommended for CAN-H (Yellow) / CAN-L (Green) runs to the UIM2852 motors (several feet)
- No dedicated CAN GND wire. All buck converters (5V ESP32, 12V Orin, 24V motors) are non-isolated with inputs fed from the same 48V- bus bar, so all output GNDs are inherently common. A separate CAN GND wire would be redundant.
- SN65HVD230 modules powered from their respective devices.

### Message Traffic

| ID       | Name              | Sender           | Frame Type      |
| -------- | ----------------- | ---------------- | --------------- |
| 0x100    | SAFETY_HEARTBEAT  | Safety           | Standard 11-bit |
| 0x120    | CONTROL_HEARTBEAT | Control          | Standard 11-bit |
| Extended | MOTOR\_UIM2852\_\* | Control / Motors | Extended 29-bit |

Planner commands and the planner heartbeat travel on the Orin UART links, not on CAN. See [common/protocol/README.md](common/protocol/README.md) for frame layouts and byte-level detail (including the `0xAA` sync byte, `orin_link_protocol` framing, and a concrete wire example).

## Orin UART Wiring

Each ESP32-C6-DevKitM-1 has **two USB-C connectors**:

- **COM** — routed through the onboard CP2102N bridge chip to UART0 (the ESP32-C6 peripheral). This is what the Orin plugs into.
- **USB** — routed directly to the ESP32-C6's native USB Serial JTAG peripheral. This is what your development laptop plugs into for `idf.py flash` / `idf.py monitor`.

The two ports are electrically independent, so flashing/monitoring from your laptop over the USB port never collides with live Orin traffic on the COM port. There are two separate cables total: **Orin ↔ Control COM** and **Orin ↔ Safety COM**. See [common/protocol/README.md](common/protocol/README.md) for which messages travel on each cable.

While the `usb_serial_link` component is installed it owns UART0 at 1 Mbaud, so the primary console (which defaults to UART0 @ 115200) is effectively silenced on the COM port. Log output still flows to the secondary USB Serial JTAG console on the native USB port, which is where `idf.py monitor` reads from anyway.

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
| ----------------- | -------------------------------------- | ----------------------------------------- |
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
