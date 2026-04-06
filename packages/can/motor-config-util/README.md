# sc3_sender

Configures a SimpleCAN 3.0 motor over CAN bus.

## What it does

1. **Starts at 500 kbps** and sends an initial command to the motor.
2. **Reboots the motor** (cw=0x7E) to cause it to save its configuration.
3. **Switches to 1000 kbps** and reconnects to the motor at the new baud rate.
4. **Sets the motor ID** based on the `MOTOR_TARGET` Kconfig option, regardless of what the motor's previous ID was.
5. **Reboots the motor again** at 1000 kbps to save the new ID.

## Kconfig options

| Option | Value | Description |
|---|---|---|
| `MOTOR_TARGET` | `1` | Braking motor — sets motor ID to 6 |
| `MOTOR_TARGET` | `2` | Steering motor — sets motor ID to 7 |

Configure via `idf.py menuconfig` → SC3 Sender.

## Usage

> **Note:** Before running, disconnect any other motors from the CAN bus. Only the motor being targeted should be connected.

```
idf.py menuconfig   # set MOTOR_TARGET, TX/RX GPIO pins
idf.py build flash monitor
```
