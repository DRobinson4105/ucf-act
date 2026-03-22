# Safety ESP32

## Install ESP-IDF

Follow the Espressif setup guide depending on platform.
- **Windows:** https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/windows-setup.html
- **Linux and macOS:** https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/linux-macos-setup.html

Make sure `idf.py` is available after installation.

## Build package and flash to ESP32

### Windows (PowerShell)
```powershell
& "~/esp/v5.5.2/esp-idf/export.ps1"
idf.py set-target esp32c6 build flash
```

### Linux or macOS
```bash
. $HOME/esp/esp-idf/export.sh
idf.py set-target esp32c6 build flash
```

## Monitor serial output

The serial monitor will display boot logs and ultrasonic distance output.

### Windows (PowerShell)
```powershell
& "~/esp/v5.5.2/esp-idf/export.ps1"
idf.py monitor
```

### Linux or macOS
```bash
. $HOME/esp/esp-idf/export.sh
idf.py monitor
```
