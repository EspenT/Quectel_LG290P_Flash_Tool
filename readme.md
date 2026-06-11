# Quectel LG290P Flash Tool

Command-line tool to update firmware on the Quectel **LG290P (03)** GNSS module.

Works on any carrier board with UART access to the chip — SparkFun breakout, **Waveshare LG290P**, Quectel EVB, or custom PCB.

Based on [SparkFun's quectool](https://github.com/sparkfun/SparkFun_LG290P_Flash_Tool). Uses [libserialport](https://sigrok.org/wiki/Libserialport) for cross-platform serial I/O.

## macOS

### Prerequisites

```bash
brew install libserialport
```

### Build

```bash
make
```

On Intel Macs with Homebrew in `/usr/local`:

```bash
make PREFIX=/usr/local
```

Or with pkg-config:

```bash
gcc $(pkg-config --cflags --libs libserialport) quectool.c -o quectool
```

### Flash firmware

1. Connect the LG290P **USB-C** port directly to the Mac (UART1 via the onboard USB–serial bridge).
2. Disconnect other UART users (e.g. ESP32 on RXD2/TXD3) during the update.
3. Find the serial port — use a **`/dev/cu.*`** device, not `/dev/tty.*`:

   ```bash
   ls /dev/cu.usb*
   ```

4. Use official LG290P `.pkg` firmware (included in this repo or from [SparkFun's firmware repo](https://github.com/sparkfun/SparkFun_RTK_Postcard/tree/main/Firmware)).

5. Run:

   ```bash
   ./quectool /dev/cu.usbmodemXXXXXXXX LG290P03AANR02A01S.pkg
   ```

The tool sends a reboot command, enters the bootloader at **460800 baud**, erases flash, uploads with CRC checks, and resets the module. Expect about 1–2 minutes total.

### Troubleshooting

| Problem | Fix |
|---------|-----|
| Hangs on "Waiting for Port …" | Use full path: `/dev/cu.usbmodem…`, not `cu.usbmodem…` |
| Sync timeout | Power-cycle the module or briefly ground **RST** while the tool is waiting |
| Port won't open | Quit QGNSS, serial monitors, or other apps using the port |
| Two USB serial ports | Try the other port (SparkFun: "Channel A" / lower enumeration) |

## Windows

```bash
gcc quectool.c -o quectool.exe -lserialport -lsetupapi -lcfgmgr32 -static
quectool.exe COM5 LG290P03AANR02A01S.pkg
```

Pre-built Windows binaries are available under [Releases](https://github.com/EspenT/Quectel_LG290P_Flash_Tool/releases).

## Linux

```bash
sudo apt install libserialport-dev   # Debian/Ubuntu
make
./quectool /dev/ttyACM0 LG290P03AANR02A01S.pkg
```

## Options

```
--version                  Print tool version
--firmware-update-enabled  Enable firmware updates (default)
--erase-only               Erase flash only, then exit
--skip-version-check       Skip current firmware version display
```

## Safety

- Use only official **LG290P03** `.pkg` files.
- Do not unplug USB or kill the process during erase/upload.
- A failed update can leave the module non-functional until you flash again; the on-chip bootloader allows recovery.

## CI builds

GitHub Actions can build binaries for Windows, macOS, Linux, and ARM. Trigger manually via **Actions → Build Tool Binaries → Run workflow**.
