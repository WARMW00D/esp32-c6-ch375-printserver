# ESP32-C6-CH375-PrintServer

Turn a plain USB printer into a WiFi network printer using an ESP32-C6
Super Mini paired with a **CH375B** external USB Host controller chip —
no Raspberry Pi, no dedicated PC required.

The ESP32-C6 has no native USB Host peripheral, so this project drives
the printer through a CH375B chip talking to the ESP32-C6 over a fast
**8-bit parallel bus** (not the much slower UART/serial mode CH375B also
supports). The board exposes the printer on your local network as a
standard **Raw/JetDirect (port 9100)** and **LPR/LPD (port 515)** printer
— the same protocols real network printers and print servers use. Any
computer on the network (Windows, macOS, Linux) can add it as a normal
TCP/IP printer, install the manufacturer's driver, and print to it
wirelessly.

Built and tested against, for example, an **HP LaserJet P2015**, but
should work with any printer exposing a standard bidirectional USB
Printer Class (07/01) interface.

## Features

- **Parallel-bus USB Host via CH375B** — the ESP32-C6 has no native USB
  Host controller, so a CH375B chip drives the printer over its 8-bit
  parallel interface (RD#/WR#/A0 + D0-D7), which is dramatically faster
  than CH375B's alternative UART mode.
- **Two network print protocols** — Raw/AppSocket on port 9100 and LPR/LPD
  on port 515, both running simultaneously. LPR handles the well-known
  Windows LPR client quirk of reporting a bogus, oversized job length by
  falling back to streaming mode instead of trusting the declared size.
- **Live PJL status monitoring** — polls `@PJL INFO STATUS` between jobs
  for PJL-capable (HP-style PCL) printers over a second, bulk-IN USB
  endpoint, and surfaces paper-out / offline / error conditions on the
  dashboard and the status LED.
- **Web dashboard** — a small dark-themed control panel served by the
  board itself: printer state (with manufacturer/model read from the
  device's own USB string descriptors where available), USB connection
  details, WiFi info, free memory, and step-by-step printer-setup
  instructions for Windows and macOS with the live IP/model substituted
  in automatically.
- **Bilingual UI (RU/EN)** — a single button toggles the whole dashboard
  between Russian and English, saved in the browser.
- **WiFi setup with network scanning** — the setup page scans for nearby
  networks instead of requiring the SSID to be typed by hand; manual
  entry is still available for hidden networks.
- **WPA2-Enterprise (802.1X) support** — in addition to regular WPA2-PSK
  (or open) networks, the setup page can join PEAP/TTLS+MSCHAPv2
  WPA2-Enterprise networks (e.g. a corporate WiFi with a RADIUS server),
  using an Identity/Username/Password prompt instead of a plain network
  password.
- **Optional portal password** — the whole web interface can be protected
  with an HTTP Basic Auth password (login `admin`). Left blank, the
  portal stays open.
- **SSDP/UPnP + mDNS discovery** — the board answers SSDP `M-SEARCH`
  requests and advertises itself via mDNS (`ESP32-C6-PrintServer.local`),
  so it shows up in your network's device list.
- **RGB status LED** — the board's onboard WS2812 LED shows, at a glance:
  WiFi connecting (green/red alternating), access-point setup mode (blue,
  blinking), printer offline (red, solid), idle and ready (green, solid),
  printing (green, fast blink), reset button held (red, blink — overrides
  everything else), and printer error / status unknown (yellow, blink).
- **Optional 0.91" I2C OLED status display** (SSD1306, 128×32) — off by
  default, enabled with a single `#define` at the top of the sketch.
- **Config-portal-first fallback** — on first boot, or whenever the
  configured WiFi network is unreachable, the board starts its own access
  point and stays fully usable as a printer through it, no home network
  required.

## Hardware

- An **ESP32-C6 Super Mini** board.
- A **CH375B** USB Host controller chip, wired in **parallel mode** (not
  serial/UART — parallel is roughly an order of magnitude faster for
  real print jobs).

| CH375B pin | Connects to |
|---|---|
| D0–D7 | ESP32-C6 GPIO18, 19, 7, 6, 3, 2, 1, 0 |
| RD# | GPIO23 |
| WR# | GPIO22 |
| A0 | GPIO21 |
| INT# | GPIO20 |
| CS# | GND (hard-wired — no other device on the bus) |
| TXD | GND through a ~1kΩ resistor — **this is a strapping pin**, it selects parallel vs. serial mode at reset, not a data line |
| RXD | not connected (unused in parallel mode) |
| RST | not connected (it's an output on this chip, not an input) |

> **Important — GPIO12/GPIO13 are off-limits.** On the ESP32-C6, these
> two pins are fixed at the silicon level to the native USB Serial/JTAG
> controller (D-/D+), even though most board pinout diagrams don't mark
> them as reserved. Repurposing them as regular GPIOs (e.g. for the
> parallel bus above) kills the board's own USB serial console the
> moment the sketch reaches `setup()` — recoverable only by forcing the
> ROM bootloader (hold BOOT while powering up) and reflashing. The pin
> mapping above already avoids them.

- The onboard **WS2812 RGB LED** (GPIO8) and the **BOOT button** (GPIO9,
  doubles as the WiFi-settings reset button — hold 10 seconds) are used
  as-is; no extra wiring needed for those.
- **Optional: a 0.91" I2C OLED display** (SSD1306, 128×32) — GPIO4 (SDA)
  and GPIO5 (SCL) on this board are free and used for it if enabled.

## Known-good / known-tricky printers

Any printer with a genuine PCL, PostScript, or similar printer-side
language and a standard USB Printer Class interface should work well,
since the print job is just a pre-rendered byte stream relayed as-is.

Cheap **"GDI" / host-based printers** (e.g. many Canon printers using the
proprietary CAPT protocol) generate the raster data on the PC and often
rely on a tight, low-latency, bidirectional exchange with the printer
during printing. Bridging that reliably over WiFi — and through a
secondary USB Host chip on top of that — is a fundamentally different,
and harder, problem than relaying a finished PCL/PostScript stream, and
hasn't been tested here. Your mileage may vary.

## Arduino IDE setup

Board package: **esp32 by Espressif Systems**.

Board settings used during development:

| Setting | Value |
|---|---|
| Board | ESP32C6 Dev Module |
| USB CDC On Boot | Enabled |
| Flash Size | 4MB (or match your module) |
| Partition Scheme | Default 4MB with spiffs |

## Libraries

- [Adafruit NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel) —
  drives the onboard WS2812 status LED. Required (installable via the
  Arduino Library Manager).
- Standard ESP32 Arduino core libraries (bundled with the core, no
  separate install): `WiFi`, `WiFiUdp`, `ESPmDNS`, `Preferences`,
  `WebServer`, `esp_wifi.h`, `esp_eap_client.h`.
- Only if the optional OLED display is enabled (`ENABLE_OLED_DISPLAY`
  uncommented at the top of the sketch): `Adafruit_SSD1306` and
  `Adafruit_GFX` (both via Library Manager). With the display disabled,
  neither is required.

## Quick start

1. Wire the CH375B to the ESP32-C6 per the pin table above, including the
   TXD strapping resistor and CS# to ground.
2. Install the board settings above in Arduino IDE and flash the sketch.
3. On first boot the board starts an access point called
   `CH375-PrintServer-Setup` (password `12345678`). **You can print right
   away through this access point** — connect a computer to it and add a
   network printer pointing at `192.168.4.1` (Raw port 9100 or LPR port
   515), no home WiFi required. This also works any time the board can't
   reach your configured network, so it always falls back to a usable
   printer instead of going silent.
4. Open `http://192.168.4.1/wifi`, pick your network from the scanned
   list (or type it manually / switch on WPA2-Enterprise), and save. The
   board reboots and joins your network. On the same page you can also
   set an optional portal password.
5. After connecting to your home network, open the board's IP address in
   a browser for the live dashboard, or visit
   `http://ESP32-C6-PrintServer.local` if your OS supports mDNS.
6. On the computer, add a new network printer pointing at the board's
   address — either **Raw/AppSocket on port 9100** or **LPR on port
   515** — and install the printer's normal driver. Step-by-step
   instructions for Windows and macOS are shown right on the dashboard,
   with your printer's actual model name and the board's current IP
   filled in automatically.

## Acknowledgements

This project was built collaboratively with **[Claude](https://claude.com)**
(Anthropic) — from wiring up the CH375B in parallel mode and chasing down
a subtle endpoint-masking bug that silently blocked all bulk-IN transfers,
through the web dashboard, bilingual UI, and status LED. 🤖

## License

MIT.
