# GPS Master Server OTA v3.7

ESP8266 time server that maintains a free-running internal clock using a NEO-6M GPS module and DS3231 RTC. It exposes local-time and UTC JSON endpoints plus a status dashboard for connected clocks and can receive firmware updates over Wi-Fi.

## Features

- GPS-derived time correction with latency compensation
- DS3231 RTC fallback and periodic GPS-to-RTC backup
- Wi-Fi access point mode with runtime-editable SSID and password
- Local and UTC JSON time endpoints
- Auto-refreshing status dashboard
- EEPROM-persisted timing and hotspot settings
- HTTP Basic Auth for settings and OTA actions
- ArduinoOTA and web-based firmware upload
- Serial diagnostics and active-low status LED

## Hardware

- ESP8266 board, such as NodeMCU 1.0
- NEO-6M GPS module
- DS3231 RTC module
- USB cable or separate 3.3 V supply
- Optional status LED using the board's built-in LED

### Wiring

| ESP8266 | Device | Signal |
|---|---|---|
| GPIO14 / D5 | NEO-6M | TX to GPS RX |
| GPIO12 / D6 | NEO-6M | RX to GPS TX |
| GPIO4 / D2 | DS3231 | SDA |
| GPIO5 / D1 | DS3231 | SCL |
| 3.3 V | NEO-6M / DS3231 | VCC |
| GND | NEO-6M / DS3231 | GND |

The GPS and RTC pins are defined in `config.h`. Verify the GPS voltage requirements before connecting it.

## Arduino IDE Setup

1. Install the ESP8266 boards package.
2. Select an ESP8266 board with at least 4 MB flash, such as **NodeMCU 1.0 (ESP-12E Module)**.
3. Install these libraries:
   - `ESP8266WiFi`
   - `ESP8266mDNS`
   - `ESP8266WebServer`
   - `ArduinoOTA`
   - `SoftwareSerial`
   - `TinyGPSPlus`
   - `RTClib`
   - `ArduinoJson`
4. Open `CLAUDE_GPS_Master_Server_OTA_v3.7expmnt.ino` and compile or upload it.

For OTA uploads, connect to the server hotspot and select **Tools > Port > gps-master-server at ...** in Arduino IDE.

## Default Network

- SSID: `GPS_Master_Server`
- Password: `123456789`
- IP: `192.168.4.1`
- mDNS: `http://gps-clock.local/`
- OTA hostname: `gps-master-server`

Settings are saved in EEPROM. A fresh EEPROM or a settings reset restores the defaults.

## Web Interface

Open `http://192.168.4.1/` while connected to the server hotspot.

| Endpoint | Method | Description |
|---|---|---|
| `/` | GET | Auto-refreshing status page |
| `/time` | GET | Detailed local-time JSON |
| `/time/utc` | GET | Detailed UTC-time JSON |
| `/status` | GET | GPS, RTC, clock, and server diagnostics JSON |
| `/settings` | GET | Basic-auth-protected settings page |
| `/settings/save` | POST | Save runtime settings |
| `/restart` | POST | Basic-auth-protected server restart |
| `/update` | GET/POST | Basic-auth-protected OTA firmware upload |

The settings and update pages are protected with HTTP Basic Auth. Defaults come from `config.h`:

- Username: `admin`
- Password: `changeme123`
- OTA password: `changeme123`

Change `ADMIN_USERNAME`, `ADMIN_PASSWORD`, and `OTA_PASSWORD` before a production deployment.

## Runtime Settings

The `/settings` page controls:

- GPS total latency compensation
- GPS correction interval
- RTC correction interval
- RTC resync interval
- Hotspot SSID
- Hotspot password

Numeric values are validated and clamped before persistence. The hotspot SSID must be 1-32 characters. The password must be 8-63 characters or can be empty for an open network.

GPS correction latency is configured with `DEFAULT_GPS_TOTAL_LATENCY_MS` and may be changed at runtime. If dynamic transmission-time measurement is enabled in `config.h`, the measured NMEA burst transmission time is added to that latency.

## Time Source Behavior

The clock engine maintains one internal time base:

1. GPS corrects the clock opportunistically while a reliable fix is available.
2. The DS3231 corrects the clock while GPS is unavailable, if the RTC is present and readable.
3. The RTC is periodically synchronized from GPS for fallback use.
4. Sync quality degrades over time since the last successful correction.

GPS and RTC values correct the internal clock; they do not replace it as the clock source.

## Serial Diagnostics

Set the serial monitor to `115200` baud. The server reports:

- Boot and setup status
- AP address and connected client count
- GPS and RTC state
- Internal time and sync quality every five seconds
- Accepted and rejected correction attempts
- OTA progress and errors

## Building from Source

A typical Arduino IDE build uses `Tools > Board > ESP8266 Boards` and the sketch filename as the project folder. Keep `config.h` and all `.cpp`/`.h` files beside the main `.ino` file.

To erase persisted settings and restore EEPROM defaults, use the reset action on `/settings` or erase flash before uploading, depending on the board partition options.
