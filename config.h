#ifndef CONFIG_H
#define CONFIG_H

// ---------------------------------------------------------------------------
// WiFi — Access Point (hotspot) mode
// This device IS the network. Client clocks connect directly to it instead
// of both devices joining a home router's WiFi.
// ---------------------------------------------------------------------------
// These are now the DEFAULT values only — the live values are runtime-
// editable via the /settings page and persisted in EEPROM (see
// SettingsManager). These constants are what a fresh device (or a
// "reset to defaults") starts from.
#define DEFAULT_AP_SSID       "GPS_Master_Server"
#define DEFAULT_AP_PASSWORD   "123456789"   // WPA2 requires 8+ chars; use "" for an open network
#define AP_CHANNEL    1               // 1, 6, or 11 avoid overlap with neighboring WiFi
#define AP_HIDDEN     false
#define AP_MAX_CLIENTS 4              // ESP8266 softAP supports up to 8

// Access point IP — clients hit this directly (e.g. http://192.168.4.1/time).
// 192.168.4.1 is the ESP8266 default; only change if it conflicts with
// something else on your setup.
#define AP_LOCAL_IP    192, 168, 4, 1
#define AP_GATEWAY_IP  192, 168, 4, 1
#define AP_SUBNET_MASK 255, 255, 255, 0

// mDNS hostname — reachable as http://gps-clock.local/ from phones/laptops
// connected to the hotspot, handy for checking the dashboard without
// remembering the IP. Client microcontrollers should just use AP_LOCAL_IP
// directly (lighter than an mDNS lookup).
#define MDNS_HOSTNAME "gps-clock"

#define HTTP_PORT 80

// ---------------------------------------------------------------------------
// GPS (NEO-6M) — SoftwareSerial pins
// ---------------------------------------------------------------------------
// Using raw GPIO numbers instead of Dx labels — the Dx macros (D5, D6, etc.)
// are only defined by some board variants (e.g. NodeMCU). Raw GPIO numbers
// work regardless of which ESP8266 board is selected.
// GPIO14 = D5 on NodeMCU, GPIO12 = D6 on NodeMCU — adjust if your board's
// silkscreen uses different labels.
#define GPS_RX_PIN 14   // ESP8266 pin connected to GPS TX
#define GPS_TX_PIN 12   // ESP8266 pin connected to GPS RX
#define GPS_BAUD   9600

// If GPS has had no valid fix update for this long, treat data as stale
#define GPS_STALE_TIMEOUT_MS 5000

// NMEA sentences take measurable time to fully arrive over serial and get
// parsed before a fix is available. The total delay between the true UTC
// second boundary and when a fix is detected has two genuinely different
// components:
//   1. Fixed chip-internal processing delay (before the module even
//      starts transmitting) — this is what this constant now represents.
//   2. Serial transmission time — calculated dynamically, per-fix, from
//      the actual character count of that fix's sentence burst (see
//      GPSManager's byte-counting; naturally accounts for burst-length
//      variation from satellite count affecting how many/long GSV
//      sentences precede RMC).
// ClockEngine adds this constant to the dynamically-measured transmission
// time to get the true total delay for each specific fix.
//
// IMPORTANT: this constant used to represent the WHOLE combined delay
// (chip delay + an implicit average transmission time baked into
// whatever was measured during calibration). Now that transmission time
// is measured separately and added on top, this needs to be RE-TUNED
// LOWER — roughly by whatever transmission time you're now seeing logged
// (see GPS_TRANSMIT_TIME_LOG_ENABLED below) — or fixes will be
// over-corrected (transmission time counted twice). Tune the same way as
// before: compare /status or the serial log against a reference clock:
//   - server reads LATE (behind reference)   -> increase this value
//   - server reads EARLY (ahead of reference) -> decrease this value
//
// This is now the DEFAULT value only — the live value is runtime-editable
// via the /settings page and persisted in EEPROM (see SettingsManager).
// This constant is what a fresh device (or a "reset to defaults") starts
// from.
#define DEFAULT_GPS_TOTAL_LATENCY_MS 600

// If true, ClockEngine adds the dynamically-measured transmission time
// (GPSSnapshot::transmissionTimeMs) to settings.gpsTotalLatencyMs() for
// each correction. If false, transmission time is ignored and behavior
// reverts to the old model (settings.gpsTotalLatencyMs() alone is the
// whole delay) — useful for comparing the two without deleting code.
#define GPS_USE_DYNAMIC_TRANSMISSION_TIME 0

// A gap this long with no bytes arriving is treated as "the previous
// second's sentence burst ended" — the next byte after this gap starts a
// new burst, resetting the character count used to calculate
// transmission time. GPS modules typically go quiet for most of each
// second between bursts, so 100ms is comfortably longer than any normal
// inter-sentence gap within a burst, but well under the ~900ms+ of
// silence between bursts.
#define GPS_BURST_GAP_THRESHOLD_MS 100

// Logs each fix's calculated transmission time to serial
// ([GPS] transmitTimeMs=...). Purely informational — doesn't affect
// GPS_USE_DYNAMIC_TRANSMISSION_TIME above either way. Set to 0 to quiet
// the serial log once you've confirmed it's behaving as expected.
#define GPS_TRANSMIT_TIME_LOG_ENABLED 1

// ---------------------------------------------------------------------------
// RTC (DS3231) — I2C
// ---------------------------------------------------------------------------
// GPIO4 = D2 on NodeMCU (SDA), GPIO5 = D1 on NodeMCU (SCL)
#define RTC_SDA_PIN 4
#define RTC_SCL_PIN 5

// How often GPS is allowed to correct the internal clock, while it's
// available. Between corrections, the internal clock just keeps ticking
// off millis() on its own — GPS is not re-applied on every single fix.
// This is now the DEFAULT value only — see DEFAULT_GPS_TOTAL_LATENCY_MS
// above; runtime-editable via /settings.
#define DEFAULT_GPS_CORRECTION_INTERVAL_MS (10UL * 1000UL)  // 10 seconds

// A GPS correction larger than this (in either direction) is treated as
// suspect rather than genuine drift — over a GPS_CORRECTION_INTERVAL_MS
// window this short, a real correction this large would imply an absurd
// crystal error. More likely cause: ESP8266 WiFi activity (AP mode, HTTP
// serving, mDNS) occasionally preempting SoftwareSerial's bit-banged GPS
// reception, delaying when a sentence finishes processing without
// corrupting it — GPS_TOTAL_LATENCY_MS assumes a fixed delay, so any extra
// one-off delay shows up as an inflated correction. Suspect corrections
// are rejected and retried on the very next fix rather than applied blindly.
#define GPS_CORRECTION_OUTLIER_THRESHOLD_MS 100

// Safety valve: after this many consecutive rejections, apply anyway
// rather than risk never correcting at all (e.g. if the "outlier" turns
// out to be real).
#define GPS_CORRECTION_OUTLIER_MAX_REJECTS 3

// Minimum time between GPS correction attempts after a rejection. Without
// this, a rejected attempt could be retried against the very next NMEA
// sentence (GGA/RMC/etc. are sent seconds apart within the same 1-second
// burst), which isn't really an independent sample — it's very likely to
// show the same issue and just burns through the reject budget in
// milliseconds. This forces retries to wait for what's genuinely closer
// to the next independent ~1Hz GPS update.
#define GPS_CORRECTION_RETRY_SPACING_MS 800

// How often GPS backs up its time to the RTC (purely as a fallback source
// for later — this write is never itself read back as the live clock).
// This is now the DEFAULT value only — runtime-editable via /settings.
#define DEFAULT_RTC_RESYNC_INTERVAL_MS (10UL * 60UL * 1000UL)  // 10 minutes

// This is now the DEFAULT value only — runtime-editable via /settings.
#define DEFAULT_RTC_CORRECTION_INTERVAL_MS (5UL * 60UL * 1000UL)  // 5 minutes

// The DS3231 has no sub-second register, so a single read at an arbitrary
// point within the current second could be off by up to ~1s from true
// time. Instead, its seconds register is polled (non-blocking, spread
// across update() calls) until observed to increment, and the correction
// is applied at that precise instant. RTC_EDGE_POLL_INTERVAL_MS throttles
// how often the actual I2C read happens while waiting (lower = tighter
// alignment, more bus traffic). RTC_EDGE_CATCH_TIMEOUT_MS is a safety
// net — a second always ticks over within 1000ms, so this should never
// normally be hit.
#define RTC_EDGE_POLL_INTERVAL_MS 5
#define RTC_EDGE_CATCH_TIMEOUT_MS 1200

// ---------------------------------------------------------------------------
// Clock engine
// ---------------------------------------------------------------------------
// If RTC hasn't been touched for longer than this, trust it less
#define RTC_STALE_TIMEOUT_MS (24UL * 60UL * 60UL * 1000UL) // 24 hours

// Time zone offset applied on top of internally-stored UTC (IST = +5:30)
#define TZ_OFFSET_MINUTES (5 * 60 + 30)

// Status LED (optional, active-low on most boards)
#define STATUS_LED_PIN LED_BUILTIN

// ---------------------------------------------------------------------------
// Admin authentication — gates /settings, /restart, and /update (firmware
// upload), since anyone connected to the hotspot could otherwise reflash
// or restart the server. Change these before deploying.
// ---------------------------------------------------------------------------
#define ADMIN_USERNAME "admin"
#define ADMIN_PASSWORD "changeme123"

// ---------------------------------------------------------------------------
// OTA (over-the-air firmware updates)
// ---------------------------------------------------------------------------
// ArduinoOTA — lets you flash new firmware over WiFi from the Arduino IDE
// (Tools -> Port -> shows up as a network port once the device is on and
// this is enabled) instead of needing a USB cable every time.
#define OTA_HOSTNAME "gps-master-server"
#define OTA_PASSWORD "changeme123"  // set to "" to disable password (not recommended)

#endif // CONFIG_H
