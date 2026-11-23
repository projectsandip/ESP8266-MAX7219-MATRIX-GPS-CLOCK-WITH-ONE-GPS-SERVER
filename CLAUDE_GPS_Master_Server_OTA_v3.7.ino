// ---------------------------------------------------------------------------
// GPS_Master_Server_v3
//
// ESP8266 server that maintains an accurate time base from a NEO-6M GPS
// module with a DS3231 RTC as backup, and serves it over HTTP as JSON
// (plus a small human-readable status page) for one or more clients
// (e.g. a MAX7219 matrix clock display) to poll.
//
// Runs a single free-running internal clock (see ClockEngine) that GPS
// and RTC only ever *correct*, never directly serve as. GPS corrects it
// opportunistically while locked; RTC corrects it occasionally, edge-
// aligned to the true second, only while GPS is unavailable. Reported
// sync quality degrades the longer it's been since either has corrected
// the clock. All three correction intervals (plus GPS latency
// compensation) are runtime-editable at /settings.
// ---------------------------------------------------------------------------

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <EEPROM.h>
#include <ArduinoOTA.h>

#include "config.h"
#include "GPSManager.h"
#include "RTCManager.h"
#include "SettingsManager.h"
#include "ClockEngine.h"
#include "JsonAPI.h"

// EEPROM size for SettingsManager's persisted struct — generous headroom
// for future settings without needing to touch this again. (The struct
// itself is currently under 130 bytes even with the SSID/password fields.)
#define EEPROM_SIZE 256

GPSManager      gpsManager;
RTCManager      rtcManager;
SettingsManager settingsManager;
ClockEngine     clockEngine(gpsManager, rtcManager, settingsManager);
JsonAPI         jsonApi(clockEngine, gpsManager, rtcManager, settingsManager, HTTP_PORT);

unsigned long lastStatusPrint = 0;
unsigned long lastClientCheck = 0;
uint8_t lastClientCount = 0;
unsigned long lastSeenCorrectionAgeMs = (unsigned long)-1;
unsigned long lastSeenGpsAttemptAgeMs = (unsigned long)-1;

void startAccessPoint() {
  WiFi.mode(WIFI_AP);

  IPAddress localIP(AP_LOCAL_IP);
  IPAddress gatewayIP(AP_GATEWAY_IP);
  IPAddress subnet(AP_SUBNET_MASK);
  WiFi.softAPConfig(localIP, gatewayIP, subnet);

  const char* ssid = settingsManager.apSsid();
  const char* password = settingsManager.apPassword();
  bool ok = WiFi.softAP(ssid, password, AP_CHANNEL, AP_HIDDEN, AP_MAX_CLIENTS);

  if (ok) {
    Serial.println("[AP] Hotspot started.");
    Serial.print("[AP] SSID = ");
    Serial.println(ssid);
    Serial.print("[AP] IP   = ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("[AP] ERROR: failed to start access point!");
  }

  if (MDNS.begin(MDNS_HOSTNAME)) {
    MDNS.addService("http", "tcp", HTTP_PORT);
    Serial.print("[mDNS] Responder started: http://");
    Serial.print(MDNS_HOSTNAME);
    Serial.println(".local/ (for browser access from a connected client)");
  } else {
    Serial.println("[mDNS] WARNING: failed to start responder.");
  }
}

// Logs when client clocks connect/disconnect from the hotspot — useful for
// confirming your client device is actually joining.
void checkConnectedClients() {
  if (millis() - lastClientCheck < 2000) return;
  lastClientCheck = millis();

  uint8_t count = WiFi.softAPgetStationNum();
  if (count != lastClientCount) {
    Serial.printf("[AP] Connected clients: %u\n", count);
    lastClientCount = count;
  }
}

// Detects when a new GPS/RTC correction has just landed (by watching for
// lastCorrectionAgeMs() resetting to near-zero) and logs it alongside how
// recently an HTTP request was serviced — lets you check whether large
// corrections correlate with WiFi/HTTP activity (see GPS_CORRECTION_
// OUTLIER_THRESHOLD_MS in config.h for the theory this is checking).
void logCorrectionEvents() {
  unsigned long currentAge = clockEngine.lastCorrectionAgeMs();

  bool isNewCorrection = (currentAge != (unsigned long)-1) &&
                         (lastSeenCorrectionAgeMs == (unsigned long)-1 || currentAge < lastSeenCorrectionAgeMs);

  if (isNewCorrection) {
    int32_t delta = clockEngine.lastCorrectionDeltaMs();
    TimeSource src = clockEngine.lastCorrectionSource();
    unsigned long reqAgo = jsonApi.msSinceLastRequest();
    bool flagged = abs(delta) > (int32_t)GPS_CORRECTION_OUTLIER_THRESHOLD_MS;

    Serial.printf("[Correction] %s delta=%+ldms%s | last HTTP request %s\n",
                  clockEngine.sourceName(src), (long)delta,
                  flagged ? " **LARGE**" : "",
                  (reqAgo == (unsigned long)-1) ? "never" : (String(reqAgo) + "ms ago").c_str());
  }

  lastSeenCorrectionAgeMs = currentAge;
}

// Same idea as logCorrectionEvents(), but fires on every GPS correction
// *attempt* — including rejected ones, which logCorrectionEvents() can't
// see since only accepted corrections update lastCorrectionAgeMs(). This
// is what actually lets you correlate rejected/outlier attempts with
// HTTP activity, not just the one that eventually got accepted.
void logGpsAttemptEvents() {
  unsigned long currentAge = clockEngine.lastGpsAttemptAgeMs();

  bool isNewAttempt = (currentAge != (unsigned long)-1) &&
                      (lastSeenGpsAttemptAgeMs == (unsigned long)-1 || currentAge < lastSeenGpsAttemptAgeMs);

  if (isNewAttempt) {
    int32_t delta = clockEngine.lastGpsAttemptDeltaMs();
    bool rejected = clockEngine.lastGpsAttemptWasRejected();
    unsigned long reqAgo = jsonApi.msSinceLastRequest();

    Serial.printf("[GpsAttempt] delta=%+ldms %s | last HTTP request %s\n",
                  (long)delta, rejected ? "REJECTED" : "accepted",
                  (reqAgo == (unsigned long)-1) ? "never" : (String(reqAgo) + "ms ago").c_str());
  }

  lastSeenGpsAttemptAgeMs = currentAge;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== GPS_Master_Server_v3 booting ===");

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH); // off (active-low on most boards)

  // Settings must load before the AP starts, since the AP's SSID/password
  // now come from here rather than fixed compile-time constants.
  EEPROM.begin(EEPROM_SIZE);
  settingsManager.begin();

  startAccessPoint();

  gpsManager.begin();

  if (rtcManager.begin()) {
    Serial.println("[RTC] DS3231 found.");
  } else {
    Serial.println("[RTC] WARNING: DS3231 not found. Running GPS + freewheel only.");
  }

  clockEngine.begin();
  jsonApi.begin();

  ArduinoOTA.setHostname(OTA_HOSTNAME);
  if (strlen(OTA_PASSWORD) > 0) {
    ArduinoOTA.setPassword(OTA_PASSWORD);
  }
  ArduinoOTA.onStart([]() { Serial.println("[OTA] Update starting..."); });
  ArduinoOTA.onEnd([]() { Serial.println("[OTA] Update complete, rebooting."); });
  ArduinoOTA.onError([](ota_error_t error) { Serial.printf("[OTA] Error [%u]\n", error); });
  ArduinoOTA.begin();
  Serial.println("[OTA] ArduinoOTA ready (flash over WiFi from Arduino IDE).");

  Serial.println("=== Setup complete, entering main loop ===");
}

void loop() {
  ArduinoOTA.handle();
  checkConnectedClients();
  MDNS.update();

  gpsManager.update();
  clockEngine.update();
  logCorrectionEvents();
  logGpsAttemptEvents();
  jsonApi.handleClient();

  // Lightweight heartbeat / diagnostics over serial every 5s
  if (millis() - lastStatusPrint > 5000) {
    lastStatusPrint = millis();
    CurrentTime ct = clockEngine.getCurrentTime();
    Serial.printf("[%04u-%02u-%02u %02u:%02u:%02u.%03u] source=%s quality=%u sats=%u\n",
                  ct.year, ct.month, ct.day, ct.hour, ct.minute, ct.second, ct.millisecond,
                  clockEngine.sourceName(ct.source), ct.syncQuality, ct.gpsSatellites);
  }

  // Blink status LED faster when we don't have a good fix, slower when locked
  static unsigned long lastBlink = 0;
  unsigned long blinkInterval = gpsManager.isReliable() ? 1000 : 200;
  if (millis() - lastBlink > blinkInterval) {
    lastBlink = millis();
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
  }
}
