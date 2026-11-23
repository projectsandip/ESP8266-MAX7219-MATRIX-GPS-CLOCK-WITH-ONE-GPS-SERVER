#include "JsonAPI.h"
#include "StatusPage.h"
#include "config.h"
#include <ArduinoJson.h>
#include <Updater.h>

JsonAPI::JsonAPI(ClockEngine &clock, GPSManager &gps, RTCManager &rtc, SettingsManager &settings, uint16_t port)
  : _server(port), _clock(clock), _gps(gps), _rtc(rtc), _settings(settings), _lastRequestMillis(0),
    _otaUploadAuthorized(false) {}

void JsonAPI::begin() {
  _server.on("/time",     HTTP_GET, [this]() { handleTime(); });
  _server.on("/time/utc", HTTP_GET, [this]() { handleTimeUTC(); });
  _server.on("/status",   HTTP_GET, [this]() { handleStatus(); });
  _server.on("/",         HTTP_GET, [this]() { handleRoot(); });

  _server.on("/settings",      HTTP_GET,  [this]() { handleSettingsPage(); });
  _server.on("/settings/save", HTTP_POST, [this]() { handleSettingsSave(); });
  _server.on("/restart",       HTTP_POST, [this]() { handleRestart(); });
  _server.on("/update",        HTTP_GET,  [this]() { handleUpdatePage(); });
  _server.on("/update", HTTP_POST,
    [this]() { handleUpdateFinish(); },
    [this]() { handleUpdateUpload(); }
  );

  _server.onNotFound([this]() { handleNotFound(); });
  _server.begin();
}

void JsonAPI::handleClient() {
  _server.handleClient();
}

template <typename T>
static void fillTimeFields(T &doc, ClockEngine &clock, const CurrentTime &ct) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%04u-%02u-%02uT%02u:%02u:%02u.%03u",
           ct.year, ct.month, ct.day, ct.hour, ct.minute, ct.second, ct.millisecond);

  doc["iso8601"]       = buf;
  doc["year"]          = ct.year;
  doc["month"]         = ct.month;
  doc["day"]           = ct.day;
  doc["hour"]          = ct.hour;
  doc["minute"]        = ct.minute;
  doc["second"]        = ct.second;
  doc["millisecond"]   = ct.millisecond;
  doc["source"]        = clock.sourceName(ct.source);
  doc["syncQuality"]   = ct.syncQuality;
  doc["secondsSinceSync"] = ct.secondsSinceSync;
  doc["gpsFixValid"]   = ct.gpsFixValid;
  doc["gpsSatellites"] = ct.gpsSatellites;
}

void JsonAPI::handleTime() {
  _lastRequestMillis = millis();
  _registry.recordSync(_server.client().remoteIP());

  CurrentTime ct = _clock.getCurrentTimeLocal();
  JsonDocument doc;
  fillTimeFields(doc, _clock, ct);
  doc["timezone"] = "local";

  String out;
  serializeJson(doc, out);
  _server.send(200, "application/json", out);
}

void JsonAPI::handleTimeUTC() {
  _lastRequestMillis = millis();
  _registry.recordSync(_server.client().remoteIP());

  CurrentTime ct = _clock.getCurrentTime();
  JsonDocument doc;
  fillTimeFields(doc, _clock, ct);
  doc["timezone"] = "UTC";

  String out;
  serializeJson(doc, out);
  _server.send(200, "application/json", out);
}

void JsonAPI::handleStatus() {
  _lastRequestMillis = millis();
  CurrentTime ct = _clock.getCurrentTime();
  GPSSnapshot gpsSnap = _gps.getSnapshot();

  JsonDocument doc;
  JsonObject timeObj = doc["time"].to<JsonObject>();
  fillTimeFields(timeObj, _clock, ct);

  JsonObject gpsObj = doc["gps"].to<JsonObject>();
  gpsObj["hasFix"]     = gpsSnap.hasFix;
  gpsObj["isStale"]    = gpsSnap.isStale;
  gpsObj["satellites"] = gpsSnap.satellites;
  gpsObj["hdopTenths"] = gpsSnap.hdopTenths;
  gpsObj["lastFixAgeMs"] = gpsSnap.hasFix ? (millis() - gpsSnap.lastFixMillis) : -1;
  gpsObj["transmissionTimeMs"] = gpsSnap.transmissionTimeMs;

  JsonObject rtcObj = doc["rtc"].to<JsonObject>();
  rtcObj["present"]   = _rtc.isPresent();
  rtcObj["lostPower"] = _rtc.lostPower();
  unsigned long sinceSync = _rtc.millisSinceLastSync();
  rtcObj["msSinceLastGpsSync"] = (sinceSync == (unsigned long)-1) ? -1 : (long)sinceSync;

  JsonObject correctionObj = doc["lastCorrection"].to<JsonObject>();
  unsigned long correctionAge = _clock.lastCorrectionAgeMs();
  correctionObj["deltaMs"] = _clock.lastCorrectionDeltaMs();
  correctionObj["source"] = _clock.sourceName(_clock.lastCorrectionSource());
  correctionObj["ageMs"] = (correctionAge == (unsigned long)-1) ? -1 : (long)correctionAge;

  JsonObject settingsObj = doc["settings"].to<JsonObject>();
  settingsObj["gpsTotalLatencyMs"] = _settings.gpsTotalLatencyMs();
  settingsObj["gpsCorrectionIntervalMs"] = _settings.gpsCorrectionIntervalMs();
  settingsObj["rtcCorrectionIntervalMs"] = _settings.rtcCorrectionIntervalMs();
  settingsObj["rtcResyncIntervalMs"] = _settings.rtcResyncIntervalMs();
  settingsObj["apSsid"] = _settings.apSsid(); // password intentionally omitted

  JsonArray clientsArr = doc["clients"].to<JsonArray>();
  for (uint8_t i = 0; i < MAX_TRACKED_CLIENTS; i++) {
    const ClientRecord &rec = _registry.get(i);
    if (!rec.inUse) continue;
    JsonObject c = clientsArr.add<JsonObject>();
    c["ip"] = rec.ip.toString();
    c["lastSyncAgoMs"] = millis() - rec.lastSyncMillis;
    c["syncCount"] = rec.syncCount;
  }

  doc["uptimeMs"] = millis();
  doc["freeHeap"] = ESP.getFreeHeap();

  String out;
  serializeJson(doc, out);
  _server.send(200, "application/json", out);
}

void JsonAPI::handleRoot() {
  _lastRequestMillis = millis();
  CurrentTime ct = _clock.getCurrentTimeLocal();
  GPSSnapshot gpsSnap = _gps.getSnapshot();
  String html = buildStatusPage(_clock, ct, gpsSnap, _rtc, _registry);
  _server.send(200, "text/html", html);
}

void JsonAPI::handleNotFound() {
  _server.send(404, "application/json", "{\"error\":\"not found\"}");
}

// ---------------------------------------------------------------------------
// Admin auth — gates /settings, /restart, /update. Simple HTTP Basic Auth;
// anyone connected to the hotspot could otherwise reflash or restart the
// server, so this isn't optional for those three. Change ADMIN_USERNAME/
// ADMIN_PASSWORD in config.h before deploying.
// ---------------------------------------------------------------------------
bool JsonAPI::requireAuth() {
  if (_server.authenticate(ADMIN_USERNAME, ADMIN_PASSWORD)) {
    return true;
  }
  _server.requestAuthentication(BASIC_AUTH, "GPS Master Server Admin");
  return false;
}

// ---------------------------------------------------------------------------
// Settings page — view/edit the runtime-configurable clock timing values.
// ---------------------------------------------------------------------------
static String htmlAttrEscape(const String &s) {
  String out;
  out.reserve(s.length());
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '\'') out += "&#39;";
    else if (c == '"') out += "&quot;";
    else if (c == '<') out += "&lt;";
    else if (c == '>') out += "&gt;";
    else if (c == '&') out += "&amp;";
    else out += c;
  }
  return out;
}

void JsonAPI::handleSettingsPage() {
  _lastRequestMillis = millis();
  if (!requireAuth()) return;

  bool saved = _server.hasArg("saved");
  bool reset = _server.hasArg("reset");
  bool apError = _server.hasArg("apError");
  bool apChanged = _server.hasArg("apChanged");

  String html;
  html.reserve(3200);
  html += F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
            "<title>Settings - GPS Master Server</title>"
            "<style>"
            "body{font-family:monospace;background:#111;color:#eee;padding:20px;max-width:480px;}"
            "h1{font-size:1.1em;color:#8ecae6;}"
            "label{display:block;margin-top:14px;color:#999;}"
            "input[type=text],input[type=password],input[type=number]{width:100%;box-sizing:border-box;padding:6px;margin-top:4px;"
            "background:#222;border:1px solid #444;color:#eee;border-radius:4px;}"
            ".hint{color:#666;font-size:0.8em;margin-top:2px;}"
            "button{margin-top:18px;padding:8px 16px;border-radius:4px;border:none;"
            "background:#8ecae6;color:#111;font-weight:bold;cursor:pointer;}"
            ".danger{background:#e74c3c;color:#fff;}"
            ".msg{background:#2ecc71;color:#111;padding:8px;border-radius:4px;margin-bottom:10px;}"
            ".msg.warn{background:#f1c40f;}"
            ".msg.err{background:#e74c3c;color:#fff;}"
            ".checkbox-row{display:flex;align-items:center;gap:8px;margin-top:10px;}"
            ".checkbox-row input{width:auto;margin:0;}"
            "a{color:#8ecae6;}"
            "</style></head><body>");

  html += F("<h1>WiFi Hotspot</h1>");
  if (apError) html += F("<div class='msg err'>SSID/password rejected — SSID needs 1-32 characters, "
                          "password needs to be empty (open network) or 8-63 characters (WPA2 minimum). "
                          "Nothing was changed.</div>");
  if (apChanged) html += F("<div class='msg warn'>Hotspot settings saved — restart the server for them to take effect.</div>");

  html += F("<form method='POST' action='/settings/save'>");
  html += "<label>Hotspot name (SSID)</label><input type='text' name='apSsid' value='" +
          htmlAttrEscape(_settings.apSsid()) + "' maxlength='32'>";
  html += F("<div class='hint'>1-32 characters.</div>");

  html += F("<label>Hotspot password</label><input type='password' name='apPassword' "
            "placeholder='(unchanged — leave blank to keep current)' maxlength='63'>");
  html += F("<div class='hint'>Leave blank to keep the current password. WPA2 requires 8-63 characters, "
            "or check below for an open (no password) network.</div>");
  html += F("<div class='checkbox-row'><input type='checkbox' name='apOpen' id='apOpen' value='1'>"
            "<label for='apOpen' style='display:inline;margin:0;'>Make it an open network (no password)</label></div>");

  html += F("<button type='submit'>Save Hotspot Settings</button>"
            "</form>");

  html += F("<hr><h1>Clock Timing Settings</h1>");
  if (saved) html += F("<div class='msg'>Settings saved.</div>");
  if (reset) html += F("<div class='msg warn'>Everything reset to defaults, including hotspot name/password — restart the server for the hotspot change to take effect.</div>");

  html += F("<form method='POST' action='/settings/save'>");

  html += "<label>GPS total latency (ms)</label><input type='number' name='gpsLatency' value='" +
          String(_settings.gpsTotalLatencyMs()) + "' min='0' max='5000'>";
  html += F("<div class='hint'>Compensates NMEA transmission/parse delay. 0-5000ms.</div>");

  html += "<label>GPS correction interval (ms)</label><input type='number' name='gpsInterval' value='" +
          String(_settings.gpsCorrectionIntervalMs()) + "' min='1000' max='3600000'>";
  html += F("<div class='hint'>How often GPS corrects the internal clock while locked. 1s-1hr.</div>");

  html += "<label>RTC correction interval (ms)</label><input type='number' name='rtcCorrection' value='" +
          String(_settings.rtcCorrectionIntervalMs()) + "' min='1000' max='3600000'>";
  html += F("<div class='hint'>How often RTC corrects the internal clock while GPS is unavailable. 1s-1hr.</div>");

  html += "<label>RTC backup-write interval (ms)</label><input type='number' name='rtcResync' value='" +
          String(_settings.rtcResyncIntervalMs()) + "' min='1000' max='86400000'>";
  html += F("<div class='hint'>How often GPS backs up its time to the RTC as a fallback. 1s-24hr.</div>");

  html += F("<button type='submit'>Save</button>"
            "</form>");

  html += F("<form method='POST' action='/settings/save' onsubmit=\"return confirm('Reset ALL settings to defaults, including the hotspot name/password? This requires a restart to fully take effect.')\">"
            "<input type='hidden' name='reset' value='1'>"
            "<button type='submit' class='danger'>Reset Everything to Defaults</button>"
            "</form>");

  html += F("<hr><h1>Server Control</h1>"
            "<form method='POST' action='/restart' onsubmit=\"return confirm('Restart the server now?')\">"
            "<button type='submit' class='danger'>Restart Server</button>"
            "</form>"
            "<p><a href='/update'>Firmware update (OTA)</a></p>"
            "<p><a href='/'>&larr; Back to dashboard</a></p>"
            "</body></html>");

  _server.send(200, "text/html", html);
}

void JsonAPI::handleSettingsSave() {
  _lastRequestMillis = millis();
  if (!requireAuth()) return;

  if (_server.hasArg("reset")) {
    _settings.resetToDefaults();
    _server.sendHeader("Location", "/settings?reset=1");
    _server.send(303, "text/plain", "");
    return;
  }

  bool apChanged = false;
  bool apError = false;

  if (_server.hasArg("apSsid")) {
    String ssid = _server.arg("apSsid");
    if (!_settings.setApSsid(ssid)) {
      apError = true;
    } else {
      apChanged = true;
    }
  }

  bool wantsOpenNetwork = _server.hasArg("apOpen");
  String newPassword = _server.hasArg("apPassword") ? _server.arg("apPassword") : "";
  if (wantsOpenNetwork) {
    // Checkbox always wins, regardless of what (if anything) was typed
    // in the password field.
    _settings.setApPassword("");
    apChanged = true;
  } else if (newPassword.length() > 0) {
    // Blank means "leave the current password unchanged" — only a
    // non-blank submission actually changes it.
    if (!_settings.setApPassword(newPassword)) {
      apError = true;
    } else {
      apChanged = true;
    }
  }

  if (_server.hasArg("gpsLatency")) {
    _settings.setGpsTotalLatencyMs(_server.arg("gpsLatency").toInt());
  }
  if (_server.hasArg("gpsInterval")) {
    _settings.setGpsCorrectionIntervalMs(_server.arg("gpsInterval").toInt());
  }
  if (_server.hasArg("rtcCorrection")) {
    _settings.setRtcCorrectionIntervalMs(_server.arg("rtcCorrection").toInt());
  }
  if (_server.hasArg("rtcResync")) {
    _settings.setRtcResyncIntervalMs(_server.arg("rtcResync").toInt());
  }
  _settings.save();

  String location = "/settings?saved=1";
  if (apChanged) location += "&apChanged=1";
  if (apError) location += "&apError=1";
  _server.sendHeader("Location", location);
  _server.send(303, "text/plain", "");
}

// ---------------------------------------------------------------------------
// Restart
// ---------------------------------------------------------------------------
void JsonAPI::handleRestart() {
  _lastRequestMillis = millis();
  if (!requireAuth()) return;

  _server.send(200, "text/html",
    "<html><body style='font-family:monospace;background:#111;color:#eee;padding:20px;'>"
    "Restarting&hellip; reconnect to the hotspot and reload the dashboard in a few seconds."
    "</body></html>");
  _server.client().flush();
  delay(300);
  ESP.restart();
}

// ---------------------------------------------------------------------------
// OTA firmware update via web upload
// ---------------------------------------------------------------------------
void JsonAPI::handleUpdatePage() {
  _lastRequestMillis = millis();
  if (!requireAuth()) return;

  String html =
    "<html><head><meta charset='utf-8'><title>Firmware Update</title>"
    "<style>body{font-family:monospace;background:#111;color:#eee;padding:20px;max-width:480px;}"
    "h1{font-size:1.1em;color:#8ecae6;} input[type=file]{margin:14px 0;}"
    "button{padding:8px 16px;border-radius:4px;border:none;background:#8ecae6;color:#111;font-weight:bold;}"
    "a{color:#8ecae6;}</style></head><body>"
    "<h1>Firmware Update (OTA)</h1>"
    "<p>Select a compiled .bin file (Sketch &rarr; Export Compiled Binary in Arduino IDE) and upload it below. "
    "The device will restart automatically once the upload completes.</p>"
    "<form method='POST' action='/update' enctype='multipart/form-data' "
    "onsubmit=\"return confirm('Upload and flash this firmware now?')\">"
    "<input type='file' name='firmware' accept='.bin' required>"
    "<br><button type='submit'>Upload &amp; Flash</button>"
    "</form>"
    "<p><a href='/settings'>&larr; Back to settings</a></p>"
    "</body></html>";

  _server.send(200, "text/html", html);
}

void JsonAPI::handleUpdateUpload() {
  // Auth is checked once at the start of the upload stream. If it fails,
  // _otaUploadAuthorized stays false for the rest of this request, so
  // subsequent WRITE/END chunks skip Update entirely rather than calling
  // write()/end() on a session that was never begin()'d.
  HTTPUpload &upload = _server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    _otaUploadAuthorized = _server.authenticate(ADMIN_USERNAME, ADMIN_PASSWORD);
    if (!_otaUploadAuthorized) {
      Serial.println("[OTA] Upload rejected: not authenticated.");
      return;
    }
    Serial.printf("[OTA] Upload starting: %s\n", upload.filename.c_str());
    // ESP8266's Updater.h has no UPDATE_SIZE_UNKNOWN constant (that's an
    // ESP32-ism) — the standard ESP8266 pattern is to compute the
    // available sketch space explicitly instead.
    uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
    if (!Update.begin(maxSketchSpace)) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (!_otaUploadAuthorized) return;
    Update.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (!_otaUploadAuthorized) return;
    bool ok = Update.end(true);
    Serial.printf("[OTA] Upload finished: %s (%u bytes)\n", ok ? "OK" : "FAILED", upload.totalSize);
  }
}

void JsonAPI::handleUpdateFinish() {
  if (!_server.authenticate(ADMIN_USERNAME, ADMIN_PASSWORD)) {
    _server.requestAuthentication(BASIC_AUTH, "GPS Master Server Admin");
    return;
  }

  bool ok = _otaUploadAuthorized && !Update.hasError();
  _otaUploadAuthorized = false; // reset for the next attempt
  _server.sendHeader("Connection", "close");
  _server.send(200, "text/html",
    ok ? "<html><body style='font-family:monospace;background:#111;color:#eee;padding:20px;'>"
         "Update OK &mdash; restarting&hellip;</body></html>"
       : "<html><body style='font-family:monospace;background:#111;color:#eee;padding:20px;'>"
         "Update FAILED. Device was not restarted &mdash; try again.</body></html>");

  if (ok) {
    delay(300);
    ESP.restart();
  }
}
