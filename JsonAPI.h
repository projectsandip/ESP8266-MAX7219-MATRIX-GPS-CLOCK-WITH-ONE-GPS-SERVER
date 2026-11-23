#ifndef JSON_API_H
#define JSON_API_H

#include <Arduino.h>
#include <ESP8266WebServer.h>
#include "ClockEngine.h"
#include "GPSManager.h"
#include "RTCManager.h"
#include "ClientRegistry.h"
#include "SettingsManager.h"

class JsonAPI {
public:
  JsonAPI(ClockEngine &clock, GPSManager &gps, RTCManager &rtc, SettingsManager &settings, uint16_t port);

  void begin();

  // Call every loop() iteration
  void handleClient();

  // How long ago the last HTTP request (any endpoint) was serviced —
  // used to correlate GPS correction outliers with WiFi/HTTP activity,
  // since SoftwareSerial's bit-banged GPS reception can be preempted by
  // WiFi interrupt bursts. (unsigned long)-1 if no request has landed yet.
  unsigned long msSinceLastRequest() const {
    if (_lastRequestMillis == 0) return (unsigned long)-1;
    return millis() - _lastRequestMillis;
  }

private:
  ESP8266WebServer _server;
  ClockEngine     &_clock;
  GPSManager      &_gps;
  RTCManager      &_rtc;
  SettingsManager &_settings;
  ClientRegistry   _registry;
  unsigned long    _lastRequestMillis;

  void handleTime();     // GET /time      -> JSON, full detail
  void handleTimeUTC();  // GET /time/utc  -> JSON, UTC instead of local
  void handleStatus();   // GET /status    -> JSON, everything incl. RTC/GPS diagnostics
  void handleRoot();     // GET /          -> human-readable status page
  void handleNotFound();

  // Admin-gated (HTTP Basic Auth, see ADMIN_USERNAME/ADMIN_PASSWORD)
  bool requireAuth();          // sends a 401 challenge and returns false if not authenticated
  void handleSettingsPage();   // GET  /settings       -> form showing current values
  void handleSettingsSave();   // POST /settings/save  -> applies + persists new values
  void handleRestart();        // POST /restart        -> restarts the device
  void handleUpdatePage();     // GET  /update          -> firmware upload form
  void handleUpdateFinish();   // POST /update          -> completes the upload, then restarts
  void handleUpdateUpload();   // upload handler (streamed chunks) for /update

  bool _otaUploadAuthorized;   // set at UPLOAD_FILE_START, checked on every subsequent chunk
};

#endif // JSON_API_H
