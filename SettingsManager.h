#ifndef SETTINGS_MANAGER_H
#define SETTINGS_MANAGER_H

#include <Arduino.h>

// Runtime-editable versions of what used to be compile-time-only clock
// timing constants (GPS latency compensation, GPS/RTC correction
// intervals), plus the AP hotspot SSID/password. Persisted to EEPROM
// (flash-emulated) so changes survive a reboot. Call begin() once in
// setup() AFTER EEPROM.begin(...).
//
// Validated/clamped on write so a bad form submission can't produce a
// value that would break the clock engine (e.g. a zero interval causing
// a busy-loop of corrections) or an invalid WiFi credential.
class SettingsManager {
public:
  SettingsManager();

  // Loads from EEPROM if valid data is present, otherwise initializes
  // with the DEFAULT_* values from config.h and writes them out.
  void begin();

  uint32_t gpsTotalLatencyMs() const { return _data.gpsTotalLatencyMs; }
  uint32_t gpsCorrectionIntervalMs() const { return _data.gpsCorrectionIntervalMs; }
  uint32_t rtcCorrectionIntervalMs() const { return _data.rtcCorrectionIntervalMs; }
  uint32_t rtcResyncIntervalMs() const { return _data.rtcResyncIntervalMs; }
  const char* apSsid() const { return _data.apSsid; }
  const char* apPassword() const { return _data.apPassword; }

  // Sets + clamps each value in memory only — call save() to persist.
  // Returns the clamped value actually stored, so callers (e.g. the
  // settings form handler) can report back what was actually applied.
  uint32_t setGpsTotalLatencyMs(uint32_t ms);
  uint32_t setGpsCorrectionIntervalMs(uint32_t ms);
  uint32_t setRtcCorrectionIntervalMs(uint32_t ms);
  uint32_t setRtcResyncIntervalMs(uint32_t ms);

  // SSID: 1-32 chars required. Password: empty (open network) or 8-63
  // chars (WPA2 minimum). Unlike the numeric setters above, these are
  // rejected outright (returning false, leaving the stored value
  // unchanged) rather than clamped — silently mangling a WiFi credential
  // would be worse than just refusing it.
  bool setApSsid(const String &ssid);
  bool setApPassword(const String &password);

  // Writes the current in-memory values to EEPROM.
  void save();

  // Resets all values to defaults (in memory) and persists immediately.
  void resetToDefaults();

private:
  struct PersistedData {
    uint32_t magic;
    uint16_t version;
    uint32_t gpsTotalLatencyMs;
    uint32_t gpsCorrectionIntervalMs;
    uint32_t rtcCorrectionIntervalMs;
    uint32_t rtcResyncIntervalMs;
    char     apSsid[33];      // 32 chars + null terminator (WiFi SSID limit)
    char     apPassword[64];  // 63 chars + null terminator (WPA2 limit)
    uint32_t checksum;
  };

  PersistedData _data;

  void loadDefaults();
  uint32_t computeChecksum(const PersistedData &d) const;
};

#endif // SETTINGS_MANAGER_H
