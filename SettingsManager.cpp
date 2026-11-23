#include "SettingsManager.h"
#include "config.h"
#include <EEPROM.h>
#include <stddef.h>

#define SETTINGS_MAGIC 0x47504353UL // "GPCS"
#define SETTINGS_VERSION 2 // bumped: v2 adds apSsid/apPassword — old v1 data
                            // must be detected as invalid (version mismatch)
                            // rather than partially reused, since reading the
                            // new, larger struct over old, shorter saved data
                            // would leave the new fields containing whatever
                            // was previously in that flash region.

// Clamp bounds — chosen to keep the clock engine's timing logic sane
// regardless of what gets submitted through the settings form.
#define MIN_GPS_LATENCY_MS 0UL
#define MAX_GPS_LATENCY_MS 5000UL
#define MIN_GPS_CORRECTION_INTERVAL_MS 1000UL
#define MAX_GPS_CORRECTION_INTERVAL_MS (3600UL * 1000UL)       // 1 hour
#define MIN_RTC_CORRECTION_INTERVAL_MS 1000UL
#define MAX_RTC_CORRECTION_INTERVAL_MS (3600UL * 1000UL)       // 1 hour
#define MIN_RTC_RESYNC_INTERVAL_MS 1000UL
#define MAX_RTC_RESYNC_INTERVAL_MS (24UL * 3600UL * 1000UL)    // 24 hours

static uint32_t clampU32(uint32_t v, uint32_t lo, uint32_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

SettingsManager::SettingsManager() {
  memset(&_data, 0, sizeof(_data));
}

uint32_t SettingsManager::computeChecksum(const PersistedData &d) const {
  // Lightweight corruption check, not cryptographic — sums every byte of
  // the struct except the checksum field itself.
  const uint8_t *bytes = reinterpret_cast<const uint8_t*>(&d);
  size_t checksumOffset = offsetof(PersistedData, checksum);
  uint32_t sum = 0;
  for (size_t i = 0; i < sizeof(PersistedData); i++) {
    if (i >= checksumOffset && i < checksumOffset + sizeof(uint32_t)) continue;
    sum += bytes[i];
    sum = (sum << 1) | (sum >> 31); // rotate, so byte order/position matters
  }
  return sum;
}

void SettingsManager::loadDefaults() {
  _data.magic = SETTINGS_MAGIC;
  _data.version = SETTINGS_VERSION;
  _data.gpsTotalLatencyMs = DEFAULT_GPS_TOTAL_LATENCY_MS;
  _data.gpsCorrectionIntervalMs = DEFAULT_GPS_CORRECTION_INTERVAL_MS;
  _data.rtcCorrectionIntervalMs = DEFAULT_RTC_CORRECTION_INTERVAL_MS;
  _data.rtcResyncIntervalMs = DEFAULT_RTC_RESYNC_INTERVAL_MS;

  strncpy(_data.apSsid, DEFAULT_AP_SSID, sizeof(_data.apSsid) - 1);
  _data.apSsid[sizeof(_data.apSsid) - 1] = '\0';
  strncpy(_data.apPassword, DEFAULT_AP_PASSWORD, sizeof(_data.apPassword) - 1);
  _data.apPassword[sizeof(_data.apPassword) - 1] = '\0';
}

void SettingsManager::begin() {
  PersistedData loaded;
  EEPROM.get(0, loaded);

  bool valid = (loaded.magic == SETTINGS_MAGIC) &&
               (loaded.version == SETTINGS_VERSION) &&
               (loaded.checksum == computeChecksum(loaded));

  if (valid) {
    _data = loaded;
    Serial.println("[Settings] Loaded from EEPROM.");
  } else {
    Serial.println("[Settings] No valid saved settings found — using defaults.");
    loadDefaults();
    save();
  }
}

uint32_t SettingsManager::setGpsTotalLatencyMs(uint32_t ms) {
  _data.gpsTotalLatencyMs = clampU32(ms, MIN_GPS_LATENCY_MS, MAX_GPS_LATENCY_MS);
  return _data.gpsTotalLatencyMs;
}

uint32_t SettingsManager::setGpsCorrectionIntervalMs(uint32_t ms) {
  _data.gpsCorrectionIntervalMs = clampU32(ms, MIN_GPS_CORRECTION_INTERVAL_MS, MAX_GPS_CORRECTION_INTERVAL_MS);
  return _data.gpsCorrectionIntervalMs;
}

uint32_t SettingsManager::setRtcCorrectionIntervalMs(uint32_t ms) {
  _data.rtcCorrectionIntervalMs = clampU32(ms, MIN_RTC_CORRECTION_INTERVAL_MS, MAX_RTC_CORRECTION_INTERVAL_MS);
  return _data.rtcCorrectionIntervalMs;
}

uint32_t SettingsManager::setRtcResyncIntervalMs(uint32_t ms) {
  _data.rtcResyncIntervalMs = clampU32(ms, MIN_RTC_RESYNC_INTERVAL_MS, MAX_RTC_RESYNC_INTERVAL_MS);
  return _data.rtcResyncIntervalMs;
}

bool SettingsManager::setApSsid(const String &ssid) {
  if (ssid.length() < 1 || ssid.length() > 32) return false;
  strncpy(_data.apSsid, ssid.c_str(), sizeof(_data.apSsid) - 1);
  _data.apSsid[sizeof(_data.apSsid) - 1] = '\0';
  return true;
}

bool SettingsManager::setApPassword(const String &password) {
  // Empty is valid (open network). Otherwise WPA2 requires 8-63 chars.
  if (password.length() != 0 && (password.length() < 8 || password.length() > 63)) {
    return false;
  }
  strncpy(_data.apPassword, password.c_str(), sizeof(_data.apPassword) - 1);
  _data.apPassword[sizeof(_data.apPassword) - 1] = '\0';
  return true;
}

void SettingsManager::save() {
  _data.magic = SETTINGS_MAGIC;
  _data.version = SETTINGS_VERSION;
  _data.checksum = computeChecksum(_data);
  EEPROM.put(0, _data);
  EEPROM.commit();
  Serial.println("[Settings] Saved to EEPROM.");
}

void SettingsManager::resetToDefaults() {
  loadDefaults();
  save();
}
