#include "RTCManager.h"
#include "config.h"
#include <Wire.h>

RTCManager::RTCManager()
  : _present(false), _lastSyncMillis(0) {}

bool RTCManager::begin() {
  // Explicitly set the I2C pins rather than relying on ESP8266 core
  // defaults (which happen to match RTC_SDA_PIN/RTC_SCL_PIN in this config,
  // but shouldn't be relied on implicitly if those ever change).
  Wire.begin(RTC_SDA_PIN, RTC_SCL_PIN);

  _present = _rtc.begin();
  if (_present && _rtc.lostPower()) {
    // Chip is present but its battery-backed time is meaningless.
    // Leave it as-is; ClockEngine will refuse to trust it until GPS syncs it.
    Serial.println("[RTC] DS3231 found but reports lost power — needs GPS sync.");
  }
  return _present;
}

bool RTCManager::isPresent() const {
  return _present;
}

bool RTCManager::lostPower() {
  if (!_present) return true;
  return _rtc.lostPower();
}

DateTime RTCManager::nowUTC() {
  return _rtc.now();
}

void RTCManager::setUTC(uint16_t year, uint8_t month, uint8_t day,
                         uint8_t hour, uint8_t minute, uint8_t second) {
  if (!_present) return;
  _rtc.adjust(DateTime(year, month, day, hour, minute, second));
  _lastSyncMillis = millis();
}

unsigned long RTCManager::millisSinceLastSync() const {
  if (_lastSyncMillis == 0) return (unsigned long)-1; // never synced
  return millis() - _lastSyncMillis;
}
