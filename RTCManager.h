#ifndef RTC_MANAGER_H
#define RTC_MANAGER_H

#include <Arduino.h>
#include <RTClib.h>

// RTC always stores/returns UTC. Timezone offset is applied elsewhere.
class RTCManager {
public:
  RTCManager();

  bool begin();               // returns false if RTC not found on the bus

  bool isPresent() const;      // did begin() find the chip?
  bool lostPower();            // did the RTC lose power since last set?
                                // (not const — RTClib's underlying call isn't const)

  DateTime nowUTC();           // current UTC time per the RTC

  // Set the RTC to a specific UTC time (e.g. from a fresh GPS fix)
  void setUTC(uint16_t year, uint8_t month, uint8_t day,
              uint8_t hour, uint8_t minute, uint8_t second);

  unsigned long millisSinceLastSync() const;

private:
  RTC_DS3231    _rtc;
  bool          _present;
  unsigned long _lastSyncMillis;
};

#endif // RTC_MANAGER_H
