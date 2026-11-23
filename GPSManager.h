#ifndef GPS_MANAGER_H
#define GPS_MANAGER_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

// Snapshot of everything the rest of the system needs to know about the
// current GPS state, fetched in one call so callers don't have to poke
// at TinyGPSPlus internals directly.
struct GPSSnapshot {
  bool     hasFix;          // true if TinyGPSPlus reports a valid location+date+time
  bool     isStale;         // true if no fresh fix in GPS_STALE_TIMEOUT_MS
  uint8_t  satellites;      // satellites in use (0 if unknown)
  int32_t  hdopTenths;      // HDOP * 10 (rough signal quality indicator), -1 if unknown
  uint16_t year;
  uint8_t  month, day, hour, minute, second;
  uint32_t centisecond;     // sub-second fraction from GPS fix (0-99)
  unsigned long lastFixMillis; // millis() timestamp of the last valid fix
  uint16_t transmissionTimeMs; // calculated serial transmission time for the
                                 // sentence burst leading up to this fix (see
                                 // GPSManager's byte-counting burst detection)
};

class GPSManager {
public:
  GPSManager();

  void begin();

  // Call every loop() iteration — feeds serial bytes into the parser
  void update();

  // Returns the latest known state
  GPSSnapshot getSnapshot() const;

  // Convenience: true if we currently have a usable, fresh fix
  bool isReliable() const;

private:
  SoftwareSerial _serial;
  TinyGPSPlus    _gps;
  GPSSnapshot    _snapshot;

  // Byte-counting burst detection: tracks how many characters have
  // arrived since the current NMEA sentence burst started (detected via
  // a gap in incoming bytes), so the serial transmission-time component
  // of GPS latency can be calculated per-fix instead of assumed as a
  // fixed constant. See GPS_BURST_GAP_THRESHOLD_MS in config.h.
  unsigned long _lastByteMillis;
  unsigned long _burstStartMillis;
  uint16_t      _charCountSinceBurst;

  void refreshSnapshot();
};

#endif // GPS_MANAGER_H
