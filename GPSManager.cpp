#include "GPSManager.h"
#include "config.h"

GPSManager::GPSManager()
  : _serial(GPS_RX_PIN, GPS_TX_PIN),
    _lastByteMillis(0), _burstStartMillis(0), _charCountSinceBurst(0) {
  memset(&_snapshot, 0, sizeof(_snapshot));
  _snapshot.hdopTenths = -1;
}

void GPSManager::begin() {
  _serial.begin(GPS_BAUD);
}

void GPSManager::update() {
  // Drain whatever bytes are available without blocking.
  while (_serial.available() > 0) {
    // Byte-counting burst detection: tracks how many characters have
    // arrived since the current NMEA sentence burst started, so the
    // serial transmission-time component of GPS latency can be
    // calculated per-fix (see refreshSnapshot()) instead of assumed as
    // part of a single fixed constant.
    unsigned long byteNow = millis();
    if (byteNow - _lastByteMillis > GPS_BURST_GAP_THRESHOLD_MS) {
      // A long enough silence means the previous burst ended — this byte
      // starts a new one.
      _burstStartMillis = byteNow;
      _charCountSinceBurst = 0;
    }
    _charCountSinceBurst++;
    _lastByteMillis = byteNow;

    _gps.encode(_serial.read());
  }

  // IMPORTANT: isValid() on TinyGPSPlus's location/date/time objects only
  // means "this field has ever been successfully set" — it latches true
  // forever once GPS achieves lock once, and does NOT reset when lock is
  // lost or when a sentence carries no new position data (e.g. GSV/GSA).
  // Relying on isValid() alone here previously meant a stale/frozen fix
  // from minutes ago could keep getting treated as "fresh" indefinitely,
  // which meant isStale never flipped and the reported source stayed GPS
  // even after lock was actually lost.
  //
  // age() gives milliseconds since the value was genuinely last updated,
  // which is what we actually need to gate freshness on.
  // IMPORTANT: gate on date.age(), not location.age(). Both GGA and RMC
  // sentences carry position, so location gets updated by whichever one
  // happens to finish parsing first each cycle — and since GGA and RMC
  // land at different points within the same 1-second NMEA burst, that
  // created two different "how long after the true fix" timings
  // depending on which sentence won the race (observed as a bimodal
  // correction distribution: a ~0-40ms cluster and a ~150ms cluster).
  // date is exclusive to RMC (GGA doesn't carry it at all), so gating on
  // date.age() guarantees every detection is anchored to RMC specifically
  // — one consistent reference sentence, not a race between two.
  //
  // HOWEVER: date.age() alone isn't enough to confirm an actual current
  // fix. Many receivers keep outputting RMC time-of-day fields from their
  // own internal clock even during a void/no-fix status, so date.age()
  // can keep resetting every second even after lock is genuinely lost —
  // which meant isStale never triggered and the reported source stayed
  // GPS indefinitely after an actual lock loss. location.age() must also
  // be checked: location is only ever updated when there's a real fix, so
  // requiring it to be reasonably fresh too confirms this reading
  // corresponds to an actual current lock, not just ongoing time output.
  bool justUpdated = _gps.location.isValid() && _gps.date.isValid() && _gps.time.isValid() &&
                      _gps.date.age() < 100 &&     // RMC-anchored precise timing
                      _gps.location.age() < 1500;  // confirms a genuinely current fix

  if (justUpdated) {
#if GPS_TRANSMIT_TIME_LOG_ENABLED
    Serial.printf("[GPS] transmitTimeMs=%u (chars=%u, elapsedSinceBurstStart=%lums)\n",
                  (uint16_t)(_charCountSinceBurst * (10.0f / (float)GPS_BAUD * 1000.0f)),
                  _charCountSinceBurst, millis() - _burstStartMillis);
#endif
    refreshSnapshot();
  }

  // Satellite count and HDOP are reported independently of the
  // location-fix gate above: GSV/GSA sentences keep carrying satellite
  // visibility data continuously, even while there's no valid position
  // fix (e.g. too few satellites in view for a 3D fix, or poor geometry).
  // Gating this behind the same "just got a fresh fix" condition as
  // above meant the reported satellite count would freeze at its last
  // good value indefinitely once lock was lost, instead of reflecting
  // the real-time drop that's actually happening.
  if (_gps.satellites.isValid() && _gps.satellites.age() < 2000) {
    _snapshot.satellites = _gps.satellites.value();
  } else {
    _snapshot.satellites = 0;
  }
  if (_gps.hdop.isValid() && _gps.hdop.age() < 2000) {
    _snapshot.hdopTenths = _gps.hdop.value();
  } else {
    _snapshot.hdopTenths = -1;
  }

  // Mark stale if too long since last good fix
  if (_snapshot.hasFix &&
      (millis() - _snapshot.lastFixMillis > GPS_STALE_TIMEOUT_MS)) {
    _snapshot.isStale = true;
  }
}

void GPSManager::refreshSnapshot() {
  _snapshot.hasFix   = true;
  _snapshot.isStale  = false;
  _snapshot.year     = _gps.date.year();
  _snapshot.month    = _gps.date.month();
  _snapshot.day      = _gps.date.day();
  _snapshot.hour     = _gps.time.hour();
  _snapshot.minute   = _gps.time.minute();
  _snapshot.second   = _gps.time.second();
  _snapshot.centisecond = _gps.time.centisecond();
  // satellites/hdopTenths are handled separately in update() now — see
  // comment there for why they're no longer set here.

  // Calculated serial transmission time for the burst leading up to this
  // fix (10 bits/char at GPS_BAUD, standard NMEA framing). This is the
  // real, per-fix transmission-time component of GPS latency — it's what
  // ClockEngine combines with the fixed/calibration portion
  // (settings.gpsTotalLatencyMs(), tuned against a reference clock) to
  // get the true total delay for this specific fix, rather than assuming
  // a constant transmission time every cycle regardless of how many
  // characters (e.g. GSV sentences, which grow with satellite count)
  // came before RMC in this burst.
  _snapshot.transmissionTimeMs = (uint16_t)(_charCountSinceBurst * (10.0f / (float)GPS_BAUD * 1000.0f));

  // Derive the actual commit instant from date.age() (see the RMC-anchoring
  // comment in update() for why date specifically, not location) rather
  // than stamping "now" — this keeps lastFixMillis accurate even though
  // update() may call refreshSnapshot() on more than one loop iteration
  // for what's really the same underlying fix.
  unsigned long commitMillis = millis() - _gps.date.age();
  _snapshot.lastFixMillis = commitMillis;
}

GPSSnapshot GPSManager::getSnapshot() const {
  return _snapshot;
}

bool GPSManager::isReliable() const {
  return _snapshot.hasFix && !_snapshot.isStale;
}
