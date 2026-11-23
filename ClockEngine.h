#ifndef CLOCK_ENGINE_H
#define CLOCK_ENGINE_H

#include <Arduino.h>
#include <RTClib.h>
#include "GPSManager.h"
#include "RTCManager.h"
#include "SettingsManager.h"

enum class TimeSource : uint8_t {
  NONE      = 0,
  GPS       = 1,
  RTC       = 2,
  FREEWHEEL = 3
};

// Everything a consumer (JSON API, display client, etc.) needs to render
// or reason about "what time is it right now".
struct CurrentTime {
  uint16_t year;
  uint8_t  month, day, hour, minute, second;
  uint16_t millisecond;     // 0-999, best-effort sub-second precision
  TimeSource source;
  uint8_t  syncQuality;     // 0-100, 100 = GPS-locked this instant
  uint32_t secondsSinceSync;// how long since the current anchor was set
  bool     gpsFixValid;
  uint8_t  gpsSatellites;
};

// ---------------------------------------------------------------------------
// ClockEngine runs a single continuously free-running internal clock (an
// anchor timestamp + millis() extrapolation) that is the one and only
// thing ever served to clients. GPS and RTC never become "the head"
// themselves — they only ever correct/discipline this internal clock:
//   - GPS corrects it only every settings.gpsCorrectionIntervalMs(), while
//     a valid lock is available (GPS is authoritative when present).
//     Between corrections the internal clock keeps ticking off millis()
//     on its own — GPS is not re-applied on every single fix.
//   - RTC corrects it only occasionally, on
//     settings.rtcCorrectionIntervalMs(), and only while GPS is
//     unavailable. Between corrections the internal clock just keeps
//     ticking off millis() on its own — RTC is not re-read every loop.
//   - RTC is also occasionally written (settings.rtcResyncIntervalMs()),
//     purely as a backup so it has something reasonably fresh to correct
//     the internal clock from later if GPS is lost. This write is sourced
//     from the internal clock itself (not raw GPS data) and only happens
//     while GPS lock is currently available — it never itself becomes the
//     live source.
// These three intervals (plus GPS latency compensation) are runtime-
// editable via SettingsManager (see /settings on the dashboard) rather
// than fixed at compile time.
// ---------------------------------------------------------------------------
class ClockEngine {
public:
  ClockEngine(GPSManager &gps, RTCManager &rtc, SettingsManager &settings);

  void begin();

  // Call every loop() iteration, after gps.update()
  void update();

  // Internal time is always UTC.
  CurrentTime getCurrentTime();

  // Convenience: same as getCurrentTime() but with TZ_OFFSET_MINUTES applied
  CurrentTime getCurrentTimeLocal();

  const char* sourceName(TimeSource s) const;

  // How much the internal clock's "now" prediction changed at the most
  // recent correction (signed ms — positive means the clock was nudged
  // forward, negative means it was nudged backward), which source did it,
  // and how long ago that was. lastCorrectionAgeMs() returns (unsigned
  // long)-1 if no correction has happened yet this boot.
  int32_t lastCorrectionDeltaMs() const { return _lastCorrectionDeltaMs; }
  TimeSource lastCorrectionSource() const { return _lastCorrectionSource; }
  unsigned long lastCorrectionAgeMs() const {
    if (_lastCorrectionSource == TimeSource::NONE) return (unsigned long)-1;
    return millis() - _lastCorrectionMillis;
  }

  // Same idea, but for every GPS correction *attempt* (accepted or
  // rejected) — useful for correlating outlier rejections with WiFi/HTTP
  // activity, since the accepted-only getters above miss rejected
  // attempts entirely. lastGpsAttemptAgeMs() returns (unsigned long)-1 if
  // no attempt has happened yet.
  int32_t lastGpsAttemptDeltaMs() const { return _lastGpsAttemptDeltaMs; }
  bool lastGpsAttemptWasRejected() const { return _lastGpsAttemptRejected; }
  unsigned long lastGpsAttemptAgeMs() const {
    if (_lastGpsAttemptMillis == 0) return (unsigned long)-1;
    return millis() - _lastGpsAttemptMillis;
  }

private:
  GPSManager &_gps;
  RTCManager &_rtc;
  SettingsManager &_settings;

  // Anchor: the last moment we trusted a time reading, plus the millis()
  // value at that instant. Freewheeling extrapolates from this anchor.
  DateTime      _anchorUTC;
  unsigned long _anchorMillis;
  uint16_t      _anchorSubMs;     // fractional ms captured at anchor time
  TimeSource    _anchorSource;

  unsigned long _lastGpsFixSeen;   // millis() of the GPS fix we last consumed
  unsigned long _lastGpsCorrection; // millis() of the last internal-clock<-GPS correction
  unsigned long _lastRtcResync;    // millis() of the last RTC<-internal-clock backup write
  unsigned long _lastRtcCorrection; // millis() of the last internal-clock<-RTC correction

  // An RTC backup write is scheduled for the next true second boundary
  // (derived from the internal clock's own live sub-second state at
  // scheduling time) rather than applied immediately — otherwise the
  // write would just reintroduce the same kind of quantization bias we're
  // trying to avoid by sourcing it from the internal clock at all.
  bool     _rtcWritePending;
  DateTime _pendingRtcDateTime;
  unsigned long _pendingWriteAtMillis;

  // Tracks the most recent correction applied to the internal clock (by
  // either GPS or RTC), so it can be reported on the dashboard/API — see
  // applyCorrection().
  int32_t       _lastCorrectionDeltaMs;
  TimeSource    _lastCorrectionSource;
  unsigned long _lastCorrectionMillis;

  // Consecutive GPS corrections rejected as outliers — see
  // GPS_CORRECTION_OUTLIER_MAX_REJECTS in config.h.
  uint8_t _consecutiveGpsOutlierRejects;

  // Tracks every GPS correction *attempt* (accepted or rejected) — used
  // both to enforce GPS_CORRECTION_RETRY_SPACING_MS between retries after
  // a rejection, and for diagnostics (see lastGpsAttempt* getters above).
  unsigned long _lastGpsAttemptMillis;
  int32_t       _lastGpsAttemptDeltaMs;
  bool          _lastGpsAttemptRejected;

  // RTC edge-catching state: the DS3231 has no sub-second register, so a
  // single read at an arbitrary point within the current second could be
  // off by up to ~1s from true time. Instead, we poll its seconds
  // register (cheap I2C reads, throttled to RTC_EDGE_POLL_INTERVAL_MS,
  // spread non-blockingly across update() calls rather than one long
  // blocking wait) until it's observed to actually increment — that
  // instant is very close to the true second boundary.
  bool          _rtcEdgeCatchInProgress;
  uint8_t       _rtcEdgeCatchLastSecond;
  unsigned long _rtcEdgeCatchStartMillis;
  unsigned long _lastRtcEdgePollMillis;

  // Returns true if the correction was actually applied, false if it was
  // rejected as an outlier (caller should NOT advance _lastGpsCorrection
  // in that case, so the next fix retries immediately rather than waiting
  // a full GPS_CORRECTION_INTERVAL_MS again).
  bool setAnchorFromGPS(const GPSSnapshot &snap);
  // Advances the RTC edge-catching state machine by one step (one cheap,
  // throttled poll). Returns true once a correction has actually been
  // applied (edge detected, or safety timeout reached) — call repeatedly
  // across successive update() calls until it returns true.
  bool tryCorrectFromRTCEdge();
  DateTime correctedGpsDateTime(const GPSSnapshot &snap) const;
  // Combines the calibrated fixed delay (settings.gpsTotalLatencyMs())
  // with the dynamically-measured per-fix transmission time (see
  // GPS_USE_DYNAMIC_TRANSMISSION_TIME in config.h) to get the true total
  // delay for a specific fix.
  uint32_t totalGpsDelayMs(const GPSSnapshot &snap) const;
  // Reads the internal clock's current instant (by extrapolating the
  // anchor forward via millis(), same math getCurrentTime() uses),
  // split into a whole-second DateTime and the 0-999ms remainder.
  void currentInternalDateTime(DateTime &outDate, uint16_t &outSubMs) const;
  // Computes what a correction to the given anchor WOULD change the
  // clock's "now" prediction by, without applying anything — used both to
  // decide whether a GPS correction looks like an outlier, and internally
  // by applyCorrection() itself.
  int32_t previewCorrectionDeltaMs(const DateTime &newAnchorUTC, unsigned long newAnchorMillis,
                                    uint16_t newAnchorSubMs) const;
  // Applies a new anchor, first recording how much this changes the
  // clock's "now" prediction compared to the outgoing anchor (both
  // evaluated at the same instant, so the comparison is apples-to-apples)
  // before committing the new anchor values.
  void applyCorrection(const DateTime &newAnchorUTC, unsigned long newAnchorMillis,
                        uint16_t newAnchorSubMs, TimeSource source);
  uint8_t computeSyncQuality(TimeSource src, unsigned long msSinceSync) const;
};

#endif // CLOCK_ENGINE_H
