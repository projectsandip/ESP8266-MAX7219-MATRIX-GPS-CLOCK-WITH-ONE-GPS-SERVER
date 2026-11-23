#include "ClockEngine.h"
#include "config.h"

ClockEngine::ClockEngine(GPSManager &gps, RTCManager &rtc, SettingsManager &settings)
  : _gps(gps), _rtc(rtc), _settings(settings),
    // NOTE: do NOT use DateTime((uint32_t)0) here. RTClib's DateTime(uint32_t)
    // constructor treats its argument as a Unix timestamp and internally
    // subtracts SECONDS_FROM_1970_TO_2000 (946684800). Since t=0 is unsigned,
    // that subtraction underflows and wraps around to ~2106-02-07 — a classic
    // 32-bit time wraparound bug. Use an explicit safe placeholder instead;
    // this only appears briefly before the first real GPS/RTC sync anyway.
    _anchorUTC(2024, 1, 1, 0, 0, 0),
    _anchorMillis(0),
    _anchorSubMs(0),
    _anchorSource(TimeSource::NONE),
    _lastGpsFixSeen(0),
    _lastGpsCorrection(0),
    _lastRtcResync(0),
    _lastRtcCorrection(0),
    _rtcWritePending(false),
    _pendingRtcDateTime((uint32_t)946684800), // harmless placeholder, never read while pending==false
    _pendingWriteAtMillis(0),
    _lastCorrectionDeltaMs(0),
    _lastCorrectionSource(TimeSource::NONE),
    _lastCorrectionMillis(0),
    _consecutiveGpsOutlierRejects(0),
    _lastGpsAttemptMillis(0),
    _lastGpsAttemptDeltaMs(0),
    _lastGpsAttemptRejected(false),
    _rtcEdgeCatchInProgress(false),
    _rtcEdgeCatchLastSecond(0),
    _rtcEdgeCatchStartMillis(0),
    _lastRtcEdgePollMillis(0) {}

void ClockEngine::begin() {
  // No immediate/unaligned RTC read here on purpose — the first RTC
  // correction (edge-aligned, see tryCorrectFromRTCEdge()) happens
  // naturally on the very first update() call if GPS isn't available yet
  // (via the _lastRtcCorrection == 0 check below), typically resolving
  // within about a second of boot. Reading RTC immediately here without
  // edge-alignment would just reintroduce the quantization error this
  // whole mechanism exists to avoid.
}

void ClockEngine::update() {
  GPSSnapshot snap = _gps.getSnapshot();

  // GPSManager only ever sets hasFix=true after TinyGPSPlus reports a
  // genuinely valid location+date+time fix — never on raw/default data
  // before an actual lock. So this is already "GPS valid, and has been
  // locked at least once" by construction; no separate latch needed.
  bool gpsValid = snap.hasFix && !snap.isStale;
  bool freshGpsFix = gpsValid && (snap.lastFixMillis != _lastGpsFixSeen);

  if (freshGpsFix) {
    _lastGpsFixSeen = snap.lastFixMillis;

    // Correct the internal clock from GPS only every
    // the configured GPS correction interval, not on every single fix — between
    // corrections it just keeps ticking off millis() on its own.
    //
    // The retry-spacing check additionally guards against re-attempting
    // immediately after a rejection: without it, a rejected attempt could
    // get retried against the very next NMEA sentence (milliseconds
    // later, from the same 1-second burst) rather than a genuinely
    // independent GPS update.
    bool intervalElapsed = (millis() - _lastGpsCorrection > _settings.gpsCorrectionIntervalMs() || _lastGpsCorrection == 0);
    bool retrySpacingOk = (millis() - _lastGpsAttemptMillis > GPS_CORRECTION_RETRY_SPACING_MS || _lastGpsAttemptMillis == 0);

    if (intervalElapsed && retrySpacingOk) {
      bool applied = setAnchorFromGPS(snap);
      if (applied) {
        _lastGpsCorrection = millis();
      }
      // else: rejected as a suspected outlier (see setAnchorFromGPS) —
      // _lastGpsCorrection is deliberately left unchanged so the gate
      // stays open and we retry once GPS_CORRECTION_RETRY_SPACING_MS has
      // passed, rather than waiting a full correction interval
      // again.
    }

    // Back up to the RTC occasionally too (settings.rtcResyncIntervalMs()), purely
    // so RTC has something reasonably fresh to correct the internal clock
    // from later if GPS is lost. Sourced from the internal clock itself
    // (not raw GPS data) — the internal clock is already disciplined and
    // accurate at this point, so we just mirror it into the RTC. The
    // write is scheduled for the next true second boundary (derived from
    // the internal clock's own live sub-second state right now) rather
    // than applied immediately, so it doesn't reintroduce a quantization
    // bias on the way in.
    //
    // IMPORTANT: gated on !_rtcWritePending too — without this, a fresh
    // GPS fix arriving (~every 1s) before a previously-scheduled write's
    // target moment was reached would re-arm it with a new target time,
    // perpetually pushing the deadline forward and the write would never
    // actually fire. _lastRtcResync only updates once a write actually
    // lands, so the "_lastRtcResync == 0" branch stays true and keeps
    // re-triggering until this guard is added.
    if (_rtc.isPresent() && !_rtcWritePending &&
        (millis() - _lastRtcResync > _settings.rtcResyncIntervalMs() || _lastRtcResync == 0)) {
      DateTime internalDate;
      uint16_t internalSubMs;
      currentInternalDateTime(internalDate, internalSubMs);

      uint32_t msUntilNextSecond = 1000UL - internalSubMs;
      _pendingRtcDateTime = internalDate + TimeSpan(1);
      _pendingWriteAtMillis = millis() + msUntilNextSecond;
      _rtcWritePending = true;
    }
  }

  // Apply a scheduled RTC backup write once its target moment has
  // actually arrived — not before, and not "whenever convenient".
  if (_rtcWritePending && millis() >= _pendingWriteAtMillis) {
    _rtc.setUTC(_pendingRtcDateTime.year(), _pendingRtcDateTime.month(), _pendingRtcDateTime.day(),
                _pendingRtcDateTime.hour(), _pendingRtcDateTime.minute(), _pendingRtcDateTime.second());
    _lastRtcResync = millis();
    _rtcWritePending = false;
  }

  if (gpsValid) {
    // GPS remains available. The internal clock was corrected above if a
    // correction interval window had elapsed, or is just ticking
    // on its own via millis() extrapolation otherwise. Either way, we do
    // NOT touch the RTC as a correction source while GPS is available.
    return;
  }

  // GPS is unavailable (lost, or never locked yet this boot). Let the
  // internal clock keep free-running off millis() on its own, only
  // nudging it from the RTC occasionally rather than every loop — and
  // when it does, aligning precisely to the true second boundary rather
  // than reading at an arbitrary point within the current second.
  if (_rtc.isPresent() && !_rtc.lostPower() &&
      (millis() - _lastRtcCorrection > _settings.rtcCorrectionIntervalMs() || _lastRtcCorrection == 0)) {
    bool corrected = tryCorrectFromRTCEdge();
    if (corrected) {
      _lastRtcCorrection = millis();
    }
    // else: edge-catching still in progress — _lastRtcCorrection stays
    // unchanged so this keeps getting called (and polled, throttled to
    // RTC_EDGE_POLL_INTERVAL_MS) on subsequent update() calls until it
    // resolves, typically within ~0-1000ms.
  }
  // else: no correction due yet (or no RTC at all) — the internal clock
  // just continues ticking from its last anchor; getCurrentTime()
  // extrapolates and eventually reports FREEWHEEL once stale enough.
}

uint32_t ClockEngine::totalGpsDelayMs(const GPSSnapshot &snap) const {
#if GPS_USE_DYNAMIC_TRANSMISSION_TIME
  return _settings.gpsTotalLatencyMs() + snap.transmissionTimeMs;
#else
  return _settings.gpsTotalLatencyMs();
#endif
}

DateTime ClockEngine::correctedGpsDateTime(const GPSSnapshot &snap) const {
  DateTime raw(snap.year, snap.month, snap.day, snap.hour, snap.minute, snap.second);
  uint32_t wholeSeconds = totalGpsDelayMs(snap) / 1000UL;
  // TimeSpan handles month/day/year rollover correctly (e.g. correcting
  // 23:59:59 -> 00:00:00 the next day), unlike naive field arithmetic.
  return raw + TimeSpan(wholeSeconds);
}

void ClockEngine::currentInternalDateTime(DateTime &outDate, uint16_t &outSubMs) const {
  // Same extrapolation math as getCurrentTime(), but returning just the
  // date/time split rather than the full CurrentTime struct (which also
  // computes source labels, quality, GPS fields, etc. that callers of
  // this helper don't need).
  unsigned long elapsedMs = millis() - _anchorMillis;
  uint32_t totalMs = (uint32_t)_anchorSubMs + elapsedMs;
  uint32_t extraSeconds = totalMs / 1000;
  outSubMs = totalMs % 1000;
  outDate = _anchorUTC + TimeSpan(extraSeconds);
}

int32_t ClockEngine::previewCorrectionDeltaMs(const DateTime &newAnchorUTC, unsigned long newAnchorMillis,
                                               uint16_t newAnchorSubMs) const {
  if (_anchorSource == TimeSource::NONE) {
    // Nothing meaningful to diff against yet (still on the boot
    // placeholder) — a zero delta rather than a bogus multi-year jump.
    return 0;
  }

  unsigned long now = millis();

  // Evaluate both the outgoing and incoming anchors at the same instant
  // ("now") so the comparison is apples-to-apples, regardless of any
  // backdating either anchor applies internally.
  unsigned long oldElapsed = now - _anchorMillis;
  uint32_t oldTotalMs = (uint32_t)_anchorSubMs + oldElapsed;
  uint32_t oldUnixAtNow = _anchorUTC.unixtime() + oldTotalMs / 1000;
  uint16_t oldSubMsAtNow = oldTotalMs % 1000;

  unsigned long newElapsed = now - newAnchorMillis;
  uint32_t newTotalMs = (uint32_t)newAnchorSubMs + newElapsed;
  uint32_t newUnixAtNow = newAnchorUTC.unixtime() + newTotalMs / 1000;
  uint16_t newSubMsAtNow = newTotalMs % 1000;

  int64_t deltaMs = (int64_t)((int32_t)newUnixAtNow - (int32_t)oldUnixAtNow) * 1000LL
                    + ((int32_t)newSubMsAtNow - (int32_t)oldSubMsAtNow);
  return (int32_t)deltaMs;
}

void ClockEngine::applyCorrection(const DateTime &newAnchorUTC, unsigned long newAnchorMillis,
                                   uint16_t newAnchorSubMs, TimeSource source) {
  _lastCorrectionDeltaMs = previewCorrectionDeltaMs(newAnchorUTC, newAnchorMillis, newAnchorSubMs);
  _lastCorrectionSource = source;
  _lastCorrectionMillis = millis();

  _anchorUTC = newAnchorUTC;
  _anchorMillis = newAnchorMillis;
  _anchorSubMs = newAnchorSubMs;
  _anchorSource = source;
}

bool ClockEngine::setAnchorFromGPS(const GPSSnapshot &snap) {
  DateTime corrected = correctedGpsDateTime(snap);
  uint16_t correctedSubMs = snap.centisecond * 10;

  // Backdate by the residual sub-second part of the total GPS delay
  // (calibrated fixed portion + dynamically-measured transmission time
  // for this specific fix): the whole-seconds part is already baked into
  // `corrected` above, but "now" (millis()) is still that many ms past
  // the true instant the corrected value describes. Guarded against
  // underflow near boot.
  uint32_t residualMs = totalGpsDelayMs(snap) % 1000UL;
  unsigned long now = millis();
  unsigned long newAnchorMillis = (now > residualMs) ? (now - residualMs) : 0;

  int32_t previewDelta = previewCorrectionDeltaMs(corrected, newAnchorMillis, correctedSubMs);
  bool looksLikeOutlier = (_anchorSource != TimeSource::NONE) &&
                          (abs(previewDelta) > (int32_t)GPS_CORRECTION_OUTLIER_THRESHOLD_MS);

  // Record this attempt regardless of outcome — used both for the
  // diagnostic getters and (by the caller) to enforce retry spacing.
  _lastGpsAttemptMillis = millis();
  _lastGpsAttemptDeltaMs = previewDelta;
  _lastGpsAttemptRejected = looksLikeOutlier && (_consecutiveGpsOutlierRejects < GPS_CORRECTION_OUTLIER_MAX_REJECTS);

  if (looksLikeOutlier && _consecutiveGpsOutlierRejects < GPS_CORRECTION_OUTLIER_MAX_REJECTS) {
    _consecutiveGpsOutlierRejects++;
    Serial.printf("[Clock] GPS correction REJECTED as outlier: %+ld ms (reject %u/%u) — retrying next fix\n",
                  (long)previewDelta, _consecutiveGpsOutlierRejects, GPS_CORRECTION_OUTLIER_MAX_REJECTS);
    return false;
  }

  if (looksLikeOutlier) {
    Serial.printf("[Clock] GPS correction accepted despite looking like an outlier (%+ld ms) after %u consecutive rejections\n",
                  (long)previewDelta, _consecutiveGpsOutlierRejects);
  }
  _consecutiveGpsOutlierRejects = 0;

  applyCorrection(corrected, newAnchorMillis, correctedSubMs, TimeSource::GPS);
  return true;
}

bool ClockEngine::tryCorrectFromRTCEdge() {
  unsigned long now = millis();

  if (!_rtcEdgeCatchInProgress) {
    // Kick off: record the current second as our baseline, then wait to
    // see it change on subsequent calls.
    _rtcEdgeCatchLastSecond = _rtc.nowUTC().second();
    _rtcEdgeCatchStartMillis = now;
    _lastRtcEdgePollMillis = now;
    _rtcEdgeCatchInProgress = true;
    return false;
  }

  // Throttle actual I2C polling rather than reading every single loop()
  // iteration — still far more than precise enough at this interval,
  // with much less bus/CPU overhead.
  if (now - _lastRtcEdgePollMillis < RTC_EDGE_POLL_INTERVAL_MS) {
    return false;
  }
  _lastRtcEdgePollMillis = now;

  DateTime rtcNow = _rtc.nowUTC();
  bool edgeDetected = (rtcNow.second() != _rtcEdgeCatchLastSecond);
  bool timedOut = (now - _rtcEdgeCatchStartMillis > RTC_EDGE_CATCH_TIMEOUT_MS);

  if (edgeDetected || timedOut) {
    // Either we caught the true second boundary (the common case — this
    // instant is accurate to within one poll interval, not up to ~1s),
    // or we hit the safety timeout (shouldn't normally happen, since a
    // second always ticks over within 1000ms — apply what we have rather
    // than risk never correcting at all).
    applyCorrection(rtcNow, now, 0, TimeSource::RTC);
    _rtcEdgeCatchInProgress = false;
    return true;
  }

  return false; // keep waiting, try again next update() call
}

uint8_t ClockEngine::computeSyncQuality(TimeSource src, unsigned long msSinceSync) const {
  switch (src) {
    case TimeSource::GPS:
      return 100;

    case TimeSource::RTC: {
      // Degrade gradually: 90 right after a GPS-disciplined RTC read,
      // decaying down to a floor of 40 over 24 hours since the RTC
      // itself was last set from GPS.
      unsigned long sinceRtcSync = _rtc.millisSinceLastSync();
      if (sinceRtcSync == (unsigned long)-1) {
        return 30; // RTC has never been GPS-synced this boot
      }
      float hours = sinceRtcSync / 3600000.0f;
      float quality = 90.0f - (hours / 24.0f) * 50.0f;
      if (quality < 40) quality = 40;
      if (quality > 90) quality = 90;
      return (uint8_t)quality;
    }

    case TimeSource::FREEWHEEL: {
      // Quality falls off fast once we're extrapolating with no anchor
      // discipline at all — millis() drift adds up quickly.
      float minutes = msSinceSync / 60000.0f;
      float quality = 25.0f - minutes; // loses ~1 point per minute
      if (quality < 1) quality = 1;
      return (uint8_t)quality;
    }

    default:
      return 0;
  }
}

CurrentTime ClockEngine::getCurrentTime() {
  CurrentTime ct;
  memset(&ct, 0, sizeof(ct));

  unsigned long elapsedMs = millis() - _anchorMillis;
  uint32_t totalSubMs = _anchorSubMs + elapsedMs;
  uint32_t extraSeconds = totalSubMs / 1000;
  uint16_t subMs = totalSubMs % 1000;

  uint32_t newUnix = _anchorUTC.unixtime() + extraSeconds;
  DateTime dt(newUnix);

  ct.year        = dt.year();
  ct.month       = dt.month();
  ct.day         = dt.day();
  ct.hour        = dt.hour();
  ct.minute      = dt.minute();
  ct.second      = dt.second();
  ct.millisecond = subMs;

  GPSSnapshot snap = _gps.getSnapshot();
  bool gpsCurrentlyValid = snap.hasFix && !snap.isStale;

  // Report FREEWHEEL if we've drifted well past what either GPS or RTC
  // would consider "fresh", even though the underlying anchor source
  // is still recorded for quality-scoring purposes.
  //
  // For GPS specifically: the correction cadence is throttled
  // (settings.gpsCorrectionIntervalMs()), so "elapsed since last correction" no
  // longer means "GPS has gone stale" the way it used to when every fix
  // corrected the anchor. As long as GPS itself is still currently
  // locked, the internal clock remains trustworthy even between
  // corrections — only fall back to FREEWHEEL once GPS has actually been
  // unavailable for GPS_STALE_TIMEOUT_MS.
  TimeSource effectiveSource = _anchorSource;
  if (_anchorSource == TimeSource::GPS) {
    if (!gpsCurrentlyValid && elapsedMs > GPS_STALE_TIMEOUT_MS) {
      effectiveSource = TimeSource::FREEWHEEL;
    }
  } else if (_anchorSource == TimeSource::RTC && elapsedMs > RTC_STALE_TIMEOUT_MS) {
    effectiveSource = TimeSource::FREEWHEEL;
  } else if (_anchorSource == TimeSource::NONE) {
    effectiveSource = TimeSource::FREEWHEEL;
  }

  ct.source = effectiveSource;
  ct.syncQuality = computeSyncQuality(effectiveSource, elapsedMs);
  ct.secondsSinceSync = elapsedMs / 1000;

  ct.gpsFixValid = gpsCurrentlyValid;
  ct.gpsSatellites = snap.satellites;

  return ct;
}

CurrentTime ClockEngine::getCurrentTimeLocal() {
  CurrentTime ct = getCurrentTime();

  uint32_t unixUtc = DateTime(ct.year, ct.month, ct.day, ct.hour, ct.minute, ct.second).unixtime();
  uint32_t unixLocal = unixUtc + (uint32_t)TZ_OFFSET_MINUTES * 60UL;
  DateTime local(unixLocal);

  ct.year   = local.year();
  ct.month  = local.month();
  ct.day    = local.day();
  ct.hour   = local.hour();
  ct.minute = local.minute();
  ct.second = local.second();
  // millisecond, source, syncQuality, secondsSinceSync, gps* fields unchanged
  return ct;
}

const char* ClockEngine::sourceName(TimeSource s) const {
  switch (s) {
    case TimeSource::GPS:       return "GPS";
    case TimeSource::RTC:       return "RTC";
    case TimeSource::FREEWHEEL: return "FREEWHEEL";
    default:                    return "NONE";
  }
}
