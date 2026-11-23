#include "StatusPage.h"
#include "config.h"

static const char* qualityColor(uint8_t q) {
  if (q >= 85) return "#2ecc71"; // green
  if (q >= 50) return "#f1c40f"; // yellow
  return "#e74c3c";              // red
}

// Converts a UTC DateTime to local display time using the same offset
// ClockEngine::getCurrentTimeLocal() applies, so the RTC's raw reading can
// be compared directly against the main (local) time shown at the top.
static DateTime toLocalDisplay(const DateTime &utc) {
  uint32_t unixLocal = utc.unixtime() + (uint32_t)TZ_OFFSET_MINUTES * 60UL;
  return DateTime(unixLocal);
}

static String formatDuration(unsigned long ms) {
  unsigned long totalSeconds = ms / 1000UL;
  unsigned long days    = totalSeconds / 86400UL;
  unsigned long hours   = (totalSeconds % 86400UL) / 3600UL;
  unsigned long minutes = (totalSeconds % 3600UL) / 60UL;
  unsigned long seconds = totalSeconds % 60UL;

  String out;
  if (days > 0)    { out += String(days) + "d "; }
  if (hours > 0 || days > 0) { out += String(hours) + "h "; }
  if (minutes > 0 || hours > 0 || days > 0) { out += String(minutes) + "m "; }
  out += String(seconds) + "s";
  return out;
}

String buildStatusPage(ClockEngine &clock, const CurrentTime &ct,
                        const GPSSnapshot &gpsSnap, RTCManager &rtc,
                        ClientRegistry &registry) {
  char timeBuf[16];
  snprintf(timeBuf, sizeof(timeBuf), "%02u:%02u:%02u.%03u",
           ct.hour, ct.minute, ct.second, ct.millisecond);

  char dateBuf[16];
  snprintf(dateBuf, sizeof(dateBuf), "%04u-%02u-%02u", ct.year, ct.month, ct.day);

  String html;
  html.reserve(3600);

  html += F("<!DOCTYPE html><html><head><meta charset='utf-8'>");
  html += F("<meta http-equiv='refresh' content='2'>");
  html += F("<title>GPS Master Server</title>");
  html += F("<style>"
            "body{font-family:monospace;background:#111;color:#eee;padding:20px;}"
            "h1{font-size:1.1em;color:#8ecae6;}"
            ".time{font-size:2.6em;margin:10px 0;}"
            ".row{margin:6px 0;}"
            ".label{color:#999;display:inline-block;width:150px;}"
            ".pill{display:inline-block;padding:2px 10px;border-radius:10px;color:#111;font-weight:bold;}"
            "table{width:100%;border-collapse:collapse;margin-top:8px;}"
            "th,td{text-align:left;padding:4px 10px;border-bottom:1px solid #333;font-size:0.9em;}"
            "th{color:#999;font-weight:normal;}"
            "</style></head><body>");

  html += F("<h1>GPS_Master_Server_v3</h1>");
  html += "<div class='time'>" + String(timeBuf) + "</div>";
  html += "<div class='row'><span class='label'>Date</span>" + String(dateBuf) + "</div>";
  html += "<div class='row'><span class='label'>Server on-time</span>" +
          formatDuration(millis()) + "</div>";

  html += "<div class='row'><span class='label'>Source</span>";
  html += String(clock.sourceName(ct.source));
  html += "</div>";

  html += "<div class='row'><span class='label'>Sync quality</span>";
  html += "<span class='pill' style='background:";
  html += qualityColor(ct.syncQuality);
  html += "'>" + String(ct.syncQuality) + "/100</span></div>";

  html += "<div class='row'><span class='label'>Since last sync</span>" +
          String(ct.secondsSinceSync) + " s</div>";

  html += "<div class='row'><span class='label'>Last correction</span>";
  {
    TimeSource correctionSrc = clock.lastCorrectionSource();
    if (correctionSrc == TimeSource::NONE) {
      html += "none yet";
    } else {
      int32_t deltaMs = clock.lastCorrectionDeltaMs();
      unsigned long ageMs = clock.lastCorrectionAgeMs();
      char deltaBuf[16];
      snprintf(deltaBuf, sizeof(deltaBuf), "%+ld ms", (long)deltaMs);
      html += String(deltaBuf) + " by " + String(clock.sourceName(correctionSrc)) +
              ", " + formatDuration(ageMs) + " ago";
    }
  }
  html += "</div>";

  html += F("<hr><h1>GPS</h1>");
  html += "<div class='row'><span class='label'>Fix</span>" +
          String(gpsSnap.hasFix ? (gpsSnap.isStale ? "STALE" : "VALID") : "NO FIX") + "</div>";
  html += "<div class='row'><span class='label'>Satellites</span>" +
          String(gpsSnap.satellites) + "</div>";
  html += "<div class='row'><span class='label'>HDOP</span>" +
          (gpsSnap.hdopTenths >= 0 ? String(gpsSnap.hdopTenths / 10.0, 1) : String("n/a")) +
          "</div>";

  html += F("<hr><h1>RTC</h1>");
  html += "<div class='row'><span class='label'>Present</span>" +
          String(rtc.isPresent() ? "yes" : "no") + "</div>";
  html += "<div class='row'><span class='label'>Lost power</span>" +
          String(rtc.lostPower() ? "yes" : "no") + "</div>";

  html += "<div class='row'><span class='label'>RTC time</span>";
  if (rtc.isPresent()) {
    DateTime rtcLocal = toLocalDisplay(rtc.nowUTC());
    char rtcBuf[24];
    snprintf(rtcBuf, sizeof(rtcBuf), "%04u-%02u-%02u %02u:%02u:%02u",
             rtcLocal.year(), rtcLocal.month(), rtcLocal.day(),
             rtcLocal.hour(), rtcLocal.minute(), rtcLocal.second());
    html += String(rtcBuf);
  } else {
    html += "n/a";
  }
  html += "</div>";

  unsigned long sinceSync = rtc.millisSinceLastSync();
  html += "<div class='row'><span class='label'>Last GPS sync</span>";
  if (sinceSync == (unsigned long)-1) {
    html += "never this boot";
  } else {
    html += String(sinceSync / 60000UL) + " min ago";
  }
  html += "</div>";

  html += F("<hr><h1>Connected Clients</h1>");
  {
    uint8_t clientCount = registry.count();
    if (clientCount == 0) {
      html += F("<div class='row'>No clients have synced yet.</div>");
    } else {
      html += F("<table><tr><th>IP</th><th>Last sync</th><th>Sync count</th><th></th></tr>");
      for (uint8_t i = 0; i < MAX_TRACKED_CLIENTS; i++) {
        const ClientRecord &rec = registry.get(i);
        if (!rec.inUse) continue;
        unsigned long agoMs = millis() - rec.lastSyncMillis;
        String ip = rec.ip.toString();
        html += "<tr><td>" + ip + "</td><td>" +
                formatDuration(agoMs) + " ago</td><td>" +
                String(rec.syncCount) + "</td><td>" +
                "<a href='http://" + ip + "/settings' target='_blank'>Settings</a>" +
                "</td></tr>";
      }
      html += F("</table>");
      html += F("<div class='row' style='color:#666;font-size:0.8em;margin-top:6px;'>"
                "Settings links assume the client exposes its own /settings page "
                "on the same hotspot &mdash; they'll 404 until that's implemented "
                "on the client.</div>");
    }
  }

  html += F("<hr><h1>Server Control</h1>"
            "<div class='row'><a href='/settings'>Clock timing settings</a></div>"
            "<div class='row'><a href='/update'>Firmware update (OTA)</a></div>"
            "<div class='row' style='color:#666;font-size:0.8em;'>"
            "Both require the admin login (see ADMIN_USERNAME/ADMIN_PASSWORD in config.h).</div>");

  html += F("<hr><small>Auto-refreshes every 2s &middot; JSON: "
            "<a style='color:#8ecae6' href='/time'>/time</a> &middot; "
            "<a style='color:#8ecae6' href='/status'>/status</a></small>");

  html += F("</body></html>");
  return html;
}
