#ifndef STATUS_PAGE_H
#define STATUS_PAGE_H

#include <Arduino.h>
#include "ClockEngine.h"
#include "GPSManager.h"
#include "RTCManager.h"
#include "ClientRegistry.h"

// Builds a small self-refreshing HTML page showing current time,
// sync source/quality, GPS/RTC diagnostics, server uptime, and connected
// clients with their last sync time. Kept dependency-free (no external
// CSS/JS) so it works standalone on the device.
String buildStatusPage(ClockEngine &clock, const CurrentTime &ct,
                        const GPSSnapshot &gpsSnap, RTCManager &rtc,
                        ClientRegistry &registry);

#endif // STATUS_PAGE_H
