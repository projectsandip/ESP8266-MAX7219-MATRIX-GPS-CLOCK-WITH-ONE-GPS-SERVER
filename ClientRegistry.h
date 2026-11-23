#ifndef CLIENT_REGISTRY_H
#define CLIENT_REGISTRY_H

#include <Arduino.h>
#include <IPAddress.h>

// Small in-memory registry of clients that have polled the time endpoints,
// so the dashboard can show who's connected and how recently they synced.
// Deliberately simple/bounded — this is a display convenience, not a
// durable client-tracking system (nothing persists across a reboot).
#define MAX_TRACKED_CLIENTS 8

struct ClientRecord {
  IPAddress     ip;
  unsigned long lastSyncMillis;
  uint32_t      syncCount;
  bool          inUse;
};

class ClientRegistry {
public:
  ClientRegistry();

  // Call whenever a client hits a time endpoint. Updates the existing
  // record for this IP if present, otherwise creates one — evicting the
  // least-recently-seen entry if the table is full.
  void recordSync(IPAddress ip);

  uint8_t count() const;
  const ClientRecord& get(uint8_t index) const;

private:
  ClientRecord _clients[MAX_TRACKED_CLIENTS];
};

#endif // CLIENT_REGISTRY_H
