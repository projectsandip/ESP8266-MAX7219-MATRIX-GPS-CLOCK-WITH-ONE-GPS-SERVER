#include "ClientRegistry.h"

ClientRegistry::ClientRegistry() {
  for (uint8_t i = 0; i < MAX_TRACKED_CLIENTS; i++) {
    _clients[i].inUse = false;
    _clients[i].syncCount = 0;
    _clients[i].lastSyncMillis = 0;
  }
}

void ClientRegistry::recordSync(IPAddress ip) {
  // Update an existing record for this IP if we have one.
  for (uint8_t i = 0; i < MAX_TRACKED_CLIENTS; i++) {
    if (_clients[i].inUse && _clients[i].ip == ip) {
      _clients[i].lastSyncMillis = millis();
      _clients[i].syncCount++;
      return;
    }
  }

  // Not seen before — use an empty slot if one's available.
  for (uint8_t i = 0; i < MAX_TRACKED_CLIENTS; i++) {
    if (!_clients[i].inUse) {
      _clients[i].inUse = true;
      _clients[i].ip = ip;
      _clients[i].lastSyncMillis = millis();
      _clients[i].syncCount = 1;
      return;
    }
  }

  // Table full — evict whichever entry has gone longest without syncing,
  // on the assumption it's the least likely to still be an active client.
  uint8_t oldestIndex = 0;
  unsigned long oldestTime = _clients[0].lastSyncMillis;
  for (uint8_t i = 1; i < MAX_TRACKED_CLIENTS; i++) {
    if (_clients[i].lastSyncMillis < oldestTime) {
      oldestTime = _clients[i].lastSyncMillis;
      oldestIndex = i;
    }
  }
  _clients[oldestIndex].ip = ip;
  _clients[oldestIndex].lastSyncMillis = millis();
  _clients[oldestIndex].syncCount = 1;
}

uint8_t ClientRegistry::count() const {
  uint8_t n = 0;
  for (uint8_t i = 0; i < MAX_TRACKED_CLIENTS; i++) {
    if (_clients[i].inUse) n++;
  }
  return n;
}

const ClientRecord& ClientRegistry::get(uint8_t index) const {
  return _clients[index];
}
