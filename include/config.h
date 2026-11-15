#pragma once
// ============================================================
//  APRS Map Viewer - user settings (edit this file only)
// ============================================================

// ---- Wi-Fi ----
#define WIFI_SSID     "YOUR_SSID"
#define WIFI_PASSWORD "YOUR_PASSWORD"

// ---- APRS-IS ----
// Your amateur radio callsign (without SSID), e.g. "JA5XXX".
#define APRS_CALLSIGN "N0CALL"
// APRS-IS passcode for your callsign.
//   Set 0 to let the firmware compute it from the callsign automatically.
//   Set -1 to log in unverified (some servers then limit the feed).
#define APRS_PASSCODE 0
#define APRS_HOST     "rotate.aprs2.net"
#define APRS_PORT     14580

// ---- Receive filter: stations around this point ----
// Default: Takamatsu, Kagawa. The firmware asks APRS-IS for
// stations within FILTER_RANGE_KM of (FILTER_LAT, FILTER_LON).
// This point is also "home" for distance/bearing in the detail view.
#define FILTER_LAT      34.3401
#define FILTER_LON      134.0434
#define FILTER_RANGE_KM 100

// ---- Stations ----
// Forget a station after this many minutes without a new packet.
#define STATION_EXPIRE_MIN 90
#define MAX_STATIONS 96

// ---- Maps (BMP files on the SD card) ----
// Each map is a 24-bit uncompressed BMP on the SD root.
// Recommended size: 640x480 (the whole map is fitted to the screen
// first; zoom steps go up to x4 from there). 320x240 also works -
// the firmware reads the real size from the BMP header.
// The four bounds MUST match the geographic area the image shows:
//   NORTH/SOUTH = latitude of the top/bottom edge
//   WEST/EAST   = longitude of the left/right edge
// The values below are examples for the Takamatsu area - replace
// them with the bounds of YOUR images (see README).
struct MapDef {
  const char* file;
  const char* name;
  double north, south, west, east;
};

static const MapDef MAPS[3] = {
  // map1: all of Kagawa prefecture
  { "/map1.bmp", "MAP1", 34.60, 33.90, 133.40, 134.90 },
  // map2: Takamatsu / eastern Kagawa
  { "/map2.bmp", "MAP2", 34.50, 34.10, 133.80, 134.50 },
  // map3: close-up (Takamatsu city)
  { "/map3.bmp", "MAP3", 34.42, 34.26, 133.95, 134.15 },
};
