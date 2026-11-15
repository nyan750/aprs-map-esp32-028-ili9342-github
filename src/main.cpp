// APRS Map Viewer for ESP32-2432S028 (standard CYD)
//  - Receives APRS stations from APRS-IS over Wi-Fi
//  - Plots them on BMP maps (map1/2/3) stored on the SD card
//  - Touch: switch map / tap a station to centre+zoom / tap again for details
//
// Display: LovyanGFX (ILI9341, backlight GPIO21) via LGFX_028.h
// Touch  : XPT2046 on its own bus (CLK25/MISO39/MOSI32/CS33/IRQ36)
// SD     : SCK18/MISO19/MOSI23/CS5 - shares the VSPI peripheral with touch,
//          so the VSPI pin routing is switched before each use.
//          Touch is polled directly (no PENIRQ gate - v4).
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <time.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <SPI.h>
#include <math.h>
#include <esp_heap_caps.h>
#include <esp_partition.h>
#include <esp_spi_flash.h>
#include <esp_task_wdt.h>
#include "pngle/pngle.h"   // vendored PNG decoder for map generation (MIT)
#include "lgfx/utility/lgfx_miniz.h"   // tinfl inflate, already linked via LovyanGFX
#include <esp_system.h>
#include <XPT2046_Touchscreen.h>
#include "LGFX_028.h"
#include "config.h"
#include "maps_data.h"   // embedded 4-bit grayscale maps (no SD card needed)

LGFX tft;
static SPIClass touchSPI(VSPI);
// NOTE: no TIRQ pin on purpose. The library's update() is gated by an
// ISR wake flag that a low-pressure read can clear, which makes polling
// die until the next interrupt edge. With tirq=255 the flag stays set
// and getPoint() always reads (same fix as the old ESP32-024 project).
static XPT2046_Touchscreen ts(33);       // CS=33

// The touch panel has the VSPI bus to itself (maps live in flash now).
static void touchRoute() { touchSPI.end(); touchSPI.begin(25, 39, 32, 33); }

// ---------------- settings (phone setup portal, Plane Radar style) ----
// config.h holds only the factory defaults; the live settings are kept
// in NVS and edited from a phone browser (captive portal on the
// "APRSMap-Setup" AP, or http://aprs-map.local / the device IP on LAN).
static String cfgSsid, cfgPass, cfgCall;
static String cfgAisKey;
static int cfgPasscode = 0;
static double homeLat = FILTER_LAT, homeLon = FILTER_LON;
static int cfgRangeKm = FILTER_RANGE_KM;
static WebServer cfgServer(80);
static DNSServer dnsServer;
static bool cfgServerStarted = false;
static bool portalApMode = false;
static uint32_t portalLastActivityMs = 0;   // last portal page access (v75)

static void loadSettings(bool skipNvs) {
  cfgSsid = WIFI_SSID;
  cfgPass = WIFI_PASSWORD;
  cfgCall = APRS_CALLSIGN;
  cfgPasscode = APRS_PASSCODE;
#ifdef AIS_API_KEY
  cfgAisKey = AIS_API_KEY;
#else
  cfgAisKey = "";
#endif
  homeLat = FILTER_LAT;
  homeLon = FILTER_LON;
  cfgRangeKm = FILTER_RANGE_KM;
  if (skipNvs) return;
  Preferences p;
  p.begin("aprsmap", true);
  cfgSsid = p.getString("ssid", cfgSsid);
  cfgPass = p.getString("pass", cfgPass);
  cfgCall = p.getString("call", cfgCall);
  cfgAisKey = p.getString("aiskey", cfgAisKey);
  cfgPasscode = p.getInt("passcode", cfgPasscode);
  homeLat = p.getDouble("lat", homeLat);
  homeLon = p.getDouble("lon", homeLon);
  cfgRangeKm = p.getInt("range", cfgRangeKm);
  p.end();
}

static String htmlEsc(const String& s) {
  String o;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '&') o += "&amp;";
    else if (c == '"') o += "&quot;";
    else if (c == '<') o += "&lt;";
    else o += c;
  }
  return o;
}

static void handleRoot() {
  portalLastActivityMs = millis();
  char latBuf[24], lonBuf[24];
  snprintf(latBuf, sizeof(latBuf), "%.4f", homeLat);
  snprintf(lonBuf, sizeof(lonBuf), "%.4f", homeLon);
  String h = "<!DOCTYPE html><html lang='ja'><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>APRS Map Setup</title><style>body{font-family:sans-serif;margin:16px}"
    "label{display:block;margin-top:12px}input{width:100%;padding:8px;font-size:16px;box-sizing:border-box}"
    "button{margin-top:16px;width:100%;padding:12px;font-size:16px}</style></head><body>"
    "<h2>APRS Map 設定</h2><form method='POST' action='/save'>";
  h += "<label>Wi-Fi SSID<input name='ssid' value='" + htmlEsc(cfgSsid) + "'></label>";
  h += "<label>Wi-Fi パスワード<input name='pass' type='password' value='" + htmlEsc(cfgPass) + "'></label>";
  h += "<label>コールサイン<input name='call' value='" + htmlEsc(cfgCall) + "'></label>";
  h += "<label>APRS パスコード (0=自動計算)<input name='passcode' type='number' value='" + String(cfgPasscode) + "'></label>";
  h += "<label>中心の緯度<input name='lat' value='" + String(latBuf) + "'></label>";
  h += "<label>中心の経度<input name='lon' value='" + String(lonBuf) + "'></label>";
  h += "<label>受信範囲 km<input name='range' type='number' value='" + String(cfgRangeKm) + "'></label>";
  h += "<label>AIS APIキー (aisstream.io)<input name='aiskey' value='" + htmlEsc(cfgAisKey) + "'></label>";
  h += "<button type='submit'>保存して再起動</button></form>"
       "<p>設定は本体に保存されます。config.h は初期値です。</p>"
       "<p><a href='/mapgen'>地図を作り直す</a>（中心の緯度経度に合わせて再生成します。数分かかります）</p></body></html>";
  cfgServer.send(200, "text/html", h);
}

static void handleSave() {
  portalLastActivityMs = millis();
  String ssid = cfgServer.arg("ssid");
  if (ssid.length() > 0) {
    Preferences p;
    p.begin("aprsmap", false);
    p.putString("ssid", ssid);
    p.putString("pass", cfgServer.arg("pass"));
    String call = cfgServer.arg("call");
    if (call.length() > 0) p.putString("call", call);
    p.putInt("passcode", cfgServer.arg("passcode").toInt());
    p.putString("aiskey", cfgServer.arg("aiskey"));
    double la = cfgServer.arg("lat").toDouble();
    double lo = cfgServer.arg("lon").toDouble();
    if (la >= -90 && la <= 90 && lo >= -180 && lo <= 180 && (la != 0 || lo != 0)) {
      p.putDouble("lat", la);
      p.putDouble("lon", lo);
    }
    int rg = cfgServer.arg("range").toInt();
    if (rg >= 10 && rg <= 500) p.putInt("range", rg);
    p.end();
  }
  cfgServer.send(200, "text/html",
    "<!DOCTYPE html><html lang='ja'><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<body style='font-family:sans-serif;margin:16px'><h2>保存しました</h2>"
    "<p>再起動します。そのままお待ちください。</p></body></html>");
  delay(1000);
  ESP.restart();
}

static void handleMapGen() {
  portalLastActivityMs = millis();
  Preferences p;
  p.begin("aprsmap", false);
  p.putBool("mgforce", true);
  p.end();
  cfgServer.send(200, "text/html",
    "<!DOCTYPE html><html lang='ja'><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<body style='font-family:sans-serif;margin:16px'><h2>地図を再生成します</h2>"
    "<p>再起動して、中心の緯度経度に合わせて地図を作り直します（数分かかります）。</p></body></html>");
  delay(1000);
  ESP.restart();
}

static void handleNotFound() {
  portalLastActivityMs = millis();
  String loc = portalApMode ? "http://192.168.4.1/" : "/";
  cfgServer.sendHeader("Location", loc);
  cfgServer.send(302, "text/plain", "");
}

static void startConfigServer() {
  if (cfgServerStarted) return;
  cfgServerStarted = true;
  cfgServer.on("/", HTTP_GET, handleRoot);
  cfgServer.on("/save", HTTP_POST, handleSave);
  cfgServer.on("/mapgen", HTTP_GET, handleMapGen);
  cfgServer.onNotFound(handleNotFound);
  cfgServer.begin();
}

static void drawPortalScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setFont(&fonts::Font2);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(10, 12);
  tft.print("SETUP MODE");
  tft.setFont(&fonts::Font0);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 48);  tft.print("1. Connect phone Wi-Fi to:");
  tft.setCursor(22, 64);  tft.print("APRSMap-Setup");
  tft.setCursor(10, 84);  tft.print("2. Open in browser:");
  tft.setCursor(22, 100); tft.print("http://192.168.4.1");
  tft.setCursor(10, 120); tft.print("(setup page may open by itself)");
  tft.setCursor(10, 140); tft.print("3. Enter Wi-Fi / callsign, save.");
}

// Captive-portal mode: serve the setup page on our own AP until the
// user saves (which reboots), or the saved Wi-Fi comes back. The
// saved-Wi-Fi retry runs at most every 5 minutes and only after the
// portal has seen no access for 5 minutes, so it can never kick the
// user out mid-typing (v75; was every 45 s regardless of activity).
static void runPortal() {
  portalApMode = true;
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("APRSMap-Setup");
  dnsServer.start(53, "*", WiFi.softAPIP());
  startConfigServer();
  drawPortalScreen();
  uint32_t lastRetry = millis();
  portalLastActivityMs = millis();
  while (true) {
    dnsServer.processNextRequest();
    cfgServer.handleClient();
    bool haveCreds = cfgSsid.length() > 0 && cfgSsid != "YOUR_SSID";
    if (haveCreds && millis() - lastRetry > 300000 &&
        millis() - portalLastActivityMs > 300000) {
      lastRetry = millis();
      WiFi.begin(cfgSsid.c_str(), cfgPass.c_str());
      uint32_t t0 = millis();
      while (millis() - t0 < 10000 && WiFi.status() != WL_CONNECTED) {
        dnsServer.processNextRequest();
        cfgServer.handleClient();
        delay(20);
      }
      if (WiFi.status() == WL_CONNECTED) {
        dnsServer.stop();
        WiFi.softAPdisconnect(true);
        WiFi.mode(WIFI_STA);
        portalApMode = false;
        MDNS.begin("aprs-map");
        return;
      }
    }
    delay(2);
  }
}

// Connect with the saved settings; fall back to the setup portal.
static void wifiPhase(bool forcePortal) {
  bool haveCreds = cfgSsid.length() > 0 && cfgSsid != "YOUR_SSID";
  if (!forcePortal && haveCreds) {
    tft.setFont(&fonts::Font0);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(10, 60);
    tft.print("WiFi connecting...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(cfgSsid.c_str(), cfgPass.c_str());
    uint32_t t0 = millis();
    while (millis() - t0 < 12000 && WiFi.status() != WL_CONNECTED) delay(100);
    if (WiFi.status() == WL_CONNECTED) {
      startConfigServer();
      MDNS.begin("aprs-map");
      return;
    }
    WiFi.disconnect();
  }
  runPortal();
  startConfigServer();
}

// BOOT (GPIO0) held AT power-on puts the ESP32 into download mode and
// the firmware never runs - so the reset gesture is: after the splash
// appears, hold BOOT for 2 s (within the first seconds) to wipe the
// saved settings and force the setup portal.
static bool bootResetWindow() {
  pinMode(0, INPUT_PULLUP);
  tft.setFont(&fonts::Font0);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 40);
  tft.print("Hold BOOT 2s: reset settings");
  uint32_t winStart = millis();
  uint32_t lowStart = 0;
  while (true) {
    if (digitalRead(0) == LOW) {
      if (!lowStart) lowStart = millis();
      if (millis() - lowStart >= 2000) {
        Preferences p;
        p.begin("aprsmap", false);
        p.clear();
        p.end();
        tft.setCursor(10, 56);
        tft.print("Settings cleared!");
        delay(800);
        return true;
      }
    } else {
      lowStart = 0;
      if (millis() - winStart > 2200) break;   // no press: boot on
    }
    if (millis() - winStart > 8000) break;
    delay(20);
  }
  return false;
}

// ---------------- aircraft (ADS-B via adsb.lol, FlightRadar-style) ----
#define MAX_AC 32
struct Aircraft {
  char flight[12] = "";
  double lat = 0, lon = 0;
  float track = 0;         // deg
  long altFt = -1;         // -1 = on ground / unknown
  int gsKt = -1;
};
static Aircraft aircraft[MAX_AC];
static int acCount = 0;
static char selAcFlight[12] = "";   // aircraft shown in the detail panel
static bool detailIsAc = false;

// Route (airline / origin / destination) per callsign, from adsbdb.
struct RouteInfo {
  bool used = false;
  char flight[12] = "";
  char airline[40] = "";
  char orig[48] = "";   // "HND Tokyo Haneda International Airport"
  char dest[48] = "";
  bool ok = false;
};
static RouteInfo routeCache[12];
static int routeNext = 0;
static uint32_t lastAcFetchMs = 0;
static uint16_t* mapChunk = nullptr;   // 60-row map render chunk (38,400 B, v76)
#define MAP_CHUNK_ROWS 60
static bool acFetchedOnce = false;
#define AC_POLL_MS 60000

static bool jsonStrField(const String& s, const char* key, int from, int to, char* out, size_t outSz) {
  int k = s.indexOf(key, from);
  if (k < 0 || k >= to) return false;
  int v = k + strlen(key);
  int e = s.indexOf('"', v);
  if (e < 0 || e > to) e = to;
  size_t n = (size_t)(e - v); if (n >= outSz) n = outSz - 1;
  memcpy(out, s.c_str() + v, n); out[n] = 0;
  return true;
}
static bool jsonNumField(const String& s, const char* key, int from, int to, double& v) {
  int k = s.indexOf(key, from);
  if (k < 0 || k >= to) return false;
  v = strtod(s.c_str() + k + strlen(key), nullptr);
  return true;
}

// Pull the aircraft within AC_DIST_NM of the filter centre from the
// community ADS-B feed and replace the table (each reply is complete).
static void fetchAircraft() {
  if (WiFi.status() != WL_CONNECTED) return;
  // The map render chunk (38,400 B) squeezes the heap that TLS and the
  // response String need; hand it back (drawMap re-allocates on demand).
  if (mapChunk) { free(mapChunk); mapChunk = nullptr; }
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(12000);
  int distNm = (int)(cfgRangeKm / 1.852);
  if (distNm < 10) distNm = 10;
  if (distNm > 250) distNm = 250;
  char url[128];
  snprintf(url, sizeof(url), "https://api.adsb.lol/v2/lat/%.4f/lon/%.4f/dist/%d",
           homeLat, homeLon, distNm);
  if (!http.begin(client, url)) return;
  int code = http.GET();
  if (code != 200) {
    Serial.printf("ADS-B http %d (largest heap %u)\n", code,
                  (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    http.end(); return;
  }
  String body = http.getString();
  http.end();
  int acPos = body.indexOf("\"ac\":");
  if (acPos < 0) { Serial.printf("ADS-B bad body, len %u\n", (unsigned)body.length()); return; }
  int n = 0;
  int pos = acPos;
  while (n < MAX_AC) {
    int h = body.indexOf("\"hex\":\"", pos);
    if (h < 0) break;
    int next = body.indexOf("\"hex\":\"", h + 8);
    int segEnd = next >= 0 ? next : (int)body.length();
    double la, lo, tr, gs;
    char fl[16] = "";
    if (jsonNumField(body, "\"lat\":", h, segEnd, la) &&
        jsonNumField(body, "\"lon\":", h, segEnd, lo)) {
      Aircraft& a = aircraft[n];
      jsonStrField(body, "\"flight\":\"", h, segEnd, fl, sizeof(fl));
      // trim the padding spaces the feed puts in callsigns
      int L = strlen(fl);
      while (L > 0 && fl[L - 1] == ' ') fl[--L] = 0;
      strncpy(a.flight, fl[0] ? fl : "----", sizeof(a.flight) - 1);
      a.flight[sizeof(a.flight) - 1] = 0;
      a.lat = la; a.lon = lo;
      a.track = jsonNumField(body, "\"track\":", h, segEnd, tr) ? (float)tr : 0;
      a.gsKt = jsonNumField(body, "\"gs\":", h, segEnd, gs) ? (int)gs : -1;
      a.altFt = -1;
      int ak = body.indexOf("\"alt_baro\":", h);
      if (ak >= 0 && ak < segEnd) {
        const char* p = body.c_str() + ak + 11;
        if (*p != '"') a.altFt = strtol(p, nullptr, 10);   // "ground" stays -1
      }
      n++;
    }
    if (next < 0) break;
    pos = next;
  }
  acCount = n;
  Serial.printf("aircraft: %d\n", acCount);
}

static RouteInfo* findRoute(const char* fl) {
  for (int i = 0; i < 12; i++)
    if (routeCache[i].used && !strcmp(routeCache[i].flight, fl)) return &routeCache[i];
  return nullptr;
}

// Fetch the route for one callsign (once, then cached). Blocking ~1 s.
static void ensureRoute(const char* fl) {
  if (findRoute(fl)) return;
  RouteInfo& r = routeCache[routeNext];
  routeNext = (routeNext + 1) % 12;
  memset(&r, 0, sizeof(r));
  r.used = true;
  strncpy(r.flight, fl, sizeof(r.flight) - 1);
  if (WiFi.status() != WL_CONNECTED) return;
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(10000);
  char url[96];
  snprintf(url, sizeof(url), "https://api.adsbdb.com/v0/callsign/%s", fl);
  if (!http.begin(client, url)) return;
  if (http.GET() != 200) { http.end(); return; }
  String body = http.getString();
  http.end();
  int fr = body.indexOf("\"flightroute\"");
  if (fr < 0) return;
  int ap = body.indexOf("\"airline\":{", fr);
  if (ap >= 0) jsonStrField(body, "\"name\":\"", ap, ap + 300, r.airline, sizeof(r.airline));
  char iata[8] = "", nm[44] = "";
  int op = body.indexOf("\"origin\":{", fr);
  int oe = body.indexOf("\"destination\":{", fr);
  if (op >= 0 && oe > op && jsonStrField(body, "\"iata_code\":\"", op, oe, iata, sizeof(iata)) &&
      jsonStrField(body, "\"name\":\"", op, oe, nm, sizeof(nm)))
    snprintf(r.orig, sizeof(r.orig), "%s %s", iata, nm);
  if (oe >= 0 && jsonStrField(body, "\"iata_code\":\"", oe, (int)body.length(), iata, sizeof(iata)) &&
      jsonStrField(body, "\"name\":\"", oe, (int)body.length(), nm, sizeof(nm)))
    snprintf(r.dest, sizeof(r.dest), "%s %s", iata, nm);
  r.ok = r.orig[0] && r.dest[0];
}

// ---------------- ships (AIS via aisstream.io) ----------------
// Live vessel positions streamed over a WebSocket (see the AIS engine
// further down). ShipStaticData (name / destination / type) arrives
// only occasionally, so those fields fill in whenever they turn up.
#define MAX_SHIPS 48
struct Ship {
  bool used = false;
  bool hasPos = false;
  uint32_t mmsi = 0;
  char name[24] = "";
  char dest[24] = "";
  double lat = 0, lon = 0;
  float sog = 0;          // knots
  float cog = 361;        // deg, 361 = not available
  int navStatus = -1;
  int type = -1;
  uint32_t lastMs = 0;
};
static Ship ships[MAX_SHIPS];
static uint32_t selShipMmsi = 0;   // ship centred / shown in the detail panel
static bool aisOnline = false;

static void trimSpaces(char* s) {
  int L = strlen(s);
  while (L > 0 && s[L - 1] == ' ') s[--L] = 0;
  int b = 0;
  while (s[b] == ' ') b++;
  if (b) memmove(s, s + b, strlen(s + b) + 1);
}

static Ship* findShip(uint32_t mmsi, bool create) {
  Ship* freeSlot = nullptr;
  Ship* oldest = nullptr;
  uint32_t oldestMs = 0xFFFFFFFF;
  for (int i = 0; i < MAX_SHIPS; i++) {
    if (ships[i].used && ships[i].mmsi == mmsi) return &ships[i];
    if (!ships[i].used && !freeSlot) freeSlot = &ships[i];
    if (ships[i].used && ships[i].lastMs < oldestMs) { oldestMs = ships[i].lastMs; oldest = &ships[i]; }
  }
  if (!create) return nullptr;
  Ship* s = freeSlot ? freeSlot : oldest;
  if (s) { memset(s, 0, sizeof(Ship)); s->cog = 361; s->navStatus = -1; s->type = -1; }
  return s;
}

static int countShips() {
  int n = 0;
  for (int i = 0; i < MAX_SHIPS; i++) if (ships[i].used && ships[i].hasPos) n++;
  return n;
}

static const char* shipTypeText(int t) {
  if (t >= 70 && t <= 79) return "CARGO";
  if (t >= 80 && t <= 89) return "TANKER";
  if (t >= 60 && t <= 69) return "PASSENGER";
  if (t >= 40 && t <= 49) return "HIGH-SPEED CRAFT";
  if (t == 30) return "FISHING";
  if (t == 36) return "SAILING";
  if (t == 37) return "PLEASURE BOAT";
  if (t == 52) return "TUG";
  if (t == 50) return "PILOT BOAT";
  if (t == 51) return "RESCUE";
  if (t == 35) return "MILITARY";
  if (t == 33) return "DREDGING";
  if (t == 55) return "LAW ENFORCEMENT";
  if (t == 31 || t == 32) return "TOWING";
  if (t >= 20 && t <= 29) return "WIG";
  return "";
}

// ---------------- stations ----------------
struct Station {
  bool used = false;
  char call[20] = "";
  double lat = 0, lon = 0;
  char symTable = '/', symCode = ' ';
  int  course = -1;        // deg
  int  speedKt = -1;
  bool hasAlt = false;
  long altFt = 0;
  char comment[80] = "";
  char via[36] = "";
  uint32_t lastMs = 0;
  uint32_t lastUnix = 0;   // real (NTP) time of last packet, 0 = unknown
  bool mobile = false;     // reported speed or actually moved (display policy)
};
static Station stations[MAX_STATIONS];
static volatile bool stationsDirty = true;
static uint32_t packetCount = 0;

// ---------------- view ----------------
static int    curMap = 0;
static double centerLat = homeLat, centerLon = homeLon;
static int    zoomLevel = 0;               // 0 = whole BMP fits the screen, +1 per x2 step (scale capped at x4)
static int    selected = -1;               // index into stations[]
static bool   detailOpen = false;
// Three screens (user request): APRS stations, Flight (aircraft) and
// AIS (ships), switched with the tabs in the top bar. Each screen
// keeps its own centre/zoom. AIS shares the Flight screen's map.
#define MODE_COUNT 3
static const int MODE_MAP[MODE_COUNT] = {0, 1, 1};   // APRS, Flight, AIS
static int    screenMode = 0;              // 0 = APRS, 1 = Flight, 2 = AIS
static double modeCenterLat[MODE_COUNT] = {0, 0, 0}, modeCenterLon[MODE_COUNT] = {0, 0, 0};
static int    modeZoom[MODE_COUNT] = {0, 0, 0};
static bool   listOpen = false;
static int    listPage = 0;
static bool   sdOk = false;

// ---- current view (filled by computeView(), defined after bmpInfo) ----
// The BMP is drawn at vScale (screen px per BMP px). The visible part of
// the BMP starts at source pixel (vRectX0, vRectY0), is vSrcW x vSrcH
// source px in size, and lands on screen at offset (vXOff, vYOff) -
// non-zero when the whole map fits with margins (e.g. 640x320 on 320x240).
static double vScale = 1, vRectX0 = 0, vRectY0 = 0, vSrcW = 320, vSrcH = 240;
static int    vXOff = 0, vYOff = 0;
static double vRectWest = 0, vRectNorth = 0;   // geo at the visible rect's top-left
static double vPxPerLon = 1, vPxPerLat = 1;    // screen px per degree
static void   computeView();
static double viewLatSpan();
static double viewLonSpan();
static bool   geoToPx(double lat, double lon, int& px, int& py);

// ---------------- BMP ----------------
// The map list lives here (not in config.h) since v19, so map changes
// never require editing the user's config.h again. Bounds must match
// the BMP files on the SD card exactly.
#undef MAPS
#define MAP_COUNT 2
static MapDef MAP_LIST[MAP_COUNT] = {   // non-const: bounds are replaced when generated maps are active (v79)
  { "/map1.bmp", "KAGAWA",  34.60, 33.90, 133.40, 134.90 },  // APRS screen: Kagawa only (z11 source)
  { "/map2.bmp", "OKAYAMA", 35.40, 33.90, 133.25, 134.95 },  // Flight screen: Okayama + Kagawa
};

// ---------------- on-device map generation (v80: synced with the M5 build v8.21) --------
// At boot, if no generated maps exist for the configured centre (or a
// regeneration was requested from the portal), the device downloads
// GSI (Geospatial Information Authority of Japan) blank + standard
// tiles for the area around home, classifies coastlines against the
// standard tiles' sea mask, downscales to the usual 640x480 4-bit
// gray + class-bit format, and stores the result in the dedicated
// "mapdata" flash partition. The renderer then reads the generated
// maps through a flash mmap instead of the baked-in Takamatsu maps,
// which remain the fallback. Map bounds mirror make_maps.py's layout
// relative to the centre. PNG decoding: pngle (c) kikuchan, MIT.
// Prototype simplifications vs the PC pipeline: no place-name text
// removal, no sea-component size filter, box min/fraction pooling
// instead of LANCZOS, no coastline thinning.
#define GEN_MAGIC 0x4D473501u            // 'MG5' v1
#define GEN_HDR_BYTES 0x1000
#define GEN_MAP_BYTES (153600 + 38400)   // gray + class per map
#define GEN_SCR_OFF  0x100000            // pass-1 aggregate scratch
#define GEN_SCR_GRAY GEN_SCR_OFF
#define GEN_SCR_SEA  (GEN_SCR_OFF + 307200)
#define GEN_TOTAL_END (GEN_SCR_SEA + 307200)
#define GEN_STAGE_OFF GEN_TOTAL_END   // v6: raw-tile stage up to partition end

struct GenHeader {
  uint32_t magic, ver;
  double homeLat, homeLon;
  double b[2][4];                        // N, S, W, E per map
  uint32_t sum[2];                       // byte sum of gray+class
  uint32_t reserved[8];
};

static const esp_partition_t* genPart = nullptr;
static const uint8_t* genMapPtr = nullptr;   // mmap base of genPart
static spi_flash_mmap_handle_t genMmapH = 0;
static bool genMapsActive = false;
static const uint8_t* mapGrayPtr[2]  = { MAP_GRAY[0],  MAP_GRAY[1] };
static const uint8_t* mapClassPtr[2] = { MAP_CLASS[0], MAP_CLASS[1] };

static double mgGx(double lon, int z) { return (lon + 180.0) / 360.0 * 256.0 * (1 << z); }
static double mgGy(double lat, int z) {
  double r = lat * M_PI / 180.0;
  return (1.0 - log(tan(r) + 1.0 / cos(r)) / M_PI) / 2.0 * 256.0 * (1 << z);
}

static void mgShow(const char* l1, const char* l2, int pct) {
  tft.fillScreen(TFT_BLACK);
  tft.setFont(&fonts::Font2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(8, 16);  tft.print("MAP GENERATION");
  tft.setFont(&fonts::Font0);
  tft.setCursor(8, 44);  tft.print("GSI tiles -> onboard maps");
  tft.setCursor(8, 64);  tft.print(l1);
  tft.setCursor(8, 84);  tft.print(l2 ? l2 : "");
  tft.drawRect(8, 108, 304, 14, TFT_WHITE);
  if (pct > 0) tft.fillRect(9, 109, 302 * (pct > 100 ? 100 : pct) / 100, 12, TFT_GREEN);
  tft.setCursor(8, 134); tft.printf("%d %%", pct);
  tft.setCursor(8, 170); tft.print("Do not power off.");
}

// ---- pass 1: tile streaming into per-output-row aggregates ----
static int mgMode = 0;                     // 0 = std tile (sea bits), 1 = blank (accumulate)
static int mgTileOx = 0, mgTileOy = 0;     // global px of tile top-left
// Generation-only scratch lives in ONE heap block (allocated in
// mgOneMap, freed on exit) - as statics it overflowed DRAM BSS.
static uint8_t* mgWork = nullptr;
static const char* mgFail = "";    // last generation failure step
static pngle_t* mgPng = nullptr;   // persistent decoder (allocated in setup)
static bool mgAllocPng();          // v6: defined near mapGenBoot
static int mgRangeKm();            // v8.13: defined near mgActivate
static bool mgAllocWorkBuffers();
static void mgFreeWorkBuffers();   // v7.1 forward declaration
// v5.17: aggregates are two separate 10 KB allocations (mgAggGray/Sea)
static int mgLastHttp = 0;         // last non-200 tile http code
static int mgTilesOk = 0;          // tiles fetched OK in this generation
static uint8_t* mgSeaBits = nullptr;       // [256*32] sea flags of the current tile
static uint16_t* mgColW = nullptr;         // [640]
static uint8_t* mgRowH = nullptr;          // [480]
static uint8_t* mgScrGrayRow = nullptr;    // [640]
static uint8_t* mgScrSeaRow = nullptr;     // [640]
static uint8_t* mgGrayCur = nullptr;       // [640] pass 2 row being emitted
static uint8_t* mgRingInk = nullptr;       // [13*80] packed bits
static uint8_t* mgRingSea = nullptr;       // [13*80] packed bits
static int mgRingRow[13];
static int mgCropX0 = 0, mgCropY0 = 0, mgCropW = 0, mgCropH = 0;
static int mgBandOy0 = 0, mgBandRows = 0;
static int mgStripX0 = 0;   // v6: output column where the current strip starts
static uint8_t* mgAggGray = nullptr;       // [bandRows][640] min luminance
static uint8_t* mgAggSea = nullptr;        // [bandRows][640] sea pixel count
static uint32_t mgWdtCtr = 0;
static long mgPixels = 0;          // v6.3: pixels drawn by the current decode
static char mgFailDetail[80] = ""; // v6.3: extra line for the FAILED screen
RTC_DATA_ATTR static uint32_t mgCkpt = 0;   // v8.6: survives warm resets:
// (map+1)<<24 | strip<<8 | phase (1=fetch, 2=decode, 3=classify)
static int mgStripNo = 0;

static void mgAccum(int gx, int gy, int r, int g, int b, int lx, int ly) {
  int cx = gx - mgCropX0, cy = gy - mgCropY0;
  if (cx < 0 || cy < 0 || cx >= mgCropW || cy >= mgCropH) return;
  int oy = (int)((double)cy * 480.0 / mgCropH);
  int ox = (int)((double)cx * 640.0 / mgCropW);
  int lo = oy - mgBandOy0;
  if (lo < 0 || lo >= mgBandRows) return;
  int lum = (r * 77 + g * 150 + b * 29) >> 8;
  int sx = ox - mgStripX0;   // v6: aggregates cover one 160-col strip
  if (sx < 0 || sx >= 160) return;
  uint8_t* gm = mgAggGray + lo * 160 + sx;
  if (lum < *gm) *gm = (uint8_t)lum;
  if (mgSeaBits[ly * 32 + (lx >> 3)] & (0x80 >> (lx & 7))) {
    uint8_t* sc = mgAggSea + lo * 160 + sx;
    if (*sc < 255) (*sc)++;
  }
}

static void mgDrawCb(pngle_t* png, uint32_t x, uint32_t y, uint32_t w, uint32_t h,
                     const uint8_t rgba[4]) {
  (void)png;
  mgPixels += (long)w * (long)h;   // v6.3 diagnostics
  for (uint32_t dy = 0; dy < h; dy++)
    for (uint32_t dx = 0; dx < w; dx++) {
      int lx = (int)x + (int)dx, ly = (int)y + (int)dy;
      if (lx > 255 || ly > 255) continue;
      if (mgMode == 0) {
        int bb = rgba[2], rr = rgba[0];
        if (bb - rr > 25 && bb > 120)
          mgSeaBits[ly * 32 + (lx >> 3)] |= (uint8_t)(0x80 >> (lx & 7));
      } else if (mgMode == 1) {
        mgAccum(mgTileOx + lx, mgTileOy + ly, rgba[0], rgba[1], rgba[2], lx, ly);
      }   // mode 2 = warm-up fetch: decode but discard every pixel
    }
  if ((++mgWdtCtr & 0x3FF) == 0) esp_task_wdt_reset();
}

static WiFiClientSecure mgTls;     // persistent: per-tile TLS handshakes
static HTTPClient mgHttp;          // exhausted the heap (http -1)
static bool mgTlsInit = false;

static bool __attribute__((noinline)) mgFetchTile(int z, int tx, int ty, const char* layer, int mode) {
  char url[96];
  snprintf(url, sizeof(url), "https://cyberjapandata.gsi.go.jp/xyz/%s/%d/%d/%d.png",
           layer, z, tx, ty);
  if (!mgTlsInit) { mgTls.setInsecure(); mgTlsInit = true; }
  for (int attempt = 0; attempt < 3; attempt++) {
    mgHttp.setReuse(true);
    mgHttp.setTimeout(15000);
    if (!mgHttp.begin(mgTls, url)) { mgLastHttp = -101; return false; }
    mgHttp.addHeader("User-Agent", "MuseAPRSMap/1.0 (personal APRS viewer project)");
    int code = mgHttp.GET();
    if (code != 200) {
      mgLastHttp = code;
      Serial.printf("MAPGEN tile %s z%d %d/%d http %d\n", layer, z, tx, ty, code);
      mgHttp.end();
      mgTls.stop();   // force a fresh handshake on the retry
      delay(400);
      continue;
    }
    mgMode = mode;
    mgTileOx = tx * 256;
    mgTileOy = ty * 256;
    if (mode == 0) memset(mgSeaBits, 0, 8192);
    if (!mgPng) { mgLastHttp = -100; mgHttp.end(); return false; }
    pngle_t* png = mgPng;   // persistent instance (see setup)
    pngle_reset(png);
    WiFiClient* stream = mgHttp.getStreamPtr();
    int total = mgHttp.getSize();   // Content-Length (-1 if unknown)
    int fed = 0;
    uint8_t buf[1024];
    bool ok = true;
    uint32_t t0 = millis();
    while (total < 0 ? mgHttp.connected() : fed < total) {
      size_t avail = stream->available();
      if (avail) {
        if (avail > sizeof(buf)) avail = sizeof(buf);
        int n = stream->readBytes(buf, avail);
        if (n > 0) {
          fed += n;
          if (pngle_feed(png, buf, n) < 0) { ok = false; break; }
        }
      } else {
        delay(1);
      }
      if (millis() - t0 > 30000) { ok = false; break; }
      esp_task_wdt_reset();
    }
    mgHttp.end();
    if (ok) { mgTilesOk++; return true; }
    mgTls.stop();
    delay(400);
  }
  snprintf(mgFailDetail, sizeof(mgFailDetail),
           "z%d tx%d ty%d %s code %d heap %u", z, tx, ty, layer,
           mgLastHttp, (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
  return false;
}

// ---- v6 staging: fetch raw PNGs to flash, decode them later ----
// The streaming design kept the TLS session, the 44 KB PNG decoder
// and the aggregation buffers resident at the same time, and the
// heap could not serve all three (v5.13-v5.19). v6 separates the
// phases per 160-column strip: Phase A fetches tiles into the stage
// area with only the decoder resident; Phase B then decodes from
// flash - no network allocations happen while decoding.
static uint8_t mgTmpBuf[2048];   // v6.1: stage/decode/merge scratch.
// These phases never nest, so one static buffer serves all three -
// as stack arrays they overflowed the 8 KB loopTask stack (on-device
// panic: "stack overflow in task loopTask", first decode of v6).
static uint32_t mgStageUsed = 0;          // padded bytes in the stage
static uint32_t mgStagePrevUsed = 0;      // v6.6: bytes to re-erase pre-strip
static uint8_t mgStageCarry[4];
static int mgStageCarryN = 0;
static uint32_t mgRecOff[10], mgRecLen[10];
static int mgRecTx[10], mgRecCount = 0;
static int mgRecTy[10]; static uint8_t mgRecLayer[10];  // v6.4 diagnostics
static int mgDumpRec = -1;   // v6.4: record to dump on serial after a failure

static bool mgStageWriteRaw(const uint8_t* data, int n) {
  while (n > 0) {
    int k = 4 - mgStageCarryN; if (k > n) k = n;
    memcpy(mgStageCarry + mgStageCarryN, data, k);
    mgStageCarryN += k; data += k; n -= k;
    if (mgStageCarryN == 4) {
      if (esp_partition_write(genPart, GEN_STAGE_OFF + mgStageUsed,
                              mgStageCarry, 4) != ESP_OK)
        return false;
      mgStageUsed += 4;
      mgStageCarryN = 0;
    }
  }
  return true;
}
static bool mgStageEndRecord() {   // flush the tail, 0-padded to 4
  if (mgStageCarryN == 0) return true;
  while (mgStageCarryN < 4) mgStageCarry[mgStageCarryN++] = 0;
  if (esp_partition_write(genPart, GEN_STAGE_OFF + mgStageUsed,
                          mgStageCarry, 4) != ESP_OK)
    return false;
  mgStageUsed += 4;
  mgStageCarryN = 0;
  return true;
}

// Fetch one tile into the stage (record mgRecCount). Same TLS
// plumbing as mgFetchTile, but the body goes to flash untouched.
static bool mgStageTile(int z, int tx, int ty, const char* layer) {
  char url[96];
  snprintf(url, sizeof(url), "https://cyberjapandata.gsi.go.jp/xyz/%s/%d/%d/%d.png",
           layer, z, tx, ty);
  if (!mgTlsInit) { mgTls.setInsecure(); mgTlsInit = true; }
  for (int attempt = 0; attempt < 3; attempt++) {
    mgHttp.setReuse(true);
    mgHttp.setTimeout(15000);
    if (!mgHttp.begin(mgTls, url)) { mgLastHttp = -101; return false; }
    mgHttp.addHeader("User-Agent", "MuseAPRSMap/1.0 (personal APRS viewer project)");
    int code = mgHttp.GET();
    if (code != 200) {
      mgLastHttp = code;
      mgHttp.end();
      mgTls.stop();
      delay(400);
      continue;
    }
    uint32_t recOff = mgStageUsed;
    int recLen = 0;
    WiFiClient* stream = mgHttp.getStreamPtr();
    int total = mgHttp.getSize();
    uint8_t* buf = mgTmpBuf;   // 1024 of the shared scratch (v6.1)
    bool ok = true;
    uint32_t t0 = millis();
    while (total < 0 ? mgHttp.connected() : recLen < total) {
      size_t avail = stream->available();
      if (avail) {
        if (avail > 1024) avail = 1024;
        int n = stream->readBytes(buf, avail);
        if (n > 0) {
          recLen += n;
          if (!mgStageWriteRaw(buf, n)) { ok = false; break; }
        }
      } else {
        delay(1);
      }
      if (millis() - t0 > 30000) { ok = false; break; }
      esp_task_wdt_reset();
    }
    mgHttp.end();   // v8.5: per-tile session again - the v8.4 reuse
    // path is suspected in the mid-fetch reboot; the reserve block
    // (not reuse) is the fragmentation guard.
    if (ok && mgStageEndRecord()) {
      if (mgRecCount < 10) {
        mgRecOff[mgRecCount] = recOff;
        mgRecLen[mgRecCount] = (uint32_t)recLen;
        mgRecTx[mgRecCount] = tx;
        mgRecTy[mgRecCount] = 0;      // filled in by the strip loop (v6.4)
        mgRecLayer[mgRecCount] = (layer[0] == 'b') ? 1 : 0;
        mgRecCount++;
      }
      mgTilesOk++;
      return true;
    }
    mgStageUsed = recOff;   // roll the partial record back
    mgStageCarryN = 0;
    mgHttp.end();
    mgTls.stop();
    delay(400);
  }
  return false;
}

// Decode one staged tile through pngle (mode 0 = std/sea, 1 = blank).
static bool mgDecodeStaged(int rec, int ty, int mode) {
  mgMode = mode;
  mgTileOx = mgRecTx[rec] * 256;
  mgTileOy = ty * 256;
  if (mode == 0) memset(mgSeaBits, 0, 8192);
  if (!genMapPtr) { mgFail = "stage mmap"; return false; }
  pngle_reset(mgPng);
  mgPixels = 0;
  // v6.2: feed the tile in ONE pngle_feed call, straight from the
  // flash mmap (chunked feeding trips a pngle boundary bug).
  int r = pngle_feed(mgPng, genMapPtr + GEN_STAGE_OFF + mgRecOff[rec],
                     mgRecLen[rec]);
  esp_task_wdt_reset();
  if (r >= 0) return true;
  // v6.3 diagnostics: capture what the staged bytes look like via
  // both paths, then retry the decode from esp_partition_read
  // chunks - the retry verdict separates "bad staged bytes / ROM
  // inflate" (retry fails too) from "mmap feed path" (retry works).
  uint8_t mm4[4] = {0}, pr4[4] = {0};
  memcpy(mm4, genMapPtr + GEN_STAGE_OFF + mgRecOff[rec], 4);
  esp_partition_read(genPart, GEN_STAGE_OFF + mgRecOff[rec], pr4, 4);
  long px1 = mgPixels;
  pngle_reset(mgPng);
  mgPixels = 0;
  bool retryOk = true;
  {
    // BSS is too tight for a static 8 KB chunk buffer; borrow heap.
    uint8_t* rbuf = (uint8_t*)heap_caps_malloc(8192, MALLOC_CAP_DMA);
    uint32_t ch = rbuf ? 8192 : 2048;
    if (!rbuf) rbuf = mgTmpBuf;
    uint32_t off = GEN_STAGE_OFF + mgRecOff[rec];
    uint32_t left = mgRecLen[rec];
    while (left > 0) {
      uint32_t n = left > ch ? ch : left;
      if (esp_partition_read(genPart, off, rbuf, n) != ESP_OK) { retryOk = false; break; }
      if (pngle_feed(mgPng, rbuf, n) < 0) { retryOk = false; break; }
      off += n; left -= n;
      esp_task_wdt_reset();
    }
    if (rbuf != mgTmpBuf) free(rbuf);
  }
  snprintf(mgFailDetail, sizeof(mgFailDetail),
           "rec%d %s %d/%d f%d px%ld len%u %02x%02x%02x%02x retry %s",
           rec, mgRecLayer[rec] ? "blank" : "std",
           mgRecTx[rec], mgRecTy[rec], r, px1, (unsigned)mgRecLen[rec],
           mm4[0], mm4[1], mm4[2], mm4[3], retryOk ? "OK" : "NG");
  return retryOk;
}

// First decode phase of a map: drop the TLS session first (its
// buffers go back to the heap), then allocate the work buffers and
// set up the per-map column/row weight tables.
static bool mgPrepWorkBuffers() {
  if (!mgWork) {
    // v6.5: the TLS connection now stays UP across Phase B. Killing
    // and re-establishing it per strip deterministically corrupted
    // the first tile fetched after each reconnect (staged bytes had
    // a genuine header, a wrong IDAT length and un-inflatable data,
    // identical CRC on every run). The strip-sized work buffers are
    // small enough to coexist with the live session.
    if (!mgAllocWorkBuffers()) { mgFail = "heap (gen buffers)"; return false; }
  }
  mgColW = (uint16_t*)(mgWork + 0);         // 1280
  mgRowH = mgWork + 1280;                   // 480
  mgScrGrayRow = mgWork + 1760;             // 640
  mgScrSeaRow = mgWork + 2400;              // 640
  mgGrayCur = mgWork + 3040;                // 640
  mgRingInk = mgWork + 3680;                // 1040
  mgRingSea = mgWork + 4720;                // 1040 -> ends 5760
  memset(mgColW, 0, 640 * 2);
  memset(mgRowH, 0, 480);
  for (int cx = 0; cx < mgCropW; cx++) mgColW[(int)(cx * 640.0 / mgCropW)]++;
  for (int cy = 0; cy < mgCropH; cy++) mgRowH[(int)(cy * 480.0 / mgCropH)]++;
  return true;
}

// ---- pass 2 helpers: classified emit from the scratch aggregates ----
static void mgLoadRow(int r) {
  int s = r % 13;
  if (mgRingRow[s] == r) return;
  esp_partition_read(genPart, GEN_SCR_GRAY + r * 640, mgScrGrayRow, 640);
  esp_partition_read(genPart, GEN_SCR_SEA + r * 640, mgScrSeaRow, 640);
  for (int xb = 0; xb < 80; xb++) { mgRingInk[s * 80 + xb] = 0; mgRingSea[s * 80 + xb] = 0; }
  for (int x = 0; x < 640; x++) {
    if (mgScrGrayRow[x] < 245)
      mgRingInk[s * 80 + (x >> 3)] |= (uint8_t)(0x80 >> (x & 7));
    int den = mgColW[x] * mgRowH[r];
    if (den > 0 && (int)mgScrSeaRow[x] * 100 >= 35 * den)
      mgRingSea[s * 80 + (x >> 3)] |= (uint8_t)(0x80 >> (x & 7));
  }
  mgRingRow[s] = r;
}
static inline bool mgRingBit(const uint8_t* ring, int r, int x) {
  if (r < 0 || r >= 480 || x < 0 || x >= 640) return false;
  mgLoadRow(r);
  return (ring[(r % 13) * 80 + (x >> 3)] >> (7 - (x & 7))) & 1;
}
static bool mgCoastAt(int r, int x) {
  if (!mgRingBit(mgRingInk, r, x)) return false;
  for (int dy = -4; dy <= 4; dy++)
    for (int dx = -4; dx <= 4; dx++)
      if (mgRingBit(mgRingSea, r + dy, x + dx)) return true;
  return false;
}

// v6.5: walk the staged records' PNG chunk structure. The poisoned
// fetch produced records whose chunk lengths do not add up to the
// record length, so this detects them before decoding.
static bool mgStripRecordsValid() {
  if (!genMapPtr) return false;
  for (int rec = 0; rec < mgRecCount; rec++) {
    const uint8_t* p = genMapPtr + GEN_STAGE_OFF + mgRecOff[rec];
    uint32_t len = mgRecLen[rec];
    if (len < 8 || memcmp(p, "\x89PNG\r\n\x1a\n", 8) != 0) return false;
    uint32_t off = 8;
    bool sawIend = false;
    while (off + 8 <= len) {
      uint32_t clen = ((uint32_t)p[off] << 24) | ((uint32_t)p[off + 1] << 16) |
                      ((uint32_t)p[off + 2] << 8) | (uint32_t)p[off + 3];
      if (clen > len) return false;
      if (memcmp(p + off + 4, "IEND", 4) == 0) { sawIend = true; off += 12 + clen; break; }
      off += 12 + clen;
    }
    if (!sawIend || off != len) return false;
  }
  return true;
}

static bool __attribute__((noinline)) mgOneMap(int mi, const double b[4], int z, uint32_t dataOff, uint32_t* sumOut) {
  mgStripNo = 0;   // v8.6
  double x0 = mgGx(b[2], z), x1 = mgGx(b[3], z);
  double y0 = mgGy(b[0], z), y1 = mgGy(b[1], z);
  mgCropW = (int)lround(x1 - x0);
  mgCropH = (int)lround(y1 - y0);
  mgCropX0 = (int)floor(x0);
  mgCropY0 = (int)floor(y0);
  if (mgCropW < 64 || mgCropH < 64) { mgFail = "crop size"; return false; }
  int tx0 = mgCropX0 >> 8, tx1 = (mgCropX0 + mgCropW - 1) >> 8;
  int ty0 = mgCropY0 >> 8, ty1 = (mgCropY0 + mgCropH - 1) >> 8;
  // (the work buffers, pointer carve and weight tables are set up by
  // mgPrepWorkBuffers() at the first decode phase - v6)
  // v6.6: NOR flash can only clear bits (1->0) when programming;
  // writing over a programmed cell yields old AND new. Every reused
  // region must be ERASED before rewriting - the old zero-write of
  // the sea scratch would have ANDed every later sea value to 0.
  // Both scratches are erased here, once per map, and pass 1 writes
  // each scratch row exactly once (RAM row staging below).
  for (uint32_t off = 0; off < 614400; off += 65536) {
    uint32_t n = 614400 - off > 65536 ? 65536 : 614400 - off;
    esp_partition_erase_range(genPart, GEN_SCR_OFF + off, n);
    esp_task_wdt_reset();
  }
  char l2[48];
  for (int ty = ty0; ty <= ty1; ty++) {
    int cyB0 = ty * 256 - mgCropY0; if (cyB0 < 0) cyB0 = 0;
    int cyB1 = ty * 256 + 255 - mgCropY0; if (cyB1 > mgCropH - 1) cyB1 = mgCropH - 1;
    if (cyB1 < cyB0) continue;
    int bandOy0 = (int)(cyB0 * 480.0 / mgCropH);
    int oyB = (int)(cyB1 * 480.0 / mgCropH);
    int bandRowsFull = oyB - bandOy0 + 1;
    for (int sb = 0; sb < bandRowsFull; sb += 32) {   // v8.21: back to the v8.7 width
      mgBandOy0 = bandOy0 + sb;
      mgBandRows = bandRowsFull - sb; if (mgBandRows > 32) mgBandRows = 32;
      // (v6.9: no RAM row accumulators - the merge writes flash)
      // v6: process the sub-band in 160-column strips. Phase A stages
      // the strip's raw tiles in flash; Phase B decodes them. The two
      // phases never hold their big allocations at the same time.
      for (int sx0 = 0; sx0 < 640; sx0 += 160) {
        mgStripX0 = sx0;
        mgStripNo++; mgCkpt = ((uint32_t)(mi + 1) << 24) | ((uint32_t)(mgStripNo & 0xFFFF) << 8) | 1;   // v8.6
        int inX0 = mgCropX0 + (int)(sx0 * (double)mgCropW / 640);
        int inX1 = mgCropX0 + (int)((sx0 + 160) * (double)mgCropW / 640);
        int txA = inX0 >> 8; if (txA < tx0) txA = tx0;
        int txB = (inX1 - 1) >> 8; if (txB > tx1) txB = tx1;
        // ---- Phase A: fetch this strip's tiles into the stage ----
        // v8.3: up to 3 attempts per strip. v8.2 died at 99% on one
        // late transient fetch failure (http -1, tiles 308); a strip
        // now restages after a pause instead of aborting the run.
        bool ok = false;
        mgFailDetail[0] = 0;   // v8.5: detail belongs to this strip
        for (int attempt = 1; attempt <= 3 && !ok; attempt++) {
          if (attempt > 1) {
            mgHttp.end(); mgTls.stop();
            mgShow("Fetch failed - retrying", "",
                   (mgBandOy0 + mgBandRows) * 85 / 480);
            delay(4000 * attempt);
            esp_task_wdt_reset();
            mgStagePrevUsed = mgStageUsed;   // erase partial stage
          }
          // v6.6: erase what the previous strip/attempt left in the
          // stage - programming over programmed cells ANDs old and
          // new bytes (the deterministic "tile decode" corruption).
          if (mgStagePrevUsed) {
            uint32_t eb = (mgStagePrevUsed + 4095) & ~4095u;
            for (uint32_t o = 0; o < eb; o += 65536) {
              uint32_t n = eb - o > 65536 ? 65536 : eb - o;
              esp_partition_erase_range(genPart, GEN_STAGE_OFF + o, n);
              esp_task_wdt_reset();
            }
            mgStagePrevUsed = 0;
          }
          mgStageUsed = 0; mgStageCarryN = 0; mgRecCount = 0;
          ok = true;
          for (int tx = txA; tx <= txB && ok; tx++) {
            if (mgStageUsed > (genPart->size - GEN_STAGE_OFF) - 131072) {
              mgFail = "stage full"; return false;
            }
            ok = mgStageTile(z, tx, ty, "std") && mgStageTile(z, tx, ty, "blank");
          }
          // v6.5: if a staged record's chunk structure is broken,
          // hard-reset the connection and refetch the strip once.
          if (ok && !mgStripRecordsValid()) {
            mgHttp.end();
            mgTls.stop();
            delay(300);
            {   // v6.6: erase the invalid records before restaging
              uint32_t eb = (mgStageUsed + 4095) & ~4095u;
              for (uint32_t o = 0; o < eb; o += 65536) {
                uint32_t n = eb - o > 65536 ? 65536 : eb - o;
                esp_partition_erase_range(genPart, GEN_STAGE_OFF + o, n);
                esp_task_wdt_reset();
              }
            }
            mgStageUsed = 0; mgStageCarryN = 0; mgRecCount = 0;
            for (int tx = txA; tx <= txB && ok; tx++) {
              ok = mgStageTile(z, tx, ty, "std") && mgStageTile(z, tx, ty, "blank");
            }
            if (ok) for (int r2 = 0; r2 < mgRecCount; r2++) mgRecTy[r2] = ty;
            if (ok && !mgStripRecordsValid()) {   // v8.3: failed attempt
              ok = false;
              if (!mgFailDetail[0])
                snprintf(mgFailDetail, sizeof(mgFailDetail),
                         "z%d ty%d sx%d bad record heap %u", z, ty, sx0,
                         (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
            }
          }
        }
        snprintf(l2, sizeof(l2), "map %d/2 rows %d/480 heap %u", mi + 1, mgBandOy0 + mgBandRows,
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
        mgShow(mi == 0 ? "APRS map (narrow area)" : "FLIGHT/AIS map (wide area)",
               l2, (mgBandOy0 + mgBandRows) * 85 / 480);
        if (!ok) { mgFail = "tile fetch"; return false; }
        for (int r2 = 0; r2 < mgRecCount; r2++) mgRecTy[r2] = ty;   // v6.4
        mgCkpt = ((uint32_t)(mi + 1) << 24) | ((uint32_t)(mgStripNo & 0xFFFF) << 8) | 2;   // v8.6
        // ---- Phase B: decode the staged tiles into the aggregates ----
        // v8.7: try the work set with the TLS session still up;
        // only if it does not fit, drop the session and retry
        // the allocation once (kept as a fallback).
        if (!mgPrepWorkBuffers()) {
          mgHttp.end(); mgTls.stop();
          if (!mgPrepWorkBuffers()) return false;
        }
        memset(mgAggGray, 0xFF, 160 * mgBandRows);
        memset(mgAggSea, 0, 160 * mgBandRows);
        for (int rec = 0; rec < mgRecCount; rec++) {
          if (!mgDecodeStaged(rec, ty, (rec & 1) ? 1 : 0)) {
            mgFail = "tile decode"; mgDumpRec = rec; return false;
          }
        }
        // v6.9: merge this strip's columns into the flash scratch.
        // AND-safety: strips own disjoint column ranges, so each
        // scratch cell is programmed with its final value exactly
        // once per map; other strips rewrite the bytes they read
        // unchanged (AND(v,v) = v). The sea baseline is the erased
        // 0xFF read as 0 - a zero-WRITE init would AND every sea
        // count to 0, which is why v6.6's RAM accumulators existed;
        // they are gone again (their 20 KB broke the heap budget).
        for (int lo = 0; lo < mgBandRows; lo++) {
          int oy = mgBandOy0 + lo;
          uint8_t* rowG = mgTmpBuf;          // shared scratch,
          uint8_t* rowS = mgTmpBuf + 640;    // not stack arrays
          esp_partition_read(genPart, GEN_SCR_GRAY + oy * 640, rowG, 640);
          esp_partition_read(genPart, GEN_SCR_SEA + oy * 640, rowS, 640);
          for (int x = sx0; x < sx0 + 160; x++) {
            int ai = lo * 160 + (x - sx0);
            if (mgAggGray[ai] < rowG[x]) rowG[x] = mgAggGray[ai];
            int base = rowS[x] == 0xFF ? 0 : rowS[x];
            int sv = base + mgAggSea[ai];
            rowS[x] = (uint8_t)(sv > 255 ? 255 : sv);
          }
          esp_partition_write(genPart, GEN_SCR_GRAY + oy * 640, rowG, 640);
          esp_partition_write(genPart, GEN_SCR_SEA + oy * 640, rowS, 640);
        }
        mgStagePrevUsed = mgStageUsed;   // v6.6: re-erase next strip
        mgFreeWorkBuffers();   // v7.1: Phase A gets a clean heap
        esp_task_wdt_reset();
      }
    }
  }

  // ---- pass 2: classify + pack + store ----
  mgCkpt = ((uint32_t)(mi + 1) << 24) | 3;   // v8.6
  // v8.1: v7.1+ frees the work buffers at each strip end; pass 2
  // uses mgWork's carve (mgGrayCur, rings), so re-acquire it here.
  if (!mgPrepWorkBuffers()) return false;
  for (int i = 0; i < 13; i++) mgRingRow[i] = -1;
  uint32_t sum = 0;
  for (int r = 0; r < 480; r++) {
    esp_partition_read(genPart, GEN_SCR_GRAY + r * 640, mgGrayCur, 640);
    uint8_t grayRow[320], classRow[80];
    memset(classRow, 0, sizeof(classRow));
    for (int x = 0; x < 640; x++) {
      bool ink = mgRingBit(mgRingInk, r, x);
      bool coast = ink && mgCoastAt(r, x);
      bool grown = false;
      if (!ink) {
        for (int dy = -1; dy <= 1 && !grown; dy++)
          for (int dx = -1; dx <= 1 && !grown; dx++)
            if ((dx || dy) && mgCoastAt(r + dy, x + dx)) grown = true;
      }
      if (coast || grown)
        classRow[x >> 3] |= (uint8_t)(0x80 >> (x & 7));
      int g = mgGrayCur[x];
      if (grown) g = 110;
      int gd = 255 - (255 - g) * 16 / 10;
      if (gd < 0) gd = 0;
      int nib = gd >> 4;
      if (x & 1) grayRow[x >> 1] |= (uint8_t)nib;
      else grayRow[x >> 1] = (uint8_t)(nib << 4);
    }
    esp_partition_write(genPart, dataOff + r * 320, grayRow, 320);
    esp_partition_write(genPart, dataOff + 153600 + r * 80, classRow, 80);
    for (int i = 0; i < 320; i++) sum += grayRow[i];
    for (int i = 0; i < 80; i++) sum += classRow[i];
    if ((r & 31) == 0) {
      mgShow("Classifying coastlines...", "", 85 + r * 15 / 480);
      esp_task_wdt_reset();
    }
  }
  *sumOut = sum;
  mgFreeWorkBuffers();   // v8.6: was kept allocated into the next
  // map, whose fetches then faced a resident 34 KB work set
  return true;
}

static bool __attribute__((noinline)) mgGenerate() {
  mgStagePrevUsed = 0;   // v6.6: full erase below covers the first strip
  // Bounds relative to home, mirroring the Takamatsu layout of
  // make_maps.py (map1 z11 narrow, map2 z10 wide).
  static const double OFF[2][4] = {
    { +0.2599, -0.4401, -0.9401, +0.8566 },   // APRS:   N S W E
    { +1.0599, -0.4401, -0.7934, +0.9066 },   // FLIGHT: N S W E
  };
  // v8.18: back to the proven fixed layout (the v8.13 range-scaled
  // geometry is removed together with the range condition below).
  static const int ZS[2] = { 11, 10 };
  double b[2][4];
  for (int i = 0; i < 2; i++) {
    b[i][0] = homeLat + OFF[i][0];
    b[i][1] = homeLat + OFF[i][1];
    b[i][2] = homeLon + OFF[i][2];
    b[i][3] = homeLon + OFF[i][3];
  }
  // v6: the PNG decoder is allocated HERE - the config server is
  // stopped and no TLS session exists yet, the emptiest heap of the
  // boot (the standalone diagnostic reached GSI with ~110 KB free in
  // these conditions). The work buffers come later still, at the
  // first decode phase, after the TLS session is dropped.
  if (!mgAllocPng()) { mgFail = "heap (gen buffers)"; return false; }
  mgShow("Erasing map area...", "", 1);
  // Erase in 64 KB chunks and feed the task watchdog: one blocking
  // erase of ~2 MB takes tens of seconds and trips the WDT (v5.5 bug).
  // v6: the erase covers the tile stage area as well.
  for (uint32_t off = 0; off < genPart->size; off += 0x10000) {
    uint32_t n = genPart->size - off; if (n > 0x10000) n = 0x10000;
    if (esp_partition_erase_range(genPart, off, n) != ESP_OK) { mgFail = "flash erase"; return false; }
    esp_task_wdt_reset();
    mgShow("Erasing map area...", "", 1 + (int)(off * 4 / genPart->size));
  }
  if (!genMapPtr) {   // v6.2: decode reads the stage through the mmap
    if (esp_partition_mmap(genPart, 0, genPart->size, SPI_FLASH_MMAP_DATA,
                           (const void**)&genMapPtr, &genMmapH) != ESP_OK) {
      mgFail = "stage mmap"; return false;
    }
  }
  uint32_t sums[2] = { 0, 0 };
  for (int i = 0; i < 2; i++)
    if (!mgOneMap(i, b[i], ZS[i], GEN_HDR_BYTES + i * GEN_MAP_BYTES, &sums[i]))
      return false;
  GenHeader h = {};
  h.magic = GEN_MAGIC;
  h.ver = 1;
  h.homeLat = homeLat;
  h.homeLon = homeLon;
  memcpy(h.b, b, sizeof(b));
  h.sum[0] = sums[0];
  h.sum[1] = sums[1];
  h.reserved[0] = (uint32_t)mgRangeKm();   // v8.13: range this map was made for
  mgShow("Done", "", 100);
  if (esp_partition_write(genPart, 0, &h, sizeof(h)) != ESP_OK) { mgFail = "header write"; return false; }
  // v8.16: a backup header copy at the end of the header page, plus
  // a read-back verify - a finished map was reported lost across a
  // power cycle, its header reading back as erased (ffffffff/-1).
  if (esp_partition_write(genPart, GEN_HDR_BYTES - sizeof(h), &h, sizeof(h)) != ESP_OK) { mgFail = "header write (backup)"; return false; }
  { GenHeader v1 = {}, v2 = {};
    bool okv = esp_partition_read(genPart, 0, &v1, sizeof(v1)) == ESP_OK && v1.magic == GEN_MAGIC
            && esp_partition_read(genPart, GEN_HDR_BYTES - sizeof(v2), &v2, sizeof(v2)) == ESP_OK && v2.magic == GEN_MAGIC;
    if (!okv) { mgFail = "header verify"; return false; } }
  return true;
}

static char mgActFail[64] = "";   // v8.1: why the last activate failed
static int mgHdrRangeKm = 100;    // v8.13: range stored in the map header
static bool mgHdrFromBackup = false;   // v8.16: activation used the backup header
// v8.13: one clamped view of the configured range, used for the map
// geometry, the header stamp and the boot-time comparison alike.
static int mgRangeKm() {
  int r = cfgRangeKm;
  if (r < 10) r = 10;
  if (r > 500) r = 500;
  return r;
}
static bool mgActivate() {
  if (!genPart) { snprintf(mgActFail, sizeof(mgActFail), "no partition"); return false; }
  if (!genMapPtr) {
    if (esp_partition_mmap(genPart, 0, genPart->size, SPI_FLASH_MMAP_DATA,
                           (const void**)&genMapPtr, &genMmapH) != ESP_OK)
      { snprintf(mgActFail, sizeof(mgActFail), "mmap failed"); return false; }
  }
  // v8.16: the header is read with esp_partition_read (not through
  // the mmap), and a backup copy in the header page takes over if
  // the primary is unreadable.
  static GenHeader hdrS;
  mgHdrFromBackup = false;
  bool hdrOk = esp_partition_read(genPart, 0, &hdrS, sizeof(hdrS)) == ESP_OK
               && hdrS.magic == GEN_MAGIC && hdrS.ver == 1;
  if (!hdrOk) {
    GenHeader bak;
    if (esp_partition_read(genPart, GEN_HDR_BYTES - sizeof(bak), &bak, sizeof(bak)) == ESP_OK
        && bak.magic == GEN_MAGIC && bak.ver == 1) {
      hdrS = bak;
      mgHdrFromBackup = true;
      hdrOk = true;
      esp_partition_write(genPart, 0, &hdrS, sizeof(hdrS));   // self-heal (works when erased)
      Serial.println("MAPGEN: primary header bad - using backup copy");
    }
  }
  const GenHeader* h = &hdrS;
  if (h->magic != GEN_MAGIC || h->ver != 1) {
    snprintf(mgActFail, sizeof(mgActFail), "bad magic/ver %08x/%d",
             (unsigned)h->magic, (int)h->ver);
    Serial.printf("MAPGEN activate: %s\n", mgActFail);
    { uint8_t d0[16], dB[16];
      esp_partition_read(genPart, 0, d0, sizeof(d0));
      esp_partition_read(genPart, GEN_HDR_BYTES - sizeof(GenHeader), dB, sizeof(dB));
      Serial.printf("MAPGEN hdr0 %02x %02x %02x %02x %02x %02x %02x %02x  hdrB %02x %02x %02x %02x %02x %02x %02x %02x\n",
                    d0[0], d0[1], d0[2], d0[3], d0[4], d0[5], d0[6], d0[7],
                    dB[0], dB[1], dB[2], dB[3], dB[4], dB[5], dB[6], dB[7]); }
    return false;
  }
  // v8.20: the header centre is NOT compared with the settings any
  // more - changing the centre in the portal no longer triggers a
  // remake. The stored map (with its own bounds from the header) is
  // used as-is until a remake is ordered (BtnC boot hold / portal).
  for (int i = 0; i < 2; i++) {
    const uint8_t* base = genMapPtr + GEN_HDR_BYTES + i * GEN_MAP_BYTES;
    uint32_t s = 0;
    for (int k = 0; k < GEN_MAP_BYTES; k++) s += base[k];
    if (s != h->sum[i]) {
      snprintf(mgActFail, sizeof(mgActFail),
               "map %d sum flash %u vs hdr %u", i, (unsigned)s, (unsigned)h->sum[i]);
      Serial.printf("MAPGEN activate: %s\n", mgActFail);
      return false;
    }
  }
  mgActFail[0] = 0;
  mgHdrRangeKm = h->reserved[0] ? (int)h->reserved[0] : 100;   // v8.13
  for (int i = 0; i < 2; i++) {
    mapGrayPtr[i] = genMapPtr + GEN_HDR_BYTES + i * GEN_MAP_BYTES;
    mapClassPtr[i] = mapGrayPtr[i] + 153600;
    MAP_LIST[i].north = h->b[i][0];
    MAP_LIST[i].south = h->b[i][1];
    MAP_LIST[i].west = h->b[i][2];
    MAP_LIST[i].east = h->b[i][3];
  }
  genMapsActive = true;
  return true;
}

static void handleMapGen();   // portal handler, defined in the portal section

// v5.19: ALL generation buffers are allocated in setup() BEFORE
// Wi-Fi starts (pristine heap). Allocating them inside mgGenerate(),
// after the TLS warm-up, failed deterministically in v5.16-v5.18:
// the warm-up handshake succeeded (http 0), but a 10 KB aggregate
// malloc then found no hole next to the live TLS session and the
// 44 KB decoder (post-cleanup largest block read 42,996 every time).
static bool mgAllocPng() {
  if (mgPng) return true;
  void* pb = heap_caps_calloc(1, PNGLE_T_SIZE, MALLOC_CAP_DMA);
  if (!pb) return false;
  mgPng = (pngle_t*)pb;
  pngle_reset(mgPng);
  pngle_set_draw_callback(mgPng, mgDrawCb);
  return true;
}
// v6: work buffers live only from the first decode phase on, and the
// aggregates are one 160-column strip wide (2,560 B each, not 10 KB).
static bool mgAllocWorkBuffers() {
  // v6.9: back to the buffer set v6.5 proved on-device (19,072 B).
  // The 20 KB of RAM row accumulators (v6.6-v6.8) never fit the
  // heap alongside the live TLS session + pngle, in any arrangement.
  if (!mgSeaBits) mgSeaBits = (uint8_t*)heap_caps_malloc(8192, MALLOC_CAP_DMA);
  if (!mgWork) mgWork = (uint8_t*)heap_caps_malloc(5760, MALLOC_CAP_DMA);
  if (!mgAggGray) mgAggGray = (uint8_t*)heap_caps_malloc(160 * 32, MALLOC_CAP_DMA);
  if (!mgAggSea) mgAggSea = (uint8_t*)heap_caps_malloc(160 * 32, MALLOC_CAP_DMA);
  return mgAggGray && mgAggSea && mgSeaBits && mgWork;
}
// v7.1: the work buffers live only inside Phase B of each strip.
// Freeing them before every Phase A keeps the heap in the proven
// "pngle only" state for TLS handshakes - with the 32-row
// aggregates resident (24 KB), a retry handshake after a dropped
// connection failed with http -1 (v7.0, tiles 14).
static void mgFreeWorkBuffers() {
  if (mgAggGray) { free(mgAggGray); mgAggGray = nullptr; }
  if (mgAggSea) { free(mgAggSea); mgAggSea = nullptr; }
  if (mgSeaBits) { free(mgSeaBits); mgSeaBits = nullptr; }
  if (mgWork) { free(mgWork); mgWork = nullptr; }
}

static void mgFreeGenBuffers() {
  if (mgPng) { free(mgPng); mgPng = nullptr; }
  if (mgAggGray) { free(mgAggGray); mgAggGray = nullptr; }
  if (mgAggSea) { free(mgAggSea); mgAggSea = nullptr; }
  if (mgSeaBits) { free(mgSeaBits); mgSeaBits = nullptr; }
  if (mgWork) { free(mgWork); mgWork = nullptr; }
}

extern "C" unsigned long mz_crc32(unsigned long crc,
                                  const unsigned char* ptr, size_t buf_len);
static void mgDumpFailed() {
  if (mgDumpRec < 0 || !genPart) return;
  int rec = mgDumpRec;
  uint32_t off = GEN_STAGE_OFF + mgRecOff[rec], len = mgRecLen[rec];
  uint32_t crc = 0;
  {
    uint32_t o = off, left = len;
    while (left > 0) {
      uint32_t n = left > 2048 ? 2048 : left;
      if (esp_partition_read(genPart, o, mgTmpBuf, n) != ESP_OK) break;
      crc = mz_crc32(crc, mgTmpBuf, n);
      o += n; left -= n;
    }
  }
  Serial.printf("DUMP rec%d %s tx%d ty%d len%u crc %08x\n",
                rec, mgRecLayer[rec] ? "blank" : "std",
                mgRecTx[rec], mgRecTy[rec], (unsigned)len, (unsigned)crc);
  static const char B64[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  uint32_t dn = len < 4096 ? len : 4096;
  Serial.println("DUMP-DATA-BEGIN");
  for (uint32_t i = 0; i < dn; i += 3) {
    uint8_t b[3] = {0, 0, 0};
    uint32_t got = 0;
    for (uint32_t k = 0; k < 3 && i + k < dn; k++) {
      if (esp_partition_read(genPart, off + i + k, &b[k], 1) == ESP_OK) got++;
    }
    char out[5];
    out[0] = B64[b[0] >> 2];
    out[1] = B64[((b[0] & 3) << 4) | (b[1] >> 4)];
    out[2] = got > 1 ? B64[((b[1] & 15) << 2) | (b[2] >> 6)] : '=';
    out[3] = got > 2 ? B64[b[2] & 63] : '=';
    out[4] = 0;
    Serial.print(out);
    if (((i / 3) + 1) % 19 == 0) Serial.println();
  }
  Serial.println();
  Serial.println("DUMP-DATA-END");
  Serial.flush();
}

static void mgDiagMaybeShow() {
  GenHeader hp = {}, hb = {};
  bool pOk = genPart && esp_partition_read(genPart, 0, &hp, sizeof(hp)) == ESP_OK
             && hp.magic == GEN_MAGIC && hp.ver == 1;
  bool bOk = genPart && esp_partition_read(genPart, GEN_HDR_BYTES - sizeof(hb), &hb, sizeof(hb)) == ESP_OK
             && hb.magic == GEN_MAGIC && hb.ver == 1;
  char lgf[64] = "", laf[64] = "";
  { Preferences p;
    if (p.begin("aprsmap", true)) {
      String s1 = p.getString("lgfail", ""), s2 = p.getString("lafail", "");
      strncpy(lgf, s1.c_str(), sizeof(lgf) - 1);
      strncpy(laf, s2.c_str(), sizeof(laf) - 1);
      p.end();
    } }
  if (pOk && !lgf[0] && !laf[0]) return;   // healthy: no screen
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  int y = 10;
  char b[80];
  auto line = [&](const char* s) { tft.setCursor(8, y); tft.print(s); y += 15; };
  line("MAP DIAG (v8.17)");
  snprintf(b, sizeof(b), "SET c %.4f,%.4f r %d", homeLat, homeLon, cfgRangeKm); line(b);
  if (pOk) { snprintf(b, sizeof(b), "HDR ok c %.4f,%.4f r %d", hp.homeLat, hp.homeLon,
                      hp.reserved[0] ? (int)hp.reserved[0] : 100); line(b); }
  else { snprintf(b, sizeof(b), "HDR BAD %08x/%d", (unsigned)hp.magic, (int)hp.ver); line(b); }
  line(bOk ? "BAK ok" : "BAK bad");
  line(WiFi.status() == WL_CONNECTED ? "WIFI ok" : "WIFI no");
  if (lgf[0]) { snprintf(b, sizeof(b), "GENFAIL %s", lgf); line(b); }
  if (laf[0]) { snprintf(b, sizeof(b), "ACTFAIL %s", laf); line(b); }
  line("photo this screen, please");
  delay(12000);
  if (pOk) {   // shown once after recovery: clear the stale records
    Preferences p;
    if (p.begin("aprsmap", false)) { p.putString("lgfail", ""); p.putString("lafail", ""); p.end(); }
  }
}

static void __attribute__((noinline)) mapGenBoot() {
  genPart = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                     ESP_PARTITION_SUBTYPE_DATA_SPIFFS, "mapdata");
  if (!genPart) {
    Serial.println("MAPGEN: no mapdata partition - baked maps only");
    return;
  }
  if (mgCkpt) {   // v8.6: a hard reset interrupted the last run
    static char cl[64];
    snprintf(cl, sizeof(cl), "reset %d at map %d strip %d phase %d",
             (int)esp_reset_reason(), (int)(mgCkpt >> 24),
             (int)((mgCkpt >> 8) & 0xFFFF), (int)(mgCkpt & 0xFF));
    Serial.printf("MAPGEN interrupted: %s\n", cl);
    mgShow("Reset during generation", cl, 0);
    delay(9000);
    mgCkpt = 0;
  }
  mgDiagMaybeShow();   // v8.17
  bool force = false;
  {
    Preferences p;
    if (p.begin("aprsmap", false)) {
      force = p.getBool("mgforce", false);
      if (force) p.putBool("mgforce", false);
      p.end();
    }
  }
  if (!force && mgActivate()) {
    // v8.18: a stored map is accepted as-is. The v8.13 range
    // condition (reject and regenerate when the header's range
    // differs from the setting) is REMOVED - in the field it sent
    // the device into an endless regeneration loop.
    Serial.println("MAPGEN: generated maps active");
    if (mgHdrFromBackup) { mgShow("Map header restored", "backup copy used", 0); delay(3000); }
    return;
  }
  if (!force && mgActFail[0]) {   // v8.2: verdict on the LCD too
    mgShow("Stored map rejected", mgActFail, 0);
    delay(6000);
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("MAPGEN: offline - baked maps for now");
    return;
  }
  Serial.println("MAPGEN: generating maps for the configured centre...");
  cfgServer.stop();          // v5.16: free every heap byte for the
  MDNS.end();                // TLS handshake (see mgGenerate)
  delay(100);
  bool genOk = mgGenerate();
  mgCkpt = 0;   // v8.6: only a hard reset leaves it set
  mgHttp.end();                 // v6.5: session's job is done
  mgTls.stop();
  if (!genOk) mgDumpFailed();   // v6.4: serial dump of the bad record
  mgFreeGenBuffers();   // hand the heap back to normal operation
  cfgServer.begin();
  MDNS.begin("aprs-map");
  bool actOk = genOk && mgActivate();
  { Preferences p;   // v8.17: persist the outcome for the diag screen
    if (p.begin("aprsmap", false)) {
      p.putString("lgfail", genOk ? "" : String(mgFail));
      p.putString("lafail", (genOk && !actOk) ? String(mgActFail) : String(""));
      p.end();
    } }
  if (actOk) {
    Serial.println("MAPGEN: generation complete");
    mgShow("Map generation complete", "", 100);
    delay(1200);
  } else {
    Serial.println("MAPGEN: generation FAILED - baked maps");
    { static char fl[48], fl2[64];
      if (!mgFail[0]) mgFail = "activate check";
      snprintf(fl, sizeof(fl), "FAILED: %s", mgFail);
      if (mgFailDetail[0]) {
        snprintf(fl2, sizeof(fl2), "%s", mgFailDetail);
      } else {
        snprintf(fl2, sizeof(fl2), "heap %u http %d tiles %d [button]",
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA),
                 mgLastHttp, mgTilesOk);
      }
      mgShow(fl, fl2, 0);
      // v5.10: keep the diagnosis on screen until the screen is
      // touched (028 port: a touch press instead of the M5 buttons)
      uint32_t ft0 = millis();
      while (millis() - ft0 < 120000) {
        touchRoute();
        if (ts.getPoint().z > 400) break;
        delay(50);
      } }
    delay(2000);
  }
}


struct BmpInfo { bool ok = false; int w = 0, h = 0; uint32_t dataOff = 0; int rowSize = 0; bool topDown = false; };
static BmpInfo bmpInfo[MAP_COUNT];

static void computeView() {
  const MapDef& m = MAP_LIST[curMap];
  const BmpInfo& bi = bmpInfo[curMap];
  int w = bi.ok ? bi.w : 320;
  int h = bi.ok ? bi.h : 240;
  double fit = min(320.0 / w, 240.0 / h);
  double s;
  if (curMap == 0) {   // APRS screen zoom table (user request)
    static const double APRS_SCALES[7] = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5};
    int li = zoomLevel; if (li < 0) li = 0; if (li > 6) li = 6;
    s = APRS_SCALES[li];
  } else {             // Flight screen: doubling zoom
    s = fit;
    for (int i = 0; i < zoomLevel; i++) s *= 2;
    if (s > 4.0) s = 4.0;
  }
  vScale = s;
  vSrcW = 320.0 / s; if (vSrcW > w) vSrcW = w;
  vSrcH = 240.0 / s; if (vSrcH > h) vSrcH = h;
  vXOff = (int)lround((320 - vSrcW * s) / 2);
  vYOff = (int)lround((240 - vSrcH * s) / 2);
  double cxPx = (centerLon - m.west) / (m.east - m.west) * w;
  double cyPx = (m.north - centerLat) / (m.north - m.south) * h;
  vRectX0 = cxPx - vSrcW / 2; if (vRectX0 < 0) vRectX0 = 0; if (vRectX0 > w - vSrcW) vRectX0 = w - vSrcW;
  vRectY0 = cyPx - vSrcH / 2; if (vRectY0 < 0) vRectY0 = 0; if (vRectY0 > h - vSrcH) vRectY0 = h - vSrcH;
  vRectWest  = m.west + vRectX0 / w * (m.east - m.west);
  vRectNorth = m.north - vRectY0 / h * (m.north - m.south);
  vPxPerLon = s * w / (m.east - m.west);
  vPxPerLat = s * h / (m.north - m.south);
}

static double viewLatSpan() {
  computeView();
  int h = bmpInfo[curMap].ok ? bmpInfo[curMap].h : 240;
  return vSrcH / h * (MAP_LIST[curMap].north - MAP_LIST[curMap].south);
}
static double viewLonSpan() {
  computeView();
  int w = bmpInfo[curMap].ok ? bmpInfo[curMap].w : 320;
  return vSrcW / w * (MAP_LIST[curMap].east - MAP_LIST[curMap].west);
}

static bool geoToPx(double lat, double lon, int& px, int& py) {
  computeView();
  px = vXOff + (int)lround((lon - vRectWest) * vPxPerLon);
  py = vYOff + (int)lround((vRectNorth - lat) * vPxPerLat);
  return px >= -20 && px < 340 && py >= -20 && py < 260;
}

// Startup/reset zoom: the whole map fitted to the screen (level 0), so
// map1 (all of Kagawa) is what you see first. Zoom in with [+] / station taps.
static int maxZoomLevel() {
  return screenMode == 0 ? 6 : 3;   // APRS has 7 levels (x0.5..x3.5), Flight 4
}
static int defaultZoomLevel() {
  return screenMode == 0 ? 1 : 0;   // APRS opens at x1, Flight at the fit view
}

static uint16_t rowPixBuf[320];

// ---- flicker-free composition ---------------------------------------
// A full-frame 16-bit sprite needs 153,600 B, more than this board's
// largest heap block (~110 KB), so it could never be allocated and G
// is the panel itself: every draw below lands on the screen as it
// happens. Flicker is therefore limited by HOW the panel is fed
// (v76): the map goes out in 60-row blocks and the top bar is only
// repainted where its text actually changed. (v77 reverted the v76
// repair batching after it triggered reboots on station taps; v78
// re-batches repairs in a DMA-safe way and skips the redraw of
// unchanged markers - see restoreMapRect and drawStations.)
static LGFX_Sprite spr(&tft);
static LovyanGFX* G = &tft;
static bool sprOk = false;
static char snapAcFlight[MAX_AC][12] = {};
static int vImgRows = 0, vImgCols = 0;
static int tapX = -1, tapY = -1;       // last registered tap (marker)
static uint32_t tapMs = 0;
static bool tapShown = false;

// (mapChunk is declared with the aircraft globals - fetchAircraft frees it)
static int chunkY0 = 0, chunkRows = 0;
static void emitLine(int y) {
  if (!mapChunk) { G->pushImage(0, y, 320, 1, rowPixBuf); return; }
  if (chunkRows == 0) chunkY0 = y;
  memcpy(mapChunk + (size_t)chunkRows * 320, rowPixBuf, 320 * 2);
  chunkRows++;
  if (chunkRows == MAP_CHUNK_ROWS || y == 239) {
    G->pushImage(0, chunkY0, 320, chunkRows, mapChunk);
    chunkRows = 0;
  }
}


// Maps are embedded in flash as 4-bit grayscale (maps_data.h) - no SD
// card is used. Downscaling averages the source footprint and upscaling
// uses bilinear interpolation, so lines stay smooth and anti-aliased.
static inline int mapGrayAt(int sx, int sy) {
  const uint8_t* m = mapGrayPtr[curMap];
  uint8_t b = m[sy * MAP_ROW_BYTES + (sx >> 1)];
  return ((sx & 1) ? (b & 0x0F) : (b >> 4)) * 17;   // 0 = line .. 255 = paper
}

// Line class of a source pixel: true = coastline (drawn white),
// false = border (drawn green). Baked into maps_data.h from the GSI
// standard tiles' sea mask.
static inline bool mapCoastAt(int sx, int sy) {
  int i = sy * MAP_W + sx;
  return (mapClassPtr[curMap][i >> 3] >> (7 - (i & 7))) & 1;
}

// Dark theme palettes by line class (user request): coastlines white,
// borders green; both ramp from the near-black background.
static uint16_t inkPalW[256];
static uint16_t inkPalG[256];
static bool inkPalInit = false;
// pushImage colour rotation on this unit (diagnosed 2026-11-08 from
// two test photos): pushed (r,g,b) displays as (g,b,r) - red shows
// cyan, green shows red, blue shows green; white/black invariant.
// Both palettes below are packed pre-rotated - (b, r, g) - so the
// map displays in the intended colours.
static void initInkPal() {
  for (int d = 0; d < 256; d++) {
    // Coastline white: quick brightness rise (thin lines reach near-
    // white), NEUTRAL ramp. (A warm cast added in v52 against the
    // old cyan report became unnecessary once the pushImage colour
    // rotation was fixed in v56, and on the thinned v59 line its
    // mid-tones read as pink - user report.)
    int wd = (int)(255.0f * powf(d / 255.0f, 0.65f));
    int r = 10 + wd * (255 - 10) / 255;
    int g = 12 + wd * (255 - 12) / 255;
    int b = 16 + wd * (255 - 16) / 255;
    inkPalW[d] = (uint16_t)(((b & 0xF8) << 8) | ((r & 0xFC) << 3) | (g >> 3));  // pre-rotated (see above)
    int gr = 10 - d * 10 / 255;
    int gg = 12 + d * (215 - 12) / 255;
    int gb = 16 - d * 16 / 255;
    inkPalG[d] = (uint16_t)(((gb & 0xF8) << 8) | ((gr & 0xFC) << 3) | (gg >> 3));  // pre-rotated
  }
  inkPalInit = true;
}

// Colour of one screen pixel of the bare map (no stations, no bar).
static uint16_t mapPixelAt(int x, int y) {
  int iy = y - vYOff;
  if (iy < 0 || iy >= vImgRows) return 0;
  int ix = x - vXOff;
  if (ix < 0 || ix >= vImgCols) return 0;
  if (vScale > 1.0) {
    double srcYf = vRectY0 + iy / vScale;
    int sy0 = (int)srcYf; if (sy0 < 0) sy0 = 0; if (sy0 >= MAP_H) sy0 = MAP_H - 1;
    int sy1 = min(sy0 + 1, MAP_H - 1);
    double fy = srcYf - floor(srcYf); if (fy < 0) fy = 0;
    double srcXf = vRectX0 + ix / vScale;
    int sx0 = (int)srcXf; if (sx0 < 0) sx0 = 0; if (sx0 >= MAP_W) sx0 = MAP_W - 1;
    int sx1 = min(sx0 + 1, MAP_W - 1);
    double fx = srcXf - floor(srcXf); if (fx < 0) fx = 0;
    int g00 = mapGrayAt(sx0, sy0), g10 = mapGrayAt(sx1, sy0);
    int g01 = mapGrayAt(sx0, sy1), g11 = mapGrayAt(sx1, sy1);
    int g = (int)(g00 * (1 - fx) * (1 - fy) + g10 * fx * (1 - fy)
                + g01 * (1 - fx) * fy + g11 * fx * fy + 0.5);
    // Zoomed-in rendering fixes (v57): the baked coastline is a dense
    // blob up close, which used to show as a thick mottled band.
    // 1) Pick the line class by a darkness^2-weighted vote over the
    //    3x3 neighbourhood instead of the single darkest pixel
    //    (kills the white/green speckle along the coast).
    // 2) Square the darkness so the anti-aliased halo falls back to
    //    the background and only the line core stays bright (thins
    //    the band on screen). Fit / 1x views use the box path below
    //    and are unchanged.
    long wCoast = 0, wBorder = 0;
    for (int dy = -1; dy <= 1; dy++)
      for (int dx = -1; dx <= 1; dx++) {
        int sx = sx0 + dx; if (sx < 0) sx = 0; if (sx >= MAP_W) sx = MAP_W - 1;
        int sy = sy0 + dy; if (sy < 0) sy = 0; if (sy >= MAP_H) sy = MAP_H - 1;
        int dd = 255 - mapGrayAt(sx, sy);
        long w = (long)dd * dd;
        if (mapCoastAt(sx, sy)) wCoast += w; else wBorder += w;
      }
    bool coast = wCoast >= wBorder;
    int d0 = 255 - g;
    int d = d0 * d0 / 255;
    return (coast ? inkPalW : inkPalG)[d];
  }
  int ys = (int)(vRectY0 + iy / vScale); if (ys < 0) ys = 0; if (ys >= MAP_H) ys = MAP_H - 1;
  int ye = (int)(vRectY0 + (iy + 1) / vScale); if (ye <= ys) ye = ys + 1; if (ye > MAP_H) ye = MAP_H;
  int xs = (int)(vRectX0 + ix / vScale); if (xs < 0) xs = 0; if (xs >= MAP_W) xs = MAP_W - 1;
  int xe = (int)(vRectX0 + (ix + 1) / vScale); if (xe <= xs) xe = xs + 1; if (xe > MAP_W) xe = MAP_W;
  long sum = 0; int cnt = 0, mn = 255;
  bool mnC = false;
  for (int sy = ys; sy < ye; sy++)
    for (int sx = xs; sx < xe; sx++) {
      int v = mapGrayAt(sx, sy); sum += v; cnt++;
      if (v < mn) { mn = v; mnC = mapCoastAt(sx, sy); }
    }
  int g = cnt ? (int)((sum / cnt + mn) / 2) : 255;
  return (mnC ? inkPalW : inkPalG)[255 - g];
}

static void drawMap() {
  if (!mapChunk)   // freed before each ADS-B fetch to give TLS the heap
    mapChunk = (uint16_t*)heap_caps_malloc(320 * MAP_CHUNK_ROWS * sizeof(uint16_t), MALLOC_CAP_INTERNAL);
  if (!inkPalInit) initInkPal();
  computeView();
  vImgRows = (int)lround(vSrcH * vScale);
  vImgCols = (int)lround(vSrcW * vScale);
  bool smooth = (vScale > 1.0);
  uint32_t t0 = millis();
  G->startWrite();
  chunkRows = 0;
  for (int y = 0; y < 240; y++) {
    int iy = y - vYOff;
    if (iy < 0 || iy >= vImgRows) {                 // letterbox margin
      for (int x = 0; x < 320; x++) rowPixBuf[x] = 0;
      emitLine(y);
      continue;
    }
    if (smooth) {
      double srcYf = vRectY0 + iy / vScale;
      int sy0 = (int)srcYf; if (sy0 < 0) sy0 = 0; if (sy0 >= MAP_H) sy0 = MAP_H - 1;
      int sy1 = min(sy0 + 1, MAP_H - 1);
      double fy = srcYf - floor(srcYf); if (fy < 0) fy = 0;
      for (int x = 0; x < 320; x++) {
        int ix = x - vXOff;
        if (ix < 0 || ix >= vImgCols) { rowPixBuf[x] = 0; continue; }
        double srcXf = vRectX0 + ix / vScale;
        int sx0 = (int)srcXf; if (sx0 < 0) sx0 = 0; if (sx0 >= MAP_W) sx0 = MAP_W - 1;
        int sx1 = min(sx0 + 1, MAP_W - 1);
        double fx = srcXf - floor(srcXf); if (fx < 0) fx = 0;
        int g00 = mapGrayAt(sx0, sy0), g10 = mapGrayAt(sx1, sy0);
        int g01 = mapGrayAt(sx0, sy1), g11 = mapGrayAt(sx1, sy1);
        int g = (int)(g00 * (1 - fx) * (1 - fy) + g10 * fx * (1 - fy)
                    + g01 * (1 - fx) * fy + g11 * fx * fy + 0.5);
        long wCoast = 0, wBorder = 0;
        for (int dy = -1; dy <= 1; dy++)
          for (int dx = -1; dx <= 1; dx++) {
            int sx = sx0 + dx; if (sx < 0) sx = 0; if (sx >= MAP_W) sx = MAP_W - 1;
            int sy = sy0 + dy; if (sy < 0) sy = 0; if (sy >= MAP_H) sy = MAP_H - 1;
            int dd = 255 - mapGrayAt(sx, sy);
            long w = (long)dd * dd;
            if (mapCoastAt(sx, sy)) wCoast += w; else wBorder += w;
          }
        bool coast = wCoast >= wBorder;
        int d0 = 255 - g;
        rowPixBuf[x] = (coast ? inkPalW : inkPalG)[d0 * d0 / 255];
      }
      emitLine(y);
      continue;
    }
    int ys = (int)(vRectY0 + iy / vScale); if (ys < 0) ys = 0; if (ys >= MAP_H) ys = MAP_H - 1;
    int ye = (int)(vRectY0 + (iy + 1) / vScale); if (ye <= ys) ye = ys + 1; if (ye > MAP_H) ye = MAP_H;
    for (int x = 0; x < 320; x++) {
      int ix = x - vXOff;
      if (ix < 0 || ix >= vImgCols) { rowPixBuf[x] = 0; continue; }
      int xs = (int)(vRectX0 + ix / vScale); if (xs < 0) xs = 0; if (xs >= MAP_W) xs = MAP_W - 1;
      int xe = (int)(vRectX0 + (ix + 1) / vScale); if (xe <= xs) xe = xs + 1; if (xe > MAP_W) xe = MAP_W;
      long sum = 0;
      int cnt = 0, mn = 255;
      bool mnC = false;
      for (int sy = ys; sy < ye; sy++)
        for (int sx = xs; sx < xe; sx++) {
          int v = mapGrayAt(sx, sy); sum += v; cnt++;
          if (v < mn) { mn = v; mnC = mapCoastAt(sx, sy); }
        }
      // blend the mean with the darkest pixel so thin lines survive
      int g = cnt ? (int)((sum / cnt + mn) / 2) : 255;
      rowPixBuf[x] = (mnC ? inkPalW : inkPalG)[255 - g];
    }
    emitLine(y);
  }
  G->endWrite();
  Serial.printf("map render: %lu ms (flash)\n", (unsigned long)(millis() - t0));
}

// ---------------- drawing ----------------
static const int BAR_H = 22;

static const char* aprsStateText();  // fwd

static char lastBarLeft[32] = "", lastBarMid[48] = "";
static void drawBarF(bool force, bool push) {
  computeView();
  char scale[8], mid[48], leftKey[4];
  if (fabs(vScale - lround(vScale)) < 0.01) snprintf(scale, sizeof(scale), "x%d", (int)lround(vScale));
  else snprintf(scale, sizeof(scale), "x%.1f", vScale);
  if (screenMode == 0) snprintf(mid, sizeof(mid), "%s %s", aprsStateText(), scale);
  else if (screenMode == 1) snprintf(mid, sizeof(mid), "AC %d %s", acCount, scale);
  else if (cfgAisKey.length() < 10) snprintf(mid, sizeof(mid), "SET AIS KEY %s", scale);
  else snprintf(mid, sizeof(mid), "SHIP %d %s", countShips(), scale);
  snprintf(leftKey, sizeof(leftKey), "%c", screenMode == 0 ? 'A' : (screenMode == 1 ? 'F' : 'S'));
  // No visible change -> leave the screen alone (user request: update
  // only when something inside the map area changed).
  if (!force && !strcmp(leftKey, lastBarLeft) && !strcmp(mid, lastBarMid)) return;
  if (!force && !strcmp(leftKey, lastBarLeft)) {
    // v76: same screen, only the status text changed (station count /
    // scale live inside mid) - repaint just that strip instead of
    // blanking and redrawing the whole bar (a visible blink on every
    // 5 s tick and every station update).
    strcpy(lastBarMid, mid);
    G->fillRect(126, 0, 102, BAR_H, TFT_NAVY);
    G->setFont(&fonts::Font0);
    G->setTextColor(TFT_WHITE, TFT_NAVY);
    G->setCursor(130, 7);
    G->print(mid);
    if (push && sprOk) spr.pushSprite(0, 0);
    return;
  }
  strcpy(lastBarLeft, leftKey);
  strcpy(lastBarMid, mid);
  G->fillRect(0, 0, 320, BAR_H, TFT_NAVY);
  // screen tabs: [APRS] [FLIGHT] [AIS] - the active one is filled white
  G->setFont(&fonts::Font0);
  static const struct { int x, w; const char* lb; } TABS[MODE_COUNT] = {
    {2, 44, "APRS"}, {48, 44, "FLIGHT"}, {94, 30, "AIS"}
  };
  for (int i = 0; i < MODE_COUNT; i++) {
    int tx = TABS[i].x, tw = TABS[i].w;
    const char* lb = TABS[i].lb;
    if (screenMode == i) {
      G->fillRect(tx, 2, tw, 18, TFT_WHITE);
      G->setTextColor(TFT_NAVY, TFT_WHITE);
    } else {
      G->drawRect(tx, 2, tw, 18, TFT_WHITE);
      G->setTextColor(TFT_WHITE, TFT_NAVY);
    }
    G->setCursor(tx + (tw - G->textWidth(lb)) / 2, 7);
    G->print(lb);
  }
  // status text in the middle
  G->setTextColor(TFT_WHITE, TFT_NAVY);
  G->setCursor(130, 7);
  G->print(mid);
  // buttons: [LIST] [-] [+]
  G->drawRect(228, 2, 38, 18, TFT_WHITE);
  G->setCursor(234, 7); G->print("LIST");
  G->drawRect(269, 2, 23, 18, TFT_WHITE);
  G->setCursor(277, 7); G->print("-");
  G->drawRect(295, 2, 23, 18, TFT_WHITE);
  G->setCursor(303, 7); G->print("+");
  if (push && sprOk) spr.pushSprite(0, 0);
}
static void drawBar() { drawBarF(true, true); }

// Snapshot of what drawStations() last put on screen, so the loop can
// skip redraws when no visible station actually moved or appeared.
static int16_t snapPx[MAX_STATIONS], snapPy[MAX_STATIONS];
static int16_t snapLabelW[MAX_STATIONS];   // label width as drawn (ghost fix)
static bool snapUsed[MAX_STATIONS] = {};
static int snapSelected = -2;
static char snapCall[MAX_STATIONS][20] = {};   // callsign as drawn (v78)
// v78: set by redraw() each frame - true only when nothing was erased
// or repainted under the markers, in which case the draw* functions
// may skip objects whose on-screen image would not change at all.
static bool skipUnchanged = false;
static bool repairErased = false;   // repairMovedObjects() restored a rect

static void drawHomeMarker() {
  int px, py;
  // home marker: red cross with a black underlay
  if (geoToPx(homeLat, homeLon, px, py) && py >= BAR_H) {
    G->drawLine(px - 6, py, px + 6, py, TFT_BLACK);
    G->drawLine(px, py - 6, px, py + 6, TFT_BLACK);
    G->drawLine(px - 5, py, px + 5, py, TFT_RED);
    G->drawLine(px, py - 5, px, py + 5, TFT_RED);
  }
}

static void drawStations() {
  int px, py;
  for (int i = 0; i < MAX_STATIONS; i++) {
    if (!stations[i].used) { snapUsed[i] = false; continue; }
    if (!geoToPx(stations[i].lat, stations[i].lon, px, py)) { snapUsed[i] = false; continue; }
    if (py < BAR_H + 4) { snapUsed[i] = false; continue; }
    if (px < 0 || px >= 320) { snapUsed[i] = false; continue; }   // label would float marker-less
    bool sel = (i == selected);
    // v78: this exact marker (position, callsign, selection state)
    // is already on screen and nothing was erased this frame - leave
    // it alone instead of redrawing every label on every update.
    if (skipUnchanged && snapUsed[i] && px == snapPx[i] && py == snapPy[i] &&
        sel == (snapSelected == i) && !strcmp(snapCall[i], stations[i].call))
      continue;
    snapUsed[i] = true; snapPx[i] = px; snapPy[i] = py;
    snapLabelW[i] = (int16_t)(strlen(stations[i].call) * 6 + 16);
    strncpy(snapCall[i], stations[i].call, sizeof(snapCall[i]) - 1);
    snapCall[i][sizeof(snapCall[i]) - 1] = 0;
    uint16_t dot = sel ? TFT_YELLOW : TFT_ORANGE;   // user-chosen colour
    G->fillCircle(px, py, sel ? 6 : 5, dot);
    G->drawCircle(px, py, sel ? 6 : 5, TFT_BLACK);
    G->fillCircle(px, py, 1, TFT_WHITE);
    // label with a full black halo so it reads on any map colour
    G->setFont(&fonts::Font0);
    G->setTextColor(TFT_BLACK);
    G->setCursor(px + 6, py - 5); G->print(stations[i].call);
    G->setCursor(px + 8, py - 5); G->print(stations[i].call);
    G->setCursor(px + 7, py - 6); G->print(stations[i].call);
    G->setCursor(px + 7, py - 4); G->print(stations[i].call);
    G->setCursor(px + 7, py - 5);
    G->setTextColor(dot);
    G->print(stations[i].call);
  }
  snapSelected = selected;
}

// Aircraft snapshot, same idea as the station snapshot.
static int16_t snapAcPx[MAX_AC], snapAcPy[MAX_AC];
static int snapAcCount = -1;
static float snapAcTrack[MAX_AC];              // heading as drawn (v78)
static char snapSelAcFlight[12] = "";          // selection as drawn (v78)
static bool snapDetailIsAc = false;

static void drawAircraft() {
  int px, py;
  for (int i = 0; i < acCount; i++) {
    if (!geoToPx(aircraft[i].lat, aircraft[i].lon, px, py)) { snapAcPx[i] = -1; continue; }
    if (py < BAR_H + 4) { snapAcPx[i] = -1; continue; }
    if (px < 0 || px >= 320) { snapAcPx[i] = -1; continue; }   // same label-spill fix
    bool sel = detailIsAc && !strcmp(aircraft[i].flight, selAcFlight);
    bool selBefore = snapDetailIsAc && !strcmp(aircraft[i].flight, snapSelAcFlight);
    // v78: skip-unchanged (position, flight, TRACK, selection) - the
    // triangle rotates with the track, so track must match too.
    if (skipUnchanged && snapAcPx[i] == px && snapAcPy[i] == py &&
        !strcmp(snapAcFlight[i], aircraft[i].flight) &&
        snapAcTrack[i] == aircraft[i].track && sel == selBefore)
      continue;
    snapAcPx[i] = px; snapAcPy[i] = py;
    strncpy(snapAcFlight[i], aircraft[i].flight, sizeof(snapAcFlight[i]) - 1);
    snapAcFlight[i][sizeof(snapAcFlight[i]) - 1] = 0;
    snapAcTrack[i] = aircraft[i].track;
    uint16_t col = sel ? TFT_YELLOW : TFT_CYAN;
    // small triangle pointing along the track, cyan like a radar plot
    float r = aircraft[i].track * (float)DEG_TO_RAD;
    float sn = sinf(r), cs = cosf(r);
    int nx = px + (int)(9 * sn),  ny = py - (int)(9 * cs);         // nose
    int bx = px - (int)(6 * sn),  by = py + (int)(6 * cs);         // tail centre
    int lx = bx + (int)(5 * cs),  ly = by + (int)(5 * sn);
    int rx = bx - (int)(5 * cs),  ry = by - (int)(5 * sn);
    G->fillTriangle(nx, ny, lx, ly, rx, ry, col);
    G->fillCircle(px, py, 1, TFT_WHITE);
    G->setFont(&fonts::Font0);
    G->setTextColor(TFT_BLACK);
    G->setCursor(px + 7, py - 5); G->print(aircraft[i].flight);
    G->setCursor(px + 9, py - 5); G->print(aircraft[i].flight);
    G->setCursor(px + 8, py - 6); G->print(aircraft[i].flight);
    G->setCursor(px + 8, py - 4); G->print(aircraft[i].flight);
    G->setCursor(px + 8, py - 5);
    G->setTextColor(col);
    G->print(aircraft[i].flight);
  }
  snapAcCount = acCount;
  strncpy(snapSelAcFlight, selAcFlight, sizeof(snapSelAcFlight) - 1);
  snapSelAcFlight[sizeof(snapSelAcFlight) - 1] = 0;
  snapDetailIsAc = detailIsAc;
}

// Ship snapshot, same idea as the aircraft snapshot.
static int16_t snapShipPx[MAX_SHIPS], snapShipPy[MAX_SHIPS];
static uint32_t snapShipMmsi[MAX_SHIPS] = {};
static int snapShipCount = -1;
static float snapShipCog[MAX_SHIPS];           // course as drawn (v78)
static bool snapShipUnder[MAX_SHIPS] = {};     // underway shape as drawn (v78)
static char snapShipName[MAX_SHIPS][24] = {};  // label as drawn (v78)
static uint32_t snapSelShipMmsi = 0;           // selection as drawn (v78)

static void drawShips() {
  int px, py;
  for (int i = 0; i < MAX_SHIPS; i++) {
    if (!ships[i].used || !ships[i].hasPos) { snapShipPx[i] = -1; continue; }
    if (!geoToPx(ships[i].lat, ships[i].lon, px, py)) { snapShipPx[i] = -1; continue; }
    if (py < BAR_H + 4) { snapShipPx[i] = -1; continue; }
    if (px < 0 || px >= 320) { snapShipPx[i] = -1; continue; }   // same label-spill fix
    char label[24];
    if (ships[i].name[0]) strncpy(label, ships[i].name, sizeof(label) - 1);
    else snprintf(label, sizeof(label), "%lu", (unsigned long)ships[i].mmsi);
    label[sizeof(label) - 1] = 0;
    bool sel = ships[i].mmsi == selShipMmsi && selShipMmsi != 0;
    bool underway = ships[i].sog >= 0.5f && ships[i].cog < 360.0f;
    // v78: skip-unchanged (position, MMSI, course, shape, label,
    // selection) - anchored ships otherwise got fully redrawn on
    // every AIS update even though they never move.
    if (skipUnchanged && snapShipPx[i] == px && snapShipPy[i] == py &&
        snapShipMmsi[i] == ships[i].mmsi && snapShipCog[i] == ships[i].cog &&
        snapShipUnder[i] == underway && !strcmp(snapShipName[i], label) &&
        sel == (snapShipMmsi[i] == snapSelShipMmsi && snapSelShipMmsi != 0))
      continue;
    snapShipPx[i] = px; snapShipPy[i] = py;
    snapShipMmsi[i] = ships[i].mmsi;
    snapShipCog[i] = ships[i].cog;
    snapShipUnder[i] = underway;
    strncpy(snapShipName[i], label, sizeof(snapShipName[i]) - 1);
    snapShipName[i][sizeof(snapShipName[i]) - 1] = 0;
    uint16_t col = sel ? TFT_YELLOW : TFT_SKYBLUE;
    if (underway) {
      // triangle pointing along the course over ground
      float r = ships[i].cog * (float)DEG_TO_RAD;
      float sn = sinf(r), cs = cosf(r);
      int nx = px + (int)(9 * sn),  ny = py - (int)(9 * cs);
      int bx = px - (int)(6 * sn),  by = py + (int)(6 * cs);
      int lx = bx + (int)(5 * cs),  ly = by + (int)(5 * sn);
      int rx = bx - (int)(5 * cs),  ry = by - (int)(5 * sn);
      G->fillTriangle(nx, ny, lx, ly, rx, ry, col);
    } else {
      // anchored / moored / drifting: a small square
      G->fillRect(px - 4, py - 4, 9, 9, col);
      G->drawRect(px - 4, py - 4, 9, 9, TFT_BLACK);
    }
    G->fillCircle(px, py, 1, TFT_WHITE);
    G->setFont(&fonts::Font0);
    G->setTextColor(TFT_BLACK);
    G->setCursor(px + 7, py - 5); G->print(label);
    G->setCursor(px + 9, py - 5); G->print(label);
    G->setCursor(px + 8, py - 6); G->print(label);
    G->setCursor(px + 8, py - 4); G->print(label);
    G->setCursor(px + 8, py - 5);
    G->setTextColor(col);
    G->print(label);
  }
  snapShipCount = countShips();
  snapSelShipMmsi = selShipMmsi;
}

// ◀ ▶ ▲ ▼ hints at the screen edges while zoomed in (the pan tap zones)
static void drawPanHints() {
  if (zoomLevel <= defaultZoomLevel()) return;
  G->fillTriangle(8, 120, 2, 112, 2, 128, TFT_SILVER);
  G->fillTriangle(311, 120, 317, 112, 317, 128, TFT_SILVER);
  G->fillTriangle(160, BAR_H + 3, 152, BAR_H + 13, 168, BAR_H + 13, TFT_SILVER);
  G->fillTriangle(160, 237, 152, 227, 168, 227, TFT_SILVER);
}

// APRS display policy (user request): fixed stations are stored and
// the screen only repaints when the visible set grows/shrinks; mobile
// stations repaint once a minute (checked in loop()).
static bool mobileOnScreenMoved() {
  int px, py;
  for (int i = 0; i < MAX_STATIONS; i++) {
    if (!stations[i].used || !stations[i].mobile) continue;
    if (!geoToPx(stations[i].lat, stations[i].lon, px, py) || py < BAR_H + 4) continue;
    if (snapUsed[i] && (px != snapPx[i] || py != snapPy[i])) return true;
  }
  return false;
}

static bool stationsOnScreenChanged() {
  int px, py;
  if (screenMode == 0) {
    if (selected != snapSelected) return true;
    for (int i = 0; i < MAX_STATIONS; i++) {
      bool vis = false;
      if (stations[i].used && geoToPx(stations[i].lat, stations[i].lon, px, py) && py >= BAR_H + 4) vis = true;
      if (vis != snapUsed[i]) return true;
    }
    return false;
  }
  if (screenMode == 2) {
    if (countShips() != snapShipCount) return true;
    for (int i = 0; i < MAX_SHIPS; i++) {
      if (!ships[i].used || !ships[i].hasPos) { if (snapShipPx[i] >= 0) return true; continue; }
      bool vis = geoToPx(ships[i].lat, ships[i].lon, px, py) && py >= BAR_H + 4;
      if (!vis) { if (snapShipPx[i] >= 0) return true; continue; }
      if (snapShipPx[i] != px || snapShipPy[i] != py) return true;
    }
    return false;
  }
  if (acCount != snapAcCount) return true;
  for (int i = 0; i < acCount; i++) {
    bool vis = geoToPx(aircraft[i].lat, aircraft[i].lon, px, py) && py >= BAR_H + 4;
    if (!vis) { if (snapAcPx[i] >= 0) return true; continue; }
    if (snapAcPx[i] != px || snapAcPy[i] != py) return true;
  }
  return false;
}

static double haversineKm(double la1, double lo1, double la2, double lo2) {
  double dLa = (la2 - la1) * DEG_TO_RAD, dLo = (lo2 - lo1) * DEG_TO_RAD;
  double a = sin(dLa / 2) * sin(dLa / 2) + cos(la1 * DEG_TO_RAD) * cos(la2 * DEG_TO_RAD) * sin(dLo / 2) * sin(dLo / 2);
  return 6371.0 * 2 * atan2(sqrt(a), sqrt(1 - a));
}
static double bearingDeg(double la1, double lo1, double la2, double lo2) {
  double y = sin((lo2 - lo1) * DEG_TO_RAD) * cos(la2 * DEG_TO_RAD);
  double x = cos(la1 * DEG_TO_RAD) * sin(la2 * DEG_TO_RAD) - sin(la1 * DEG_TO_RAD) * cos(la2 * DEG_TO_RAD) * cos((lo2 - lo1) * DEG_TO_RAD);
  double b = atan2(y, x) * RAD_TO_DEG;
  return b < 0 ? b + 360 : b;
}

static void drawAcDetail() {
  int idx = -1;
  for (int i = 0; i < acCount; i++)
    if (!strcmp(aircraft[i].flight, selAcFlight)) { idx = i; break; }
  if (idx < 0) { detailOpen = false; detailIsAc = false; return; }
  const Aircraft& a = aircraft[idx];
  const RouteInfo* r = findRoute(a.flight);
  G->fillRoundRect(8, 30, 304, 190, 6, 0x1082);   // dark slate
  G->drawRoundRect(8, 30, 304, 190, 6, TFT_WHITE);
  G->setTextColor(TFT_RED, 0x1082);
  G->setFont(&fonts::Font2);
  G->setCursor(288, 34); G->print("X");
  G->setTextColor(TFT_CYAN, 0x1082);
  G->setCursor(16, 34);
  G->print(a.flight);
  G->setFont(&fonts::Font0);
  G->setTextColor(TFT_WHITE, 0x1082);
  int y = 58;
  char buf[96];
  if (r && r->airline[0]) { G->setCursor(16, y); G->print(r->airline); y += 14; }
  if (r && r->ok) {
    snprintf(buf, sizeof(buf), "FROM %s", r->orig);
    G->setCursor(16, y); G->print(buf); y += 12;
    snprintf(buf, sizeof(buf), "TO   %s", r->dest);
    G->setCursor(16, y); G->print(buf); y += 14;
  } else {
    G->setCursor(16, y); G->print("ROUTE UNKNOWN"); y += 14;
  }
  if (a.altFt >= 0)
    snprintf(buf, sizeof(buf), "ALT %ld ft (%ld m)", a.altFt, (long)(a.altFt * 0.3048));
  else
    snprintf(buf, sizeof(buf), "ALT ---- (on ground?)");
  G->setCursor(16, y); G->print(buf); y += 12;
  if (a.gsKt >= 0) {
    snprintf(buf, sizeof(buf), "GS %d kt (%d km/h)  HDG %03d", a.gsKt, (int)(a.gsKt * 1.852), (int)a.track);
    G->setCursor(16, y); G->print(buf); y += 12;
  }
  double d = haversineKm(homeLat, homeLon, a.lat, a.lon);
  double brg = bearingDeg(homeLat, homeLon, a.lat, a.lon);
  snprintf(buf, sizeof(buf), "DIST %.1f km  BRG %.0f deg", d, brg);
  G->setCursor(16, y); G->print(buf);
}

static void drawShipDetail() {
  Ship* s = selShipMmsi ? findShip(selShipMmsi, false) : nullptr;
  if (!s || !s->hasPos) { detailOpen = false; return; }
  G->fillRoundRect(8, 30, 304, 190, 6, 0x1082);   // dark slate
  G->drawRoundRect(8, 30, 304, 190, 6, TFT_WHITE);
  G->setTextColor(TFT_RED, 0x1082);
  G->setFont(&fonts::Font2);
  G->setCursor(288, 34); G->print("X");
  G->setTextColor(TFT_SKYBLUE, 0x1082);
  G->setCursor(16, 34);
  char title[28];
  if (s->name[0]) strncpy(title, s->name, sizeof(title) - 1);
  else snprintf(title, sizeof(title), "%lu", (unsigned long)s->mmsi);
  title[sizeof(title) - 1] = 0;
  G->print(title);
  G->setFont(&fonts::Font0);
  G->setTextColor(TFT_WHITE, 0x1082);
  int y = 58;
  char buf[96];
  snprintf(buf, sizeof(buf), "MMSI %lu", (unsigned long)s->mmsi);
  G->setCursor(16, y); G->print(buf); y += 12;
  const char* tt = shipTypeText(s->type);
  if (tt[0]) {
    snprintf(buf, sizeof(buf), "TYPE %s", tt);
    G->setCursor(16, y); G->print(buf); y += 12;
  }
  snprintf(buf, sizeof(buf), "LAT %.4f  LON %.4f", s->lat, s->lon);
  G->setCursor(16, y); G->print(buf); y += 12;
  double d = haversineKm(homeLat, homeLon, s->lat, s->lon);
  double brg = bearingDeg(homeLat, homeLon, s->lat, s->lon);
  snprintf(buf, sizeof(buf), "DIST %.1f km  BRG %.0f deg", d, brg);
  G->setCursor(16, y); G->print(buf); y += 12;
  if (s->sog >= 0.5f && s->cog < 360.0f)
    snprintf(buf, sizeof(buf), "SOG %.1f kt (%d km/h)  COG %03d", s->sog, (int)(s->sog * 1.852), (int)s->cog);
  else
    snprintf(buf, sizeof(buf), "SOG %.1f kt (anchored / slow)", s->sog);
  G->setCursor(16, y); G->print(buf); y += 12;
  if (s->dest[0]) {
    snprintf(buf, sizeof(buf), "DEST %.30s", s->dest);
    G->setCursor(16, y); G->print(buf); y += 12;
  }
  snprintf(buf, sizeof(buf), "LAST %lu min ago", (unsigned long)((millis() - s->lastMs) / 60000));
  G->setCursor(16, y); G->print(buf);
}

static void drawDetail() {
  if (screenMode == 2) { drawShipDetail(); return; }
  if (detailIsAc) { drawAcDetail(); return; }
  if (selected < 0 || !stations[selected].used) { detailOpen = false; return; }
  const Station& s = stations[selected];
  G->fillRoundRect(8, 30, 304, 190, 6, 0x1082);   // dark slate
  G->drawRoundRect(8, 30, 304, 190, 6, TFT_WHITE);
  G->setTextColor(TFT_RED, 0x1082);
  G->setFont(&fonts::Font2);
  G->setCursor(288, 34); G->print("X");
  G->setTextColor(TFT_YELLOW, 0x1082);
  G->setCursor(16, 34);
  G->print(s.call);
  G->setFont(&fonts::Font0);
  G->setTextColor(TFT_WHITE, 0x1082);
  int y = 58;
  char buf[96];
  snprintf(buf, sizeof(buf), "LAT %.4f  LON %.4f", s.lat, s.lon);
  G->setCursor(16, y); G->print(buf); y += 12;
  double d = haversineKm(homeLat, homeLon, s.lat, s.lon);
  double brg = bearingDeg(homeLat, homeLon, s.lat, s.lon);
  snprintf(buf, sizeof(buf), "DIST %.1f km  BRG %.0f deg", d, brg);
  G->setCursor(16, y); G->print(buf); y += 12;
  if (s.course >= 0 && s.speedKt >= 0) {
    snprintf(buf, sizeof(buf), "COURSE %03d  SPEED %d kt (%d km/h)", s.course, s.speedKt, (int)(s.speedKt * 1.852));
    G->setCursor(16, y); G->print(buf); y += 12;
  }
  if (s.hasAlt) {
    snprintf(buf, sizeof(buf), "ALT %ld ft (%ld m)", s.altFt, (long)(s.altFt * 0.3048));
    G->setCursor(16, y); G->print(buf); y += 12;
  }
  uint32_t ageMin = (s.lastUnix && time(nullptr) > 1700000000)
      ? ((uint32_t)time(nullptr) - s.lastUnix) / 60
      : (millis() - s.lastMs) / 60000;
  snprintf(buf, sizeof(buf), "LAST %lu min ago   SYM %c%c", (unsigned long)ageMin, s.symTable, s.symCode);
  G->setCursor(16, y); G->print(buf); y += 12;
  if (s.via[0]) {
    snprintf(buf, sizeof(buf), "VIA %.40s", s.via);
    G->setCursor(16, y); G->print(buf); y += 12;
  }
  y += 4;
  G->setTextColor(TFT_CYAN, 0x1082);
  // comment, wrapped at ~48 chars, up to 5 lines
  const char* c = s.comment;
  while (*c && y < 210) {
    char line[52];
    int n = 0;
    while (c[n] && n < 48 && c[n] != '\n') n++;
    memcpy(line, c, n); line[n] = 0;
    G->setCursor(16, y); G->print(line);
    y += 12;
    c += n; if (*c == '\n') c++;
  }
}

static bool fullRedraw = true;
static bool mapDirty = true;              // view changed -> re-render the map from SD
static uint16_t* restoreChunk = nullptr;   // v78: dedicated 320x24 staging buffer (15,360 B)
static void restoreMapRect(int x0, int y0, int x1, int y1) {
  if (x0 < 0) x0 = 0; if (y0 < BAR_H) y0 = BAR_H;
  if (x1 > 319) x1 = 319; if (y1 > 239) y1 = 239;
  if (x1 < x0 || y1 < y0) return;
  int w = x1 - x0 + 1;
  // v78: batched again, but NOT the way v76 did it. v76 staged into
  // the SHARED mapChunk and pushed several blocks under one held
  // startWrite transaction - the buffer could be restaged while a
  // DMA transfer from it was still in flight, and the device began
  // rebooting on station taps (fixed by the v77 revert). Here each
  // chunk is pushed with a standalone pushImage (its own transaction,
  // which returns only after the transfer completes, so the buffer
  // is never reused early), and the staging buffer is dedicated to
  // this function instead of being shared with drawMap. Marker erase
  // is 1 push, the detail-panel area ~8 pushes, the full map area
  // ~10 - instead of 23 / 190 / 218 line-by-line transfers.
  if (!restoreChunk)
    restoreChunk = (uint16_t*)heap_caps_malloc(320 * 24 * sizeof(uint16_t), MALLOC_CAP_INTERNAL);
  if (!restoreChunk) {   // allocation failed: v75 line-by-line fallback
    for (int y = y0; y <= y1; y++) {
      for (int x = x0; x <= x1; x++) rowPixBuf[x - x0] = mapPixelAt(x, y);
      G->pushImage(x0, y, w, 1, rowPixBuf);
    }
    return;
  }
  int rowsPerPush = (320 * 24) / w;
  if (rowsPerPush < 1) rowsPerPush = 1;
  for (int y = y0; y <= y1;) {
    int n = min(rowsPerPush, y1 - y + 1);
    for (int r = 0; r < n; r++)
      for (int x = x0; x <= x1; x++)
        restoreChunk[r * w + (x - x0)] = mapPixelAt(x, y + r);
    G->pushImage(x0, y, w, n, restoreChunk);
    y += n;
  }
}

// When only markers moved, repaint just the bare map under their old
// spots (all markers are redrawn right after), not the whole map.
static void repairMovedObjects() {
  int px, py;
  for (int i = 0; i < MAX_STATIONS; i++) {
    if (!snapUsed[i]) continue;
    bool same = stations[i].used && geoToPx(stations[i].lat, stations[i].lon, px, py) &&
                py >= BAR_H + 4 && px == snapPx[i] && py == snapPy[i] &&
                i != selected && i != snapSelected &&
                !strcmp(snapCall[i], stations[i].call);   // v78: slot reused by another call
    if (same) continue;
    // Erase with the width that was DRAWN: if this slot has since been
    // reused by a station with a shorter callsign, using the current
    // call left the tail of the previous label on screen (user report).
    int w = snapLabelW[i];
    repairErased = true;
    restoreMapRect(snapPx[i] - 9, snapPy[i] - 11, snapPx[i] + w, snapPy[i] + 11);
  }
  for (int i = 0; i < MAX_AC; i++) {
    if (snapAcPx[i] < 0) continue;
    bool same = i < acCount && !strcmp(aircraft[i].flight, snapAcFlight[i]) &&
                geoToPx(aircraft[i].lat, aircraft[i].lon, px, py) &&
                py >= BAR_H + 4 && px == snapAcPx[i] && py == snapAcPy[i] &&
                snapAcTrack[i] == aircraft[i].track;   // v78: rotated triangle is not "same"
    if (same) continue;
    int w = (int)strlen(snapAcFlight[i]) * 6 + 16;
    repairErased = true;
    restoreMapRect(snapAcPx[i] - 13, snapAcPy[i] - 13, snapAcPx[i] + w, snapAcPy[i] + 13);
  }
  for (int i = 0; i < MAX_SHIPS; i++) {
    if (snapShipPx[i] < 0) continue;
    char labelNow[24];
    if (ships[i].name[0]) strncpy(labelNow, ships[i].name, sizeof(labelNow) - 1);
    else snprintf(labelNow, sizeof(labelNow), "%lu", (unsigned long)ships[i].mmsi);
    labelNow[sizeof(labelNow) - 1] = 0;
    bool underwayNow = ships[i].sog >= 0.5f && ships[i].cog < 360.0f;
    bool same = ships[i].used && ships[i].hasPos && ships[i].mmsi == snapShipMmsi[i] &&
                geoToPx(ships[i].lat, ships[i].lon, px, py) &&
                py >= BAR_H + 4 && px == snapShipPx[i] && py == snapShipPy[i] &&
                ships[i].cog == snapShipCog[i] && underwayNow == snapShipUnder[i] &&
                !strcmp(labelNow, snapShipName[i]);   // v78: course/shape/label all as drawn
    if (same) continue;
    char oldLabel[24];
    // label width as DRAWN (v78: from the snapshot, not the current
    // slot occupant - the ship may have been replaced in its slot)
    if (snapShipName[i][0]) strncpy(oldLabel, snapShipName[i], sizeof(oldLabel) - 1);
    else snprintf(oldLabel, sizeof(oldLabel), "%lu", (unsigned long)snapShipMmsi[i]);
    oldLabel[sizeof(oldLabel) - 1] = 0;
    int w = (int)strlen(oldLabel) * 6 + 16;
    repairErased = true;
    restoreMapRect(snapShipPx[i] - 13, snapShipPy[i] - 13, snapShipPx[i] + w, snapShipPy[i] + 13);
  }
}

static bool markerRepair = false;
// ---- visible-object list (LIST button, both screens) ----------------
#define LIST_ROWS 11
#define LIST_Y0 (BAR_H + 26)
#define LIST_ROWH 15
static int listIdx[MAX_STATIONS + MAX_AC];
static double listDist[MAX_STATIONS + MAX_AC];
static int listCount = 0;

static void buildList() {
  listCount = 0;
  int px, py;
  if (screenMode == 0) {
    for (int i = 0; i < MAX_STATIONS; i++) {
      if (!stations[i].used) continue;
      if (!geoToPx(stations[i].lat, stations[i].lon, px, py) || py < BAR_H) continue;
      listIdx[listCount] = i;
      listDist[listCount] = haversineKm(homeLat, homeLon, stations[i].lat, stations[i].lon);
      listCount++;
    }
  } else if (screenMode == 1) {
    for (int i = 0; i < acCount; i++) {
      if (!geoToPx(aircraft[i].lat, aircraft[i].lon, px, py) || py < BAR_H) continue;
      listIdx[listCount] = i;
      listDist[listCount] = haversineKm(homeLat, homeLon, aircraft[i].lat, aircraft[i].lon);
      listCount++;
    }
  } else {
    for (int i = 0; i < MAX_SHIPS; i++) {
      if (!ships[i].used || !ships[i].hasPos) continue;
      if (!geoToPx(ships[i].lat, ships[i].lon, px, py) || py < BAR_H) continue;
      listIdx[listCount] = i;
      listDist[listCount] = haversineKm(homeLat, homeLon, ships[i].lat, ships[i].lon);
      listCount++;
    }
  }
  for (int i = 1; i < listCount; i++) {   // nearest first
    int id = listIdx[i]; double dd = listDist[i]; int j = i - 1;
    while (j >= 0 && listDist[j] > dd) { listIdx[j + 1] = listIdx[j]; listDist[j + 1] = listDist[j]; j--; }
    listIdx[j + 1] = id; listDist[j + 1] = dd;
  }
}

static void drawList() {
  buildList();
  int pages = max(1, (listCount + LIST_ROWS - 1) / LIST_ROWS);
  if (listPage >= pages) listPage = pages - 1;
  if (listPage < 0) listPage = 0;
  G->fillRect(0, BAR_H, 320, 240 - BAR_H, 0x1082);
  G->setFont(&fonts::Font2);
  G->setTextColor(TFT_YELLOW, 0x1082);
  G->setCursor(10, BAR_H + 3);
  G->print(screenMode == 0 ? "APRS LIST" : (screenMode == 1 ? "FLIGHT LIST" : "SHIP LIST"));
  G->setTextColor(TFT_RED, 0x1082);
  G->setCursor(294, BAR_H + 3);
  G->print("X");
  G->setFont(&fonts::Font0);
  if (listCount == 0) {
    G->setTextColor(TFT_WHITE, 0x1082);
    const char* msg = screenMode == 0 ? "NO STATIONS VISIBLE"
                    : (screenMode == 1 ? "NO AIRCRAFT VISIBLE" : "NO SHIPS VISIBLE");
    G->setCursor((320 - G->textWidth(msg)) / 2, 120);
    G->print(msg);
  }
  int start = listPage * LIST_ROWS;
  for (int r = 0; r < LIST_ROWS && start + r < listCount; r++) {
    int id = listIdx[start + r];
    int y = LIST_Y0 + r * LIST_ROWH;
    char right[40];
    G->setTextColor(TFT_WHITE, 0x1082);
    G->setCursor(10, y + 3);
    if (screenMode == 0) {
      G->print(stations[id].call);
      snprintf(right, sizeof(right), "%.1f km", listDist[start + r]);
    } else if (screenMode == 1) {
      G->print(aircraft[id].flight);
      if (aircraft[id].altFt >= 0)
        snprintf(right, sizeof(right), "%.0f km  %d ft", listDist[start + r], aircraft[id].altFt);
      else
        snprintf(right, sizeof(right), "%.0f km", listDist[start + r]);
    } else {
      if (ships[id].name[0]) G->print(ships[id].name);
      else {
        char mb[16];
        snprintf(mb, sizeof(mb), "%lu", (unsigned long)ships[id].mmsi);
        G->print(mb);
      }
      snprintf(right, sizeof(right), "%.1f km  %.0f kn", listDist[start + r], ships[id].sog);
    }
    G->setCursor(310 - G->textWidth(right), y + 3);
    G->print(right);
    G->drawLine(8, y + LIST_ROWH - 2, 312, y + LIST_ROWH - 2, TFT_NAVY);
  }
  // paging footer
  G->setTextColor(TFT_WHITE, 0x1082);
  if (listPage > 0) { G->setCursor(10, 226); G->print("< PREV"); }
  if (listPage < pages - 1) { G->setCursor(268, 226); G->print("NEXT >"); }
  char pc[12];
  snprintf(pc, sizeof(pc), "%d/%d", listPage + 1, pages);
  G->setCursor((320 - G->textWidth(pc)) / 2, 226);
  G->print(pc);
}

// v78: the bare map was repainted under every marker (full map draw
// or a panel/list restore), so all "already on screen" snapshots are
// void and the draw* functions must draw everything again.
static void invalidateSnapshots() {
  for (int i = 0; i < MAX_STATIONS; i++) snapUsed[i] = false;
  snapSelected = -2;
  for (int i = 0; i < MAX_AC; i++) snapAcPx[i] = -1;
  snapAcCount = -1;
  for (int i = 0; i < MAX_SHIPS; i++) snapShipPx[i] = -1;
  snapShipCount = -1;
}

static void redraw() {
  static bool prevDetail = false, prevList = false;
  computeView();
  vImgRows = (int)lround(vSrcH * vScale);
  vImgCols = (int)lround(vSrcW * vScale);
  if (!inkPalInit) initInkPal();
  static double lastViewLat = 0, lastViewLon = 0;
  static int lastViewZoom = -1, lastViewMap = -1;
  bool viewChanged = centerLat != lastViewLat || centerLon != lastViewLon
                  || zoomLevel != lastViewZoom || curMap != lastViewMap;
  bool mapRedrawn = false;
  repairErased = false;
  if (mapDirty || viewChanged) {
    drawMap(); mapDirty = false;
    mapRedrawn = true;
    lastViewLat = centerLat; lastViewLon = centerLon;
    lastViewZoom = zoomLevel; lastViewMap = curMap;
    invalidateSnapshots();
  } else {
    // panel or list closed -> repaint the map area under it (the
    // list close had no repaint path before v76 and left the slate
    // background on screen).
    if (prevList && !listOpen) {
      restoreMapRect(0, BAR_H, 319, 239);   // the list covered everything
      invalidateSnapshots();
    } else if (prevDetail && !detailOpen) {
      restoreMapRect(7, 29, 312, 220);      // v78: only the panel's own rect
      invalidateSnapshots();
    } else repairMovedObjects();
    if (markerRepair) {
      restoreMapRect(tapX - 12, tapY - 12, tapX + 12, tapY + 12);
      markerRepair = false;
      repairErased = true;   // the tap ring may have clipped a marker
    }
  }
  // v78: unchanged markers may be skipped only when nothing was
  // erased or repainted under them this frame.
  skipUnchanged = !repairErased && !mapRedrawn;
  prevDetail = detailOpen;
  prevList = listOpen;
  // v76: a full map redraw overwrites the bar area too, so the bar
  // must be repainted; on a marker-only update the bar is untouched
  // on screen, so leave it to drawBarF's own change check instead of
  // forcing a full-bar blink on every update.
  if (mapRedrawn) drawBarF(true, false);
  else drawBarF(false, false);
  drawHomeMarker();
  if (screenMode == 0) drawStations();
  else if (screenMode == 1) drawAircraft();
  else drawShips();
  drawPanHints();
  if (detailOpen) drawDetail();
  if (listOpen) drawList();
  if (tapShown) {
    G->drawCircle(tapX, tapY, 9, TFT_YELLOW);
    G->drawCircle(tapX, tapY, 10, TFT_YELLOW);
  }
  if (sprOk) spr.pushSprite(0, 0);
  fullRedraw = false;
  stationsDirty = false;
}

// ---------------- APRS parsing ----------------
static bool isDig(char c) { return c >= '0' && c <= '9'; }

// Parse an uncompressed position starting at p ("DDMM.mmN/DDDMM.mmWc...")
static bool parseUncomp(const char* p, Station& st, const char*& rest) {
  if (!isDig(p[0]) || !isDig(p[1]) || !isDig(p[2]) || !isDig(p[3]) || p[4] != '.' ||
      !isDig(p[5]) || !isDig(p[6]) || (p[7] != 'N' && p[7] != 'S')) return false;
  double lat = (p[0] - '0') * 10 + (p[1] - '0')
             + ((p[2] - '0') * 10 + (p[3] - '0') + ((p[5] - '0') * 10 + (p[6] - '0')) / 100.0) / 60.0;
  if (p[7] == 'S') lat = -lat;
  st.symTable = p[8];
  const char* q = p + 9;
  if (!isDig(q[0]) || !isDig(q[1]) || !isDig(q[2]) || !isDig(q[3]) || !isDig(q[4]) || q[5] != '.' ||
      !isDig(q[6]) || !isDig(q[7]) || (q[8] != 'E' && q[8] != 'W')) return false;
  double lon = (q[0] - '0') * 100 + (q[1] - '0') * 10 + (q[2] - '0')
             + ((q[3] - '0') * 10 + (q[4] - '0') + ((q[6] - '0') * 10 + (q[7] - '0')) / 100.0) / 60.0;
  if (q[8] == 'W') lon = -lon;
  st.symCode = q[9];
  st.lat = lat; st.lon = lon;
  rest = q + 10;
  return true;
}

// Parse a compressed position starting at p (table + 4 + 4 base91 + code ...)
static bool parseComp(const char* p, Station& st, const char*& rest) {
  for (int i = 1; i <= 8; i++) if (p[i] < '!' || p[i] > '{') return false;
  st.symTable = p[0];
  long yv = 0, xv = 0;
  for (int i = 0; i < 4; i++) yv = yv * 91 + (p[1 + i] - 33);
  for (int i = 0; i < 4; i++) xv = xv * 91 + (p[5 + i] - 33);
  st.lat = 90.0 - yv / 380926.0;
  st.lon = -180.0 + xv / 190463.0;
  st.symCode = p[9];
  rest = p + 10;
  // optional csT (course/speed or altitude): 3 base91-range bytes
  if (p[10] >= '!' && p[10] <= '{' && p[11] >= '!' && p[11] <= '{' && p[12] >= '!' && p[12] <= '{') {
    int cs1 = p[10] - 33, cs2 = p[11] - 33, tt = p[12] - 33;
    if (tt & 0x10) {
      st.hasAlt = true;
      st.altFt = (long)pow(1.002, cs1 * 91 + cs2);
    } else {
      st.course = cs1 * 4;
      st.speedKt = (int)(pow(1.08, cs2) - 1);
      if (st.course > 360 || st.speedKt > 799) { st.course = -1; st.speedKt = -1; }
    }
    rest = p + 13;
  }
  return true;
}

static bool parsePositionAt(const char* p, Station& st, const char*& rest) {
  if (isDig(p[0])) return parseUncomp(p, st, rest);
  return parseComp(p, st, rest);
}

// Mic-E digit from a destination-callsign character.
// flag: 0 = '0'-'9', 1 = 'A'-'J' (message bits, chars 1-3 only),
//       2 = 'P'-'Y' (the hemisphere/offset marks in chars 4-6)
static int micDigit(char c, int& flag) {
  flag = 0;
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'J') { flag = 1; return c - 'A'; }
  if (c >= 'P' && c <= 'Y') { flag = 2; return c - 'P'; }
  return 0; // K, L, Z = space
}

static bool parseMicE(const char* dst, const uint8_t* info, int infoLen, Station& st) {
  if (strlen(dst) < 6 || infoLen < 8) return false;
  int d[6], f[6];
  for (int i = 0; i < 6; i++) d[i] = micDigit(dst[i], f[i]);
  double lat = d[0] * 10 + d[1] + ((d[2] * 10 + d[3]) + (d[4] * 10 + d[5]) / 100.0) / 60.0;
  // Hemisphere/offset flags follow Dire Wolf's aprs_mic_e exactly:
  // char4: '0'-'9'/'L' = South, 'P'-'Z' = North (v12 had this backwards;
  // verified against JO4HJX-9>S4SXR4 - char4 'X', really in Mie, N).
  if (f[3] != 2) lat = -lat;
  bool lonOff = (f[4] == 2);                   // char5: 'P'-'Y' = +100 deg
  bool west = (f[5] == 2);                     // char6: 'P'-'Y' = West
  // Longitude bytes use wrapped ranges (also Dire Wolf's rules):
  int ch = info[0], lonDeg;
  if (lonOff && ch >= 118 && ch <= 127) lonDeg = ch - 118;
  else if (!lonOff && ch >= 38 && ch <= 127) lonDeg = ch - 38 + 10;
  else if (lonOff && ch >= 108 && ch <= 117) lonDeg = ch - 108 + 100;
  else if (lonOff && ch >= 38 && ch <= 107) lonDeg = ch - 38 + 110;
  else return false;
  ch = info[1];
  int lonMin;
  if (ch >= 88 && ch <= 97) lonMin = ch - 88;
  else if (ch >= 38 && ch <= 87) lonMin = ch - 38 + 10;
  else return false;
  ch = info[2];
  if (ch < 28 || ch > 127) return false;
  double lon = lonDeg + (lonMin + (ch - 28) / 100.0) / 60.0;
  if (west) lon = -lon;
  int sp = (info[3] - 28) * 10 + (info[4] - 28) / 10;
  int crs = ((info[4] - 28) % 10) * 100 + (info[5] - 28);
  if (crs >= 400) crs -= 400;
  if (sp >= 800) sp -= 800;
  st.lat = lat; st.lon = lon;
  st.symCode = (char)info[6];
  st.symTable = (char)info[7];
  if (sp < 200) st.speedKt = sp;
  if (crs < 360) st.course = crs;
  // comment follows the symbol table char
  if (infoLen > 8) {
    int n = infoLen - 8; if (n > 78) n = 78;
    memcpy(st.comment, info + 8, n); st.comment[n] = 0;
  }
  return true;
}

static void finishStation(Station& st, const char* rest) {
  // course/speed prefix "DDD/SSS" at the start of the comment
  const char* c = rest;
  if (isDig(c[0]) && isDig(c[1]) && isDig(c[2]) && c[3] == '/' && isDig(c[4]) && isDig(c[5]) && isDig(c[6])) {
    st.course = (c[0] - '0') * 100 + (c[1] - '0') * 10 + (c[2] - '0');
    st.speedKt = (c[4] - '0') * 100 + (c[5] - '0') * 10 + (c[6] - '0');
    c += 7;
    if (*c == ' ') c++;
  }
  strncpy(st.comment, c, sizeof(st.comment) - 1);
  st.comment[sizeof(st.comment) - 1] = 0;
  // altitude "/A=001234" anywhere in the comment
  const char* a = strstr(st.comment, "/A=");
  if (a) {
    char* end = nullptr;
    long v = strtol(a + 3, &end, 10);
    if (end != a + 3) { st.hasAlt = true; st.altFt = v; }
  }
}

static bool stPersistDirty = false;

static int findSlot(const char* call) {
  int slot = -1, oldest = 0;
  uint32_t oldestMs = 0xFFFFFFFF;
  for (int i = 0; i < MAX_STATIONS; i++) {
    if (stations[i].used && strcmp(stations[i].call, call) == 0) { slot = i; break; }
    if (!stations[i].used && slot < 0) slot = i;
    if (stations[i].used && stations[i].lastMs < oldestMs) { oldestMs = stations[i].lastMs; oldest = i; }
  }
  if (slot < 0) slot = oldest;
  return slot;
}

static void upsert(Station& st) {
  int slot = findSlot(st.call);
  bool mob = stations[slot].mobile;
  if (stations[slot].used &&
      (fabs(stations[slot].lat - st.lat) > 0.00005 ||
       fabs(stations[slot].lon - st.lon) > 0.00005)) mob = true;
  if (st.speedKt > 0) mob = true;
  st.mobile = mob;
  st.lastMs = millis();
  time_t t = time(nullptr);
  st.lastUnix = t > 1700000000 ? (uint32_t)t : 0;
  st.used = true;
  stations[slot] = st;
  stationsDirty = true;
  stPersistDirty = true;
  Serial.printf("STN %s lat %.4f lon %.4f\n", st.call, st.lat, st.lon);
}

// ---- station persistence: show the last-heard stations immediately
// after boot instead of waiting for each one to beacon again. The
// table is kept in NVS with real (NTP) timestamps; APRS-IS then keeps
// it live as before. (aprs.fi's API cannot replace this: it only
// answers per-callsign lookups, with no area search.)
struct PersistStation {          // v41: slimmer record (104 B)
  char call[16];
  float lat, lon;
  int16_t course, speedKt;
  int32_t altFt;
  uint8_t hasAlt;
  char comment[40];
  char via[24];
  uint32_t lastUnix;
};
struct PersistStationV1 {        // v35-v40 layout (144 B), for migration
  char call[20];
  float lat, lon;
  int16_t course, speedKt;
  int32_t altFt;
  uint8_t hasAlt;
  char comment[64];
  char via[36];
  uint32_t lastUnix;
};
#define MAX_PERSIST 56
static uint32_t lastPersistMs = 0;
static bool ntpStarted = false, stRestored = false;

static void saveStations() {
  time_t t = time(nullptr);
  if (t < 1700000000) return;
  static PersistStation buf[MAX_STATIONS];
  int n = 0;
  uint32_t nowM = millis();
  for (int i = 0; i < MAX_STATIONS; i++) {
    if (!stations[i].used) continue;
    PersistStation& p = buf[n++];
    memset(&p, 0, sizeof(p));
    strncpy(p.call, stations[i].call, sizeof(p.call) - 1);
    p.lat = (float)stations[i].lat;
    p.lon = (float)stations[i].lon;
    p.course = (int16_t)stations[i].course;
    p.speedKt = (int16_t)stations[i].speedKt;
    p.altFt = (int32_t)stations[i].altFt;
    p.hasAlt = stations[i].hasAlt ? 1 : 0;
    strncpy(p.comment, stations[i].comment, sizeof(p.comment) - 1);
    strncpy(p.via, stations[i].via, sizeof(p.via) - 1);
    p.lastUnix = stations[i].lastUnix ? stations[i].lastUnix
                                      : (uint32_t)t - (nowM - stations[i].lastMs) / 1000;
  }
  if (n > MAX_PERSIST) {   // keep the freshest ones (blob stays ~6 KB)
    for (int i = 0; i < MAX_PERSIST; i++) {
      int best = i;
      for (int j = i + 1; j < n; j++) if (buf[j].lastUnix > buf[best].lastUnix) best = j;
      if (best != i) { PersistStation tmp = buf[i]; buf[i] = buf[best]; buf[best] = tmp; }
    }
    n = MAX_PERSIST;
  }
  // v73: save in 8-station chunks (832 B each) instead of one 5824 B blob.
  // A single big blob needs one contiguous free run in the 20 KB NVS
  // partition and kept failing with "persist: short write 0/5824" once
  // the table filled up; each chunk only needs ~832 B free.
  Preferences pr;
  pr.begin("aprsst", false);
  const int CH = 8, NCH = (MAX_PERSIST + CH - 1) / CH;   // 7 chunks
  int nch = (n + CH - 1) / CH;
  bool ok = true;
  for (int c = 0; c < NCH; c++) {
    char key[4]; snprintf(key, sizeof(key), "t%d", c);
    if (c >= nch) { pr.remove(key); continue; }   // drop stale chunks
    int cn = min(CH, n - c * CH);
    size_t want = cn * sizeof(PersistStation);
    size_t w = pr.putBytes(key, buf + c * CH, want);
    if (w != want) {   // retry once after removing just this chunk
      Serial.printf("persist: chunk %d short write %u/%u - retry\n", c, (unsigned)w, (unsigned)want);
      pr.remove(key);
      w = pr.putBytes(key, buf + c * CH, want);
      Serial.printf("persist: chunk %d retry %u/%u\n", c, (unsigned)w, (unsigned)want);
    }
    if (w != want) ok = false;
  }
  if (ok) pr.remove("tbl");   // old single blob no longer needed
  pr.putInt("count", ok ? n : 0);
  if (ok && n) Serial.printf("persist: saved %d stations in %d chunks\n", n, nch);
  pr.end();
}

static void restoreStations() {
  time_t nowT = time(nullptr);
  if (nowT < 1700000000) return;
  Preferences pr;
  pr.begin("aprsst", true);
  int n = pr.getInt("count", 0);
  if (n <= 0 || n > MAX_STATIONS) { Serial.printf("persist: no data (count %d)\n", n); pr.end(); return; }
  static PersistStation buf[MAX_STATIONS];
  // v73: chunked layout t0..t6 (8 stations each) first; fall back to the
  // old single "tbl" blob (incl. v35-v40 records) when no chunks exist.
  if (pr.getBytesLength("t0") > 0) {
    int gotN = 0;
    const int CH = 8;
    for (int c = 0; c * CH < n; c++) {
      char key[4]; snprintf(key, sizeof(key), "t%d", c);
      int cn = min(CH, n - c * CH);
      size_t got = pr.getBytes(key, buf + c * CH, cn * sizeof(PersistStation));
      if (got != (size_t)(cn * sizeof(PersistStation))) {
        Serial.printf("persist: chunk %d short read %u/%u\n", c, (unsigned)got,
                      (unsigned)(cn * sizeof(PersistStation)));
        break;
      }
      gotN += cn;
    }
    n = gotN;
    if (!n) { Serial.println("persist: no chunk data"); pr.end(); return; }
  } else {
    size_t got = pr.getBytes("tbl", buf, n * sizeof(PersistStation));
    if (got != (size_t)(n * sizeof(PersistStation))) {
      // maybe a v35-v40 blob (bigger records): convert it
      // v79: buf1 was a 13.8 KB function static; BSS no longer has
      // room for it next to the map-generation statics, and this
      // one-time migration path can borrow the heap instead.
      PersistStationV1* buf1 =
          (PersistStationV1*)malloc(n * sizeof(PersistStationV1));
      size_t got1 = buf1 ? pr.getBytes("tbl", buf1, n * sizeof(PersistStationV1)) : 0;
      if (buf1 && got1 == (size_t)(n * sizeof(PersistStationV1))) {
        for (int i = 0; i < n; i++) {
          memset(&buf[i], 0, sizeof(buf[i]));
          strncpy(buf[i].call, buf1[i].call, sizeof(buf[i].call) - 1);
          buf[i].lat = buf1[i].lat; buf[i].lon = buf1[i].lon;
          buf[i].course = buf1[i].course; buf[i].speedKt = buf1[i].speedKt;
          buf[i].altFt = buf1[i].altFt; buf[i].hasAlt = buf1[i].hasAlt;
          strncpy(buf[i].comment, buf1[i].comment, sizeof(buf[i].comment) - 1);
          strncpy(buf[i].via, buf1[i].via, sizeof(buf[i].via) - 1);
          buf[i].lastUnix = buf1[i].lastUnix;
        }
        Serial.println("persist: legacy blob migrated");
      } else {
        Serial.printf("persist: size mismatch (count %d, got %u/%u)\n",
                      n, (unsigned)got, (unsigned)got1);
        free(buf1);
        pr.end();
        return;
      }
      free(buf1);
    }
  }
  pr.end();
  int added = 0;
  for (int i = 0; i < n; i++) {
    if (!buf[i].call[0] || !buf[i].lastUnix) continue;
    uint32_t age = (uint32_t)nowT - buf[i].lastUnix;
    if (age > (uint32_t)STATION_EXPIRE_MIN * 60) continue;
    Station st;
    strncpy(st.call, buf[i].call, sizeof(st.call) - 1);
    st.call[sizeof(st.call) - 1] = 0;
    st.lat = buf[i].lat;
    st.lon = buf[i].lon;
    st.course = buf[i].course;
    st.speedKt = buf[i].speedKt;
    st.hasAlt = buf[i].hasAlt != 0;
    st.altFt = buf[i].altFt;
    strncpy(st.comment, buf[i].comment, sizeof(st.comment) - 1);
    strncpy(st.via, buf[i].via, sizeof(st.via) - 1);
    st.lastUnix = buf[i].lastUnix;
    uint32_t ageMs = age * 1000UL;
    st.lastMs = ageMs < millis() ? millis() - ageMs : 0;
    st.used = true;
    stations[findSlot(st.call)] = st;
    added++;
  }
  if (added) {
    stationsDirty = true;
    Serial.printf("restored %d stations\n", added);
  } else {
    Serial.printf("persist: %d entries, none fresh\n", n);
  }
}

static void parsePacket(const char* line) {
  const char* gt = strchr(line, '>');
  const char* colon = gt ? strchr(gt, ':') : nullptr;
  if (!gt || !colon || colon == gt + 1) return;
  char src[20];
  int sl = min((int)(gt - line), 19);
  memcpy(src, line, sl); src[sl] = 0;
  char dst[16];
  const char* comma = strchr(gt + 1, ',');
  const char* dstEnd = (comma && comma < colon) ? comma : colon;
  int dl = min((int)(dstEnd - (gt + 1)), 15);
  memcpy(dst, gt + 1, dl); dst[dl] = 0;

  const char* info = colon + 1;
  if (!*info) return;
  Station st;
  strncpy(st.call, src, sizeof(st.call) - 1);
  if (comma && comma < colon) {
    int vl = min((int)(colon - (comma + 1)), (int)sizeof(st.via) - 1);
    memcpy(st.via, comma + 1, vl); st.via[vl] = 0;
  }
  bool ok = false;
  const char* rest = "";
  switch (info[0]) {
    case '!': case '=':
      ok = parsePositionAt(info + 1, st, rest);
      break;
    case '/': case '@':
      if (strlen(info) > 8) ok = parsePositionAt(info + 8, st, rest);
      break;
    case ';': {  // object
      if (strlen(info) < 18 || info[10] == '_') return;  // killed object
      char name[12];
      memcpy(name, info + 1, 9); name[9] = 0;
      for (int i = 8; i >= 0; i--) { if (name[i] == ' ') name[i] = 0; else break; }
      strncpy(st.call, name, sizeof(st.call) - 1);
      ok = parsePositionAt(info + 18, st, rest);
      break;
    }
    case ')': {  // item
      int i = 1;
      while (info[i] && info[i] != '!' && info[i] != '_' && i < 11) i++;
      if (!info[i] || info[i] == '_') return;
      char name[12];
      int nl = min(i - 1, 11);
      memcpy(name, info + 1, nl); name[nl] = 0;
      strncpy(st.call, name, sizeof(st.call) - 1);
      ok = parsePositionAt(info + i + 1, st, rest);
      break;
    }
    case '`': case '\'':
      ok = parseMicE(dst, (const uint8_t*)info + 1, strlen(info) - 1, st);
      if (ok) { upsert(st); packetCount++; return; }
      return;
    default:
      return;
  }
  if (!ok) return;
  if (st.lat < -90 || st.lat > 90 || st.lon < -180 || st.lon > 180) return;
  if (st.lat == 0 && st.lon == 0) return;
  finishStation(st, rest);
  upsert(st);
  packetCount++;
}

// ---------------- APRS-IS ----------------
static WiFiClient client;
enum AprsState { A_WIFI, A_CONNECT, A_LOGIN, A_ONLINE };
static AprsState aState = A_WIFI;
static uint32_t aStateMs = 0, lastRxMs = 0, lastKeepaliveMs = 0;
static char lineBuf[1024];
static int lineLen = 0;
static bool filterSent = false;

static const char* aprsStateText() {
  static char buf[40];
  // Count only stations inside the current map: activity outside the
  // map must not change the screen (user request).
  int n = 0;
  const MapDef& m = MAP_LIST[curMap];
  for (int i = 0; i < MAX_STATIONS; i++)
    if (stations[i].used && stations[i].lat <= m.north && stations[i].lat >= m.south &&
        stations[i].lon >= m.west && stations[i].lon <= m.east) n++;
  switch (aState) {
    case A_WIFI:   return "WiFi...";
    case A_CONNECT: return "APRS...";
    case A_LOGIN:  return "LOGIN...";
    default:
      snprintf(buf, sizeof(buf), "APRS %d st", n);
      return buf;
  }
}

static int aprsPasscode(const char* call) {
  char base[16];
  int i = 0;
  while (call[i] && call[i] != '-' && i < 15) { base[i] = toupper((uint8_t)call[i]); i++; }
  base[i] = 0;
  int h = 0x73e2;
  int len = strlen(base);
  for (int k = 0; k + 1 < len; k += 2) { h ^= (base[k] << 8); h ^= base[k + 1]; }
  if (len % 2) h ^= (base[len - 1] << 8);
  return h & 0x7FFF;
}

static void aprsSetup() {
  // wifiPhase() in setup() has already connected (or portal-configured)
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(cfgSsid.c_str(), cfgPass.c_str());
  }
  aState = (WiFi.status() == WL_CONNECTED) ? A_CONNECT : A_WIFI;
  aStateMs = millis();
}

static void aprsLoop() {
  uint32_t now = millis();
  switch (aState) {
    case A_WIFI:
      if (WiFi.status() == WL_CONNECTED) { aState = A_CONNECT; aStateMs = now; }
      else if (now - aStateMs > 20000) { WiFi.disconnect(); WiFi.begin(cfgSsid.c_str(), cfgPass.c_str()); aStateMs = now; }
      break;
    case A_CONNECT:
      if (WiFi.status() != WL_CONNECTED) { aState = A_WIFI; aStateMs = now; break; }
      client.stop();
      if (client.connect(APRS_HOST, APRS_PORT)) {
        int pass = cfgPasscode;
        if (pass == 0) pass = aprsPasscode(cfgCall.c_str());
        client.printf("user %s pass %d vers MuseAPRSMap 1.0\r\n", cfgCall.c_str(), pass);
        filterSent = false;
        lineLen = 0;
        lastRxMs = now;
        aState = A_LOGIN; aStateMs = now;
      } else if (now - aStateMs > 10000) {
        aStateMs = now;  // retry
      }
      break;
    case A_LOGIN:
    case A_ONLINE: {
      if (!client.connected()) { aState = A_CONNECT; aStateMs = now; break; }
      while (client.available()) {
        char ch = (char)client.read();
        lastRxMs = now;
        if (ch == '\n' || ch == '\r') {
          if (lineLen > 0) {
            lineBuf[lineLen] = 0;
            if (lineBuf[0] == '#') {
              if (aState == A_LOGIN && strstr(lineBuf, "logresp")) {
                client.printf("#filter r/%.2f/%.2f/%d\r\n", homeLat, homeLon, cfgRangeKm);
                filterSent = true;
                aState = A_ONLINE;
              }
            } else {
              parsePacket(lineBuf);
            }
            lineLen = 0;
          }
        } else if (lineLen < (int)sizeof(lineBuf) - 1) {
          lineBuf[lineLen++] = ch;
        }
      }
      if (aState == A_LOGIN && now - aStateMs > 6000 && !filterSent) {
        // server gave no logresp line; send the filter anyway
        client.printf("#filter r/%.2f/%.2f/%d\r\n", homeLat, homeLon, cfgRangeKm);
        filterSent = true;
        aState = A_ONLINE;
      }
      if (aState == A_ONLINE) {
        if (now - lastKeepaliveMs > 240000) { client.print("# keepalive\r\n"); lastKeepaliveMs = now; }
        if (now - lastRxMs > 300000) { client.stop(); aState = A_CONNECT; aStateMs = now; }
      }
      break;
    }
  }
}

// ---------------- AIS stream (aisstream.io) ----------------
// One persistent WebSocket; the subscription names the Flight/AIS map
// area, so every ship received belongs on that map. Position reports
// arrive every few seconds per ship; static data (name / destination /
// type) only now and then. Reconnect is handled by the library.
static bool aisSubConfirmed = false;
static uint32_t aisLastSubMs = 0;
static bool wsSendFrame(uint8_t opcode, const uint8_t* data, size_t len);   // fwd

static bool sendAisSubscription() {
  // Bounds of MAP_LIST[1] (the Flight/AIS map), as [lat, lon] pairs.
  char sub[320];
  snprintf(sub, sizeof(sub),
    "{\"APIKey\":\"%s\",\"BoundingBoxes\":[[[%.4f,%.4f],[%.4f,%.4f]]],"
    "\"FilterMessageTypes\":[\"PositionReport\",\"ShipStaticData\"]}",
    cfgAisKey.c_str(), MAP_LIST[1].south, MAP_LIST[1].west,
    MAP_LIST[1].north, MAP_LIST[1].east);
  aisLastSubMs = millis();
  return wsSendFrame(0x1, (const uint8_t*)sub, strlen(sub));
}

static void parseAis(const String& msg) {
  if (msg.indexOf("\"SubscriptionConfirmation\"") >= 0) {
    aisSubConfirmed = true;
    Serial.println("AIS subscription confirmed");
    return;
  }
  int mk = msg.indexOf("\"MMSI\":");
  if (mk < 0) return;
  uint32_t mmsi = (uint32_t)strtoul(msg.c_str() + mk + 7, nullptr, 10);
  if (!mmsi) return;
  int len = (int)msg.length();
  if (msg.indexOf("\"PositionReport\"") >= 0) {
    int seg = msg.indexOf("\"PositionReport\":{");
    if (seg < 0) return;
    double la, lo;
    bool okPos = jsonNumField(msg, "\"Latitude\":", seg, len, la) &&
                 jsonNumField(msg, "\"Longitude\":", seg, len, lo);
    if (!okPos) {   // fall back to the MetaData position
      okPos = jsonNumField(msg, "\"latitude\":", 0, len, la) &&
              jsonNumField(msg, "\"longitude\":", 0, len, lo);
    }
    if (!okPos) return;
    Ship* s = findShip(mmsi, true);
    if (!s) return;
    bool isNew = !s->used || !s->hasPos;
    s->used = true; s->hasPos = true; s->mmsi = mmsi;
    s->lat = la; s->lon = lo;
    double v;
    if (jsonNumField(msg, "\"Sog\":", seg, len, v)) s->sog = (v <= 102.2) ? (float)v : 0;
    if (jsonNumField(msg, "\"Cog\":", seg, len, v)) s->cog = (float)v;
    if (jsonNumField(msg, "\"NavigationalStatus\":", seg, len, v)) s->navStatus = (int)v;
    char nm[28];
    if (jsonStrField(msg, "\"ShipName\":\"", 0, len, nm, sizeof(nm))) {
      trimSpaces(nm);
      if (nm[0]) { strncpy(s->name, nm, sizeof(s->name) - 1); s->name[sizeof(s->name) - 1] = 0; }
    }
    s->lastMs = millis();
    stationsDirty = true;
    if (isNew) Serial.printf("AIS ship: %s (%lu) total %d\n",
                             s->name[0] ? s->name : "?", (unsigned long)mmsi, countShips());
  } else if (msg.indexOf("\"ShipStaticData\"") >= 0) {
    Ship* s = findShip(mmsi, true);
    if (!s) return;
    s->used = true; s->mmsi = mmsi;
    char buf[28];
    if (jsonStrField(msg, "\"Name\":\"", 0, len, buf, sizeof(buf))) {
      trimSpaces(buf);
      if (buf[0]) { strncpy(s->name, buf, sizeof(s->name) - 1); s->name[sizeof(s->name) - 1] = 0; }
    }
    if (jsonStrField(msg, "\"Destination\":\"", 0, len, buf, sizeof(buf))) {
      trimSpaces(buf);
      strncpy(s->dest, buf, sizeof(s->dest) - 1); s->dest[sizeof(s->dest) - 1] = 0;
    }
    double v;
    if (jsonNumField(msg, "\"Type\":", 0, len, v)) s->type = (int)v;
    if (s->hasPos) { s->lastMs = millis(); stationsDirty = true; }
  }
}

// ---- hand-rolled WebSocket client with permessage-deflate --------
// aisstream silently ignores subscriptions from clients that do not
// negotiate compression (only a deflate-capable test client ever
// received data, incl. the SubscriptionConfirmation). The WebSockets
// library cannot negotiate permessage-deflate, so the protocol runs
// here directly on WiFiClientSecure, with miniz/tinfl inflating. The
// streaming inflater keeps its dictionary between messages, so the
// server's context takeover works (validated on a host build with
// python-generated takeover streams before this was written).
static WiFiClientSecure aisSock;
static bool aisSockUp = false;
static uint32_t aisNextTryMs = 0;
static uint32_t aisLastRxMs = 0;
static bool aisDeflate = false;
static String wsBuf;              // received bytes not yet framed
static String wsMsg;              // payload of the message being assembled
static bool wsMsgCompressed = false;
static bool wsMsgStarted = false;

static lgfx_tinfl_decompressor* inflState = nullptr;
static lgfx_mz_uint8* inflDict = nullptr;
static size_t inflDictPos = 0;

static void wsInflateReset() {
  if (inflState) { lgfx_tinfl_init(inflState); inflDictPos = 0; }
}

// Inflate one permessage-deflate message (its sync-flush tail was
// stripped on the wire; it is appended back here). The decompressor
// and its dictionary persist across messages (context takeover).
static bool wsInflate(const uint8_t* in, size_t inLen, String& out) {
  if (!inflState) {
    if (mapChunk) { free(mapChunk); mapChunk = nullptr; }   // make room first
    inflState = (lgfx_tinfl_decompressor*)malloc(sizeof(lgfx_tinfl_decompressor));
    inflDict = (lgfx_mz_uint8*)malloc(TINFL_LZ_DICT_SIZE);
    if (!inflState || !inflDict) return false;
    lgfx_tinfl_init(inflState);
    inflDictPos = 0;
  }
  static const lgfx_mz_uint8 tail[4] = {0x00, 0x00, 0xFF, 0xFF};
  size_t off = 0;
  int iter = 0;
  for (;;) {
    const lgfx_mz_uint8* chunk;
    size_t chunkLen;
    if (off < inLen) { chunk = in + off; chunkLen = inLen - off; }
    else if (off < inLen + 4) { chunk = tail; chunkLen = 4 - (off - inLen); }
    else break;
    size_t inSize = chunkLen;
    size_t outSize = TINFL_LZ_DICT_SIZE - inflDictPos;
    lgfx_mz_uint8* outNext = inflDict + inflDictPos;
    lgfx_tinfl_status st = lgfx_tinfl_decompress(inflState, chunk, &inSize, inflDict, outNext, &outSize,
                                       TINFL_FLAG_HAS_MORE_INPUT);
    off += inSize;
    if (outSize) {
      out.concat((const char*)outNext, (unsigned int)outSize);
      inflDictPos = (inflDictPos + outSize) & (TINFL_LZ_DICT_SIZE - 1);
      if (out.length() > 16384) return false;
    }
    if (st == TINFL_STATUS_FAILED || st == TINFL_STATUS_BAD_PARAM) { wsInflateReset(); return false; }
    if (off >= inLen + 4 && outSize == 0) break;   // flush fully consumed
    if (inSize == 0 && outSize == 0) break;        // at a block boundary
    if (++iter > 10000) return false;
  }
  return true;
}

static bool wsSendFrame(uint8_t opcode, const uint8_t* data, size_t len) {
  if (!aisSockUp) return false;
  uint8_t hdr[14];
  int h = 0;
  hdr[h++] = 0x80 | (opcode & 0x0F);
  uint8_t maskKey[4];
  for (int i = 0; i < 4; i++) maskKey[i] = (uint8_t)esp_random();
  if (len < 126) { hdr[h++] = 0x80 | (uint8_t)len; }
  else if (len < 65536) { hdr[h++] = 0x80 | 126; hdr[h++] = (uint8_t)((len >> 8) & 0xFF); hdr[h++] = (uint8_t)(len & 0xFF); }
  else return false;
  memcpy(hdr + h, maskKey, 4); h += 4;
  if (aisSock.write(hdr, h) != (size_t)h) return false;
  uint8_t buf[256];
  size_t sent = 0;
  while (sent < len) {
    size_t n = len - sent; if (n > sizeof(buf)) n = sizeof(buf);
    for (size_t i = 0; i < n; i++) buf[i] = data[sent + i] ^ maskKey[(sent + i) & 3];
    if (aisSock.write(buf, n) != n) return false;
    sent += n;
  }
  return true;
}

static void wsMessageDone() {
  String plain;
  const String* msgp = &wsMsg;
  if (wsMsgCompressed) {
    if (!wsInflate((const uint8_t*)wsMsg.c_str(), wsMsg.length(), plain)) {
      wsMsg = "";
      return;
    }
    msgp = &plain;
  }
  parseAis(*msgp);
  wsMsg = "";
}

// Parse and dispatch every complete frame currently in wsBuf.
static void wsProcess() {
  for (;;) {
    if (wsBuf.length() < 2) return;
    const uint8_t* b = (const uint8_t*)wsBuf.c_str();
    bool fin = (b[0] & 0x80) != 0;
    bool rsv1 = (b[0] & 0x40) != 0;
    uint8_t op = b[0] & 0x0F;
    bool masked = (b[1] & 0x80) != 0;
    uint64_t len = b[1] & 0x7F;
    int hdrLen = 2;
    if (len == 126) {
      if (wsBuf.length() < 4) return;
      len = ((uint64_t)b[2] << 8) | b[3]; hdrLen = 4;
    } else if (len == 127) {
      if (wsBuf.length() < 10) return;
      len = 0; for (int i = 0; i < 8; i++) len = (len << 8) | b[2 + i];
      hdrLen = 10;
    }
    int maskOfs = hdrLen;
    if (masked) hdrLen += 4;
    if (len > 65536) { aisSock.stop(); return; }   // stream desync
    if ((uint64_t)wsBuf.length() < (uint64_t)hdrLen + len) return;   // incomplete: wait
    String payload;
    payload.reserve((unsigned int)len + 1);
    const uint8_t* p = b + hdrLen;
    if (masked) {
      const uint8_t* mk = b + maskOfs;
      for (uint64_t i = 0; i < len; i++) payload += (char)(p[i] ^ mk[i & 3]);
    } else {
      for (uint64_t i = 0; i < len; i++) payload += (char)p[i];
    }
    wsBuf.remove(0, (unsigned int)(hdrLen + (size_t)len));
    if (op == 0x9) { wsSendFrame(0xA, (const uint8_t*)payload.c_str(), payload.length()); continue; }
    if (op == 0xA) continue;
    if (op == 0x8) { aisSock.stop(); return; }
    if (op == 0x1 || op == 0x2) { wsMsg = payload; wsMsgCompressed = rsv1; wsMsgStarted = true; }
    else if (op == 0x0 && wsMsgStarted) { wsMsg += payload; }
    else continue;
    if (fin && wsMsgStarted) { wsMsgStarted = false; wsMessageDone(); }
  }
}

static const char WS_B64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
static void makeWsKey(char* out) {   // Sec-WebSocket-Key: base64 of 16 random bytes
  uint8_t r[16];
  for (int i = 0; i < 16; i++) r[i] = (uint8_t)esp_random();
  int o = 0;
  for (int i = 0; i < 16; i += 3) {
    int n = min(3, 16 - i);
    uint32_t v = ((uint32_t)r[i] << 16) | (n > 1 ? ((uint32_t)r[i + 1] << 8) : 0) | (n > 2 ? r[i + 2] : 0);
    out[o++] = WS_B64[(v >> 18) & 63];
    out[o++] = WS_B64[(v >> 12) & 63];
    out[o++] = n > 1 ? WS_B64[(v >> 6) & 63] : '=';
    out[o++] = n > 2 ? WS_B64[v & 63] : '=';
  }
  out[o] = 0;
}

static bool aisConnect() {
  if (mapChunk) { free(mapChunk); mapChunk = nullptr; }   // TLS handshake headroom
  aisSock.setInsecure();
  aisSock.setTimeout(10);
  if (!aisSock.connect("stream.aisstream.io", 443)) return false;
  char key[28];
  makeWsKey(key);
  String req;
  req.reserve(384);
  req += "GET /v0/stream HTTP/1.1\r\n";
  req += "Host: stream.aisstream.io\r\n";
  req += "Upgrade: websocket\r\n";
  req += "Connection: Upgrade\r\n";
  req += "Sec-WebSocket-Key: "; req += key; req += "\r\n";
  req += "Sec-WebSocket-Version: 13\r\n";
  req += "Sec-WebSocket-Extensions: permessage-deflate; client_max_window_bits\r\n";
  req += "User-Agent: APRSMap-ESP32\r\n\r\n";
  aisSock.print(req);
  String resp;
  uint32_t t0 = millis();
  while (millis() - t0 < 10000 && !resp.endsWith("\r\n\r\n")) {
    while (aisSock.available()) {
      resp += (char)aisSock.read();
      if (resp.length() > 4096) break;
    }
    if (!aisSock.connected() && !aisSock.available()) break;
    delay(2);
  }
  if (resp.indexOf(" 101") < 0) {
    aisSock.stop();
    return false;
  }
  aisDeflate = resp.indexOf("permessage-deflate") >= 0;
  aisSockUp = true;
  aisOnline = true;
  aisSubConfirmed = false;
  aisLastRxMs = millis();
  wsBuf = ""; wsMsg = ""; wsMsgStarted = false;
  wsInflateReset();
  sendAisSubscription();
  Serial.println("AIS connected");
  return true;
}

static void aisLoop() {
  if (cfgAisKey.length() < 10) return;   // no key set: AIS screen stays empty
  if (WiFi.status() != WL_CONNECTED) {
    if (aisSockUp) { aisSock.stop(); aisSockUp = false; aisOnline = false; }
    return;
  }
  uint32_t now = millis();
  if (!aisSockUp) {
    if (now >= aisNextTryMs) {
      if (!aisConnect()) aisNextTryMs = now + 15000;
    }
  } else if (!aisSock.connected()) {
    aisSockUp = false; aisOnline = false; aisSubConfirmed = false;
    aisNextTryMs = now + 5000;
    Serial.println("AIS disconnected");
  } else {
    int guard = 4096;
    while (aisSock.available() && guard-- > 0) {
      int c = aisSock.read();
      if (c < 0) break;
      wsBuf += (char)c;
      aisLastRxMs = now;
      if (wsBuf.length() > 16384) { aisSock.stop(); break; }
    }
    if (aisSockUp) wsProcess();
    if (aisSockUp && now - aisLastRxMs > 120000) {
      aisSock.stop(); aisSockUp = false; aisOnline = false;
      aisNextTryMs = now + 5000;
    }
    // If the confirmation never comes back, resend the subscription
    // (the server treats a resend as a harmless replacement).
    if (aisSockUp && !aisSubConfirmed && now - aisLastSubMs > 10000)
      sendAisSubscription();
    static uint32_t lastPingMs = 0;
    if (aisSockUp && now - lastPingMs > 30000) {
      lastPingMs = now;
      wsSendFrame(0x9, nullptr, 0);   // keepalive ping
    }
  }
}

// ---------------- touch ----------------
static uint32_t lastTouchMs = 0;

static void clampCenter() {
  const MapDef& m = MAP_LIST[curMap];
  double latSpan = viewLatSpan(), lonSpan = viewLonSpan();
  if (centerLat + latSpan / 2 > m.north) centerLat = m.north - latSpan / 2;
  if (centerLat - latSpan / 2 < m.south) centerLat = m.south + latSpan / 2;
  if (centerLon + lonSpan / 2 > m.east)  centerLon = m.east - lonSpan / 2;
  if (centerLon - lonSpan / 2 < m.west)  centerLon = m.west + lonSpan / 2;
}


// ---- screen switching ------------------------------------------------
static void switchMode(int m) {
  if (m == screenMode) return;
  modeCenterLat[screenMode] = centerLat;
  modeCenterLon[screenMode] = centerLon;
  modeZoom[screenMode] = zoomLevel;
  screenMode = m;
  curMap = MODE_MAP[m];
  centerLat = modeCenterLat[m];
  centerLon = modeCenterLon[m];
  zoomLevel = modeZoom[m];
  selected = -1;
  selAcFlight[0] = 0;
  selShipMmsi = 0;
  detailIsAc = false;
  detailOpen = false;
  listOpen = false;
  mapDirty = true;
  fullRedraw = true;
}

// ---- list touch -------------------------------------------------------
static void handleListTouch(int x, int y) {
  buildList();
  int pages = max(1, (listCount + LIST_ROWS - 1) / LIST_ROWS);
  if (y >= 218) {   // paging footer
    if (x < 100 && listPage > 0) listPage--;
    else if (x >= 220 && listPage < pages - 1) listPage++;
    fullRedraw = true;
    return;
  }
  if (y < LIST_Y0 - 4 && x >= 280) { listOpen = false; fullRedraw = true; return; }   // X
  int row = (y - LIST_Y0) / LIST_ROWH;
  if (row < 0 || row >= LIST_ROWS) return;
  int idx = listPage * LIST_ROWS + row;
  if (idx >= listCount) return;
  int id = listIdx[idx];
  listOpen = false;
  if (zoomLevel < 1) zoomLevel = 1;   // so the centre move is visible
  if (screenMode == 0) {
    selected = id;
    selAcFlight[0] = 0; detailIsAc = false;
    centerLat = stations[id].lat;
    centerLon = stations[id].lon;
    clampCenter();
    detailOpen = true;
  } else if (screenMode == 2) {
    selShipMmsi = ships[id].mmsi;
    selected = -1; selAcFlight[0] = 0; detailIsAc = false;
    centerLat = ships[id].lat;
    centerLon = ships[id].lon;
    clampCenter();
    detailOpen = true;
  } else {
    strncpy(selAcFlight, aircraft[id].flight, sizeof(selAcFlight) - 1);
    selAcFlight[sizeof(selAcFlight) - 1] = 0;
    selected = -1; detailIsAc = true;
    centerLat = aircraft[id].lat;
    centerLon = aircraft[id].lon;
    clampCenter();
    detailOpen = true;
    ensureRoute(selAcFlight);
  }
  mapDirty = true;
  fullRedraw = true;
}

static void handleTouch(int x, int y) {
  tapX = x; tapY = y; tapMs = millis(); tapShown = false;
  if (listOpen) {
    if (y < BAR_H) {   // bar stays live: tabs switch, LIST closes
      if (x >= 2 && x < 46) switchMode(0);
      else if (x >= 48 && x < 92) switchMode(1);
      else if (x >= 94 && x < 124) switchMode(2);
      else if (x >= 228 && x < 266) listOpen = false;
      fullRedraw = true;
      return;
    }
    handleListTouch(x, y);
    return;
  }
  if (detailOpen) { detailOpen = false; fullRedraw = true; return; }
  // bar: tabs, [LIST] [-] [+]
  if (y < BAR_H) {
    if (x >= 2 && x < 46) switchMode(0);
    else if (x >= 48 && x < 92) switchMode(1);
    else if (x >= 94 && x < 124) switchMode(2);
    else if (x >= 228 && x < 266) { listOpen = true; listPage = 0; }
    else if (x >= 269 && x < 292) { if (zoomLevel > 0) { zoomLevel--; clampCenter(); } }
    else if (x >= 295) { if (zoomLevel < maxZoomLevel()) { zoomLevel++; clampCenter(); } }
    mapDirty = true;
    fullRedraw = true;
    return;
  }
  // nearest visible object of the current screen within 16 px
  int best = -1, bestD = 16 * 16;
  if (screenMode == 0) {
    for (int i = 0; i < MAX_STATIONS; i++) {
      if (!stations[i].used) continue;
      int px, py;
      if (!geoToPx(stations[i].lat, stations[i].lon, px, py)) continue;
      int d = (px - x) * (px - x) + (py - y) * (py - y);
      if (d < bestD) { bestD = d; best = i; }
    }
  }
  int bestAc = -1, bestAcD = 16 * 16;
  if (screenMode == 1) {
    for (int i = 0; i < acCount; i++) {
      int px, py;
      if (!geoToPx(aircraft[i].lat, aircraft[i].lon, px, py)) continue;
      int d = (px - x) * (px - x) + (py - y) * (py - y);
      if (d < bestAcD) { bestAcD = d; bestAc = i; }
    }
  }
  int bestShip = -1, bestShipD = 16 * 16;
  if (screenMode == 2) {
    for (int i = 0; i < MAX_SHIPS; i++) {
      if (!ships[i].used || !ships[i].hasPos) continue;
      int px, py;
      if (!geoToPx(ships[i].lat, ships[i].lon, px, py)) continue;
      int d = (px - x) * (px - x) + (py - y) * (py - y);
      if (d < bestShipD) { bestShipD = d; bestShip = i; }
    }
  }
  if (best >= 0) {
    if (best == selected) {
      detailIsAc = false;
      detailOpen = true;
    } else {
      selected = best;
      selAcFlight[0] = 0; detailIsAc = false;
      centerLat = stations[best].lat;
      centerLon = stations[best].lon;
      if (zoomLevel < maxZoomLevel()) zoomLevel++;
      clampCenter();
    }
  } else if (bestAc >= 0) {
    // aircraft: straight to the detail panel (flight + route)
    strncpy(selAcFlight, aircraft[bestAc].flight, sizeof(selAcFlight) - 1);
    selAcFlight[sizeof(selAcFlight) - 1] = 0;
    selected = -1;
    detailIsAc = true;
    detailOpen = true;
    ensureRoute(selAcFlight);   // ~1 s on first tap per flight
  } else if (bestShip >= 0) {
    // ships move slowly: station-style - first tap centres + zooms,
    // a second tap on the same ship opens the detail panel
    if (selShipMmsi != 0 && ships[bestShip].mmsi == selShipMmsi) {
      detailOpen = true;
    } else {
      selShipMmsi = ships[bestShip].mmsi;
      selected = -1; selAcFlight[0] = 0; detailIsAc = false;
      centerLat = ships[bestShip].lat;
      centerLon = ships[bestShip].lon;
      if (zoomLevel < maxZoomLevel()) zoomLevel++;
      clampCenter();
    }
  } else if (x < 48 || x >= 320 - 48 || y < BAR_H + 40 || y >= 240 - 48) {
    // pan: tap any screen edge to shift the view a quarter screen that way
    int dx = (x >= 320 - 48) ? 1 : (x < 48 ? -1 : 0);
    int dy = (y >= 240 - 48) ? -1 : (y < BAR_H + 40 ? 1 : 0);
    centerLon += dx * viewLonSpan() * 0.25;
    centerLat += dy * viewLatSpan() * 0.25;
    clampCenter();
  } else if (selected >= 0 || selShipMmsi != 0 || zoomLevel != defaultZoomLevel()) {
    selected = -1; selAcFlight[0] = 0; selShipMmsi = 0; detailIsAc = false;
    zoomLevel = defaultZoomLevel();
    centerLat = (MAP_LIST[curMap].north + MAP_LIST[curMap].south) / 2;
    centerLon = (MAP_LIST[curMap].east + MAP_LIST[curMap].west) / 2;
  }
  mapDirty = true;
  fullRedraw = true;
}

static uint32_t lastPollMs = 0;
static bool touchWasDown = false;
static void touchLoop() {
  uint32_t now = millis();
  if (now - lastPollMs < 40) return;           // poll at ~25 Hz
  lastPollMs = now;
  touchRoute();                                // route VSPI to the touch pins
  TS_Point p = ts.getPoint();                  // direct poll - no PENIRQ gate
  if (p.z < 400) { touchWasDown = false; return; }
  // One action per press (v73 fix): while the finger stays down, do not
  // fire handleTouch again. A held press used to re-fire after the
  // 300 ms debounce at the SAME screen spot - but the first fire had
  // already re-centred the view, so the second fire missed the station
  // and fell into the reset branch (or opened and instantly closed the
  // detail panel). That made the second tap on a station unusable.
  if (touchWasDown) return;
  touchWasDown = true;
  if (now - lastTouchMs < 300) return;         // action debounce
  lastTouchMs = now;
  int x = constrain(map(p.x, 200, 3700, 0, 319), 0, 319);   // 2432S028 standard
  int y = constrain(map(p.y, 240, 3800, 0, 239), 0, 239);
  handleTouch(x, y);
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(2);   // new unit: ILI9342 panel, ROT 2 (diag v4/v5, 2026-11-15)
  tft.setBrightness(255);
  tft.fillScreen(TFT_BLACK);
  tft.setFont(&fonts::Font2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 10);
  tft.print("APRS Map Viewer 028 v81N");
  // v77 diagnostic: show WHY the previous boot ended, on the splash
  // itself and on serial. The user saw occasional reboots on station
  // taps with NOTHING on the serial port; a panic or watchdog would
  // print there, a silent reset points at power - this line settles
  // it without needing to capture serial at the right moment.
  {
    esp_reset_reason_t rr = esp_reset_reason();
    const char* rs = rr == ESP_RST_POWERON   ? "POWERON"   :
                     rr == ESP_RST_EXT       ? "EXT_PIN"   :
                     rr == ESP_RST_SW        ? "SW"        :
                     rr == ESP_RST_PANIC     ? "PANIC"     :
                     rr == ESP_RST_INT_WDT   ? "INT_WDT"   :
                     rr == ESP_RST_TASK_WDT  ? "TASK_WDT"  :
                     rr == ESP_RST_WDT       ? "WDT"       :
                     rr == ESP_RST_DEEPSLEEP ? "DEEPSLEEP" :
                     rr == ESP_RST_BROWNOUT  ? "BROWNOUT"  : "OTHER";
    Serial.printf("reset reason: %s (%d)\n", rs, (int)rr);
    tft.setFont(&fonts::Font0);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(10, 30);
    tft.printf("prev reset: %s", rs);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
  }

  touchSPI.begin(25, 39, 32, 33);
  ts.begin(touchSPI);
  ts.setRotation(1);

  // maps are embedded in flash: fixed 640x480 geometry, always available
  for (int i = 0; i < MAP_COUNT; i++) {
    bmpInfo[i].ok = true;
    bmpInfo[i].w = MAP_W;
    bmpInfo[i].h = MAP_H;
    bmpInfo[i].topDown = true;
  }
  sdOk = true;   // legacy flag: now means "maps available"
  // (mode centres are initialised after mapGenBoot(), once the final
  // map bounds - baked or generated - are known)

  // Phone-based setup (Plane Radar style): settings live in NVS and
  // config.h is only the factory default. Holding BOOT for 2 s after
  // the splash wipes them and forces the setup portal + help screen.
  loadSettings(false);
  bool forcePortal = bootResetWindow();
  if (forcePortal) loadSettings(true);
  wifiPhase(forcePortal);

  mapGenBoot();   // v79: activate generated maps, or generate them first
  mgFreeGenBuffers();   // covers mapGenBoot's early returns
  for (int i = 0; i < MODE_COUNT; i++) {
    const MapDef& mm = MAP_LIST[MODE_MAP[i]];
    modeCenterLat[i] = (mm.north + mm.south) / 2;
    modeCenterLon[i] = (mm.east + mm.west) / 2;
    modeZoom[i] = (i == 0) ? 1 : 0;   // APRS opens at x1
  }
  centerLat = modeCenterLat[0];
  centerLon = modeCenterLon[0];
  zoomLevel = modeZoom[0];
  // (no frame sprite on this board - see the composition note at G)

  aprsSetup();
  redraw();
}

static uint32_t lastBarMs = 0, lastPurgeMs = 0, lastSdRetryMs = 0, lastStationRedrawMs = 0;
static uint32_t lastMobileDrawMs = 0, lastMobileCheckMs = 0;
void loop() {
  if (cfgServerStarted) cfgServer.handleClient();   // LAN reconfigure page
  aprsLoop();
  aisLoop();
  touchLoop();
  uint32_t now = millis();
  if ((!acFetchedOnce || now - lastAcFetchMs > AC_POLL_MS) && WiFi.status() == WL_CONNECTED) {
    lastAcFetchMs = now;
    acFetchedOnce = true;
    fetchAircraft();
    stationsDirty = true;   // let the batched check redraw if anything moved
  }
  // NTP clock for the persisted station timestamps
  if (!ntpStarted && WiFi.status() == WL_CONNECTED) {
    configTime(0, 0, "pool.ntp.org", "ntp.nict.jp");
    ntpStarted = true;
  }
  if (!stRestored && ntpStarted) {
    if (time(nullptr) > 1700000000) { restoreStations(); stRestored = true; }
    else if (now > 150000) stRestored = true;   // no NTP: skip restore
  }
  if (stPersistDirty && now - lastPersistMs > 20000) {
    lastPersistMs = now;
    stPersistDirty = false;
    saveStations();
  }
  if (now - lastPurgeMs > 30000) {
    lastPurgeMs = now;
    bool changed = false;
    time_t nowT = time(nullptr);
    bool unixOk = nowT > 1700000000;
    for (int i = 0; i < MAX_STATIONS; i++) {
      if (!stations[i].used) continue;
      bool expired = (unixOk && stations[i].lastUnix)
          ? ((uint32_t)nowT - stations[i].lastUnix > (uint32_t)STATION_EXPIRE_MIN * 60)
          : (now - stations[i].lastMs > (uint32_t)STATION_EXPIRE_MIN * 60000UL);
      if (expired) {
        stations[i].used = false;
        if (selected == i) { selected = -1; detailOpen = false; }
        changed = true;
      }
    }
    for (int i = 0; i < MAX_SHIPS; i++) {
      if (!ships[i].used) continue;
      // ships go quiet for good once they leave the area / switch off
      bool expired = !ships[i].hasPos || (now - ships[i].lastMs > 30UL * 60000UL);
      if (expired) {
        if (selShipMmsi == ships[i].mmsi) { selShipMmsi = 0; detailOpen = false; }
        ships[i].used = false;
        changed = true;
      }
    }
    if (changed) { stationsDirty = true; stPersistDirty = true; }
  }
  if (fullRedraw) { redraw(); return; }
  if (stationsDirty && !detailOpen && now - lastStationRedrawMs > 3000) {
    lastStationRedrawMs = now;   // batch station updates: check at most 1x / 3 s
    if (stationsOnScreenChanged()) {
      if (screenMode == 0) lastMobileDrawMs = now;
      redraw(); return;
    }
    stationsDirty = false;       // nothing to show: skip the redraw
  }
  if (stationsDirty && detailOpen) stationsDirty = false;  // don't disturb the panel
  // mobile stations on the APRS screen: repaint once a minute
  if (screenMode == 0 && !detailOpen && now - lastMobileCheckMs > 5000) {
    lastMobileCheckMs = now;
    if (now - lastMobileDrawMs > 60000 && mobileOnScreenMoved()) {
      lastMobileDrawMs = now;
      redraw(); return;
    }
  }
  if (now - lastBarMs > 5000) { lastBarMs = now; if (!detailOpen) drawBarF(false, true); }
  // tap marker: shown/hidden through composed redraws (no direct draw)
  bool markerNow = (tapX >= 0 && now - tapMs < 500);
  if (markerNow != tapShown) {
    if (tapShown) markerRepair = true;
    tapShown = markerNow;
    fullRedraw = true;
  }
  delay(10);
}
