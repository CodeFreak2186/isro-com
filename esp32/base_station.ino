/*
================================================================================
  Drone Base Station — ESP32
  Polls /data from the server non-stop, shows battery voltage on OLED.
================================================================================
  Wiring:
    OLED VCC  →  3.3V
    OLED GND  →  GND
    OLED SDA  →  GPIO 21
    OLED SCL  →  GPIO 22

  Libraries (Arduino IDE → Manage Libraries):
    • Adafruit SSD1306
    • Adafruit GFX Library
    • ArduinoJson  (v7.x by Benoit Blanchon)
================================================================================
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ── Change these ─────────────────────────────────────────────────────────────
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* SERVER_URL    = "http://192.168.1.100:5000/data";
// ─────────────────────────────────────────────────────────────────────────────

#define SCREEN_W  128
#define SCREEN_H   64
#define OLED_ADDR 0x3C

Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &Wire, -1);

// latest battery value received from server
float  g_voltage    = 0.0;
bool   g_hasData    = false;
String g_fetchError = "";

unsigned long lastPoll = 0;
const unsigned long POLL_MS = 1000;   // fetch every 1 second


// ── Draw everything on the OLED ───────────────────────────────────────────────
void drawScreen() {
  oled.clearDisplay();

  // ── Header bar ───────────────────────────────────────────────────────────
  oled.fillRoundRect(0, 0, SCREEN_W, 14, 3, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(1);
  oled.setCursor(22, 3);
  oled.print("DRONE  BATTERY");
  oled.setTextColor(SSD1306_WHITE);

  if (!g_hasData) {
    // ── Waiting / error state ───────────────────────────────────────────────
    oled.setCursor(0, 22);
    oled.setTextSize(1);
    oled.println("Connecting to");
    oled.println("server...");
    if (g_fetchError.length()) {
      oled.setCursor(0, 48);
      oled.print(g_fetchError.substring(0, 21));
    }
    oled.display();
    return;
  }

  // ── Large voltage number ──────────────────────────────────────────────────
  oled.setTextSize(3);
  oled.setCursor(8, 18);
  char buf[8];
  dtostrf(g_voltage, 5, 2, buf);   // e.g. "12.34"
  oled.print(buf);

  oled.setTextSize(2);
  oled.setCursor(104, 24);
  oled.print("V");

  // ── Percentage bar (10.5 V = 0%, 12.6 V = 100%) ──────────────────────────
  float pct = ((g_voltage - 10.5f) / 2.1f) * 100.0f;
  pct = constrain(pct, 0.0f, 100.0f);

  // bar outline
  oled.drawRoundRect(0, 48, SCREEN_W, 12, 3, SSD1306_WHITE);
  // bar fill
  int fill = (int)((pct / 100.0f) * (float)(SCREEN_W - 4));
  fill = constrain(fill, 0, SCREEN_W - 4);
  oled.fillRoundRect(2, 50, fill, 8, 2, SSD1306_WHITE);

  // percentage text inside / next to bar
  oled.setTextSize(1);
  char pctBuf[6];
  snprintf(pctBuf, sizeof(pctBuf), "%d%%", (int)pct);
  // position it centred over the bar
  int txtX = (SCREEN_W - strlen(pctBuf) * 6) / 2;
  // draw black text on white fill if fill is wide enough, else white on black
  if (fill > txtX + (int)(strlen(pctBuf) * 6)) {
    oled.setTextColor(SSD1306_BLACK);
  } else {
    oled.setTextColor(SSD1306_WHITE);
  }
  oled.setCursor(txtX, 51);
  oled.print(pctBuf);

  oled.display();
}


// ── Fetch battery from server ─────────────────────────────────────────────────
void fetchBattery() {
  if (WiFi.status() != WL_CONNECTED) {
    g_fetchError = "No WiFi";
    return;
  }

  HTTPClient http;
  http.begin(SERVER_URL);
  http.setTimeout(3000);

  int code = http.GET();

  if (code != HTTP_CODE_OK) {
    g_fetchError = "HTTP " + String(code);
    http.end();
    return;
  }

  String body = http.getString();
  http.end();

  JsonDocument doc;
  if (deserializeJson(doc, body)) {
    g_fetchError = "JSON err";
    return;
  }

  g_voltage    = doc["battery_voltage"] | 0.0f;
  g_hasData    = true;
  g_fetchError = "";

  Serial.printf("[BATTERY] %.2f V  (%.0f%%)\n",
    g_voltage,
    constrain(((g_voltage - 10.5f) / 2.1f) * 100.0f, 0.0f, 100.0f));
}


// ── WiFi auto-reconnect ───────────────────────────────────────────────────────
void maintainWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.println("[WiFi] Reconnecting...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  delay(3000);
}


// ═════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  // OLED
  Wire.begin(21, 22);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init failed!");
    while (true) delay(500);
  }

  // Splash
  oled.clearDisplay();
  oled.fillRoundRect(0, 0, SCREEN_W, 14, 3, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(1);
  oled.setCursor(22, 3);
  oled.print("DRONE  BATTERY");
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(20, 22);
  oled.setTextSize(1);
  oled.println("Connecting WiFi");
  oled.setCursor(28, 36);
  oled.print(WIFI_SSID);
  oled.display();

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("Connecting to %s", WIFI_SSID);
  int t = 0;
  while (WiFi.status() != WL_CONNECTED && t < 40) {
    delay(500); Serial.print("."); t++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nWiFi failed — will retry in loop");
  }
}


void loop() {
  unsigned long now = millis();

  if (now - lastPoll >= POLL_MS) {
    lastPoll = now;
    fetchBattery();
    drawScreen();
  }

  maintainWiFi();
}
