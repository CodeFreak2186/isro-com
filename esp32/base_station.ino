/*
================================================================================
  Drone Base Station — ESP32
  • Polls /data from the server every second
  • Displays battery voltage + charge bar on SSD1306 OLED
  • Shows "IMAGE UPLOADING..." screen when website sends an image to the drone
================================================================================
  Wiring (I2C):
    OLED VCC  →  3.3 V
    OLED GND  →  GND
    OLED SDA  →  GPIO 21
    OLED SCL  →  GPIO 22

  Libraries — install via Arduino IDE → Tools → Manage Libraries:
    • Adafruit SSD1306        (search "Adafruit SSD1306")
    • Adafruit GFX Library   (installed as a dependency)
    • ArduinoJson            (v7.x — search "ArduinoJson Blanchon")

  Board: ESP32 Dev Module
================================================================================
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ── ★ Change these before flashing ★ ─────────────────────────────────────────
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* DATA_URL      = "http://192.168.1.100:5000/data";
//                                    ^^^^^^^^^^^^^ your server IP or Render URL
// ─────────────────────────────────────────────────────────────────────────────

// OLED
#define SCREEN_W    128
#define SCREEN_H     64
#define OLED_ADDR   0x3C
#define SDA_PIN      21
#define SCL_PIN      22

Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &Wire, -1);

// Poll interval
const unsigned long POLL_MS = 1000;   // 1 second

// ── Runtime state ─────────────────────────────────────────────────────────────
float  g_voltage        = 0.0f;
bool   g_hasData        = false;
bool   g_imageUploading = false;
String g_error          = "";

unsigned long g_lastPoll    = 0;
unsigned long g_lastBlink   = 0;
bool          g_blinkState  = false;


// ═════════════════════════════════════════════════════════════════════════════
//  OLED SCREENS
// ═════════════════════════════════════════════════════════════════════════════

// ── Shared header bar ─────────────────────────────────────────────────────────
void drawHeader(const char* title) {
  oled.fillRect(0, 0, SCREEN_W, 13, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(1);
  // Centre the title
  int16_t x1, y1;
  uint16_t tw, th;
  oled.getTextBounds(title, 0, 0, &x1, &y1, &tw, &th);
  oled.setCursor((SCREEN_W - tw) / 2, 3);
  oled.print(title);
  oled.setTextColor(SSD1306_WHITE);
}

// ── Waiting / error screen ────────────────────────────────────────────────────
void drawWaiting() {
  oled.clearDisplay();
  drawHeader("DRONE BATTERY");

  oled.setTextSize(1);
  oled.setCursor(4, 18);
  oled.println("Connecting to");
  oled.setCursor(4, 28);
  oled.println("server...");

  if (g_error.length() > 0) {
    oled.setCursor(0, 50);
    oled.setTextSize(1);
    // Truncate to fit screen (21 chars max at size 1)
    String errLine = g_error.substring(0, 21);
    oled.print(errLine);
  }

  oled.display();
}

// ── Image uploading screen ────────────────────────────────────────────────────
void drawImageUploading() {
  oled.clearDisplay();
  drawHeader("DRONE BATTERY");

  // Blinking upload icon area
  oled.setTextSize(1);
  oled.setCursor(4, 18);
  oled.println("Image uploading");
  oled.setCursor(4, 28);
  oled.println("to drone...");

  // Animated dots — blink every 400 ms
  if (millis() - g_lastBlink >= 400) {
    g_lastBlink  = millis();
    g_blinkState = !g_blinkState;
  }
  if (g_blinkState) {
    // Draw a simple upload arrow  ↑  using lines
    oled.drawLine(64, 52, 64, 40, SSD1306_WHITE);  // stem
    oled.drawLine(64, 40, 58, 46, SSD1306_WHITE);  // left wing
    oled.drawLine(64, 40, 70, 46, SSD1306_WHITE);  // right wing
    oled.drawLine(56, 54, 72, 54, SSD1306_WHITE);  // base line
  }

  oled.display();
}

// ── Main battery screen ───────────────────────────────────────────────────────
void drawBattery() {
  oled.clearDisplay();
  drawHeader("DRONE BATTERY");

  // ── Battery percentage ────────────────────────────────────────────────────
  // Voltage range: 10.5 V = 0%, 12.6 V = 100%
  float pct = ((g_voltage - 10.5f) / 2.1f) * 100.0f;
  pct = constrain(pct, 0.0f, 100.0f);

  // ── Large voltage number  e.g. "12.34 V" ─────────────────────────────────
  char voltBuf[8];
  dtostrf(g_voltage, 5, 2, voltBuf);  // "12.34" — always 5 chars wide

  oled.setTextSize(2);
  // Calculate width to centre it
  int16_t x1, y1;
  uint16_t tw, th;
  oled.getTextBounds(voltBuf, 0, 0, &x1, &y1, &tw, &th);
  oled.setCursor((SCREEN_W - tw - 12) / 2, 16);   // leave room for "V"
  oled.print(voltBuf);

  // "V" unit — smaller
  oled.setTextSize(1);
  oled.setCursor((SCREEN_W - tw - 12) / 2 + tw + 2, 22);
  oled.print("V");

  // ── Percentage label ──────────────────────────────────────────────────────
  char pctBuf[8];
  snprintf(pctBuf, sizeof(pctBuf), "%d%%", (int)pct);
  oled.setTextSize(1);
  oled.getTextBounds(pctBuf, 0, 0, &x1, &y1, &tw, &th);
  oled.setCursor((SCREEN_W - tw) / 2, 36);
  oled.print(pctBuf);

  // ── Charge bar  ───────────────────────────────────────────────────────────
  // Outline
  oled.drawRoundRect(4, 47, SCREEN_W - 8, 13, 3, SSD1306_WHITE);

  // Fill — proportional to percentage
  int fillW = (int)(((float)(SCREEN_W - 12)) * pct / 100.0f);
  fillW = constrain(fillW, 0, SCREEN_W - 12);
  if (fillW > 0) {
    oled.fillRoundRect(6, 49, fillW, 9, 2, SSD1306_WHITE);
  }

  oled.display();
}


// ═════════════════════════════════════════════════════════════════════════════
//  HTTP FETCH
// ═════════════════════════════════════════════════════════════════════════════
void fetchData() {
  if (WiFi.status() != WL_CONNECTED) {
    g_error   = "No WiFi";
    g_hasData = false;
    return;
  }

  HTTPClient http;
  http.begin(DATA_URL);
  http.setTimeout(3000);   // 3-second timeout

  int code = http.GET();

  if (code != HTTP_CODE_OK) {
    g_error   = "Server err: " + String(code);
    g_hasData = false;
    http.end();
    return;
  }

  String body = http.getString();
  http.end();

  // ── Parse JSON ─────────────────────────────────────────────────────────────
  JsonDocument doc;
  DeserializationError jsonErr = deserializeJson(doc, body);
  if (jsonErr) {
    g_error   = "JSON: " + String(jsonErr.c_str());
    g_hasData = false;
    return;
  }

  g_voltage        = doc["battery_voltage"] | 0.0f;
  g_imageUploading = doc["image_uploading"]  | false;
  g_hasData        = true;
  g_error          = "";

  float pct = constrain(((g_voltage - 10.5f) / 2.1f) * 100.0f, 0.0f, 100.0f);
  Serial.printf("[DATA] Batt: %.2f V  (%.0f%%)  imgUpload: %s\n",
                g_voltage, pct, g_imageUploading ? "YES" : "no");
}


// ═════════════════════════════════════════════════════════════════════════════
//  WiFi — non-blocking reconnect
// ═════════════════════════════════════════════════════════════════════════════
void maintainWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.println("[WiFi] Lost — reconnecting...");
  WiFi.disconnect(true);
  delay(100);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Wait up to 8 s without blocking the full loop
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 8000) {
    delay(250);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[WiFi] Reconnected: " + WiFi.localIP().toString());
  } else {
    Serial.println("[WiFi] Still offline — will retry next loop.");
  }
}


// ═════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[BOOT] Drone Base Station starting...");

  // ── I2C + OLED ─────────────────────────────────────────────────────────────
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[ERROR] OLED not found — check wiring!");
    // Blink built-in LED so user knows something is wrong
    pinMode(2, OUTPUT);
    while (true) {
      digitalWrite(2, HIGH); delay(150);
      digitalWrite(2, LOW);  delay(150);
    }
  }
  Serial.println("[OLED] OK");

  // ── Splash screen ──────────────────────────────────────────────────────────
  oled.clearDisplay();
  drawHeader("DRONE BATTERY");
  oled.setTextSize(1);
  oled.setCursor(4, 18);
  oled.print("Connecting to WiFi");
  oled.setCursor(4, 30);
  oled.print(WIFI_SSID);
  oled.display();

  // ── WiFi ───────────────────────────────────────────────────────────────────
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[WiFi] Connected: " + WiFi.localIP().toString());
    oled.clearDisplay();
    drawHeader("DRONE BATTERY");
    oled.setTextSize(1);
    oled.setCursor(4, 20);
    oled.print("WiFi connected!");
    oled.setCursor(4, 32);
    oled.print(WiFi.localIP().toString());
    oled.display();
    delay(1500);
  } else {
    Serial.println("[WiFi] Failed — will retry in loop.");
  }
}


// ═════════════════════════════════════════════════════════════════════════════
//  LOOP
// ═════════════════════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  // ── Fetch + draw every POLL_MS ────────────────────────────────────────────
  if (now - g_lastPoll >= POLL_MS) {
    g_lastPoll = now;

    fetchData();

    if (!g_hasData) {
      drawWaiting();
    } else if (g_imageUploading) {
      drawImageUploading();
    } else {
      drawBattery();
    }
  }

  // ── Keep image uploading animation smooth (redraws between polls) ─────────
  if (g_hasData && g_imageUploading) {
    drawImageUploading();
  }

  // ── WiFi watchdog ─────────────────────────────────────────────────────────
  maintainWiFi();
}
