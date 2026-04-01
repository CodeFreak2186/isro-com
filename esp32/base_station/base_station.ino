/*
================================================================================
  Drone Communication System — ESP32 Base Station
================================================================================
  Hardware:
    • Any ESP32 dev board (ESP32-WROOM, NodeMCU-32, etc.)
    • SSD1306 OLED 128×64 connected via I2C

  Wiring:
    OLED VCC  →  3.3 V
    OLED GND  →  GND
    OLED SDA  →  GPIO 21   (hardware I2C SDA on most ESP32 boards)
    OLED SCL  →  GPIO 22   (hardware I2C SCL on most ESP32 boards)

  Libraries — install via Arduino IDE → Tools → Manage Libraries:
    • Adafruit SSD1306         (search "SSD1306 Adafruit")
    • Adafruit GFX Library     (installed automatically as a dependency)
    • ArduinoJson              by Benoit Blanchon  v7.x
    WiFi.h and HTTPClient.h come with the ESP32 Arduino core — no install needed.

  Board setting: ESP32 Dev Module  (or your specific board)
  Upload speed:  115200 / 921600   (either works)
================================================================================
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------------------------------------------------------------------------
// ★ CHANGE THESE before flashing ★
// ---------------------------------------------------------------------------
const char* WIFI_SSID     = "sam";
const char* WIFI_PASSWORD = "SANS2186";
const char* DATA_URL      = "https://isro-com.onrender.com/telemetry";
//                                   ^^^^^^^^^^^^^ your server IP

// How often to hit the server (milliseconds).
// 1000 ms = once per second — feels real-time on the display.
const unsigned long POLL_MS = 1000;

// ---------------------------------------------------------------------------
// OLED setup   128 × 64 pixels, I2C address 0x3C (most common)
// ---------------------------------------------------------------------------
#define SCREEN_W   128
#define SCREEN_H    64
#define OLED_ADDR  0x3C
#define OLED_RESET  -1    // share reset with ESP32 board

Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &Wire, OLED_RESET);

// ---------------------------------------------------------------------------
// Built-in LED (GPIO 2 on most dev boards) — used for low-battery warning
// ---------------------------------------------------------------------------
#define LED_PIN 2

// ---------------------------------------------------------------------------
// Globals — latest parsed telemetry
// ---------------------------------------------------------------------------
float  g_battery  = 0.0;
float  g_altitude = 0.0;
String g_status   = "---";
bool   g_hasData  = false;    // false until first successful fetch

unsigned long lastPollMs  = 0;
unsigned long lastBlinkMs = 0;
bool          ledState    = false;


// ===========================================================================
// OLED helpers
// ===========================================================================

// Draw a solid horizontal bar.
//   x, y  : top-left corner
//   w, h  : maximum width and height of the bar container
//   pct   : fill percentage 0–100
void drawBar(int x, int y, int w, int h, float pct) {
  int fill = (int)((pct / 100.0f) * (float)(w - 2));
  fill = constrain(fill, 0, w - 2);
  oled.drawRect(x, y, w, h, SSD1306_WHITE);         // outer border
  oled.fillRect(x + 1, y + 1, fill, h - 2, SSD1306_WHITE); // inner fill
}

// Show a splash screen while connecting to WiFi.
void splashScreen(const char* line1, const char* line2 = "") {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 20);
  oled.println(line1);
  oled.setCursor(0, 34);
  oled.println(line2);
  oled.display();
}

// Draw the full telemetry dashboard.
void drawDashboard() {
  oled.clearDisplay();

  // ── Title bar ─────────────────────────────────────────────────────────────
  oled.fillRect(0, 0, SCREEN_W, 11, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(1);
  oled.setCursor(22, 2);
  oled.print("DRONE  MONITOR");

  // Switch back to white text for the rest
  oled.setTextColor(SSD1306_WHITE);

  // ── Battery section ───────────────────────────────────────────────────────
  //  Row starts at y=14
  oled.setCursor(0, 14);
  oled.print("BAT:");

  // voltage number — slightly larger
  oled.setTextSize(1);
  oled.setCursor(28, 14);
  char batBuf[10];
  dtostrf(g_battery, 5, 2, batBuf);   // "12.34"
  oled.print(batBuf);
  oled.print("V");

  // Battery percentage bar  (10.5 V = 0%, 12.6 V = 100%)
  float batPct = ((g_battery - 10.5f) / 2.1f) * 100.0f;
  batPct = constrain(batPct, 0.0f, 100.0f);
  drawBar(0, 24, 80, 7, batPct);

  // Percentage text to the right of bar
  oled.setCursor(84, 24);
  oled.print((int)batPct);
  oled.print("%");

  // ── Altitude section ──────────────────────────────────────────────────────
  oled.setCursor(0, 35);
  oled.print("ALT:");
  oled.setCursor(28, 35);
  char altBuf[10];
  dtostrf(g_altitude, 5, 1, altBuf);  // " 12.5"
  oled.print(altBuf);
  oled.print(" m");

  // ── Status section ────────────────────────────────────────────────────────
  oled.setCursor(0, 46);
  oled.print("STS:");
  oled.setCursor(28, 46);
  // Capitalise first letter
  String disp = g_status;
  if (disp.length() > 0) disp[0] = toupper((unsigned char)disp[0]);
  oled.print(disp);

  // ── WiFi RSSI in bottom-right corner ──────────────────────────────────────
  oled.setCursor(84, 57);
  oled.setTextSize(1);
  int rssi = WiFi.RSSI();
  if (rssi > -60)       oled.print("WiFi:OK");
  else if (rssi > -80)  oled.print("WiFi:~");
  else                  oled.print("WiFi:!");

  // ── Bottom separator ──────────────────────────────────────────────────────
  oled.drawLine(0, 54, SCREEN_W, 54, SSD1306_WHITE);

  oled.display();
}

// Show an error screen when the server can't be reached.
void drawError(const char* msg) {
  oled.clearDisplay();
  oled.fillRect(0, 0, SCREEN_W, 11, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(1);
  oled.setCursor(28, 2);
  oled.print("CONNECTION");

  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 16);
  oled.println("Server unreachable.");
  oled.setCursor(0, 28);
  oled.println(msg);
  oled.setCursor(0, 44);
  oled.println("Retrying...");
  oled.display();
}


// ===========================================================================
// Setup
// ===========================================================================
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // ── I2C + OLED init ───────────────────────────────────────────────────────
  Wire.begin(21, 22);   // SDA=21, SCL=22
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[OLED] Init FAILED — check wiring/address!");
    // Halt here so the user knows something is wrong
    while (true) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(200);
    }
  }

  Serial.println("[OLED] Initialised OK");

  // ── Connect to WiFi ───────────────────────────────────────────────────────
  splashScreen("Connecting WiFi...", WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("[WiFi] Connecting to %s ", WIFI_SSID);

  int tries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    tries++;
    if (tries > 40) {
      Serial.println("\n[WiFi] Timeout — restarting...");
      splashScreen("WiFi failed!", "Restarting...");
      delay(2000);
      ESP.restart();
    }
  }

  Serial.println();
  Serial.print("[WiFi] Connected — IP: ");
  Serial.println(WiFi.localIP());

  char ipStr[24];
  WiFi.localIP().toString().toCharArray(ipStr, sizeof(ipStr));
  splashScreen("WiFi Connected!", ipStr);
  delay(1200);

  Serial.printf("[Server] Polling: %s every %lu ms\n", DATA_URL, POLL_MS);
}


// ===========================================================================
// Fetch data from server and update globals
// ===========================================================================
bool fetchData() {
  if (WiFi.status() != WL_CONNECTED) return false;

  HTTPClient http;
  http.begin(DATA_URL);
  http.setTimeout(4000);   // 4-second timeout

  int code = http.GET();

  if (code != HTTP_CODE_OK) {
    Serial.printf("[HTTP] Error: %d\n", code);
    http.end();
    return false;
  }

  String body = http.getString();
  http.end();

  // ── Parse JSON ────────────────────────────────────────────────────────────
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, body);
  if (err) {
    Serial.printf("[JSON] Parse error: %s\n", err.c_str());
    return false;
  }

  g_battery  = doc["battery_voltage"] | 0.0f;
  g_altitude = doc["altitude"]        | 0.0f;
  g_status   = (const char*)(doc["status"] | "unknown");
  g_hasData  = true;

  return true;
}


// ===========================================================================
// Low-battery LED blink (non-blocking)
//   < 11.0 V  →  fast blink
//   >= 11.0 V →  LED off
// ===========================================================================
void handleBatteryLed() {
  if (!g_hasData) return;

  if (g_battery > 0.0f && g_battery < 11.0f) {
    // Fast blink every 200 ms
    if (millis() - lastBlinkMs >= 200) {
      lastBlinkMs = millis();
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
    }
  } else {
    // Battery OK — LED off
    digitalWrite(LED_PIN, LOW);
    ledState = false;
  }
}


// ===========================================================================
// WiFi watchdog — auto-reconnect if dropped
// ===========================================================================
void maintainWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Lost connection — reconnecting...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    // Give it a moment; next loop will retry if still not connected
    delay(3000);
  }
}


// ===========================================================================
// Main loop
// ===========================================================================
void loop() {
  unsigned long now = millis();

  // ── Non-stop data fetch on interval ────────────────────────────────────
  if (now - lastPollMs >= POLL_MS) {
    lastPollMs = now;

    bool ok = fetchData();

    if (ok) {
      // Print to Serial as well — useful for debugging
      Serial.printf("[DATA] batt=%.2fV  alt=%.1fm  status=%s\n",
                    g_battery, g_altitude, g_status.c_str());
      drawDashboard();

    } else {
      // Show error on OLED
      char buf[32];
      snprintf(buf, sizeof(buf), "HTTP err / no WiFi");
      drawError(buf);
    }
  }

  // ── Battery LED ────────────────────────────────────────────────────────
  handleBatteryLed();

  // ── WiFi watchdog ──────────────────────────────────────────────────────
  maintainWiFi();
}
