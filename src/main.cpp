/*
 * ============================================================
 *  Dock Station — ESP-WROOM-32
 *  Sensors : DS18B20 (water temp) · DHT22 (air temp + humidity)
 *            HC-SR04 (water level)
 *  Comms   : WiFi (native) · MQTT → 164.92.68.188:1883
 *  OTA     : GitHub Releases via HTTPUpdate (HTTPS)
 * ============================================================
 *
 *  Wiring
 *  ──────
 *  DS18B20  DATA  → GPIO 4   (+ 4.7 kΩ pull-up to 3.3 V)
 *  DHT22    DATA  → GPIO 15  (+ 10 kΩ pull-up to 3.3 V)
 *  HC-SR04  TRIG  → GPIO 5
 *  HC-SR04  ECHO  → GPIO 18  (use voltage divider — 1kΩ + 2kΩ to GND)
 *
 *  MQTT Topics Published
 *  ─────────────────────
 *  home/dock/watertemp      – water temp °F         (float string, 2 dp)
 *  home/dock/airtemp        – air temp °F           (float string, 2 dp)
 *  home/dock/humidity       – relative humidity %   (float string, 2 dp)
 *  home/dock/level          – distance sensor → in  (float string, 2 dp)
 *  home/dock/data           – all four as JSON snapshot
 *  stations/DOCK1/status    – "online" / "offline"  (retained)
 *  stations/DOCK1/version   – firmware version      (retained)
 *  stations/DOCK1/ota/status – OTA progress/result  (JSON)
 *
 *  MQTT Topics Subscribed
 *  ──────────────────────
 *  stations/DOCK1/ota/cmd
 *    {"cmd":"update","url":"https://github.com/.../firmware.bin"}
 *    {"cmd":"reboot"}
 *
 *  GitHub OTA workflow
 *  ───────────────────
 *  1. Build in PlatformIO  →  .pio/build/esp32dev/firmware.bin
 *  2. Create a GitHub Release, attach firmware.bin as an asset
 *  3. Publish MQTT cmd with the direct asset download URL
 *  4. ESP32 fetches, flashes, reboots automatically
 *
 *  The GitHub root CA is embedded below. If it ever expires/rotates,
 *  update GITHUB_ROOT_CA with the new DigiCert cert (see note in code).
 *
 *  ⚠️  WiFi credentials live in secrets.h — DO NOT commit that file.
 *      Copy secrets.h.template → secrets.h and fill in your values.
 * ============================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include "secrets.h"   // WIFI_SSID, WIFI_PASSWORD — not committed to git

// ── Firmware identity ────────────────────────────────────────
#define STATION_ID      "DOCK1"
#define FW_VERSION      "1.0.0"

// ── MQTT broker ──────────────────────────────────────────────
#define MQTT_HOST       "164.92.68.188"
#define MQTT_PORT       1883
#define MQTT_CLIENT_ID  "dock-station-" STATION_ID

// ── MQTT topics ──────────────────────────────────────────────
#define TOPIC_WATER_T   "home/dock/watertemp"
#define TOPIC_AIR_T     "home/dock/airtemp"
#define TOPIC_HUMIDITY  "home/dock/humidity"
#define TOPIC_LEVEL     "home/dock/level"
#define TOPIC_DATA      "home/dock/data"
#define TOPIC_STATUS    "stations/" STATION_ID "/status"
#define TOPIC_VERSION   "stations/" STATION_ID "/version"
#define TOPIC_OTA_CMD   "stations/" STATION_ID "/ota/cmd"
#define TOPIC_OTA_STAT  "stations/" STATION_ID "/ota/status"

// ── Pin assignments ──────────────────────────────────────────
#define PIN_ONE_WIRE    4
#define PIN_DHT         13 // 15 
#define PIN_TRIG        5
#define PIN_ECHO        18

// ── Sensor config ────────────────────────────────────────────
#define DHT_TYPE        DHT22
#define SOUND_CM_US     0.01715f
#define CM_TO_INCH      0.393701f

// ── Timing ───────────────────────────────────────────────────
#define READ_INTERVAL_MS   15000UL
#define MQTT_RETRY_MS       5000UL
#define WIFI_RETRY_MS      10000UL

// ── GitHub root CA certificate ───────────────────────────────
// DigiCert Global Root CA — used by objects.githubusercontent.com
// To update: openssl s_client -connect objects.githubusercontent.com:443
//            -showcerts 2>/dev/null | openssl x509 -noout -text
// Expires: 2031-11-10
static const char GITHUB_ROOT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD
QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMB8GA1UdIwQYMBaA
FAPeUDVW0Uy7ZvCj4hsbw5eyPdFVMA8GA1UdEwEB/wQFMAMBAf8wDgYDVR0PAQH/
BAQDAgGGMA0GCSqGSIb3DQEBBQUAA4IBAQBs1HZEUO5pQMiXR8V8+J/LHbYnMEW+
bnQlYqZJXrjL7ceMFRkJGFvW3TkWXW+wTiFMpU8ACzL6ESYiS73c/sVzMOOYlgMX
MRPTasVAGvHpjdH8nU0CZqOkLmfV7N6S6rHvj0FXKW0oHNOqJPrFkYDSgBhAVe9
Z6k43Y7KmNYlBHB9Kj5QRYocNakSOmWrFOe7HXQP3WkZWS7bUwPyLVhLNYVXMQI
vJoFnAqB5N4AVzMC1ZRtxRkMiKWzCB0s0kE7ZZNQ5Q5vFwjt5VsXbdSPxQdLj8r
Hmjz6A2S0LVmkf6AZr7CXL8iCFOJnbvW5ARe0EFBhYyvRMGHCQ0=
-----END CERTIFICATE-----
)EOF";

// ── Objects ──────────────────────────────────────────────────
OneWire           oneWire(PIN_ONE_WIRE);
DallasTemperature ds18b20(&oneWire);
DHT               dht(PIN_DHT, DHT_TYPE);
WiFiClient        wifiClient;
PubSubClient      mqtt(wifiClient);

// ── State ────────────────────────────────────────────────────
unsigned long lastReadMs  = 0;
unsigned long lastMqttMs  = 0;
unsigned long lastWifiMs  = 0;
bool          otaActive   = false;

// ── Pending OTA URL (set in MQTT callback, executed in loop) ─
String        pendingOtaUrl = "";

// ─────────────────────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────────────────────
void publishStr(const char* topic, const char* payload, bool retained = false) {
    mqtt.publish(topic, payload, retained);
}

void publishFloat(const char* topic, float value) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%.2f", value);
    mqtt.publish(topic, buf);
}

// ─────────────────────────────────────────────────────────────
//  HC-SR04 — returns distance cm, or -1.0 on timeout
// ─────────────────────────────────────────────────────────────
float readUltrasonicCm() {
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);
    long duration = pulseIn(PIN_ECHO, HIGH, 30000UL);
    if (duration == 0) return -1.0f;
    return duration * SOUND_CM_US;
}

// ─────────────────────────────────────────────────────────────
//  GitHub OTA update
//  Called from loop() so MQTT callback stays non-blocking
// ─────────────────────────────────────────────────────────────
void performOtaUpdate(const String& url) {
    otaActive = true;
    Serial.println("[OTA] Starting GitHub update...");
    Serial.println(url);

    publishStr(TOPIC_OTA_STAT, "{\"status\":\"downloading\"}");
    mqtt.loop();

    WiFiClientSecure secureClient;
    secureClient.setCACert(GITHUB_ROOT_CA);

    httpUpdate.onProgress([](int cur, int total) {
        static int lastPct = -1;
        int pct = (total > 0) ? (cur * 100 / total) : 0;
        if (pct != lastPct && pct % 10 == 0) {
            Serial.printf("[OTA] %d%%\n", pct);
            lastPct = pct;
        }
    });

    t_httpUpdate_return result = httpUpdate.update(secureClient, url);

    switch (result) {
        case HTTP_UPDATE_FAILED:
            Serial.printf("[OTA] Failed: %s\n", httpUpdate.getLastErrorString().c_str());
            publishStr(TOPIC_OTA_STAT, "{\"status\":\"failed\"}");
            otaActive = false;
            break;
        case HTTP_UPDATE_NO_UPDATES:
            Serial.println("[OTA] No update available");
            publishStr(TOPIC_OTA_STAT, "{\"status\":\"no_update\"}");
            otaActive = false;
            break;
        case HTTP_UPDATE_OK:
            Serial.println("[OTA] Success — rebooting");
            break;
    }
}

// ─────────────────────────────────────────────────────────────
//  WiFi
// ─────────────────────────────────────────────────────────────
void connectWiFi() {
    if (WiFi.status() == WL_CONNECTED) return;
    unsigned long now = millis();
    if (now - lastWifiMs < WIFI_RETRY_MS) return;
    lastWifiMs = now;

    Serial.print("[WiFi] Connecting to ");
    Serial.println(WIFI_SSID);
    WiFi.disconnect(true);
    delay(100);
    WiFi.mode(WIFI_STA);
    delay(100);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    uint8_t attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500);
        Serial.print('.');
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("\n[WiFi] Connected — IP: ");
        Serial.println(WiFi.localIP());
        delay(1000);
    } else {
        Serial.println("\n[WiFi] Failed — will retry");
    }
}

// ─────────────────────────────────────────────────────────────
//  MQTT callback
// ─────────────────────────────────────────────────────────────
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    if (strcmp(topic, TOPIC_OTA_CMD) != 0) return;

    StaticJsonDocument<256> doc;
    if (deserializeJson(doc, payload, length) != DeserializationError::Ok) return;

    const char* cmd = doc["cmd"];
    if (!cmd) return;

    if (strcmp(cmd, "update") == 0) {
        const char* url = doc["url"];
        if (!url) {
            publishStr(TOPIC_OTA_STAT, "{\"status\":\"error\",\"msg\":\"no url\"}");
            return;
        }
        pendingOtaUrl = String(url);
        publishStr(TOPIC_OTA_STAT, "{\"status\":\"pending\"}");
        Serial.print("[OTA] Update queued: ");
        Serial.println(url);
    }

    if (strcmp(cmd, "reboot") == 0) {
        publishStr(TOPIC_OTA_STAT, "{\"status\":\"rebooting\"}");
        delay(300);
        ESP.restart();
    }
}

// ─────────────────────────────────────────────────────────────
//  MQTT connect / reconnect
// ─────────────────────────────────────────────────────────────
void connectMQTT() {
    if (mqtt.connected()) return;
    unsigned long now = millis();
    if (now - lastMqttMs < MQTT_RETRY_MS) return;
    lastMqttMs = now;

    Serial.print("[MQTT] Connecting...");
    bool ok = mqtt.connect(
        MQTT_CLIENT_ID,
        nullptr, nullptr,
        TOPIC_STATUS, 1, true,
        "offline"
    );

    if (ok) {
        Serial.println(" connected");
        publishStr(TOPIC_STATUS,  "online",   true);
        publishStr(TOPIC_VERSION, FW_VERSION, true);
        mqtt.subscribe(TOPIC_OTA_CMD);
    } else {
        Serial.print(" failed, rc=");
        Serial.println(mqtt.state());
    }
}

// ─────────────────────────────────────────────────────────────
//  Read & publish all sensors
// ─────────────────────────────────────────────────────────────
void readAndPublish() {
    float waterTempF = NAN;
    // float waterTempC = NAN;
    float airTempF   = NAN;
    float humidity   = NAN;
    float distInch   = NAN;

    // ── DS18B20 water temp ────────────────────────────────────
    ds18b20.requestTemperatures();
    float waterTempC = ds18b20.getTempCByIndex(0);
    Serial.printf("[DS18B20] Raw temp C: %.4f\n", waterTempC);
    Serial.printf("[DS18B20] Device count: %d\n", ds18b20.getDeviceCount());
    if (waterTempC != DEVICE_DISCONNECTED_C) {
        waterTempF = (waterTempC * 9.0f / 5.0f) + 32.0f;
        publishFloat(TOPIC_WATER_T, waterTempF);
        Serial.printf("[DS18B20] Water: %.2f °F\n", waterTempF);
    } else {
        Serial.println("[DS18B20] Sensor error / disconnected");
    }

    // ── DHT22 air temp + humidity ─────────────────────────────
    float airTempC = dht.readTemperature();
    humidity       = dht.readHumidity();
    Serial.printf("[DHT22] Raw tempC: %.4f  Raw humidity: %.4f\n", airTempC, humidity);
    if (!isnan(airTempC) && !isnan(humidity)) {
        airTempF = (airTempC * 9.0f / 5.0f) + 32.0f;
        publishFloat(TOPIC_AIR_T,    airTempF);
        publishFloat(TOPIC_HUMIDITY, humidity);
        Serial.printf("[DHT22]   Air: %.2f °F  Humidity: %.1f%%\n", airTempF, humidity);
    } else {
        Serial.println("[DHT22] Read failed");
    }

    // ── HC-SR04 water level ──────────────────────────────────
    float distCm = readUltrasonicCm();
    if (distCm > 0) {
        distInch = distCm * CM_TO_INCH;
        publishFloat(TOPIC_LEVEL, distInch);
        Serial.printf("[HC-SR04] Level: %.2f in (%.1f cm)\n", distInch, distCm);
    } else {
        Serial.println("[HC-SR04] Echo timeout");
    }

    // ── JSON summary ─────────────────────────────────────────
    StaticJsonDocument<128> doc;
    if (!isnan(waterTempF)) doc["watertemp"] = serialized(String(waterTempF, 2));
    if (!isnan(airTempF))   doc["airtemp"]   = serialized(String(airTempF,   2));
    if (!isnan(humidity))   doc["humidity"]  = serialized(String(humidity,   2));
    if (!isnan(distInch))   doc["level"]     = serialized(String(distInch,   2));

    char jsonBuf[128];
    serializeJson(doc, jsonBuf, sizeof(jsonBuf));
    mqtt.publish(TOPIC_DATA, jsonBuf);
    Serial.printf("[JSON]    %s\n", jsonBuf);
}

// ─────────────────────────────────────────────────────────────
//  setup()
// ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== Dock Station " STATION_ID " — FW " FW_VERSION " ===");

    ds18b20.begin();
    Serial.printf("[DS18B20] Devices found on bus: %d\n", ds18b20.getDeviceCount());
    dht.begin();
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
    digitalWrite(PIN_TRIG, LOW);

    connectWiFi();

    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setCallback(mqttCallback);
    mqtt.setBufferSize(512);
    connectMQTT();
}

// ─────────────────────────────────────────────────────────────
//  loop()
// ─────────────────────────────────────────────────────────────
void loop() {
    if (pendingOtaUrl.length() > 0) {
        String url = pendingOtaUrl;
        pendingOtaUrl = "";
        performOtaUpdate(url);
        return;
    }

    if (otaActive) return;

    if (WiFi.status() != WL_CONNECTED) {
        connectWiFi();
        return;
    }
    connectMQTT();
    mqtt.loop();

    unsigned long now = millis();
    if (now - lastReadMs >= READ_INTERVAL_MS) {
        lastReadMs = now;
        if (mqtt.connected()) readAndPublish();
    }
}
