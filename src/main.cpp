/*
 * ============================================================
 *  Dock Station — ESP-WROOM-32
 *  Sensors : DS18B20 x2 (water temp + air temp, shared OneWire bus)
 *            XKC-KL200-2M-UART LIDAR (water level)
 *  Comms   : WiFi (native) · MQTT → 164.92.68.188:1883
 *  OTA     : GitHub Releases via HTTPUpdate (HTTPS)
 * ============================================================
 *
 *  Wiring
 *  ──────
 *  DS18B20 x2  DATA → GPIO 4   (shared OneWire bus + shared pull-up
 *                                on breakout board, both probes'
 *                                wires land in the same terminals)
 *                     9/3/26   After much debugging, it was determined
 *                                that GPIO4 was fried. We swapped to GPIO13
 *                                and all OneWire sensors worked fine.
 * 
 *  LIDAR   TXD (Yellow) → GPIO 18  (ESP32 RX, via voltage divider
 *                                    down to 3.3V — LIDAR TXD is
 *                                    open-collector, pulled to 5V)
 *  LIDAR   RXD (Black)  → GPIO 5   (ESP32 TX, straight 3.3V logic,
 *                                    LIDAR RX accepts 3.3V directly)
 *  LIDAR   VCC (Brown)  → 5V (from ESP32 expansion board)
 *  LIDAR   GND (Blue)   → GND
 *
 *  Note: GPIO19-as-soft-GND trick from the old HC-SR04 wiring is
 *  NOT used here — not needed with this connector layout.
 *
 *  MQTT Topics Published
 *  ─────────────────────
 *  home/dock/watertemp      – water temp °F         (float string, 2 dp)
 *  home/dock/airtemp        – air temp °F           (float string, 2 dp)
 *  home/dock/level          – LIDAR-derived level → in (float string, 2 dp)
 *  home/dock/data           – all three as JSON snapshot
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
#include "secrets.h"   // WIFI_SSID, WIFI_PASSWORD — not committed to git
#include <esp_task_wdt.h>
#include "esp_crt_bundle.h"
extern const uint8_t rootca_crt_bundle_start[] asm("_binary_x509_crt_bundle_start"); // added this to replace fixed CA.

// ── Firmware identity ────────────────────────────────────────
#define STATION_ID      "DOCK1"
//#define FW_VERSION      "2.0.0"   // bumped: LIDAR + dual-DS18B20 rewrite
//#define FW_VERSION      "2.0.1"   // bumped: spillway/sensor calibration update
//#define FW_VERSION      "2.0.2"   // bumped: 9/8/26 new CA certificate.
//#define FW_VERSION      "2.0.3"   // bumped: 9/8/26 deleted CA, replaced with 'rootca_crt_bundle_start'.
#define FW_VERSION      "2.0.4"   // bumped: 9/8/26 deleted CA, replaced with 'rootca_crt_bundle_start'.


// ── MQTT broker ──────────────────────────────────────────────
#define MQTT_HOST       "164.92.68.188"
#define MQTT_PORT       1883
#define MQTT_CLIENT_ID  "dock-station-" STATION_ID

// ── MQTT topics ──────────────────────────────────────────────
#define TOPIC_WATER_T   "home/dock/watertemp"
#define TOPIC_AIR_T     "home/dock/airtemp"
#define TOPIC_LEVEL     "home/dock/level"
#define TOPIC_DATA      "home/dock/data"
#define TOPIC_STATUS    "stations/" STATION_ID "/status"
#define TOPIC_VERSION   "stations/" STATION_ID "/version"
#define TOPIC_OTA_CMD   "stations/" STATION_ID "/ota/cmd"
#define TOPIC_OTA_STAT  "stations/" STATION_ID "/ota/status"

// ── Pin assignments ──────────────────────────────────────────
#define PIN_ONE_WIRE    13 //4
#define PIN_LIDAR_RX    18   // ESP32 RX ← LIDAR TXD (via divider)
#define PIN_LIDAR_TX    5    // ESP32 TX → LIDAR RXD (direct 3.3V)
#define LIDAR_BAUD      9600

// ── DS18B20 probe addresses (hardcoded — bus order is not reliable) ──
// Confirmed via ds18b20_address_scanner.ino
DeviceAddress airProbeAddr   = { 0x28, 0x68, 0xA8, 0x81, 0xE3, 0xDF, 0x3C, 0x59 };
DeviceAddress waterProbeAddr = { 0x28, 0x4B, 0x3F, 0xA9, 0x9E, 0x23, 0x0B, 0x2A };
#define DS18B20_RESOLUTION_BITS 9   // whole-degree resolution, ~94ms conversion

// ── Sensor config ────────────────────────────────────────────
#define MM_TO_INCH      0.0393701f
#define LIDAR_OUT_OF_RANGE_MM 4000
#define LIDAR_START_BYTE 0x62

const float dockAboveSpillway  = 25.637;   // Inches, dock height above spillway level (was 26.2 — re-measured 2026-08-21)
const float sensorAboveDock    = -6.0;     // Inches, negative = sensor sits below dock (was -8.0 — re-measured 2026-08-21)

// ── Timing ───────────────────────────────────────────────────
#define READ_INTERVAL_MS   1800000UL   // 30 min — temp/level don't need finer resolution;
                                        // see 2"/hr-rain-event math from project chat
// #define READ_INTERVAL_MS 15000UL   // TEMP: 15s for LIDAR debug — revert to 1800000UL (30min) when done
#define MQTT_RETRY_MS       5000UL
#define WIFI_RETRY_MS      10000UL
#define WDT_TIMEOUT_S 25  // bumped from 15 — covers TLS handshake latency to GitHub during OTA
bool firstReadDone = false;   // forces an immediate read/publish on boot


// ── Objects ──────────────────────────────────────────────────
OneWire           oneWire(PIN_ONE_WIRE);
DallasTemperature ds18b20(&oneWire);
WiFiClient        wifiClient;
PubSubClient      mqtt(wifiClient);
// HardwareSerial Serial2 is predefined on ESP32 — just call begin() with pins

// ── State ────────────────────────────────────────────────────
unsigned long lastReadMs  = 0;
unsigned long lastMqttMs  = 0;
unsigned long lastWifiMs  = 0;
bool          otaActive   = false;

// ── Pending OTA URL (set in MQTT callback, executed in loop) ─
String        pendingOtaUrl = "";

// ── LIDAR state (updated asynchronously by pollLidar() in loop) ─
byte          lidarMsgBuffer[9];
float         lastDistanceMm     = NAN;
unsigned long lastLidarUpdateMs  = 0;

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

void setupWatchdog() {
    esp_task_wdt_init(WDT_TIMEOUT_S, true);
    esp_task_wdt_add(NULL);
}

// void setupWatchdog() {
//     esp_task_wdt_config_t wdtConfig = {
//         .timeout_ms = WDT_TIMEOUT_S * 1000,
//         .idle_core_mask = 0,
//         .trigger_panic = true
//     };
//     esp_err_t err = esp_task_wdt_init(&wdtConfig);
//     if (err == ESP_ERR_INVALID_STATE) {
//         esp_task_wdt_reconfigure(&wdtConfig);  // framework already owns it — reconfigure instead
//     }
//     esp_task_wdt_add(NULL);
// }

// ─────────────────────────────────────────────────────────────
//  LIDAR — XKC-KL200-2M-UART
//  Protocol bytes and checksum straight from the bench-tested
//  LIDAR_Round sketch. Auto-mode command is sent once in setup();
//  after that the sensor streams a 9-byte frame roughly once a
//  second on its own, so loop() just has to catch and parse it.
// ─────────────────────────────────────────────────────────────

byte lidarChecksum(byte message[]) {
    byte checksum = 0;
    for (int i = 0; i < 8; i++) {
        checksum = checksum ^ message[i];
    }
    return checksum;
}

void initLidar() {
    // 0 = manual query, 1 = automatic serialization (page-10 convention —
    // confirmed correct by bench test, page-4 defaults table disagrees
    // and is wrong)
    byte setAutoMode[] = {0x62, 0x34, 0x09, 0xFF, 0xFF, 0x0, 0x1, 0x0, 0x0};
    setAutoMode[8] = lidarChecksum(setAutoMode);

    while (Serial2.available()) Serial2.read();   // clear any boot noise

    size_t sent = Serial2.write(setAutoMode, 9);
    Serial2.flush();
    Serial.printf("[LIDAR] Set auto-mode bytes sent: %d\n", sent);
    delay(150);   // one-time setup delay — let the sensor reply fully before moving on

    byte ackBuf[9];
    size_t ackLen = Serial2.readBytes(ackBuf, 9);   // blocks up to default timeout, waits for full frame
    if (ackLen > 0) {
        Serial.print("[LIDAR] ACK: X");
        for (size_t j = 0; j < ackLen; j++) Serial.printf("%02X\t", ackBuf[j]);
        Serial.printf("Xend (%d bytes)\n", ackLen);
        if (ackLen < 9) {
            Serial.println("[LIDAR] WARNING: incomplete ACK — possible wiring/signal issue");
        }
    } else {
        Serial.println("[LIDAR] No ACK received — check wiring/power");
    }
}


// Non-blocking — call every loop() pass. Updates lastDistanceMm
// whenever a complete, validated 9-byte frame has arrived.
// Rejects frames with a bad start byte or failed checksum, and
// resyncs by discarding one byte at a time rather than trusting
// whatever 9 bytes happen to be at the front of the buffer.
void pollLidar() {
    while (Serial2.available() >= 9) {
        if (Serial2.peek() != LIDAR_START_BYTE) {
            Serial2.read();           // discard one byte, try to resync
            continue;
        }

        size_t bytesReceived = Serial2.readBytes(lidarMsgBuffer, 9);
        if (bytesReceived != 9) break;   // shouldn't happen given available() check, but be safe

        byte checksum = 0;
        for (int i = 0; i < 8; i++) checksum ^= lidarMsgBuffer[i];

        if (checksum != lidarMsgBuffer[8]) {
            Serial.println("[LIDAR] Bad checksum — frame discarded");
            continue;   // buffer already advanced past this bad frame; loop checks for more
        }

        int distance = lidarMsgBuffer[5] * 256 + lidarMsgBuffer[6];
        lastDistanceMm    = (float)distance;
        lastLidarUpdateMs = millis();
    }
}

// ─────────────────────────────────────────────────────────────
//  GitHub OTA update
//  Called from loop() so MQTT callback stays non-blocking
// ─────────────────────────────────────────────────────────────
//new
void performOtaUpdate(const String& url) {
    otaActive = true;
    Serial.println("[OTA] Starting GitHub update...");
    Serial.println(url);

    publishStr(TOPIC_OTA_STAT, "{\"status\":\"downloading\"}");
    mqtt.loop();

    WiFiClientSecure secureClient;
    secureClient.setCACertBundle(rootca_crt_bundle_start);   // was: setCACert(GITHUB_ROOT_CA)

    httpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);

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
        case HTTP_UPDATE_FAILED: {
            String err = httpUpdate.getLastErrorString();
            Serial.printf("[OTA] Failed: %s\n", err.c_str());
            String payload = "{\"status\":\"failed\",\"error\":\"" + err + "\"}";
            publishStr(TOPIC_OTA_STAT, payload.c_str());
            otaActive = false;
            break;
        }
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
//new
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
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
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

    JsonDocument doc;
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
    float airTempF   = NAN;
    float distInch   = NAN;

    // ── DS18B20 water + air temp ──────────────────────────────
    ds18b20.requestTemperatures();   // broadcasts to both probes on the bus at once

    float waterTempC = ds18b20.getTempC(waterProbeAddr);
    if (waterTempC != DEVICE_DISCONNECTED_C) {
        waterTempF = (waterTempC * 9.0f / 5.0f) + 32.0f;
        publishFloat(TOPIC_WATER_T, waterTempF);
        Serial.printf("[DS18B20] Water: %.2f °F\n", waterTempF);
    } else {
        Serial.println("[DS18B20] Water probe error / disconnected");
    }

    float airTempC = ds18b20.getTempC(airProbeAddr);
    if (airTempC != DEVICE_DISCONNECTED_C) {
        airTempF = (airTempC * 9.0f / 5.0f) + 32.0f;
        publishFloat(TOPIC_AIR_T, airTempF);
        Serial.printf("[DS18B20] Air: %.2f °F\n", airTempF);
    } else {
        Serial.println("[DS18B20] Air probe error / disconnected");
    }

    // ── LIDAR water level ─────────────────────────────────────
    // Uses whatever pollLidar() last cached — LIDAR streams on its
    // own schedule (~1/sec), this just reads the latest value.
    if (!isnan(lastDistanceMm) && lastDistanceMm < LIDAR_OUT_OF_RANGE_MM) {
        distInch = lastDistanceMm * MM_TO_INCH;
        distInch = dockAboveSpillway + sensorAboveDock - distInch;   // convert to spillway-relative
        publishFloat(TOPIC_LEVEL, distInch);
        Serial.printf("[LIDAR] Level: %.2f in (%.0f mm)\n", distInch, lastDistanceMm);
    } else if (isnan(lastDistanceMm)) {
        Serial.println("[LIDAR] No data received yet");
    } else {
        Serial.println("[LIDAR] Out of range");
    }

    // ── JSON summary ─────────────────────────────────────────
    JsonDocument doc;
    if (!isnan(waterTempF)) doc["watertemp"] = serialized(String(waterTempF, 2));
    if (!isnan(airTempF))   doc["airtemp"]   = serialized(String(airTempF,   2));
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
    setupWatchdog();
    delay(1000);
    Serial.println("\n=== Dock Station " STATION_ID " — FW " FW_VERSION " ===");

    ds18b20.begin();
    Serial.printf("[DS18B20] Devices found on bus: %d\n", ds18b20.getDeviceCount());
    ds18b20.setResolution(waterProbeAddr, DS18B20_RESOLUTION_BITS);
    ds18b20.setResolution(airProbeAddr,   DS18B20_RESOLUTION_BITS);

    Serial2.begin(LIDAR_BAUD, SERIAL_8N1, PIN_LIDAR_RX, PIN_LIDAR_TX);
    initLidar();

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
    esp_task_wdt_reset();
    pollLidar();   // non-blocking — catch any streamed LIDAR frame every pass
//  // TEMP TEST — remove after confirming reset
//     static bool tested = false;
//     if (!tested && millis() > 30000) {  // wait 30s so you see normal operation first
//         tested = true;
//         Serial.println("[TEST] Forcing WDT timeout...");
//         delay(WDT_TIMEOUT_S * 1000 + 2000);  // sit past the timeout
//     }

//     //end of temporary test
    
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
    if (!firstReadDone || (now - lastReadMs >= READ_INTERVAL_MS)) {
        lastReadMs = now;
        firstReadDone = true;
        if (mqtt.connected()) readAndPublish();
    }
}
