/*
  Conveyor sensor node - ESP32-S3-WROOM-1
  SCT-013 (RMS phase current) + MLX90614 (object + ambient temperature)
  Every second: readable line on Serial + JSON over MQTT to plant/conveyor/telemetry
  Libraries: PubSubClient (Nick O'Leary), Adafruit MLX90614
  Board: ESP32S3 Dev Module, USB CDC On Boot: Enabled
*/
#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <Adafruit_MLX90614.h>

// ===== FILL IN =====
const char* WIFI_SSID = "Benismail's A57";
const char* WIFI_PASS = "skanskan12";              // remove before putting the code in the thesis
const char* MQTT_HOST = "10.111.171.174";          // IP of the PC with Mosquitto (cmd: ipconfig)
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USER = "esp32";
const char* MQTT_PASS = "esp32pass2026";

// ===== SENSORS =====
const int SCT_PIN = 4;            // ADC1 pin (works with Wi-Fi on)
float AMPS_PER_VOLT = 10.0;       // SCT-013-010
float CAL = 1.0;                  // adjust against a true-RMS clamp meter
const float NOISE_FLOOR_A = 0.0;  // 0 for the test; later ~2x the "motor stopped" value
const bool DIAG = true;           // true = show pin voltage at the end of each line (set false later)

const char* TOPIC_TELE   = "plant/conveyor/telemetry";
const char* TOPIC_STATUS = "plant/esp32/status";

Adafruit_MLX90614 mlx;
bool mlxOk = false;
WiFiClient net;
PubSubClient mqtt(net);
uint32_t lastSample = 0, lastWifiTry = 0, lastMqttTry = 0, lastMlxTry = 0;
String clientId;

// diagnostic values of the last burst
float pinMean = 0, pinMin = 0, pinMax = 0;
int pinN = 0;

// ---------- RMS current over ~200 ms ----------
float readCurrentRMS() {
  const int N = 1000;
  static float buf[N];
  double sum = 0;
  int n = 0;
  float vmin = 1e9, vmax = -1e9;
  uint32_t start = millis();
  while (millis() - start < 200 && n < N) {
    buf[n] = analogReadMilliVolts(SCT_PIN);
    sum += buf[n];
    if (buf[n] < vmin) vmin = buf[n];
    if (buf[n] > vmax) vmax = buf[n];
    n++;
    delayMicroseconds(150);       // spreads the samples over ~200 ms
  }
  if (n < 10) return NAN;
  double mean = sum / n;          // = bias voltage, should be ~1650 mV
  pinMean = mean; pinMin = vmin; pinMax = vmax; pinN = n;
  double sq = 0;
  for (int i = 0; i < n; i++) sq += (buf[i] - mean) * (buf[i] - mean);
  float amps = sqrt(sq / n) / 1000.0 * AMPS_PER_VOLT * CAL;
  return (amps < NOISE_FLOOR_A) ? 0.0f : amps;
}

// ---------- Wi-Fi / MQTT, reconnect without blocking ----------
void ensureWifi() {
  if (WiFi.status() == WL_CONNECTED || millis() - lastWifiTry < 10000) return;
  lastWifiTry = millis();
  Serial.println("WiFi: connecting...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

void ensureMqtt() {
  if (WiFi.status() != WL_CONNECTED || mqtt.connected() || millis() - lastMqttTry < 5000) return;
  lastMqttTry = millis();
  Serial.printf("MQTT: connecting to %s ... ", MQTT_HOST);
  if (mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS, TOPIC_STATUS, 1, true, "offline")) {
    Serial.println("connected");
    mqtt.publish(TOPIC_STATUS, "online", true);
  } else {
    // -2 = broker not reachable (IP/firewall), 4 = wrong user/password, 5 = not authorised
    Serial.printf("failed, state=%d\n", mqtt.state());
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(8, 9);
  Wire.setClock(100000);
  analogReadResolution(12);
  analogSetPinAttenuation(SCT_PIN, ADC_11db);   // if this does not compile: ADC_ATTEN_DB_12

  mlxOk = mlx.begin();
  Serial.println(mlxOk ? "MLX90614 found" : "MLX90614 NOT found - retrying every 10 s");

  clientId = "esp32-conveyor-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  lastWifiTry = millis();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setKeepAlive(60);
}

void loop() {
  ensureWifi();
  ensureMqtt();
  mqtt.loop();

  if (!mlxOk && millis() - lastMlxTry > 10000) {
    lastMlxTry = millis();
    mlxOk = mlx.begin();
  }

  if (millis() - lastSample < 1000) return;
  lastSample = millis();

  float amps = readCurrentRMS();
  float tObj = NAN, tAmb = NAN;
  if (mlxOk) { tObj = mlx.readObjectTempC(); tAmb = mlx.readAmbientTempC(); }
  bool okObj = !isnan(tObj) && tObj > -40 && tObj < 380;
  bool okAmb = !isnan(tAmb) && tAmb > -40 && tAmb < 125;

  // JSON for MQTT - a failed reading is sent as null
  char sCur[16], sObj[16], sAmb[16], payload[128];
  if (isnan(amps)) strcpy(sCur, "null"); else snprintf(sCur, sizeof sCur, "%.3f", amps);
  if (okObj) snprintf(sObj, sizeof sObj, "%.2f", tObj); else strcpy(sObj, "null");
  if (okAmb) snprintf(sAmb, sizeof sAmb, "%.2f", tAmb); else strcpy(sAmb, "null");
  snprintf(payload, sizeof payload, "{\"current\":%s,\"temp_obj\":%s,\"temp_amb\":%s}", sCur, sObj, sAmb);

  // readable line - this exact format is what 3_record_conveyor_usb.bat records
  Serial.printf("current = %.4f A | temp_obj = %.2f °C | temp_amb = %.2f °C",
                amps, okObj ? tObj : NAN, okAmb ? tAmb : NAN);
  Serial.print(mqtt.connected() ? "  -> sent" : "  (not sent: no MQTT)");
  if (mqtt.connected()) mqtt.publish(TOPIC_TELE, payload);
  if (DIAG) Serial.printf("   [pin mean %.0f mV, min %.0f, max %.0f, %d samples]", pinMean, pinMin, pinMax, pinN);
  Serial.println();
}