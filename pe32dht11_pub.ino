/**
 * pe32dht11 // Collect temperature+humidity from DHT11, export to MQTT.
 *
 * Components:
 * - ESP8266MOD+Wifi
 * - DHT11 digital temperature and humidity sensor
 * - attach 3VC<->VCC, GND<->GND, D5<->DAT
 *
 * Building:
 * - Arduino IDE setup: add
 *   https://arduino.esp8266.com/stable/package_esp8266com_index.json
 *   to "Additional Boards Managers URL".
 * - Install "ESP8266" boards.
 * - Select "Generic ESP8266 Module" board.
 * - Install "ArduinoMqttClient" module.
 * - Install "DHT sensor library for ESPx"
 *
 * Configuration:
 * - Set Wifi SSID+password in config.h
 * - Set MQTT broker details in config.h
 * - Set up broker and read values to process them further.
 */
#include <ArduinoMqttClient.h>
#include <DHTesp.h>
#include <ESP8266WiFi.h>

/* In config.h, you should have:
const char wifi_ssid[] = "<ssid>";
const char wifi_password[] = "<password>";
const char mqtt_broker[] = "192.168.1.2";
const int  mqtt_port = 1883;
const char mqtt_topic[] = "some/topic";
*/
#include "config.h"

#define VERSION "v0"
//#define DEBUG

/* Interesting stuff for another time:
 * We can read/write the first 512 bytes in the EEPROM:
 *   #include <EEPROM.h>
 *   EEPROM.read(0), EEPROM.read(1), etc..
 *   EEPROM.write(0, 'a'); EEPROM.write(1, 'b');
 * We could use this to store updated/new passwords/ssids, etc..
 */

// TODO: #include <SimpleKalmanFilter.h>
// TODO: https://github.com/denyssene/SimpleKalmanFilter/blob/master/examples/BasicKalmanFilterExample/BasicKalmanFilterExample.ino
//  SimpleKalmanFilter(e_mea, e_est, q);
//  e_mea: Measurement Uncertainty
//  e_est: Estimation Uncertainty
//  q: Process Noise
// SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);

/* We use the guid to store something unique to identify the device by.
 * For now, we'll populate it with the ESP8266 Wifi MAC address. */
char guid[24]; // "EUI48:11:22:33:44:55:66"

const int DHTpin = 14; // D5 of ESP8266 (NodeMCU) is GPIO14

DHTesp dht;

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

void ensure_wifi();
void ensure_mqtt();

void setup()
{
  strncpy(guid, "undefined_guid", sizeof(guid) - 1);

  // Setup (should we skip this if !DEBUG?)
  // TODO: figure out whether this speed it better or worse than slow
  // speed (for resources? and if we should skip this)
  Serial.begin(115200);
  delay(500);

  // Setup GUID
  strncpy(guid, "EUI48:", 6);
  strncpy(guid + 6, WiFi.macAddress().c_str(), sizeof(guid) - (6 + 1));

  // Setup pin
  dht.setup(DHTpin, DHTesp::DHT11);

  // Welcome message
  Serial.print("Booted pe32dht11 " VERSION " guid ");
  Serial.println(guid);

  // Initial connect
  ensure_wifi();
  ensure_mqtt();
}

void loop()
{
  float humidity_pct = dht.getHumidity();
  float temperature_c = dht.getTemperature();
  float heat_index = dht.computeHeatIndex(temperature_c, humidity_pct, false);

  if (strcmp(dht.getStatusString(), "OK") != 0) {
    Serial.println("Sampling failed, waiting 30s before next attempt");
    delay(30000); // 30s, or: dht.getMinimumSamplingPeriod()
    return;
  }

  Serial.print("Collected: device_id=");
  Serial.print(guid);
  Serial.print("&temperature_celcius=");
  Serial.print(temperature_c);
  Serial.print("&humidity_percent=");
  Serial.print(humidity_pct);
  Serial.print("&heat_index=");
  Serial.println(heat_index);

  Serial.print("Attempting push to broker topic ");
  Serial.println(mqtt_topic);

  ensure_wifi();
  ensure_mqtt();

  mqttClient.beginMessage(mqtt_topic);
  mqttClient.print("device_id=");
  mqttClient.print(guid);
  mqttClient.print("&temperature_celcius=");
  mqttClient.print(temperature_c);
  mqttClient.print("&humidity_percent=");
  mqttClient.print(humidity_pct);
  mqttClient.print("&heat_index=");
  mqttClient.print(heat_index);
  mqttClient.endMessage();

  delay(60000); // 60s, or 30s, or: dht.getMinimumSamplingPeriod()
}

/**
 * Check that Wifi is up, or connect when not connected.
 */
void ensure_wifi()
{
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(wifi_ssid, wifi_password);
    for (int i = 30; i >= 0; --i) {
      if (WiFi.status() == WL_CONNECTED) {
        break;
      }
      delay(1000);
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("Wifi UP on \"");
      Serial.print(wifi_ssid);
      Serial.print("\", Local IP: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.print("Wifi NOT UP on \"");
      Serial.print(wifi_ssid);
      Serial.println("\".");
    }
  }
}

/**
 * Check that the MQTT connection is up or connect if it isn't.
 */
void ensure_mqtt()
{
  mqttClient.poll();
  if (!mqttClient.connected()) {
    if (mqttClient.connect(mqtt_broker, mqtt_port)) {
      Serial.print("MQTT connected: ");
      Serial.println(mqtt_broker);
    } else {
      Serial.print("MQTT connection to ");
      Serial.print(mqtt_broker);
      Serial.print(" failed! Error code = ");
      Serial.println(mqttClient.connectError());
    }
  }
}

// vim: set ts=8 sw=2 sts=2 et ai:
