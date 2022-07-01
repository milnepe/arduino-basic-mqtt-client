/***************************************************
  A robust basic Arduino MQTT client that connects
  to DesgnSpark's ESDK

  Version: 1.1
  Date: 1st July 2022
  Author: Peter Milne

  Copywrite 2022 Peter Milne
  Released under GNU GENERAL PUBLIC LICENSE
  Version 3, 29 June 2007

 ****************************************************/

// Un-comment for debugging
// System will not run in DEBUG until serial monitor attaches!
#define DEBUG

#include <WiFiNINA.h>
#include "arduino_secrets.h"
#include <SPI.h>
#include <PubSubClient.h>
#include "ArduinoJson.h"

// ESDK host
// You may need to substiture its IP address on your network
//const char broker[] = "192.168.0.75";
const char broker[] = "airquality";
int        port     = 1883;

// ESDK topic root
#define TOPIC "airquality/#"

/////// Enter sensitive data in arduino_secrets.h
const char ssid[] = SECRET_SSID;  // Network SSID
const char pass[] = SECRET_PASS;  // WPA key

WiFiClient wifiClient;
long lastReconnectWIFIAttempt = 0;
PubSubClient mqttClient(wifiClient);
long lastReconnectMQTTAttempt = 0;

unsigned long delayStart = 0; // time the delay started
bool delayRunning = false; // true if still waiting for delay to finish
bool printFlag = false;

int co2 = 0;
double temperature = 0;
double humidity = 0;
int tvoc = 0;
int pm = 0;

void setup() {
  Serial.begin(115200);
#ifdef DEBUG
  while (!Serial) {
    ; // wait for serial port to connect
  }
#endif

  Serial.println("Basic MQTT client");

  // Print firmware version on the module
  String fv = WiFi.firmwareVersion();
  String latestFv;
  Serial.print("Firmware version installed: ");
  Serial.println(fv);

  mqttClient.setServer(broker, port);
  mqttClient.setCallback(callback);
  mqttClient.setBufferSize(384);

  delayStart = millis();
  delayRunning = true;

  Serial.println("Attempting WiFi connection...");
  delay(1000);
}

void loop() {

  // Attempt to reconnect
  // Wifi.begin blocks until connect or failure timeout
  if (WiFi.status() != WL_CONNECTED) {
    reconnectWiFi();
  }

  if (!mqttClient.connected()) {
    // Attempt to reconnect without blocking
    // Stops too many connection attemps which
    // can give you a bad day!
    long now = millis();
    if (now - lastReconnectMQTTAttempt > 5000) {
      lastReconnectMQTTAttempt = now;
      if (reconnectMQTT()) {
        lastReconnectMQTTAttempt = 0;
      }
    }
  } else {
    mqttClient.loop();
  }

#ifdef DEBUG
  if (printFlag) {
    printSensorReadings();
    printFlag = false;
  }
#endif

  // Code that must always run
  Serial.println("Still running...");
  delay(500);
}

int reconnectWiFi() {
  // WL_IDLE_STATUS     = 0
  // WL_NO_SSID_AVAIL   = 1
  // WL_SCAN_COMPLETED  = 2
  // WL_CONNECTED       = 3
  // WL_CONNECT_FAILED  = 4
  // WL_CONNECTION_LOST = 5
  // WL_DISCONNECTED    = 6
  printWiFiStatus(WiFi.status());
  // Always force Wifi drv to disconnect for safety
  int disconnect_result = WiFi.disconnect();
  Serial.print("Disconnect state: ");
  Serial.println(disconnect_result);
  printWiFiStatus(WiFi.status());
  delay(1000);
  WiFi.begin(ssid, pass);
  printWiFiStatus(WiFi.status());
  return WiFi.status();
}

void printWiFiStatus(int state) {
  switch (state) {
    case WL_IDLE_STATUS:
      Serial.println("WiFi IDLE");
      break;
    case WL_NO_SSID_AVAIL:
      Serial.println("WiFi NO SSID AVAILABLE");
      break;
    case WL_SCAN_COMPLETED:
      Serial.println("WiFi SCAN COMPLETED");
      break;
    case WL_CONNECTED:
      Serial.println("WiFi CONNECTED");
      break;
    case WL_CONNECT_FAILED:
      Serial.println("WiFi CONNECTION FAILED");
      break;
    case WL_CONNECTION_LOST:
      Serial.println("WiFi CONNECTION LOST");
      break;
    case WL_DISCONNECTED:
      Serial.println("WiFi DISCONNECTED");
  }
}

boolean reconnectMQTT() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Attempting connection to the MQTT server: ");
    Serial.println(broker);
    if (mqttClient.connect("arduinoClient")) {
      mqttClient.subscribe(TOPIC);
      Serial.println("MQTT connected");
    }
  }
  return mqttClient.connected();
}

// Update sensor variables each time a message is received
void callback(char* topic, byte * payload, unsigned int length) {
#ifdef DEBUG
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
#endif
  // ESDK sends a large JSON payload
  // - ensure you have enough memory allocated
  StaticJsonDocument<384> doc;
  deserializeJson(doc, payload, length);
  co2 = doc["co2"]["co2"];
  temperature = doc["thv"]["temperature"];
  humidity = doc["thv"]["humidity"];
  tvoc = doc["thv"]["vocIndex"];
  pm = doc["pm"]["pm2.5"];

  printFlag = true;
}

void printSensorReadings() {
  Serial.print("CO2: ");
  Serial.println(co2);
  Serial.print("Temperature: ");
  Serial.println(temperature);
  Serial.print("Humidity: ");
  Serial.println(humidity);
  Serial.print("TVOC: ");
  Serial.println(tvoc);
  Serial.print("PM2.5: ");
  Serial.println(pm);
}
