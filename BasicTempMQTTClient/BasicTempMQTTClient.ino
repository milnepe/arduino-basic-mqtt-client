/***************************************************
  A basic Arduino MQTT client with temperature
  sensors

  Test with client
  mosquitto_sub -h airquality -t '/beer/test' -F '%x : %l'

  Version: 0.1.0
  Date: 18 December 2022
  Author: Peter Milne

  Copywrite 2022 Peter Milne
  Released under GNU GENERAL PUBLIC LICENSE
  Version 3, 29 June 2007

 ****************************************************/

#include <SPI.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h"
#include <PubSubClient.h>  // https://pubsubclient.knolleary.net/api
#include "ArduinoJson.h"

// Un-comment for debugging
// System will not run in DEBUG until serial monitor attaches!
#define DEBUG

#define HEARTBEAT_LED 5
#define RGB_LED_GREEN 25
#define RGB_LED_RED 26

const char* soft_version = "0.1.0";

// ESDK host
// You may need to substiture its IP address on your network
//const char broker[] = "192.168.0.75";
const char broker[] = "airquality";
int port = 1883;

// ESDK topic root
#define TOPIC "/beer/test"

/////// Enter sensitive data in arduino_secrets.h
const char ssid[] = SECRET_SSID;  // Network SSID
const char pass[] = SECRET_PASS;  // WPA key

WiFiClient wifiClient;
byte mac[6];  // MAC address of WiFi Module
char macstr[18];


PubSubClient mqttClient(wifiClient);

unsigned long lastReconnectMQTTAttempt = 0;
boolean printFlag = false;
boolean heartbeat = true;

int32_t raw_reading = -30;  // Raw readings
byte buffer[4] = { 0 };     // Readings converted to bytes

void setup() {
  WiFiDrv::pinMode(RGB_LED_GREEN, OUTPUT);  //define green pin
  WiFiDrv::pinMode(RGB_LED_RED, OUTPUT);    //define red pin
  pinMode(HEARTBEAT_LED, OUTPUT);

  Serial.begin(115200);
#ifdef DEBUG
  while (!Serial) {
    ;  // wait for serial port to connect
  }
#endif

  Serial.println("Basic Temp MQTT client");
  Serial.print("Version: ");
  Serial.println(soft_version);

  WiFi.macAddress(mac);
  snprintf(macstr, 18, "%02x:%02x:%02x:%02x:%02x:%02x", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
  Serial.print("MAC: ");
  Serial.println(macstr);

  // Print firmware version on the module
  String fv = WiFi.firmwareVersion();
  String latestFv;
  Serial.print("Firmware version installed: ");
  Serial.println(fv);

  mqttClient.setServer(broker, port);
  // mqttClient.setCallback(callback);
  mqttClient.setBufferSize(384);

  WiFiDrv::analogWrite(RGB_LED_GREEN, 0);  // Green off
  WiFiDrv::analogWrite(RGB_LED_RED, 255);  // Red on (disconnected)

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
    unsigned long now = millis();
    if (now - lastReconnectMQTTAttempt > 5000) {
      lastReconnectMQTTAttempt = now;
      if (reconnectMQTT()) {
        lastReconnectMQTTAttempt = 0;
        WiFiDrv::analogWrite(RGB_LED_GREEN, 255);  // Green on (connected)
        WiFiDrv::analogWrite(26, 0);               // Red off
      }
    }
  } else {
    mqttClient.loop();
  }
  raw_reading++;  // mock
  convert_reading_to_bytes(raw_reading);
  mqttClient.publish(TOPIC, buffer, 4);  // Send a reading as 2 bytes

#ifdef DEBUG
  printSensorReadings();
#endif

  // Code that must always run
  digitalWrite(HEARTBEAT_LED, (heartbeat = !heartbeat));
  Serial.println(heartbeat);
  Serial.println("Still running...");
  delay(1000);
}  // end loop

// Convert a raw reading and hold it in buffer as high and low bytes
void convert_reading_to_bytes(int32_t reading) {
  buffer[0] = (reading >> 24) & 0xFF;
  buffer[1] = (reading >> 16) & 0xFF;
  buffer[2] = (reading >> 8) & 0xFF;
  buffer[3] = reading & 0xFF;
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
    if (mqttClient.connect(macstr)) {  // use wifi mac addr
      mqttClient.subscribe(TOPIC);
      Serial.println("MQTT connected");
    }
  }
  return mqttClient.connected();
}

// Update sensor variables each time a message is received
// void callback(char* topic, byte * payload, unsigned int length) {
//   Serial.print("Message arrived [");
//   Serial.print(topic);
//   Serial.print("] ");
//   for (unsigned int i = 0; i < length; i++) {
//     Serial.print((char)payload[i]);
//   }
//   Serial.println();
//   // ESDK sends a large JSON payload
//   // - ensure you have enough memory allocated
//   StaticJsonDocument<384> doc;
//   deserializeJson(doc, payload, length);
//   co2 = doc["co2"]["co2"];
//   temperature = doc["thv"]["temperature"];
//   humidity = doc["thv"]["humidity"];
//   tvoc = doc["thv"]["vocIndex"];
//   pm = doc["pm"]["pm2.5"];

//   printFlag = true;
// }

void printSensorReadings() {
  Serial.print("Reading: ");
  Serial.print(raw_reading);
  Serial.print(" ");
  Serial.print(buffer[0], HEX);
  Serial.print(" ");
  Serial.print(buffer[1], HEX);
  Serial.print(" ");
  Serial.print(buffer[2], HEX);
  Serial.print(" ");
  Serial.print(buffer[3], HEX);
  Serial.print(" check: ");
  int32_t read_back = ((buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3]);
  Serial.println(read_back);
}
