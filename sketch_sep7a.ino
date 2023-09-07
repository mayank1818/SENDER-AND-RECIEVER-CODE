#include <ArduinoMqttClient.h>
#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
#include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
#include <WiFi101.h>
#elif defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#endif

char wifiSsid[] = "realme Narzo 10";
char wifiPassword[] = "mayank1818";

const int trigPin = 3;
const int echoPin = 4;

float echoDuration, distance;

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char mqttBroker[] = "mqtt-dashboard.com";
int mqttPort = 1883;
const char mqttTopic[] = "MAYANK";

const long measurementInterval = 1000;
unsigned long previousMeasurementMillis = 0;

int measurementCount = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }

  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(wifiSsid);
  while (WiFi.begin(wifiSsid, wifiPassword) != WL_CONNECTED) {
    Serial.print(".");
    delay(6000);
  }

  Serial.println("Connected to the network");
  Serial.println();

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(mqttBroker);

  if (!mqttClient.connect(mqttBroker, mqttPort)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    while (1);
  }

  Serial.println("Connected to the MQTT broker!");
  Serial.println();
}

void loop() {
  mqttClient.poll();

  unsigned long currentMillis = millis();

  if (currentMillis - previousMeasurementMillis >= measurementInterval) {
    previousMeasurementMillis = currentMillis;

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    echoDuration = pulseIn(echoPin, HIGH);
    distance = (echoDuration * 0.0343) / 2;
    Serial.print("Distance: ");
    Serial.println(distance);
    if (distance < 12) {
      mqttClient.beginMessage(mqttTopic);
      mqttClient.print("MAYANK : Wave detected, ");
      mqttClient.print("Distance: ");
      mqttClient.print(distance);
      mqttClient.endMessage();
      delay(3000);
    }

    Serial.println();

    measurementCount++;
  }
}
