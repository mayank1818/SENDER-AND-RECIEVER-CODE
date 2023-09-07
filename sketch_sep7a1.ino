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

char ssid[] = "realme Narzo 10";    // Your Wi-Fi network SSID (name)
char pass[] = "mayank1818";    // Your Wi-Fi network password

int ledPin = 5; // Pin connected to the LED

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "mqtt-dashboard.com"; // MQTT broker address
int port = 1883;             // MQTT broker port
const char topic[] = "MAYANK"; // MQTT topic to subscribe to

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);

  // Connect to Wi-Fi network
  Serial.print("Connecting to Wi-Fi network: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    Serial.print(".");
    delay(5000);
  }
  Serial.println("Connected to the Wi-Fi network");

  Serial.print("Connecting to MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    while (1);
  }
  Serial.println("Connected to the MQTT broker");

  Serial.print("Subscribing to MQTT topic: ");
  Serial.println(topic);

  mqttClient.subscribe(topic);
  Serial.println("Waiting for MQTT messages");
}

void loop() {
  int messageSize = mqttClient.parseMessage();
  if (messageSize) {
    Serial.print("Received an MQTT message with topic '");
    Serial.print(mqttClient.messageTopic());
    Serial.print("', length ");
    Serial.print(messageSize);
    Serial.println(" bytes:");

    while (mqttClient.available()) {
      Serial.print((char)mqttClient.read());
    }
    Serial.println();

    // Blink the LED
    for (int i = 0; i < 3; i++) {
      digitalWrite(ledPin, HIGH);
      delay(2000);
      digitalWrite(ledPin, LOW);
      delay(2000);
    }

    Serial.println();
  }
}