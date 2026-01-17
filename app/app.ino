#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>

// Configuration - Update these values for your setup
#define WIFI_SSID "your-wifi-ssid"
#define WIFI_PASSWORD "your-wifi-password"

// Local Mosquitto MQTT configuration (no TLS, no auth)
#define MQTT_SERVER "192.168.1.100"
#define MQTT_PORT 1883

// MQTT Topics
#define MQTT_TOPIC_LED_STATE "esp32/led/state"
#define MQTT_TOPIC_LED_CONTROL "esp32/led/control"
#define MQTT_TOPIC_BUZZER_STATE "esp32/buzzer/state"
#define MQTT_TOPIC_BUZZER_CONTROL "esp32/buzzer/control"

// Pin definitions
#define LED_PIN 4
#define BUZZER_PIN 2

WiFiClient espClient;
PubSubClient mqttClient(espClient);

unsigned long lastMqttReconnect = 0;

void setup() {
  Serial.begin(115200);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Setup MQTT for local Mosquitto broker (no TLS, no auth)
  // ESP32 uses native MQTT protocol on port 1883
  // WebSocket is enabled on port 9001 for browser dashboard clients
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(512);
  
  connectMQTT();
}

void loop() {
  // Reconnect to MQTT if disconnected
  if (!mqttClient.connected()) {
    unsigned long now = millis();
    if (now - lastMqttReconnect > 5000) {
      lastMqttReconnect = now;
      connectMQTT();
    }
  }
  mqttClient.loop();
  
  // Publish LED and buzzer states periodically
  static unsigned long lastStatePublish = 0;
  unsigned long now = millis();
  if (now - lastStatePublish > 2000) {  // Publish every 2 seconds
    lastStatePublish = now;
    publishPinStates();
  }
}

int getLedState() {
    return digitalRead(LED_PIN);
}

int getBuzzerState() {
    return digitalRead(BUZZER_PIN);
}

void connectMQTT() {
  Serial.print("Attempting MQTT connection...");
  
  // Create a unique client ID
  String clientId = "ESP32Client-";
  clientId += String(random(0xffff), HEX);
  
  // Attempt to connect to local Mosquitto broker (no auth)
  if (mqttClient.connect(clientId.c_str())) {
    Serial.println("connected to Mosquitto broker");
    
    // Subscribe to control topics
    mqttClient.subscribe(MQTT_TOPIC_LED_CONTROL);
    mqttClient.subscribe(MQTT_TOPIC_BUZZER_CONTROL);
    Serial.println("Subscribed to control topics");
  } else {
    Serial.print("failed, rc=");
    Serial.print(mqttClient.state());
    Serial.println(" try again in 5 seconds");
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  Serial.println(message);
  
  // Control LED
  // Note: ESP32 responds to all control messages regardless of AI control state
  // The dashboard and YOLOv8 script handle AI control logic and prevent
  // manual control messages when AI control is enabled
  if (String(topic) == MQTT_TOPIC_LED_CONTROL) {
    if (message == "1" || message == "ON" || message == "on") {
      digitalWrite(LED_PIN, HIGH);
      Serial.println("LED turned ON");
      publishPinStates();
    } else if (message == "0" || message == "OFF" || message == "off") {
      digitalWrite(LED_PIN, LOW);
      Serial.println("LED turned OFF");
      publishPinStates();
    }
  }
  
  // Control Buzzer
  if (String(topic) == MQTT_TOPIC_BUZZER_CONTROL) {
    if (message == "1" || message == "ON" || message == "on") {
      digitalWrite(BUZZER_PIN, HIGH);
      Serial.println("Buzzer turned ON");
      publishPinStates();
    } else if (message == "0" || message == "OFF" || message == "off") {
      digitalWrite(BUZZER_PIN, LOW);
      Serial.println("Buzzer turned OFF");
      publishPinStates();
    }
  }
}

void publishPinStates() {
  if (mqttClient.connected()) {
    // Publish LED state
    String ledState = String(getLedState());
    mqttClient.publish(MQTT_TOPIC_LED_STATE, ledState.c_str());
    Serial.print("Published LED state: ");
    Serial.println(ledState);
    
    // Publish Buzzer state
    String buzzerState = String(getBuzzerState());
    mqttClient.publish(MQTT_TOPIC_BUZZER_STATE, buzzerState.c_str());
    Serial.print("Published Buzzer state: ");
    Serial.println(buzzerState);
  } else {
    Serial.println("MQTT not connected, cannot publish");
  }
}
