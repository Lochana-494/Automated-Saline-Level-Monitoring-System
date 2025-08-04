#include <WiFi.h>
#include <PubSubClient.h>
#include "HX711.h"
#include <ESP32Servo.h>

// Pin definitions
#define MOTOR_PIN 23         // Motor control pin
#define BUZZER_PIN 19    

uint8_t dataPin = 5;          // Data pin for load cell
uint8_t clockPin = 18;        // Clock pin for load cell

// WiFi credentials
const char* ssid = "ssid";         // Replace with your WiFi SSID
const char* password = "passwd"; // Replace with your WiFi password

// MQTT Broker settings
const char* mqtt_server = "192.168.xxx.xx"; // Replace with your MQTT broker IP
const int mqtt_port = 1883;

// Initialize HX711, servo
HX711 scale;
Servo myservo;

bool servoState = false;  
bool buzzerState = false; // To track if the buzzer is currently on
int flowRate = 0;  // Stores received flow rate value

// Initialize WiFi and MQTT clients
WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);

  scale.begin(dataPin, clockPin);
  //scale.set_offset(2500);  // Set the pre-calibrated offset
  scale.set_scale(400.195435);  // Set the pre-calibrated scale factor
  scale.tare(20);

  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW); // Turn off the motor initially
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  connectToWiFi();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback); // Set the callback function for receiving commands

  myservo.attach(MOTOR_PIN);
  myservo.write(0); // Servo starts at 0 degrees
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (scale.is_ready()) {
    
    float weight = scale.get_units(10); // Average 10 readings

    Serial.print("Weight: ");
    Serial.println(weight);

    char weightStr[8];
    dtostrf(weight, 1, 2, weightStr); // Convert float to string
    client.publish("loadcell/weight", weightStr);

    // Buzzer logic
    if (weight < 10 && !buzzerState) {
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerState = true;
      Serial.println("Buzzer ON - Weight below 10%");
    } else if (weight >= 10 && buzzerState) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzerState = false;
      Serial.println("Buzzer OFF - Weight back to normal");
    }

    // Automatic servo motor control
    if (weight <= 5 && !servoState) {
      myservo.write(90); // Turn to 90 degrees
      servoState = true;
      Serial.println("Servo moved to 90 degrees");
    } else if (weight > 5 && servoState) {
      myservo.write(0); // Turn to 0 degrees
      servoState = false;
      Serial.println("Servo moved to 0 degrees");
    }
  }

  delay(1000);
}

void connectToWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      client.subscribe("motor/control");
      client.subscribe("saline/flowRate"); // ✅ Subscribe to flow rate
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" trying again in 5 seconds");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  message.trim();
  Serial.print("Message received: ");
  Serial.println(message);

  if (String(topic) == "motor/control") {
    if (message == "on") {
      myservo.write(90); 
      servoState = true;
      Serial.println("Manual: Servo moved to 90 degrees");
    } else if (message == "off") {
      myservo.write(0);
      servoState = false;
      Serial.println("Manual: Servo moved to 0 degrees");
    } else {
      Serial.println("Invalid message received. Ignoring...");
    }
  } 
  
  else if (String(topic) == "saline/flowRate") {  // ✅ Handle flow rate messages
    int receivedFlowRate = message.toInt();
    if (receivedFlowRate >= 0 && receivedFlowRate <= 100) {
      flowRate = receivedFlowRate;
      int servoAngle = map(flowRate, 0, 100, 90, 0);  // Map flow rate to servo angle
      myservo.write(servoAngle);
      Serial.print("Flow rate set to: ");
      Serial.print(flowRate);
      Serial.print("% - Servo moved to angle: ");
      Serial.println(servoAngle);
    } else {
      Serial.println("Invalid flow rate received. Ignoring...");
    }
  } 

  else {
    Serial.println("Unknown topic. Ignoring...");
  }
}
