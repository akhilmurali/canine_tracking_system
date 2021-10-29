#include <SPI.h>
#include <LoRa.h>
#include "WiFi.h"
#include "PubSubClient.h" // Connect and publish to the MQTT broker
//define the pins used by the transceiver module
#define ss 5
#define rst 14
#define dio0 2

// WiFi
const char* ssid = "Airtel_9618553130";// Your personal network SSID
const char* wifi_password = "XXXXX"; // Your personal network password

// MQTT
const char* mqtt_server = "192.168.1.8";  // IP of the MQTT broker
const char* local_proxy_topic = "topic/cts_local_proxy_topic";
const char* mqtt_username = "pine_mos"; // MQTT username
const char* mqtt_password = "1234"; // MQTT password
const char* clientID = "local_proxy_client_ID"; // MQTT client ID

// Initialise the WiFi and MQTT Client objects
WiFiClient wifiClient;
PubSubClient client(mqtt_server, 1883, wifiClient); // 1883 is the listener port for the Broker

void setup() {
  //initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Receiver");

  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  
  //replace the LoRa.begin(---E-) argument with your location's frequency 
  //433E6 for Asia
  //866E6 for Europe
  //915E6 for North America
  while (!LoRa.begin(433E6)) {
    Serial.println(".");
    delay(500);
  }
   // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");

  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect to the WiFi
  WiFi.begin(ssid, wifi_password);

  // Wait until the connection has been confirmed before continuing
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (client.connect(clientID, mqtt_username, mqtt_password)) {
    Serial.println("Connected to MQTT Broker!");
  }
  else {
    Serial.println("Connection to MQTT Broker failed...");
  }
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  String LoRaData;
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");
    // read packet
    while (LoRa.available()) {
      LoRaData = LoRa.readString();
      Serial.print(LoRaData); 
    }
    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
    int str_len = LoRaData.length() + 1; 
    char char_array[str_len];
    LoRaData.toCharArray(char_array, str_len);
    
    if (client.publish(local_proxy_topic, char_array)) {
     Serial.println("Published data to local broker.");
    }
    else 
    {
     Serial.println("Message failed to send. Reconnecting to MQTT Broker and trying again");
     client.connect(clientID, mqtt_username, mqtt_password);
     delay(10); 
     client.publish(local_proxy_topic, char_array);
    }
  }
}