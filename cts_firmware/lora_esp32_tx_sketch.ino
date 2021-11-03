#include <SPI.h>
#include <LoRa.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <WiFi.h>
#include <TinyGPSPlus.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;


//Lora module pin configuration
#define ss 5
#define rst 14
#define dio0 2

// Sleep mode configuration
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 5        /* Time ESP32 will go to sleep (in seconds) */
#define EEPROM_SIZE 1

// Software serial initilalization for GPS
static const int RXPin = 3, TXPin = 1;
SoftwareSerial softwareserial(RXPin, TXPin);
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void setModemSleep() {
    WiFi.setSleep(true);
}


void setup() {
  //initialize Serial Monitor
  Serial.println("Initializing ESP32");
  Serial.begin(115200);
  while (!Serial)
    delay(10);
  // Initialize MPU 6050:
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  // Setup a wake interrupt on GPIO 4
  //esp_sleep_enable_ext0_wakeup(GPIO_NUM_4,0);
  //  softwareserial.begin(GPSBaud);
  //  while (!Serial);
  Serial.println("LoRa Sender");

  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);

  //433E6 for Asia
  while (!LoRa.begin(433E6)) {
    Serial.println(".");
    delay(500);
  }
   // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");
  setModemSleep();
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.write(0, 0); // Use this to check if the setup part is invoked after sleep.
}

void loop() {
  print_wakeup_reason(); //Print the wakeup reason for ESP32
  int counter = EEPROM.read(0);
//  while (softwareserial.available() > 0){
//    gps.encode(softwareserial.read());
//    Serial.print("Latitude= "); 
//    Serial.print(gps.location.lat(), 6);
//    Serial.print(" Longitude= "); 
//    Serial.println(gps.location.lng(), 6);
//  }
  //Send LoRa packet to receiver
  LoRa.beginPacket();
  LoRa.print("hello ");
  LoRa.print(counter);
  LoRa.endPacket();
  Serial.print("Sent packet: ");
  Serial.print(counter);
  Serial.print("\n");
  EEPROM.write(0, counter + 1);
  EEPROM.commit();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float xa=a.acceleration.x;
  float ya=a.acceleration.y;
  float za=a.acceleration.z;
  float ff = sqrt(xa*xa + ya*ya + za*za);
  Serial.println("Printing resultant:");
  Serial.println(ff);
  // Initiate low power mode sequence:
  Serial.println("Going to light-sleep now");
  Serial.flush();
  delay(1000);
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); // ESP32 wakes up every 60 seconds 
  esp_light_sleep_start();
}