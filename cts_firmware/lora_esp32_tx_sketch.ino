#include <SPI.h>
#include <LoRa.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <WiFi.h>
#include <TinyGPSPlus.h>
#include <Wire.h>

//Lora module pin configuration
#define ss 5
#define rst 14
#define dio0 2

// Sleep mode configuration
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 60        /* Time ESP32 will go to sleep (in seconds) */
#define EEPROM_SIZE 1

// MPU 6050 configuration
#define INTERRUPT_PIN 4

#define MOTION_THRESHOLD 8
#define MOTION_EVENT_DURATION 40
#define SIGNAL_PATH_RESET 0x68
#define I2C_SLV0_ADDR 0x37
#define ACCEL_CONFIG 0x1C
#define MOT_THR 0x1F // Motion detection threshold bits [7:0]
#define MOT_DUR 0x20 // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define MOT_DETECT_CTRL 0x69
#define INT_ENABLE 0x38
#define WHO_AM_I_MPU6050 0x75 // Should return 0x68
#define INT_STATUS 0x3A
//when nothing connected to AD0 than address is 0x68
#define ADO 0
#if ADO
#define MPU6050_ADDRESS 0x69 // Device address when ADO = 1
#else
#define MPU6050_ADDRESS 0x68 // Device address when ADO = 0
#endif
#define EXT0_WAKEUP 1
#define TIMER_WAKEUP 0

// Software serial initilalization for GPS
static const int RXPin = 16, TXPin = 17;
SoftwareSerial softwareserial(RXPin, TXPin);
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;

int wakePin = 4; // pin used for waking up  
int flag = 0;

int get_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  int wakeup_reason_value = 10;
  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : 
        wakeup_reason_value = EXT0_WAKEUP;
        Serial.println("Wakeup caused by external signal using RTC_IO"); 
        break;
    case ESP_SLEEP_WAKEUP_EXT1 : 
        Serial.println("Wakeup caused by external signal using RTC_CNTL"); 
        break;
    case ESP_SLEEP_WAKEUP_TIMER : 
        wakeup_reason_value = TIMER_WAKEUP; 
        Serial.println("Wakeup caused by timer"); 
        break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : 
        Serial.println("Wakeup caused by touchpad"); 
        break;
    case ESP_SLEEP_WAKEUP_ULP : 
        Serial.println("Wakeup caused by ULP program"); 
        break;
    default : 
        Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); 
        break;
  }
  return wakeup_reason_value;
}

void setModemSleep() {
    WiFi.setSleep(true);
}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
  Wire.begin();
  Wire.beginTransmission(address); // Initialize the Tx buffer
  Wire.write(subAddress); // Put slave register address in Tx buffer
  Wire.write(data); // Put data in Tx buffer
  Wire.endTransmission(); // Send the Tx buffer
}

//example showing using readbytev   ----    readByte(MPU6050_ADDRESS, GYRO_CONFIG);
uint8_t readByte(uint8_t address, uint8_t subAddress) {
  uint8_t data; // `data` will store the register data   
  Wire.beginTransmission(address); // Initialize the Tx buffer
  Wire.write(subAddress); // Put slave register address in Tx buffer
  Wire.endTransmission(false); // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1); // Read one byte from slave register address 
  data = Wire.read(); // Fill Rx buffer with result
  return data; // Return data read from slave register
}

void setup() {
  //initialize Serial Monitor
  Serial.println("Initializing ESP32");
  Serial.begin(115200);
  while (!Serial)
    delay(10);
  // Setup a wake interrupt on GPIO 4
  pinMode(4, INPUT_PULLUP);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_4,0);
  softwareserial.begin(GPSBaud);
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
  setupMPU();
}

void setupMPU(){
  writeByte(MPU6050_ADDRESS, 0x6B, 0x00);
  writeByte(MPU6050_ADDRESS, SIGNAL_PATH_RESET, 0x07); //Reset all internal signal paths in the MPU-6050 by writing 0x07 to register 0x68;
  writeByte(MPU6050_ADDRESS, I2C_SLV0_ADDR, 0x20); //write register 0x37 to select how to use the interrupt pin. For an active high, push-pull signal that stays until register (decimal) 58 is read, write 0x20.
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x01); //Write register 28 (==0x1C) to set the Digital High Pass Filter, bits 3:0. For example set it to 0x01 for 5Hz. (These 3 bits are grey in the data sheet, but they are used! Leaving them 0 means the filter always outputs 0.)
  writeByte(MPU6050_ADDRESS, MOT_THR, 10); //Write the desired Motion threshold to register 0x1F (For example, write decimal 20).  
  writeByte(MPU6050_ADDRESS, MOT_DUR, 40); //Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate  
  writeByte(MPU6050_ADDRESS, MOT_DETECT_CTRL, 0x15); //to register 0x69, write the motion detection decrement and a few other settings (for example write 0x15 to set both free-fall and motion decrements to 1 and accelerometer start-up delay to 5ms total by adding 1ms. )   
  writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x40); //write register 0x38, bit 6 (0x40), to enable motion detection interrupt.     
  writeByte(MPU6050_ADDRESS, 0x37, 160); // now INT pin is active low
}

void loop() {
  int wakeUpReason = get_wakeup_reason(); //Print the wakeup reason for ESP32
  double lat = 0.00, lon = 0.00;
  if (wakeUpReason == EXT0_WAKEUP) {
    readByte(MPU6050_ADDRESS, 0x3A);
  }
  int counter = EEPROM.read(0);
  while (softwareserial.available() > 0){
    gps.encode(softwareserial.read());
    Serial.print("Latitude= "); 
    Serial.print(gps.location.lat(), 6);
    lat = gps.location.lat();
    Serial.print(" Longitude= "); 
    Serial.println(gps.location.lng(), 6);
    lon = gps.location.lng();
  }
  //Send LoRa packet to receiver
  LoRa.beginPacket();
  LoRa.print("LAT|");
  LoRa.print(lat);
  LoRa.print("|LON|");
  LoRa.print(lon);
  LoRa.print("|ACCIDENT|");
  LoRa.print(wakeUpReason);
  LoRa.endPacket();
  Serial.print("Sent Lora packet");
  
  delay(50);
  // Initiate low power mode sequence:
  Serial.println("Going to light-sleep now");
  Serial.flush();
  delay(1000);
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); // ESP32 wakes up every 60 seconds 
  esp_light_sleep_start();
}