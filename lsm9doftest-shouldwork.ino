#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <WiFi101.h>
#include <PubSubClient.h>
#include <avr/dtostrf.h>
#include <ArduinoJson.h>
#include "credentials.h"

#define TOPIC "uark/csce5013/KneeCompression"
#define accel1TOPIC "uark/csce5013/KneeCompression/accel1"
#define accel2TOPIC "uark/csce5013/KneeCompression/accel2"
#define USERNAME "crp016"
#define CONVERT_G_TO_MS2 9.80665f
#define FREQUENCY_HZ 104
#define INTERVAL_MS (1000 / (FREQUENCY_HZ + 1))

char ssid[] = SSID;
char password[] = PASSKEY;
int status = WL_IDLE_STATUS;
const char* mqtt_server = "broker.hivemq.com";
String accelx_str;
StaticJsonDocument<256> doc1;
StaticJsonDocument<256> doc2;
static char payload1[256];
static char payload2[256];

WiFiClient wifiClient;
PubSubClient client(wifiClient);

long lastMsg = 0;
char msg[100];
int value = 0;

// i2c
Adafruit_LSM9DS0 lsm1 = Adafruit_LSM9DS0();
Adafruit_LSM9DS0 lsm2 = Adafruit_LSM9DS0();

// You can also use software SPI
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(13, 12, 11, 10, 9);
// Or hardware SPI! In this case, only CS pins are passed in
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(10, 9);

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm1.setupAccel(lsm1.LSM9DS0_ACCELRANGE_2G);
  lsm2.setupAccel(lsm2.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm1.setupMag(lsm1.LSM9DS0_MAGGAIN_2GAUSS);
  lsm2.setupMag(lsm2.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm1.setupGyro(lsm1.LSM9DS0_GYROSCALE_245DPS);
  lsm2.setupGyro(lsm2.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

void setup_wifi() {
  delay(1000);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  while (status != WL_CONNECTED) {
    status = WiFi.begin(ssid, password);
  }

  Serial.println("");
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  if(length >2){
    for (int i = 0; i < length; i++){
      Serial.print((char)payload[i]);
    }
  }
  Serial.println("");
}

void setup() 
{
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif
  Serial.begin(9600);
  Serial.println("LSM raw read demo");
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm1.begin() ||  !lsm2.begin())
  {
    Serial.println("Oops ... unable to initialize both of the LSM9DS0 sensors. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS0 9DOF");
  Serial.println("");
  Serial.println("");
  Serial.println("Setting up LSM9DS0 9DOF");
  setupSensor();
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  delay(1);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(USERNAME)) {
      Serial.println("connected");
      // Once connected, subscribe to input channel
      client.subscribe(TOPIC);
      client.subscribe(accel1TOPIC);
      client.subscribe(accel2TOPIC);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop() 
{
  if (!client.connected()) {
    reconnect();
  }
  static unsigned long last_interval_ms1 = 0;
  static unsigned long last_interval_ms2 = 0;

  if (millis() > last_interval_ms1 + 500)
  {
    lsm1.read();
    doc1["ACC1_X"] = lsm1.accelData.x;
    doc1["ACC1_Y"] = lsm1.accelData.y;
    doc1["ACC1_Z"] = lsm1.accelData.z;
    serializeJsonPretty(doc1, payload1);
    client.publish(accel1TOPIC, payload1);
    
    last_interval_ms1 = millis();
  }

  if (millis() > last_interval_ms2 + 500)
    {
      lsm2.read();
      doc2["ACC2_X"] = lsm2.accelData.x;
      doc2["ACC2_Y"] = lsm2.accelData.y;
      doc2["ACC2_Z"] = lsm2.accelData.z;
      serializeJsonPretty(doc2, payload2);
      client.publish(accel2TOPIC, payload2);
      last_interval_ms2 = millis();
    }

  client.loop();
  
  //Serial.print("Accel X: "); Serial.print((int)lsm.accelData.x); Serial.println(" ");
//  Serial.print("Y: "); Serial.print((int)lsm.accelData.y);       Serial.print(" ");
//  Serial.print("Z: "); Serial.println((int)lsm.accelData.z);     Serial.print(" ");
//  Serial.print("Mag X: "); Serial.print((int)lsm.magData.x);     Serial.print(" ");
//  Serial.print("Y: "); Serial.print((int)lsm.magData.y);         Serial.print(" ");
//  Serial.print("Z: "); Serial.println((int)lsm.magData.z);       Serial.print(" ");
//  Serial.print("Gyro X: "); Serial.print((int)lsm.gyroData.x);   Serial.print(" ");
//  Serial.print("Y: "); Serial.print((int)lsm.gyroData.y);        Serial.print(" ");
//  Serial.print("Z: "); Serial.println((int)lsm.gyroData.z);      Serial.println(" ");
//  Serial.print("Temp: "); Serial.print((int)lsm.temperature);    Serial.println(" ");
  delay(100);
}
