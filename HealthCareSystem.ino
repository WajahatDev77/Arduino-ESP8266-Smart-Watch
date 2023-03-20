//adding TFT Screen Libraries
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>
#include <SD.h>
#define REPORTING_PERIOD_MS 5000
uint32_t tsLastReport = 0;

//Wifi Libraries
#include <ESP8266WiFi.h>

//adding firebase libraires
#include <FirebaseCloudMessaging.h>
#include <Firebase.h>
#include <FirebaseHttpClient.h>
#include <FirebaseArduino.h>
#include <FirebaseError.h>
#include <FirebaseObject.h>
#include <FirebaseArduino.h>
#include <ArduinoJson.h>
#include <ESP8266HTTPClient.h>
//Define an NTP client to get date and time.
#include <NTPClient.h>
#include <WiFiUdp.h>

//Online time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);// Define NTP Client to get time

//Heart Rate Libraries
#include <Wire.h>
#include <ESP8266Webhook.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;
#define MAX_BRIGHTNESS 255

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid


#define TFT_CS  0    // Chip select line for TFT display  Change according to the ESP8266  GPIO0-D3
#define TFT_RST  2  // Reset line for TFT (or see below...)  GPIO2-D4
#define TFT_DC   15  // Data/command line for TFT   GPIO15-D8
//GPIO14-D5-HSCLK for SCK and GPIO13-D7-HMOSI for SDA

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

const char* ssid     = "OPPO A54";
const char* password = "agent123";
#define FIREBASE_AUTH "FzL3WKDQ6GUUyGpzoPbBZYrdHgOw845t9oyC0vtq"
#define FIREBASE_HOST "health-app-f2ec5-default-rtdb.firebaseio.com"

String formattedDate; //store the date formate
String currentDate; //store current date
String currentTime; //store current time

//unsigned long sendDataPrevMillis = 0;
//int count = 0;
//WiFiServer server(80);

unsigned char bloodPressure[10];
int i = 0;
int xPos = 0;

void send_event(const char *event, String value);
const char *host = "maker.ifttt.com";
const char *privateKey = "oqSsMYFY2de_AlFpKyFGKfl9oZbuZaCGQ-TxHbde3uU";

#define KEY "oqSsMYFY2de_AlFpKyFGKfl9oZbuZaCGQ-TxHbde3uU"        // Webhooks Key
#define EVENT "Alert!"      // Webhooks Event Name

Webhook webhook(KEY, EVENT);    // Create an object.

void setup()
{
  Serial.begin(115200);

  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1); // Landscape
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 10);
  tft.println("IoT Health Care System");

  delay(1000);

  // Initialize sensor
  while (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    tft.println(F("MAX30100 was not found. Please check wiring/power."));
    delay(100);
  }


  //byte ledBrightness = 180;
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  //byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte sampleAverage = 2;
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  delay(1000);

  WiFi.begin(ssid, password);
  tft.println("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    tft.print(".");
    delay(300);
  }
  tft.println();
  tft.print("Connected with IP: ");
  tft.println(WiFi.localIP());
  tft.println();


  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps
  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 10);

  timeClient.begin();
  timeClient.setTimeOffset(18000); //Pakistan time offset
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);

}

void loop()
{

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
  for (byte i = 25; i < 100; i++)
  {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25] = irBuffer[i];
  }

  //take 25 sets of samples before calculating the heart rate.
  for (byte i = 75; i < 100; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

  }

  //After gathering 25 new samples recalculate HR and SP02
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  if (millis() - tsLastReport > REPORTING_PERIOD_MS)
  {
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(0, 10);

    particleSensor.enableDIETEMPRDY(); //Enable the temp ready interrupt. This is required.
    float temperature = particleSensor.readTemperature();

    Serial.write(0xFD);
    delay(10);
    Serial.write(0x20);
    delay(10);

    Serial.write(0);
    delay(10);
    Serial.write(0x20);
    delay(10);

    Serial.write(0);
    delay(10);
    Serial.write(0x20);
    delay(10);

    Serial.write(0);
    delay(10);
    Serial.write(0x20);
    delay(10);

    Serial.write(0);
    delay(10);
    Serial.write(0x20);
    delay(10);

    Serial.write(0);
    delay(10);
    Serial.write(0x20);
    delay(10);

    i = 0;
    while (Serial.available() > 0)
    {
      bloodPressure[i] = Serial.read();
      i++;
      delay(10);
    }

    tft.print("Systolic Blood Pressure (mmHg): ");
    tft.println(bloodPressure[1]);
    tft.print("diastolic Blood Pressure (mmHg): ");
    tft.println(bloodPressure[2]);
    tft.print("Heart Rate (Bpm): ");
    tft.println(bloodPressure[3]);
    tft.print("spO2: ");
    if (validSPO2)
    {
      tft.println(spo2);
    }
    else
    {
      tft.println("INVALID");
    }
    tft.print("Body Temperature: ");
    tft.println(temperature);

    GetCurrentDateTime(); //Get time and date
    //sending data to firebase
    if (bloodPressure[1] != 0 && bloodPressure[1] != 255 && bloodPressure[2] != 0 && bloodPressure[2] != 255 && bloodPressure[3] != 0 && bloodPressure[3] != 255) {
      String addBP = "Doctor/-NDRulOcLUH-IfSv_xS4/AllSensorsData/Date/" + currentDate + "/BloodPressure/" + currentTime;
      Firebase.setInt (addBP, bloodPressure[1]);
      String addHR = "Doctor/-NDRulOcLUH-IfSv_xS4/AllSensorsData/Date/" + currentDate + "/HeartRate/" + currentTime;
      Firebase.setInt (addHR, bloodPressure[3]);
      String addSpO2 = "Doctor/-NDRulOcLUH-IfSv_xS4/AllSensorsData/Date/" + currentDate + "/SpO2/" + currentTime;
      Firebase.setInt (addSpO2, spo2);
      String addTemp = "Doctor/-NDRulOcLUH-IfSv_xS4/AllSensorsData/Date/" + currentDate + "/Temperature/" + currentTime;
      Firebase.setFloat (addTemp, temperature);
    }

    if (bloodPressure[1] != 255)
    {
      Firebase.setFloat ("Doctor/-NDRulOcLUH-IfSv_xS4/BP", bloodPressure[1]);
      Firebase.setFloat ("Doctor/-NDRulOcLUH-IfSv_xS4/DBP", bloodPressure[2]);
      Firebase.setFloat ("Doctor/-NDRulOcLUH-IfSv_xS4/HR", bloodPressure[3]);
    }
    if (validSPO2)
    {
      Firebase.setFloat ("Doctor/-NDRulOcLUH-IfSv_xS4/SpO2", spo2);
    }
    Firebase.setFloat ("Doctor/-NDRulOcLUH-IfSv_xS4/Temp", temperature);

    if (spo2 == 100) {
      String value1 = "Maximum_SpO2: " + String(spo2);
      send_event("Alert!", value1);
    }
    if(temperature == 40){
      String value1 = "High_Temperature_Detected: " + String(temperature);
      send_event("Alert!", value1);
      }
    tsLastReport = millis();
    particleSensor.disableDIETEMPRDY(); //Enable the temp ready interrupt. This is required.
  }
  delay(1000);
}

void GetCurrentDateTime() {
  //while condition to start updating the timeclient forcefully if it is not updated
  while (!timeClient.update()) {
    timeClient.forceUpdate();
  }

  formattedDate = timeClient.getFormattedDate(); //getting the date
  //Serial.println(formattedDate); //printing the formated date

  // Extract date
  int splitT = formattedDate.indexOf("T");
  currentDate = formattedDate.substring(0, splitT);

  // Extract time
  currentTime = formattedDate.substring(splitT + 1, formattedDate.length() - 1);

}

void send_event(const char *event, String value)
{
  Serial.print("Connecting to ");
  Serial.println(host);
  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  const int httpPort = 80;
  if (!client.connect(host, httpPort)) {
    Serial.println("Connection failed");
    return;
  }
  // We now create a URI for the request
  String url = "/trigger/";
  url += event;
  url += "/with/key/";
  url += privateKey;
  url += "?value1=";
  url += value;
  Serial.print("Requesting URL: ");
  Serial.println(url);
  // This will send the request to the server
  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "Connection: close\r\n\r\n");
  while (client.connected())
  {
    if (client.available())
    {
      String line = client.readStringUntil('\r');
      Serial.print(line);
    } else {
      // No data yet, wait a bit
      delay(50);
    };
  }
}
