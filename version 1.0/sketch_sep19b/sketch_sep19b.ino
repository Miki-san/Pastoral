#include <ESP32Time.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Arduino.h>
#include <ESPFlash.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>
//#include <GyverOLED.h>
#include "vl53l1_api.h"
#include "SparkFun_AS7265X.h"

#define SEALEVELPRESSURE_HPA (1013.25)
#define GMTplus3 (10800)
#define uS_TO_S_FACTOR (1000000)
#define TIME_TO_SLEEP (60)

//OLED
//GyverOLED<SSD1306_128x64, OLED_NO_BUFFER> oled;
//BME280
Adafruit_BME280 bme;
float temp[10];
float press[10];
float alt[10];
float hum[10];
//JSON
DynamicJsonDocument doc(10000);
String formattedDate;
String json;
//WIFI
const char* ssid = "Miki";
const char* password = "angmihmax";
AsyncWebServer server(80);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
//RTC
ESP32Time rtc(GMTplus3);
//VL53L1
VL53L1_Dev_t dev;
VL53L1_DEV Dev = &dev;
int vl_status;
VL53L1_UserRoi_t roiConfig;
int matrix_size = 7;
//SOIL
const int ntcPin = 00;
const int ecfPin = 01;
float ntc[10];
float ecf[10];
//AS7265X
AS7265X sensor;
float AS_G[30];
float AS_H[30];
float AS_R[30];
float AS_I[30];
float AS_S[30];
float AS_J[30];
float AS_T[30];
float AS_U[30];
float AS_V[30];
float AS_W[30];
float AS_K[30];
float AS_L[30];
//OTHER
unsigned int delayTime = 100;
unsigned int sensorTime = 5000;
byte count_data = 0;
byte id = 0;
bool flag = false;
bool status;
long counter = 0;

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  Serial.println("-- Default Test --");
  Serial.println();
  //oledInit();
  bmeConnection();
  vl53l1Init();
  AS7265XInit();
  wifiConnection();
  timeClientConnection();
  webSerialConnection();
  delay(5000);
}

void loop() {
  if (id < 10) {
    if (count_data < 30) {
      if (count_data < 10) {
        temp[count_data] = bme.readTemperature();
        press[count_data] = bme.readPressure() / 100.0F;
        alt[count_data] = bme.readAltitude(SEALEVELPRESSURE_HPA);
        hum[count_data] = bme.readHumidity();
        ntc[count_data] = analogRead(ntcPin);
        ecf[count_data] = analogRead(ecfPin);
      }
      sensor.takeMeasurements();
      AS_G[count_data] = sensor.getCalibratedG();  //560nm
      AS_H[count_data] = sensor.getCalibratedH();  //585nm
      AS_R[count_data] = sensor.getCalibratedR();  //610nm
      AS_I[count_data] = sensor.getCalibratedI();  //645nm
      AS_S[count_data] = sensor.getCalibratedS();  //680nm
      AS_J[count_data] = sensor.getCalibratedJ();  //705nm
      AS_T[count_data] = sensor.getCalibratedT();  //730nm
      AS_U[count_data] = sensor.getCalibratedU();  //760nm
      AS_V[count_data] = sensor.getCalibratedV();  //810nm
      AS_W[count_data] = sensor.getCalibratedW();  //860nm
      AS_K[count_data] = sensor.getCalibratedK();  //900nm
      AS_L[count_data] = sensor.getCalibratedL();  //940nm
      count_data++;
      delay(delayTime);
    } else {
      count_data = 0;
      doc["id"] = id;
      formattedDate = String(rtc.getDay()) + "." + String(rtc.getMonth() + 1) + "." + rtc.getTime("%Y %H:%M:%S");
      doc["timestamp"] = formattedDate;
      sorting(temp, 10);
      doc["bme"]["temp"] = median(temp, 10) / 10.0;
      sorting(press, 10);
      doc["bme"]["press"] = median(press, 10) / 10.0;
      sorting(alt, 10);
      doc["bme"]["alt"] = median(alt, 10) / 10.0;
      sorting(hum, 10);
      doc["bme"]["hum"] = median(hum, 10) / 10.0;
      sorting(ntc, 10);
      doc["soil"]["temp"] = median(ntc, 10) / 10.0;
      sorting(ecf, 10);
      doc["soil"]["hum"] = median(ecf, 10) / 10.0;
      sorting(AS_G, 30);
      doc["AS7265X"]["G"] = median(AS_G, 30) / 30.0;
      sorting(AS_H, 30);
      doc["AS7265X"]["H"] = median(AS_H, 30) / 30.0;
      sorting(AS_R, 30);
      doc["AS7265X"]["R"] = median(AS_R, 30) / 30.0;
      sorting(AS_I, 30);
      doc["AS7265X"]["I"] = median(AS_I, 30) / 30.0;
      sorting(AS_S, 30);
      doc["AS7265X"]["S"] = median(AS_S, 30) / 30.0;
      sorting(AS_J, 30);
      doc["AS7265X"]["J"] = median(AS_J, 30) / 30.0;
      sorting(AS_T, 30);
      doc["AS7265X"]["T"] = median(AS_T, 30) / 30.0;
      sorting(AS_U, 30);
      doc["AS7265X"]["U"] = median(AS_U, 30) / 30.0;
      sorting(AS_V, 30);
      doc["AS7265X"]["V"] = median(AS_V, 30) / 30.0;
      sorting(AS_W, 30);
      doc["AS7265X"]["W"] = median(AS_W, 30) / 30.0;
      sorting(AS_K, 30);
      doc["AS7265X"]["K"] = median(AS_K, 30) / 30.0;
      sorting(AS_L, 30);
      doc["AS7265X"]["L"] = median(AS_L, 30) / 30.0;
      vl53l1GetData();
      serializeJsonPretty(doc, Serial);
      Serial.println();
      Serial.println();
      serializeJson(doc, json);
      WebSerial.println(json);
      id++;
      delay(sensorTime);
    }
  } else {
    id = 0;
    Serial.flush();
    delay(TIME_TO_SLEEP * 1000);
  }
  counter++;
  //Serial.println(ESP.getFreeHeap());
}

/*void oledInit() {
  oled.init();
  oled.clear();
  oled.setScale(1);
  oled.home();
}*/

void AS7265XInit() {
  if (sensor.begin() == false) {
    Serial.println("Sensor does not appear to be connected. Please check wiring. Freezing...");
    while (1)
      ;
  }
}

void bmeConnection() {
  status = bme.begin(0x76);
  while (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    status = bme.begin(0x76);
    delay(1000);
  }
}

void vl53l1Init() {
  Dev->I2cDevAddr = 0x52;
  VL53L1_software_reset(Dev);
  vl53l1Instrictions();
  while (vl_status) {
    Serial.println(vl_status);
    vl53l1Instrictions();
    delay(1000);
  };
}

void vl53l1Instrictions() {
  vl_status = VL53L1_WaitDeviceBooted(Dev);
  vl_status = VL53L1_DataInit(Dev);
  vl_status = VL53L1_StaticInit(Dev);
  vl_status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_LONG);
  vl_status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, 200000);
  vl_status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, 50);
  vl_status = VL53L1_StartMeasurement(Dev);
}

void vl53l1GetData() {
  doc["vl53l1x"]["settings"]["m_size"] = matrix_size;
  String num_of_meas = "";
  static VL53L1_RangingMeasurementData_t RangingData;
  vl_status = VL53L1_WaitMeasurementDataReady(Dev);
  if (vl_status == 0) {
    vl_status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
    if (vl_status == 0) {
      for (int y = 0; y < matrix_size; y++) {
        for (int x = 0; x < matrix_size; x++) {
          roiConfig = { 2 * x, (15 - 2 * y), (2 * x + 3), (12 - 2 * y) };
          vl_status = VL53L1_SetUserROI(Dev, &roiConfig);
          vl_status = VL53L1_WaitMeasurementDataReady(Dev);
          if (!vl_status) vl_status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
          VL53L1_clear_interrupt_and_enable_next_range(Dev, VL53L1_DEVICEMEASUREMENTMODE_SINGLESHOT);
          if (vl_status == 0) {
            if ((RangingData.RangeMilliMeter < 1000) and (RangingData.RangeMilliMeter > 99)) {
              //Serial.print(" ");
            } else if ((RangingData.RangeMilliMeter < 100) and (RangingData.RangeMilliMeter > 9)) {
              //Serial.print("  ");
            }
            num_of_meas += String(RangingData.RangeMilliMeter);
            if (x != 6) {
              num_of_meas += " ";
            }
          }
        }
        doc["vl53l1x"]["data"][y] = num_of_meas;
        num_of_meas = "";
      }
      vl_status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
    }
  }
}

void wifiConnection() {
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  //oled.print(WiFi.localIP());
}

void timeClientConnection() {
  timeClient.begin();
  while (!timeClient.update()) {
    timeClient.forceUpdate();
    delay(100);
  }
  rtc.setTime(timeClient.getEpochTime());
}

void webSerialConnection() {
  WebSerial.begin(&server);
  WebSerial.msgCallback(recvMsg);
  server.begin();
}

int sort_desc(const void* cmp1, const void* cmp2) {
  float b = *((float*)cmp1);
  float a = *((float*)cmp2);
  return a > b ? -1 : (a < b ? 1 : 0);
}

void sorting(float* arraySort, int arraySize) {
  qsort(arraySort, arraySize, sizeof(float), sort_desc);
}

void printArray(float* arrayPrint, int arraySize) {
  for (int i = 0; i < arraySize; i++) {
    Serial.print(arrayPrint[i]);
    Serial.print(" ");
  }
  Serial.println();
}

long median(float* arrayMed, int arraySize) {
  int med = arraySize / 2 - 1;

  return (long)((arrayMed[med] + arrayMed[med + 1]) * (arraySize / 2));
}

void recvMsg(uint8_t* data, size_t len) {
  WebSerial.println("Received Data...");
  String d = "";
  for (int i = 0; i < len; i++) {
    d += char(data[i]);
  }
  WebSerial.println(d);
}
