#include <Arduino.h>  //Librerias
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_BMP280.h>
#include <Statistic.h>

#define CHANNEL 1 //Comunicacion y control
esp_now_peer_info_t slave;
bool dataSent = false;

#define uS_TO_S_FACTOR 1000000  //Manejo de hibernación
uint16_t TIME_TO_SLEEP = 300;
uint64_t real_sleep = TIME_TO_SLEEP * uS_TO_S_FACTOR;
uint16_t maxSmpl = 100;
long previousMillis = 0;
uint8_t intentos = 10; 

#define SDA_S 9  //Hardware ATMÓSFERA Perif
#define SCL_S 8  
//#define SDA_S 7  //Hardware ATMÓSFERA Central
//#define SCL_S 6 
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_SHT31 sht31;
Adafruit_BMP280 bmp; //


typedef struct Estructura {
  uint8_t id;
  float data;
  bool status;
} Estructura;
Estructura TValue;
Estructura HRValue;
Estructura PValue;


void ScanForSlave(){ //Conexión con central
  uint8_t scanResults = WiFi.scanNetworks();
  for (int i = 0; i < scanResults; ++i){
    String SSID = WiFi.SSID(i);
    String BSSIDstr = WiFi.BSSIDstr(i);
    if (SSID.indexOf("TX") == 0) {
      int mac[6];
      //if (6 == sscanf(BSSIDstr, "%x:%x:%x:%x:%x:%x", &mac[1],&mac[2],&mac[3],&mac[4],&mac[5],&mac[6])){
      if (6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x", &mac[0],&mac[1],&mac[2],&mac[3],&mac[4],&mac[5])){
        for (int ii = 0; ii < 6; ++ii){
          slave.peer_addr[ii] = (uint8_t) mac[ii];
          //Serial.println(mac[ii]);
        }
      }
      slave.channel = CHANNEL;
      slave.encrypt = 0;
      break;
    }
  }
}

void SensTemp(){ //Lecturas e interpretación.
  
  float t = sht31.readTemperature();
  if (! isnan(t)) {  
    TValue.id = 13;
    TValue.data = t;
    TValue.status= true;
  }
  if(TValue.status){
    while(!dataSent){
      ScanForSlave();
      esp_now_add_peer(&slave);
      delay(100);
      esp_now_send(slave.peer_addr, (uint8_t *) &TValue, sizeof(TValue));
      Serial.println("WL");
      delay(100);
    }
  }
}
void SensHumi(){
  float h = sht31.readHumidity();
  if (! isnan(h)) {  
    HRValue.id = 14;
    HRValue.data = h;
    HRValue.status = true;    
  }
  if(HRValue.status){
    while(!dataSent){
      ScanForSlave();
      esp_now_add_peer(&slave);
      delay(100);
      esp_now_send(slave.peer_addr, (uint8_t *) &HRValue, sizeof(HRValue));
      Serial.println("WL");
      delay(100);
    }
  }
}
void SensPres(){
  float p = bmp.readPressure() / 100.0F;
  if (! isnan(p)) {  
    PValue.id = 15;
    PValue.data = p;
    PValue.status = true;    
  }
  if(PValue.status){
    while(!dataSent){
      ScanForSlave();
      esp_now_add_peer(&slave);
      delay(100);
      esp_now_send(slave.peer_addr, (uint8_t *) &PValue, sizeof(PValue));
      Serial.println("WL");
      delay(100);
    }
  }
  if(TValue.status && HRValue.status && PValue.status){
    esp_sleep_enable_timer_wakeup(real_sleep);
    //Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
    Serial.println("Sleeping");
    Serial.flush();
    esp_deep_sleep_start();
  }
} 
void SendESPNOW(const uint8_t *macAddr, esp_now_send_status_t status){
    status == ESP_NOW_SEND_SUCCESS ? dataSent = true : dataSent = false;
} 
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_S, SCL_S);
  WiFi.mode(WIFI_STA);
  delay(500);
  esp_now_init();
  esp_now_register_send_cb(SendESPNOW);
  ScanForSlave();
  while(esp_now_add_peer(&slave) != ESP_OK){
    Serial.println("Peer");
    delay(100);
  }

  while(!sht31.begin(0x44)) {   // Set to 0x45 for alternate I2C address
    Serial.println("Couldn't find SHT31");
    delay(100);
  }
  while(!bmp.begin(0x76)) {
    Serial.println("Could not find  BMP280");
    delay(100);
  }
  TValue.status = false;
  HRValue.status = false;
  PValue.status = false;
}
void loop() { 
  long currentMillis = millis();
  if(currentMillis - previousMillis >= 1000){
    SensTemp();
    SensHumi();
    SensPres();
    previousMillis = currentMillis;
  }
  delay(800);
}
