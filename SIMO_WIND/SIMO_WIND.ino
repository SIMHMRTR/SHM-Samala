#include <WiFi.h> //Librerias
#include <esp_now.h>
#include <Statistic.h>
#include "AS5600.h"

#define CHANNEL 1 //Comunicación y control
esp_now_peer_info_t slave;
bool dataSent = false;

#define uS_TO_S_FACTOR 1000000  //Manejo de hibernación
uint16_t TIME_TO_SLEEP = 10;
uint64_t real_sleep = TIME_TO_SLEEP * uS_TO_S_FACTOR;
uint16_t maxSmpl = 25;
long previousMillis = 0;

#define SDA_D 8 //Harware Viento Central
#define SCL_D 9
#define SDA_V 6
#define SCL_V 7 
AS5600 as5600;  //Hardware Viento

typedef struct Estructura {   //Datos para transmitir por aire.
  uint8_t station;
  uint8_t variable;
  uint16_t priority;
  float recent;
  float average;
  float minimum;
  float maximum;
  float standard_dev;

} Estructura;
Estructura direccion;
Estructura velocidad;

statistic::Statistic<float, uint32_t, true> StatsDir; //Procesamiento
statistic::Statistic<float, uint32_t, true> StatsVel; //Procesamiento
float angulo = 0.0f;
float velo = 0.0f;
bool dataSelect = true; //true = velocidad, false = dirección.
bool selected = false;
unsigned long lastTime = 0;  
unsigned long timerDelay = 20;  // send readings timer miliseconds

typedef struct nowTask {    // Intrucciones de control
  char samples;
  char sleep;
} nowTask;
nowTask taskViento;

int magnet_status = 0;

void ScanForSlave(){
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
void SendESPNOW(const uint8_t *macAddr, esp_now_send_status_t status){
    status == ESP_NOW_SEND_SUCCESS ? dataSent = true : dataSent = false;
} 

void RecvESPNOW(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len){
  memcpy(&taskViento, incomingData, sizeof(taskViento));
  Serial.println("Muestras");
  Serial.println(taskViento.samples);

  Serial.println("Tiempo sueño");
  Serial.println(taskViento.sleep);

}
void readVelocity(){
  if ((millis() - lastTime) > timerDelay) {
    Serial.print("V ");
    delay(50);
    velo = as5600.getAngularSpeed(); //* 0.0037699;
    if(!isnan(velo) ){    
      if(velo >= 0){
        StatsVel.add(velo);
        Serial.println(velo,5);
      }
    }else{
      Serial.print("Failed to read velocity");
    }
    lastTime = millis();
  }
} 
void readDirection(){
  if ((millis() - lastTime) > timerDelay) {
    Serial.print("D ");
    delay(50);
    angulo = as5600.rawAngle() * AS5600_RAW_TO_DEGREES;
    if(!isnan(angulo)){
      StatsDir.add(angulo);
      Serial.println(angulo);
    }else{
      Serial.println("Failed to read direction");
    }
    lastTime = millis();
  }
}
void VelStatCheck(){
  if(StatsVel.count() >= maxSmpl*8){
    velocidad.station = 1;
    velocidad.variable = 6;
    velocidad.priority = 6;
    velocidad.recent = velo;
    velocidad.average = StatsVel.average();
    velocidad.minimum = StatsVel.minimum();
    velocidad.maximum = StatsVel.maximum();
    velocidad.standard_dev = StatsVel.pop_stdev(); 
    esp_now_send(slave.peer_addr, (uint8_t *) &velocidad, sizeof(velocidad));
    delay(500);
    while(!dataSent){
        ScanForSlave();
        esp_now_add_peer(&slave);
        delay(100);
        esp_now_send(slave.peer_addr, (uint8_t *) &velocidad, sizeof(velocidad));
        Serial.println("V");
        delay(500);
    }
    StatsVel.clear();
    dataSent = false; 
    dataSelect = false;
    selected = false;
    Wire.end();
  }
}
void DirStatCheck(){
  if(StatsDir.count() >= maxSmpl){
    
    direccion.station = 1;
    direccion.variable = 7;
    direccion.priority = 6;
    direccion.recent = angulo;
    direccion.average = StatsDir.average();
    direccion.minimum = StatsDir.minimum();
    direccion.maximum = StatsDir.maximum();
    direccion.standard_dev = StatsDir.pop_stdev(); 
    esp_now_send(slave.peer_addr, (uint8_t *) &direccion, sizeof(direccion));
    delay(500);
    while(!dataSent){
        ScanForSlave();
        esp_now_add_peer(&slave);
        delay(100);
        esp_now_send(slave.peer_addr, (uint8_t *) &direccion, sizeof(direccion));
        Serial.println("D");
        delay(500);
    }
    StatsDir.clear();
    dataSent = false;
    dataSelect = true;
    selected = false;
    Wire.end();
    esp_sleep_enable_timer_wakeup(real_sleep);
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
    Serial.println("Going to sleep now");
    Serial.flush();
    esp_deep_sleep_start();
  }
}
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  delay(500);
  esp_now_init();
  esp_now_register_send_cb(SendESPNOW);
  esp_now_register_recv_cb(RecvESPNOW);
  ScanForSlave();
  while(esp_now_add_peer(&slave) != ESP_OK){
    delay(100);
  }
  StatsDir.clear();
  StatsVel.clear();
  delay(1000);
}
 
void loop() {
  if (dataSelect) {
    if(!selected){
      Wire.begin(SDA_V, SCL_V);
      Wire.setClock(800000L);
      timerDelay = 20;
      selected = true;
      delay(100);
      as5600.begin(4);
    }
    readVelocity();
    VelStatCheck();
  }else{ 
    if(!selected){
      Wire.begin(SDA_D, SCL_D);
      Wire.setClock(800000L);
      timerDelay = 500;
      selected = true;
      delay(100);
      as5600.begin(4);
    } 
    readDirection();
    DirStatCheck();  
  }
}