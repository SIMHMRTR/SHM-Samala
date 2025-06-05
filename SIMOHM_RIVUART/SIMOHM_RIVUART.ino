#include <WiFi.h>     //Librerias
#include <esp_now.h>
#include <HardwareSerial.h>
#include <Statistic.h>


#define CHANNEL 1 //Comunicacion y control
//uint8_t broadcastMacAdd[] = {0x7C,0x9E,0xBD,0x62,0x03,0xD4}; 
esp_now_peer_info_t slave;
bool dataSent = false;

#define uS_TO_S_FACTOR 1000000  //Manejo de hibernación
uint16_t TIME_TO_SLEEP = 30;
uint64_t real_sleep = TIME_TO_SLEEP * uS_TO_S_FACTOR;
uint16_t maxSmpl = 200;
long previousMillis = 0;
uint8_t intentos = 10; 

const int pinRX = 8; // HARDWARE RIO
const int pinTX = 7;
unsigned char data_buffer[4] = {0};

// Integer to store distance
int distance = 0;
float d = 0.0f;

// Variable to hold checksum
unsigned char CS;

// Object to represent software serial port
//HardwareSerial Serial1(1);
typedef struct Estructura {
  uint8_t id;
  float data;
  bool status;
} Estructura;

Estructura WLValue;

statistic::Statistic<float, uint32_t, true> WLStats; //Procesamiento

typedef struct nowTask {    // Intrucciones de control
  char samples;
  char sleep;
} nowTask;
nowTask taskRiver;

void ScanForSlave(){    //Conexión con central
  uint8_t scanResults = WiFi.scanNetworks();
  for (int i = 0; i < scanResults; ++i){
    String SSID = WiFi.SSID(i);
    String BSSIDstr = WiFi.BSSIDstr(i);
    if (SSID.indexOf("RX") == 0) {
      int mac[6];
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
void RecvESPNOW(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len){ //Funciones callback
  memcpy(&taskRiver, incomingData, sizeof(taskRiver));
}
void SendESPNOW(const uint8_t *macAddr, esp_now_send_status_t status){
    status == ESP_NOW_SEND_SUCCESS ? dataSent = true : dataSent = false;
} 

void hidrometricuart(){
  
  if (Serial1.available() > 0) {     // If anything comes in Serial1 (pins 4 & 5)
    delay(4);
    // Check for packet header character 0xff

    if (Serial1.read() == 0xff) {
      // Insert header into array
      data_buffer[0] = 0xff;
      // Read remaining 3 characters of data and insert into array
      for (int i = 1; i < 4; i++) {
        data_buffer[i] = Serial1.read();
      }
      //Compute checksum
      CS = data_buffer[0] + data_buffer[1] + data_buffer[2];
      // If checksum is valid compose distance from data
      if (data_buffer[3] == CS) {
        distance = (data_buffer[1] << 8) + data_buffer[2];
        d = float(distance) / 10.0;
        
        Serial.print("Distancia: ");
        Serial.println(d);
        if(distance >= 250){
          WLValue.data = d;
          WLValue.status = true;
        }
      }

    }
  }
          
  if(WLValue.status){
    while(!dataSent){
      ScanForSlave();
      esp_now_add_peer(&slave);
      delay(100);
      esp_now_send(slave.peer_addr, (uint8_t *) &WLValue, sizeof(WLValue));
      Serial.println("WL");
    }
    esp_sleep_enable_timer_wakeup(real_sleep);
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
    Serial.println("Going to sleep now");
    Serial.flush();
    esp_deep_sleep_start();
  }
 
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600,SERIAL_8N1, pinRX, pinTX); 
  
  //Serial1.begin(9600,pinRX, pinTX);
  delay(100);
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_send_cb(SendESPNOW);
  esp_now_register_recv_cb(RecvESPNOW);
  ScanForSlave();
  while(esp_now_add_peer(&slave) != ESP_OK){
    delay(100);
    Serial.println("Peer");
  }
  
  WLValue.id = 11; 
  WLValue.status = false;
  Serial.println("Setup complete.");
  delay(100);
}
void loop() {
  //hidrometric();
  hidrometricuart();
  delay(300);
}