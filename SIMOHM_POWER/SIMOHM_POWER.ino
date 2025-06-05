#include <Arduino.h>  //Librerias
#include <WiFi.h>
#include <esp_now.h>
#include <ArduinoJson.h>

#define CHANNEL 1 
esp_now_peer_info_t slave;


#define rio 25
#define atmo 26
#define viento 32
#define lluvia 33
#define lluvSgn 27

typedef struct Estructura {
  uint8_t id;
  float data;
  bool status;
} Estructura;
Estructura RecvStruct; //ESP-NOW Arie
String recibido = ""; //Serial USB
bool vari[9] = {false, false, false, false, false, false, false, false, false};

void RecvESPNOW(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len){
  memcpy(&RecvStruct, incomingData, sizeof(RecvStruct));
  JsonDocument doc;
  doc["id"] = RecvStruct.id;
  doc["dt"] = RecvStruct.data;
  doc["sts"] = RecvStruct.status;
  if(serializeJson(doc, Serial) == 0) {
    Serial.println(F("Err"));
  }
  if(RecvStruct.status){
    vari[(RecvStruct.id % 10) - 1] = true;
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.begin();
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("RX_2","RX_2_PSWRD", CHANNEL, 0); 
  esp_now_init();
  esp_now_register_recv_cb(RecvESPNOW);
  pinMode(rio, OUTPUT);
  digitalWrite(rio, 0);
  pinMode(atmo, OUTPUT);
  digitalWrite(atmo, 0);
  pinMode(viento, OUTPUT);
  digitalWrite(viento, 0);
  pinMode(lluvia, OUTPUT);
  digitalWrite(lluvia, 1);
  pinMode(lluvSgn, OUTPUT);
  digitalWrite(lluvSgn, 1);
  
}
void loop() { 
  listen_serial();
  if(vari[0])digitalWrite(rio, 0);
  if(vari[2] && vari[3] && vari[4]) digitalWrite(atmo, 0);
  if(vari[5] && vari[6]) digitalWrite(viento, 0);
}

void listen_serial(){
  while (Serial.available()) {
      char incomingChar = Serial.read();  // Read each character from the buffer
      if (incomingChar == '\n') {  // Check if the user pressed Enter (new line character)
        int len = recibido.length();
        if(len == 3){
          bool state = false;;
 //         Serial.println("msj: ");
 //         Serial.print(recibido);
          if(recibido[0] == '0')state = false;
          if(recibido[0] == '1')state = true;
          switch(recibido[1]){
            case 'T':
              digitalWrite(rio, state);
              digitalWrite(atmo, state);
              digitalWrite(viento, state);
              if(state){
                for(int i = 0; i < 9; i++){
                    vari[i] = false;
                }
                digitalWrite(lluvSgn, 0);Kare
                delay(250);
                digitalWrite(lluvSgn, 1);
              }
              //digitalWrite(lluvia, state);
              break;
            case 'R':
              digitalWrite(lluvia, 0);
              delay(250);
              digitalWrite(lluvia, 1);
              break;
            default:
              break;
          }
        }
        recibido = "";
      } else {
        recibido += incomingChar;
      }
    }
}