#include <WiFi.h>
#include <esp_now.h>

uint8_t broadcastAddress[] = { 0x7C, 0x9E, 0xBD, 0x62, 0x03, 0xD4 };  //Dirección del dispositivo destino.
esp_now_peer_info_t slave;
#define CHANNEL 1

bool dataSent = false;
bool rainSent = false;
bool reading = false;
bool sending = false;
bool reseting = false;
#define sendPin 5
#define pluviometerPin 4
#define pwrPin 3
#define pHpin 2
#define soilPin 1  //Pin No. 14 de interrupción ingreso datos de pluviometro
volatile int pulseCounter = 0;
int contador = 0;
const int interval = 100;
unsigned long previousMillis = 0;
long tiempoReporte = 0;
uint8_t maxSmpl = 100;

unsigned long lastTime = 0;
unsigned long timerDelay = 1000;

float calibration_value = 21.34;
int phval = 0; 
unsigned long int avgval; 
int buffer_arr[10],temp;

typedef struct Estructura {
  uint8_t id;
  float data;
  bool status;
} Estructura;

Estructura Pluvio;
Estructura pH;
Estructura SoilH;

void ScanForSlave() {
  uint8_t scanResults = WiFi.scanNetworks();
  for (int i = 0; i < scanResults; ++i) {
    String SSID = WiFi.SSID(i);
    String BSSIDstr = WiFi.BSSIDstr(i);
    if (SSID.indexOf("RX") == 0) {
      int mac[6];
      //if (6 == sscanf(BSSIDstr, "%x:%x:%x:%x:%x:%x", &mac[1],&mac[2],&mac[3],&mac[4],&mac[5],&mac[6])){
      if (6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5])) {
        for (int ii = 0; ii < 6; ++ii) {
          slave.peer_addr[ii] = (uint8_t)mac[ii];
          //Serial.println(mac[ii]);
        }
      }
      slave.channel = CHANNEL;
      slave.encrypt = 0;
      break;
    }
  }
}
void SendESPNOW(const uint8_t *macAddr, esp_now_send_status_t status) {
  status == ESP_NOW_SEND_SUCCESS ? dataSent = true : dataSent = false;
}


//ICACHE_RAM_ATTR 
void IRAM_ATTR contadorPulsos(){

  unsigned long currentMillis = millis();
  if ((currentMillis - previousMillis) >= interval) {
    //long delta_time = currentMillis - previousMillis;
    //float minutos = float(delta_time) / 60000.0;
    previousMillis = currentMillis;
    pulseCounter++;
  }
}
void IRAM_ATTR enviar(){
  
    Pluviometro();
}
float prevRain = 0.0f; //Crecimiento
float accumRain = 0.0f;

void Pluviometro() {
    const float mm = 0.1045;              // calibrar la cantidad de mm que cae en el balancin
    accumRain = pulseCounter * mm;  //Litros por m2 caidos en el lapso de reporte
    //float delta = accumRain - prevRain;
    //prevRain = accumRain;
    Pluvio.id = 12;
    Pluvio.data = accumRain;
    esp_now_send(slave.peer_addr, (uint8_t *) &Pluvio, sizeof(Pluvio));
    delay(200);
    while(!dataSent){
      ScanForSlave();
      esp_now_add_peer(&slave);
      delay(100);
      esp_now_send(slave.peer_addr, (uint8_t *) &Pluvio, sizeof(Pluvio));
      Serial.println("Ll");
      delay(100);
    }
}

void pHread(){
    //Serial.println("Leyendo"); 
    for(int i=0;i<10;i++){ 
        buffer_arr[i]=analogRead(pHpin);
        delay(30);
    }
    for(int i=0;i<9;i++){
        for(int j=i+1;j<10;j++){
            if(buffer_arr[i]>buffer_arr[j]){
                temp=buffer_arr[i];
                buffer_arr[i]=buffer_arr[j];
                buffer_arr[j]=temp;
            }
        }
    }
    avgval=0;
    for(int i=2;i<8;i++) avgval+=buffer_arr[i];
    float volt=(float)avgval*3.3/4096/6;
    float ph_act = -5.70 * volt + calibration_value;
    pH.id = 19;
    pH.data = ph_act; 
    esp_now_send(slave.peer_addr, (uint8_t *) &pH, sizeof(pH));
    delay(200);
    while(!dataSent){
      ScanForSlave();
      esp_now_add_peer(&slave);
      delay(100);
      esp_now_send(slave.peer_addr, (uint8_t *) &pH, sizeof(pH));
      Serial.println("pH");
      delay(100);
    }
    
}
void Soilread(){
    int suelo = analogRead(soilPin);
    Serial.print("Suelo ");
    Serial.println(suelo);
    float Ssuelo = (float)suelo/1.0;
    SoilH.id = 18;
    SoilH.data = Ssuelo;
    esp_now_send(slave.peer_addr, (uint8_t *) &SoilH, sizeof(SoilH));
    delay(500);
    while(!dataSent){
      ScanForSlave();
      esp_now_add_peer(&slave);
      delay(100);
      esp_now_send(slave.peer_addr, (uint8_t *) &SoilH, sizeof(SoilH));
      Serial.println("Soil");
      delay(100);
    }
    //dataSent = false;
    digitalWrite(pwrPin, 0);
}
void setup() {
  Serial.begin(115200);
  pinMode(pluviometerPin, INPUT_PULLUP);
  pinMode(sendPin, INPUT_PULLUP);
  WiFi.mode(WIFI_STA);
  delay(100);
  attachInterrupt(digitalPinToInterrupt(sendPin), contadorPulsos, FALLING);
  attachInterrupt(digitalPinToInterrupt(pluviometerPin), enviar, FALLING);
  Serial.println("Attached");
  delay(500);
  esp_now_init();
  esp_now_register_send_cb(SendESPNOW);
  ScanForSlave();
  while(esp_now_add_peer(&slave) != ESP_OK){
    Serial.println("Peer");
    delay(100);
  }
  pinMode(pwrPin, OUTPUT);
  digitalWrite(pwrPin, 0);

  Serial.println("Initiated");
  delay(500);
}
void loop() {

}