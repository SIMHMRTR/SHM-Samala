#include <WiFi.h>     //Librerias
#include <esp_now.h>
#include <HardwareSerial.h>
#include <Statistic.h>


#define CHANNEL 1 //Comunicacion y control
//uint8_t broadcastMacAdd[] = {0x7C,0x9E,0xBD,0x62,0x03,0xD4}; 
esp_now_peer_info_t slave;
bool dataSent = false;

#define uS_TO_S_FACTOR 1000000  //Manejo de hibernación
uint16_t TIME_TO_SLEEP = 300;
uint64_t real_sleep = TIME_TO_SLEEP * uS_TO_S_FACTOR;
uint16_t maxSmpl = 200;

#define pinTX 6  //Hardware RIO
#define pinRX 7
#define TRIGGER_LOW 0
#define TRIGGER_HIGH 1
#define TRIGGER_MODE TRIGGER_LOW  //Hardware RIO
// Integer to store distance
int distance = 0;

// Variable to hold checksum
unsigned char CS;

// Object to represent software serial port
//HardwareSerial Serial1(1);
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


void pwm_trigger(){//E. HIDROMETRICA    //Lecturas e interpretación
  #if TRIGGER_MODE == TRIGGER_LOW
        digitalWrite( pinTX, LOW ); // trigger disparo de salida
  #else
        digitalWrite( pinTX, HIGH ); // Trigger diparador no emite
  #endif
}
void pwm_stop_trigger(){//E. HIDROMETRICA
  #if TRIGGER_MODE == TRIGGER_LOW
        digitalWrite( pinTX, HIGH ); // disparar
  #else
        digitalWrite( pinTX, LOW ); // disparar
  #endif
}

void hidrometric(){
  static uint32_t trigger_cnt = millis();
    if ( millis() - trigger_cnt > 500 ) {    // disparo cada  min. SENSOR ULTRASONICO
      pwm_trigger(); // Trigger salida
      uint32_t trigger_stop_cnt = micros();
      while ( digitalRead( pinRX ) == LOW ) {//{} espera a q termine el nivel bajo
          if ( micros() - trigger_stop_cnt > 100 ){
              pwm_stop_trigger();
              //Serial.println("100 micros");
              break;
          }
          //Serial.println("WaitLow");
      }
      while ( digitalRead( pinRX ) == LOW ) {} // esperar a que empiece el disparo del pulso
      uint32_t pluse_time = micros();
      while ( digitalRead( pinRX ) == HIGH ) {} // esperar a que termine el pulso
      pluse_time = micros() - pluse_time; // calcular el tiempo
      float d = ((1.0003*(pluse_time / 57.5))-0.1021);
      Serial.println(d);
      WLStats.add(d);
      trigger_cnt = millis();
      if(WLStats.count() >= 60){
        WLValue.station = 1;
        WLValue.variable = 1;
        if(WLStats.average() < 100.00) WLValue.priority = 3;
        if(WLStats.average() > 100.00 && WLStats.average() < 300.00) WLValue.priority = 2;
        if(WLStats.average() > 300.00 && WLStats.average() < 600.00) WLValue.priority = 1;
        if(WLStats.average() > 600.00) WLValue.priority = 0;
        WLValue.recent = d;
        WLValue.average = WLStats.average();
        WLValue.minimum = WLStats.minimum();
        WLValue.maximum = WLStats.maximum();
        WLValue.standard_dev = WLStats.pop_stdev();    
        esp_now_send(slave.peer_addr, (uint8_t *) &WLValue, sizeof(WLValue));
        delay(500);
        while(!dataSent){
          ScanForSlave();
          esp_now_add_peer(&slave);
          delay(100);
          esp_now_send(slave.peer_addr, (uint8_t *) &WLValue, sizeof(WLValue));
          Serial.println("Sending water level");
          delay(500);
        }
        WLStats.clear(); 
        dataSent = false;
        esp_sleep_enable_timer_wakeup(real_sleep);
        Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
        Serial.println("Going to sleep now");
        Serial.flush();
        esp_deep_sleep_start();
      }
    }
}
/*
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
        float d = float(distance) / 100.0;
        
        Serial.print("Distancia: ");
        Serial.print(d);
        WLStats.add(d);
        Serial.println("m");

        //float n = 750.0 - d //Nivel del rio, ubicando el sensor a 7.5m del fondo del cauce.
      }

    }
  }      
  if(WLStats.count() >= 60){
        WLValue.station = 1;
        WLValue.variable = 1;
        WLValue.priority = 6;
        WLValue.recent = d;
        WLValue.average = WLStats.average();
        WLValue.minimum = WLStats.minimum();
        WLValue.maximum = WLStats.maximum();
        WLValue.standard_dev = WLStats.pop_stdev();    
        esp_now_send(slave.peer_addr, (uint8_t *) &WLValue, sizeof(WLValue));
        delay(500);
        while(!dataSent){
          ScanForSlave();
          esp_now_add_peer(&slave);
          delay(100);
          esp_now_send(slave.peer_addr, (uint8_t *) &WLValue, sizeof(WLValue));
          Serial.println("Sending water level");
          delay(500);
        }
        WLStats.clear(); 
        dataSent = false;
        esp_sleep_enable_timer_wakeup(real_sleep);
        Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
        Serial.println("Going to sleep now");
        Serial.flush();
        esp_deep_sleep_start();
  }
}*/
//
void setup() {
  Serial.begin(115200);
  //Serial1.begin(9600,SERIAL_8N1, pinRX, pinTX); 
  
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
  pinMode(pinTX, OUTPUT );
  pwm_stop_trigger(); 
  pinMode(pinRX, INPUT);

  Serial.println("Setup complete.");
  delay(100);
}
void loop() {
  hidrometric();
}