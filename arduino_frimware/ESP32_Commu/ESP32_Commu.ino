#include <WiFi.h>
#include "ESPAsyncWebServer.h"
#define LEDPIN 5
const char* ssid = "TP-Link_A550";
const char* password = "23172845";

AsyncWebServer server(80);

IPAddress local_IP(192, 168, 0, 62);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8); //optional
IPAddress secondaryDNS(8, 8, 4, 4); //optional

int speed_a, speed_b, speed_c, speed_d;
//int travel_time;
uint32_t num;
uint8_t power = 150;
union packed_int {
  int16_t i;
  byte b[2];
} travel_time;
String getSens() {    
  return String(num++);
}


void setup() {
  // Serial port for debugging purposes
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);
  Serial.begin(115200);
  Serial2.begin(57600);
  Serial.println();

    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  digitalWrite(LEDPIN, LOW);
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  //server.begin();
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {

    int paramsNr = request->params();
    //Serial.println(paramsNr);

    for (int i = 0; i < paramsNr; i++) {

      AsyncWebParameter* p = request->getParam(i);

   if (p->name() == "A") {
        speed_a = (p->value()).toInt();
        //Serial.println(speed_a);
    
      }
      ////////////////////////////////////////////////////
      if (p->name() == "B") {
        speed_b = (p->value()).toInt();
      }
      ////////////////////////////////////////////////////
      if (p->name() == "C") {
        speed_c = (p->value()).toInt();
        
      }
      ////////////////////////////////////////////////////
      if (p->name() == "D") {
        speed_d = (p->value()).toInt();
      }
      ////////////////////////////////////////////////////
      if (p->name() == "T") {
        travel_time.i = (p->value()).toInt();
      }


    }
    const char cmd[10] = {'#','b',byte(speed_a),byte(speed_b),byte(speed_c),byte(speed_d),byte(travel_time.b[1]),byte(travel_time.b[0]),'\r','\n'};
    for(uint8_t i=0; i<10; i++){
      Serial.write(cmd[i]);
      Serial2.write(cmd[i]);
    }   
    request->send(200, "text/plain", "message received");
  });

  server.begin();
}
void loop() {
  delay(1000);
}
