#include <WiFi.h>
#include "ESPAsyncWebServer.h"

const char* ssid = "GSF-WIFI1200";
const char* password = "gsfrobotics";

AsyncWebServer server(80);

IPAddress local_IP(192, 168, 8, 150);
IPAddress gateway(192, 168, 8, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8); //optional
IPAddress secondaryDNS(8, 8, 4, 4); //optional


uint32_t num;
uint8_t power = 150;
String getSens() {
  return String(num++);
}


void setup() {
  // Serial port for debugging purposes
  Serial.begin(115200);
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
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  //server.begin();
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {`

    int paramsNr = request->params();
    //Serial.println(paramsNr);

    for (int i = 0; i < paramsNr; i++) {

      AsyncWebParameter* p = request->getParam(i);

   if (p->name() == "A") {
        speed_a = (p->value()).toInt();
        //control("1");
        if (speed_a > 0) {

          digitalWrite(motor1Pin1, LOW);
          ledcWrite(1, abs(speed_a));
        }
        else if (speed_a < 0) {
          digitalWrite(motor1Pin1, HIGH);
          ledcWrite(1, abs(speed_a));
        }
        else {
          digitalWrite(motor1Pin1, LOW);
          ledcWrite(1, 0);
        }
    
      }
      ////////////////////////////////////////////////////
      if (p->name() == "B") {
        speed_b = (p->value()).toInt();
        //control("2");
        if (speed_b > 0) {

          digitalWrite(motor2Pin1, LOW);
          ledcWrite(2, abs(speed_b));
        }
        else if (speed_b < 0) {
          digitalWrite(motor2Pin1, HIGH);
          ledcWrite(2, abs(speed_b));
        }
        else {
          digitalWrite(motor2Pin1, LOW);
          ledcWrite(2, 0);
        }
      
      }
      ////////////////////////////////////////////////////
      if (p->name() == "C") {
        speed_c = (p->value()).toInt();
        //control("3");
        if (speed_c > 0) {

          digitalWrite(motor3Pin1, LOW);
          ledcWrite(3, abs(speed_c));
        }
        else if (speed_c < 0) {
          digitalWrite(motor3Pin1, HIGH);
          ledcWrite(3, abs(speed_c));
        }
        else {
          digitalWrite(motor3Pin1, LOW);
          ledcWrite(3, 0);
        }
        
      }
      ////////////////////////////////////////////////////
      if (p->name() == "D") {
        speed_d = (p->value()).toInt();
        //control("4");
        if (speed_d > 0) {

          digitalWrite(motor4Pin1, LOW);
          ledcWrite(4, abs(speed_d));
        }
        else if (speed_d < 0) {
          digitalWrite(motor4Pin1, HIGH);
          ledcWrite(4, abs(speed_d));
        }
        else {
          digitalWrite(motor4Pin1, LOW);
          ledcWrite(4, 0);
        }
      }

      
    }

    request->send(200, "text/plain", "message received");
  });

  server.begin();
}
void loop() {

}
