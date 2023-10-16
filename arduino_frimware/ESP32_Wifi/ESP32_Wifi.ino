#include <WiFi.h>
#include "ESPAsyncWebServer.h"
#define motor1Pin1 21    // Set pin for Motor a
//#define motor1Pin2 18p
#define motor2Pin1 22    // Set pin for Motor b
//#define motor2Pin2 19
#define motor3Pin1 25    // Set pin for Motor c
//#define motor3Pin2 33
#define motor4Pin1 27    // Set pin for Motor d
//#define motor4Pin2 26
#define motor1Speed 5
#define motor2Speed 16
#define motor3Speed 4
#define motor4Speed 2

const char* ssid = "TP-Link_A550";
const char* password = "23172845";

AsyncWebServer server(80);

IPAddress local_IP(192, 168, 0, 62);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8); //optional
IPAddress secondaryDNS(8, 8, 4, 4); //optional

const uint8_t EN1A = 34;
const uint8_t EN1B = 35;
volatile int counter;
int M1_speed;

void IRAM_ATTR isr(){
  if(digitalRead(EN1A) == digitalRead(EN1B)) {
    //Clockwise
    counter--;
  } else {
    //Counter Clockwise
    counter++;
  }
}

uint32_t num;
uint8_t power = 150;
int16_t speed_a, speed_b, speed_c, speed_d;
//bool forward = false;
String getSens() {
  return String(num++);
}
void handleFunc();
hw_timer_t *My_timer = NULL;
void IRAM_ATTR onTimer(){
M1_speed = (1*counter)/(6*1);
counter = 0;
}

void control(String cmd) {
  if (cmd == "1") { //forward
    digitalWrite(motor1Pin1, LOW);
    //digitalWrite(motor1Pin2, HIGH);
    ledcWrite(1, power);
    digitalWrite(motor2Pin1, LOW);
    //digitalWrite(motor2Pin2, HIGH);
    ledcWrite(2, power);
    digitalWrite(motor3Pin1, LOW);
    //digitalWrite(motor3Pin2, HIGH);
    ledcWrite(3, power);
    digitalWrite(motor4Pin1, LOW);
    //digitalWrite(motor4Pin2, HIGH);
    ledcWrite(4, power);
  }
  else if (cmd == "2") {  //left
    digitalWrite(motor1Pin1, LOW);
    //digitalWrite(motor1Pin2, HIGH);
    ledcWrite(1, power);
    digitalWrite(motor2Pin1, HIGH);
    //digitalWrite(motor2Pin2, LOW);
    ledcWrite(2, power);
    digitalWrite(motor3Pin1, HIGH);
    //digitalWrite(motor3Pin2, LOW);
    ledcWrite(3, power);
    digitalWrite(motor4Pin1, LOW);
    //digitalWrite(motor4Pin2, HIGH);
    ledcWrite(4, power);
  }
  else if (cmd == "3") {  //right
    digitalWrite(motor1Pin1, HIGH);
    //digitalWrite(motor1Pin2, LOW);
    ledcWrite(1, power);
    digitalWrite(motor2Pin1, LOW);
    //digitalWrite(motor2Pin2, HIGH);
    ledcWrite(2, power);
    digitalWrite(motor3Pin1, LOW);
    //digitalWrite(motor3Pin2, HIGH);
    ledcWrite(3, power);
    digitalWrite(motor4Pin1, HIGH);
    //digitalWrite(motor4Pin2, LOW);
    ledcWrite(4, power);
  }
  else if (cmd == "4") {  //backward
    digitalWrite(motor1Pin1, HIGH);
    //digitalWrite(motor1Pin2, LOW);
    ledcWrite(1, power);
    digitalWrite(motor2Pin1, HIGH);
    //digitalWrite(motor2Pin2, LOW);
    ledcWrite(2, power);
    digitalWrite(motor3Pin1, HIGH);
    //digitalWrite(motor3Pin2, LOW);
    ledcWrite(3, power);
    digitalWrite(motor4Pin1, HIGH);
    //digitalWrite(motor4Pin2, LOW);
    ledcWrite(4, power);
  }
  else {
    digitalWrite(motor1Pin1, LOW);
    //digitalWrite(motor1Pin2, LOW);
    ledcWrite(1, 0);
    digitalWrite(motor2Pin1, LOW);
    //digitalWrite(motor2Pin2, LOW);
    ledcWrite(2, 0);
    digitalWrite(motor3Pin1, LOW);
    //digitalWrite(motor3Pin2, LOW);
    ledcWrite(3, 0);
    digitalWrite(motor4Pin1, LOW);
    //digitalWrite(motor4Pin2, LOW);
    ledcWrite(4, 0);

  }
}

void setup() {
  // Serial port for debugging purposes
  Serial.begin(115200);
  Serial.println();
  attachInterrupt(EN1A, isr, RISING);

  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, 1000000, true);
  timerAlarmEnable(My_timer);

  pinMode(motor1Pin1, OUTPUT);
  //pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  //pinMode(motor2Pin2, OUTPUT);
  pinMode(motor3Pin1, OUTPUT);
  //pinMode(motor3Pin2, OUTPUT);
  pinMode(motor4Pin1, OUTPUT);
  //pinMode(motor4Pin2, OUTPUT);
  pinMode(EN1A, INPUT);
  pinMode(EN1B, INPUT);

  ledcSetup(1, 5000, 8);  ledcAttachPin(motor1Speed, 1);
  ledcSetup(2, 5000, 8);  ledcAttachPin(motor2Speed, 2);
  ledcSetup(3, 5000, 8);  ledcAttachPin(motor3Speed, 3);
  ledcSetup(4, 5000, 8);  ledcAttachPin(motor4Speed, 4);

//    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
//    Serial.println("STA Failed to configure");
//  }
//
//  WiFi.begin(ssid, password);
//  while (WiFi.status() != WL_CONNECTED) {
//    delay(500);
//    Serial.print(".");
//  }
//  Serial.println("");
//  Serial.println("WiFi connected");
//
//  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
//
//    int paramsNr = request->params();
//    //Serial.println(paramsNr);
//
//    for (int i = 0; i < paramsNr; i++) {
//
//      AsyncWebParameter* p = request->getParam(i);
//
//   if (p->name() == "A") {
//        speed_a = (p->value()).toInt();
//        //control("1");
//        if (speed_a > 0) {
//
//          digitalWrite(motor1Pin1, LOW);
//          ledcWrite(1, abs(speed_a));
//        }
//        else if (speed_a < 0) {
//          digitalWrite(motor1Pin1, HIGH);
//          ledcWrite(1, abs(speed_a));
//        }
//        else {
//          digitalWrite(motor1Pin1, LOW);
//          ledcWrite(1, 0);
//        }
//    
//      }
//      ////////////////////////////////////////////////////
//      if (p->name() == "B") {
//        speed_b = (p->value()).toInt();
//        //control("2");
//        if (speed_b > 0) {
//
//          digitalWrite(motor2Pin1, LOW);
//          ledcWrite(2, abs(speed_b));
//        }
//        else if (speed_b < 0) {
//          digitalWrite(motor2Pin1, HIGH);
//          ledcWrite(2, abs(speed_b));
//        }
//        else {
//          digitalWrite(motor2Pin1, LOW);
//          ledcWrite(2, 0);
//        }
//      
//      }
//      ////////////////////////////////////////////////////
//      if (p->name() == "C") {
//        speed_c = (p->value()).toInt();
//        //control("3");
//        if (speed_c > 0) {
//
//          digitalWrite(motor3Pin1, LOW);
//          ledcWrite(3, abs(speed_c));
//        }
//        else if (speed_c < 0) {
//          digitalWrite(motor3Pin1, HIGH);
//          ledcWrite(3, abs(speed_c));
//        }
//        else {
//          digitalWrite(motor3Pin1, LOW);
//          ledcWrite(3, 0);
//        }
//        
//      }
//      ////////////////////////////////////////////////////
//      if (p->name() == "D") {
//        speed_d = (p->value()).toInt();
//        //control("4");
//        if (speed_d > 0) {
//
//          digitalWrite(motor4Pin1, LOW);
//          ledcWrite(4, abs(speed_d));
//        }
//        else if (speed_d < 0) {
//          digitalWrite(motor4Pin1, HIGH);
//          ledcWrite(4, abs(speed_d));
//        }
//        else {
//          digitalWrite(motor4Pin1, LOW);
//          ledcWrite(4, 0);
//        }
//      }
//
//      
//    }
//
//    request->send(200, "text/plain", "message received");
//  });
//
//  server.begin();
}
void loop() {
  control("1");
Serial.println(M1_speed);
}
