

#include <HardwareSerial.h>
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
float PPR = 360;
AsyncWebServer server(80);

IPAddress local_IP(192, 168, 0, 62);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8); //optional
IPAddress secondaryDNS(8, 8, 4, 4); //optional



// PID control parameters
float kp = 0.8;
float ki = 2.0;
float kd = 0.0;


//1 Wheel control parameter
unsigned long curTime1, prevTime1, dt1;
long curTick1, prevTick1, diffTick1;
double err1, prev_err1, sumErr1, dErr1, setRPM1;
double control_out1;
double measuredRPM1;

//2 Wheel control parameter
unsigned long curTime2, prevTime2, dt2;
long curTick2, prevTick2, diffTick2;
double err2, prev_err2, sumErr2, dErr2, setRPM2;
double control_out2;
double measuredRPM2;

//3 Wheel control parameter
unsigned long curTime3, prevTime3, dt3;
long curTick3, prevTick3, diffTick3;
double err3, prev_err3, sumErr3, dErr3, setRPM3;
double control_out3;
double measuredRPM3;

//4 Wheel control parameter
unsigned long curTime4, prevTime4, dt4;
long curTick4, prevTick4, diffTick4;
double err4, prev_err4, sumErr4, dErr4, setRPM4;
double control_out4;
double measuredRPM4;


double desiredRPM2, desiredRPM1,desiredRPM4,desiredRPM3;


const uint8_t EN1A = 34;
const uint8_t EN1B = 35;
int M1_speed;
volatile long counter1;


const uint8_t EN2A = 32;
const uint8_t EN2B = 26;
int M2_speed;
volatile long counter2;

const uint8_t EN3A = 12;
const uint8_t EN3B = 13;
volatile long counter3;
int M3_speed;

const uint8_t EN4A = 19;
const uint8_t EN4B = 18;
volatile long counter4;
int M4_speed;

void IRAM_ATTR isr1(){
  // if(digitalRead(EN1A) == digitalRead(EN1B)) {
  //   //Clockwise
  //   counter1--;
  // } else {
  //   //Counter Clockwise
  //   counter1++;
  // }
  counter1++;
}

void IRAM_ATTR isr2(){
  // if(digitalRead(EN2A) == digitalRead(EN2B)) {
  //   //Clockwise
  //   counter2++;
  // } else {
  //   //Counter Clockwise
  //   counter2--;
  // }

   counter2++;
}

void IRAM_ATTR isr3(){
  // if(digitalRead(EN3A) == digitalRead(EN3B)) {
  //   //Clockwise
  //   counter3++;
  // } else {
  //   //Counter Clockwise
  //   counter3--;
  // }

  counter3++;
}

void IRAM_ATTR isr4(){
  // if(digitalRead(EN4A) == digitalRead(EN4B)) {
  //   //Clockwise
  //   counter4++;
  // } else {
  //   //Counter Clockwise
  //   counter4--;
  // }

  counter4++;
}

uint32_t num;
uint8_t power = 250;
int16_t speed_a, speed_b, speed_c, speed_d;
//bool forward = false;
String getSens() {
  return String(num++);
}
void handleFunc();
hw_timer_t *My_timer = NULL;
  
void IRAM_ATTR onTimer(){
  M1_speed = (1*counter1)/(6*1);
  // counter1 = 0;
}

void control(String cmd) {
  if (cmd == "1") { //forward
    //digitalWrite(motor1Pin1, LOW);
    //ledcWrite(1, power);
    digitalWrite(motor2Pin1, LOW);
    ledcWrite(2, power);
    digitalWrite(motor3Pin1, LOW);
    ledcWrite(3, power);
    digitalWrite(motor4Pin1, LOW);
    ledcWrite(4, power);
  }
  else if (cmd == "2") {  //left
    //digitalWrite(motor1Pin1, LOW);
    //ledcWrite(1, power);
    digitalWrite(motor2Pin1, HIGH);
    ledcWrite(2, power);
    digitalWrite(motor3Pin1, HIGH);
    ledcWrite(3, power);
    digitalWrite(motor4Pin1, LOW);
    ledcWrite(4, power);
  }
  else if (cmd == "3") {  //right
    //digitalWrite(motor1Pin1, HIGH);
    //ledcWrite(1, power);
    digitalWrite(motor2Pin1, LOW);
    ledcWrite(2, power);
    digitalWrite(motor3Pin1, LOW);
    ledcWrite(3, power);
    digitalWrite(motor4Pin1, HIGH);
    ledcWrite(4, power);
  }
  else if (cmd == "4") {  //backward
    //digitalWrite(motor1Pin1, HIGH);
    //ledcWrite(1, power);
    digitalWrite(motor2Pin1, HIGH);
    ledcWrite(2, power);
    digitalWrite(motor3Pin1, HIGH);
    ledcWrite(3, power);
    digitalWrite(motor4Pin1, HIGH);
    ledcWrite(4, power);
  }
  else {
    digitalWrite(motor1Pin1, LOW);
    ledcWrite(1, 0);
    digitalWrite(motor2Pin1, LOW);
    ledcWrite(2, 0);
    digitalWrite(motor3Pin1, LOW);
    ledcWrite(3, 0);
    digitalWrite(motor4Pin1, LOW);
    ledcWrite(4, 0);

  }
}

void setup() {
  // Serial port for debugging purposes
  Serial.begin(115200);
  Serial.println();
  attachInterrupt(EN1A, isr1, RISING);
  attachInterrupt(EN2B, isr2, RISING);
  attachInterrupt(EN3A, isr3, RISING);
  attachInterrupt(EN4A, isr4, RISING);


  // My_timer = timerBegin(0, 80, true);
  // timerAttachInterrupt(My_timer, &onTimer, true);
  // timerAlarmWrite(My_timer, 1000000, true);
  // timerAlarmEnable(My_timer);

  pinMode(motor1Pin1, OUTPUT);
  //pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  //pinMode(motor2Pin2, OUTPUT);
  pinMode(motor3Pin1, OUTPUT);
  //pinMode(motor3Pin2, OUTPUT);
  pinMode(motor4Pin1, OUTPUT);
  //pinMode(motor4Pin2, OUTPUT);
  pinMode(EN1A, INPUT);
  //pinMode(EN1B, INPUT);
  pinMode(EN2A, INPUT);
  //pinMode(EN2B, INPUT);
  pinMode(EN3A, INPUT);
  //pinMode(EN3B, INPUT);
  pinMode(EN4A, INPUT);
  //pinMode(EN4B, INPUT);

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

int ctrlLoop = 10;
double currentTime = 0.0;
void loop() {
  currentTime = millis();
//  digitalWrite(motor1Pin1, LOW);
//  ledcWrite(1, 100);
//   digitalWrite(motor2Pin1, LOW);
//  ledcWrite(2, 100);
//   digitalWrite(motor3Pin1, LOW);
//  ledcWrite(3, 100);
//   digitalWrite(motor4Pin1, LOW);
//  ledcWrite(4, 100);
  //control("1");
  computePID1(100.0,counter1);
  computePID2(100.0,counter2);
  //computePID3(100.0,counter3);
  //computePID4(100.0,counter4);
  // Serial.print("Loop");
  Serial.println(String(counter1) + "," +  String(counter2) + "," + String(counter3) + "," + String(counter4));
  while((millis() - currentTime <= ctrlLoop));
  
}
