/* Encoder Library - TwoKnobs Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */
#include <TimerOne.h>
#include <Encoder.h>
#define LEDPIN 13
#define motor1Pin1 16
#define motor1Speed 1
#define motor2Pin1 17
#define motor2Speed 18
#define motor3Pin1 20   
#define motor3Speed 19
#define motor4Pin1 21    // Set pin for Motor a
#define motor4Speed 22

Encoder encM1(3, 4); // Motor3
Encoder encM2(11, 12); // Motor3
Encoder encM3(6, 5); // Motor3
Encoder encM4(9, 10); //Motor 4

// PID control parameters
float kp[4] = {2.0, 2.5, 2.0, 2.0};
float ki[4] = {0.1, 0.1, 0.1, 0.1};
float kd[4] = {0.0, 0.0, 0.0, 0.0};

volatile long counter1;
int M1_speed;

volatile long counter2;
int M2_speed;

volatile long counter3;
int M3_speed;

volatile long counter4;
int M4_speed;


//1 Wheel control parameter
unsigned long curTime1, prevTime1, dt1;
long curTick1, prevTick1;
double diffTick1;
double err1, prev_err1, sumErr1, dErr1, setRPM1;
double control_out1;
double measuredRPM1;

//2 Wheel control parameter
unsigned long curTime2, prevTime2, dt2;
long curTick2, prevTick2;
double diffTick2;
double err2, prev_err2, sumErr2, dErr2, setRPM2;
double control_out2;
double measuredRPM2;

//3 Wheel control parameter
unsigned long curTime3, prevTime3, dt3;
long curTick3, prevTick3;
double diffTick3;
double err3, prev_err3, sumErr3, dErr3, setRPM3;
double control_out3;
double measuredRPM3;

//4 Wheel control parameter
unsigned long curTime4, prevTime4, dt4;
long curTick4, prevTick4;
double diffTick4;
double err4, prev_err4, sumErr4, dErr4, setRPM4;
double control_out4;
double measuredRPM4;

float PPR = 360.0;
double desiredRPM2, desiredRPM1,desiredRPM4,desiredRPM3;

int positionLeft = -999;
int positionRight = -999;
float setpointM1 = 0.0, setpointM2 = 0.0, setpointM3=0.0, setpointM4 = 0.0;
uint16_t travel_time=0;
uint8_t idx = 0;
#define idx_length 10
volatile char _buffer[idx_length * 2] = {0};

unsigned long recvtime = 0;

union packed_int {
  int num;
  byte b[2];
} num_rec;

void setALLsetpoint(float setpoint){
  
  setpointM1 = setpoint;
  setpointM2 = setpoint;
  setpointM3 = setpoint;
  setpointM4 = setpoint;
  delay(0.1);
}
void loopControl(void)
{
  encRead();
  computePID1(setpointM1,counter1);
  computePID2(setpointM2,counter2);
  computePID3(setpointM3,counter3);
  computePID4(setpointM4,counter4);
}

void encRead(){
  counter1 = encM1.read();
  counter2 = encM2.read();
  counter3 = encM3.read();
  counter4 = encM4.read();
  //Serial.println(measuredRPM3);
}
int16_t getIntFromSerial(volatile unsigned char packet[], int i1, int i2) {
  num_rec.b[0] = packet[i1];
  num_rec.b[1] = packet[i2];
  int16_t out = num_rec.num;
  return out;
}
void timeControl(){
  //long currentTime = millis();
  if((millis() - recvtime) <= travel_time){
    //Serial.println(millis() - currentTime);
      digitalWrite(LEDPIN,LOW);
  }
  else{
   setALLsetpoint(0.0);
   digitalWrite(LEDPIN,HIGH);
  }
}

void SerialEvent3(){
  while(Serial3.available()){
    Serial.print("Active...");
    byte cmd = (byte) Serial3.read();
    Serial.println(byte(cmd));
    _buffer[idx] = cmd;
    if((idx == 0) && _buffer[0] != '#'){
      setALLsetpoint(0.0);
    }
    if((_buffer[idx - 9] == '#' && (_buffer[idx - 8] == 'b')
    && (_buffer[idx - 1] == '\r') && _buffer[idx] == '\n')){
      //Serial.print("speed2" + String(int8_t(_buffer[3])));
      setpointM1 = int8_t(_buffer[2]);
      setpointM2 = int8_t(_buffer[3]);
      setpointM3 = int8_t(_buffer[4]);
      setpointM4 = int8_t(_buffer[5]);
      travel_time = getIntFromSerial(_buffer, 7 , 6);
      recvtime = millis();
      //timeControl(int(_buffer[2]),int(_buffer[3]),int(_buffer[4]),int(_buffer[5]),getIntFromSerial(_buffer, 7 , 6));
    idx = 0;  
    }
    else{
      idx < idx_length*2 ? ++idx : idx = 0;
    }
    
    
  }
}
void setup() {
  pinMode(LEDPIN, OUTPUT);
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Speed, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Speed, OUTPUT);
  pinMode(motor3Pin1, OUTPUT);
  pinMode(motor3Speed, OUTPUT);
  pinMode(motor4Pin1, OUTPUT);
  pinMode(motor4Speed, OUTPUT);
  //setALLsetpoint(0.0);
  Timer1.initialize(10000); //pin 7,8
  Timer1.attachInterrupt(loopControl); //run every 0.15s
  Serial.begin(115200);
  Serial3.begin(57600);
  digitalWrite(LEDPIN, HIGH);
}

void loop() {
  SerialEvent3();
  timeControl();
  //setpointM1 = 50.0;
  //setpointM2 = 50.0;
  //setpointM3 = 50.0;
  //setpointM4 = 50.0;
  //digitalWrite(motor1Pin1, LOW);
  //analogWrite(motor1Speed, 50);
  //digitalWrite(motor3Pin1, LOW);
  //analogWrite(motor3Speed, 50);
  //Serial.println(counter1);
  //testDistance();

}
