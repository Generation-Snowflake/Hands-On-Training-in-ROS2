/* Encoder Library - TwoKnobs Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */
#include <TimerOne.h>
#include <Encoder.h>
#define motor1Pin1 16
#define motor1Speed 13
#define motor2Pin1 17
#define motor2Speed 18
#define motor3Pin1 20   
#define motor3Speed 19
#define motor4Pin1 21    // Set pin for Motor a
#define motor4Speed 22

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability

// PID control parameters
float kp = 2.0;
float ki = 0.01;
float kd = 0.0;

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
Encoder encM1(3, 4); // Motor3
Encoder encM2(11, 12); // Motor3
Encoder encM3(5, 6); // Motor3
Encoder encM4(9, 10); //Motor 4
//   avoid using pins with LEDs attached
int positionLeft = -999;
int positionRight = -999;
float setpoint = 0.0;
void loopControl(void)
{
  computePID1(setpoint,counter1);
  computePID2(setpoint,counter2);
  computePID3(setpoint,counter3);
  computePID4(setpoint,counter4);
}
void encRead(){
  counter1 = encM1.read();
  counter2 = encM2.read();
  counter3 = encM3.read();
  counter4 = encM4.read();
  Serial.println(measuredRPM3);
}
void setup() {
  Serial.begin(115200);
  Serial.println("TwoKnobs Encoder Test:");
  Timer1.initialize(10000); //pin 7,8
  Timer1.attachInterrupt(loopControl); // blinkLED to run every 0.15 seconds
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Speed, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Speed, OUTPUT);
  pinMode(motor3Pin1, OUTPUT);
  pinMode(motor3Speed, OUTPUT);
  pinMode(motor4Pin1, OUTPUT);
  pinMode(motor4Speed, OUTPUT);
  encRead();
}

void loop() {
  encRead();
  setpoint = 50.0;
  long currentTime = millis();
  int ctrlLoop = 4800;
  while((millis() - currentTime <= ctrlLoop)){
    
      setpoint = 50.0;
      encRead();
  }
  while(1){
    encRead();
    setpoint = 0.0;
  }

  

//  

//  if (counter3 != positionLeft || counter4 != positionRight) {
//    Serial.print("Motor3 = ");
//    Serial.print(counter3);
//    Serial.print(", Motor4 = ");
//    Serial.print(counter4);
//    Serial.println();
//    positionLeft = counter3;
//    positionRight = counter4;
//  }
//  // if a character is sent from the serial monitor,
//  // reset both back to zero.
//  if (Serial.available()) {
//    Serial.read();
//    Serial.println("Reset both knobs to zero");
//    knobLeft.write(0);
//    knobRight.write(0);
//  }
//  computePID3(100.0,counter3);
//  computePID4(100.0,counter4);
///  //  digitalWrite(motor3Pin1, LOW);
/////  analogWrite(motor3Speed, 100);
/////  digitalWrite(motor4Pin1, LOW);
/////  analogWrite(motor4Speed, 100);
//  while((millis() - currentTime <= ctrlLoop));

}
