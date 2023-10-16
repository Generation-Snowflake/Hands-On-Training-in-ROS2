void computePID1(double control_cmd,
                 long inTick)
{
  int dirs = 1;  //1 for forward direction
  //int dirm = 1;  //1 for forward direction

  curTick1 = inTick;

  setRPM1 = control_cmd;
  dirs = control_cmd / abs(control_cmd);

  diffTick1 = curTick1 - prevTick1;

  measuredRPM1 = (((diffTick1/4.0f)  * (PPR/360.0f)) / (0.01f * 6.0f)) ;

  err1 = abs(setRPM1) - abs(measuredRPM1);

  sumErr1 += err1;

  dErr1 = (err1 - prev_err1);

  control_out1 = kp * err1 + ki * sumErr1 + kd * dErr1;

  if (control_out1 < 0) {
    control_out1 = 0;
  }
  prev_err1 = err1;
  prevTick1 = curTick1;
  control_out1 = constrain(control_out1,0 , 255);
  if (dirs > 0.5) {
    digitalWrite(motor1Pin1, LOW);
    analogWrite(motor1Speed, control_out1);
  }
  else if (dirs < -0.5) {
    digitalWrite(motor1Pin1, HIGH);
    analogWrite(motor1Speed, control_out1);
  }
  else {
    digitalWrite(motor1Pin1, LOW);
    analogWrite(motor1Speed, 0);
  }
  //Serial.println(measuredRPM1);

  //Serial.println("Sum error  "+String(sumErr1));

  

}

void computePID2(double control_cmd,
                 long inTick)
{
  int dirs = 1;  //1 for forward direction
  //int dirm = 1;  //1 for forward direction

  curTick2 = inTick;

  setRPM2 = control_cmd;
  dirs = control_cmd / abs(control_cmd);

  diffTick2 = curTick2 - prevTick2;

  measuredRPM2 = (((diffTick2/4.0f) * (PPR/360.0f)) / (0.01f * 6.0f));

  err2 = abs(setRPM2) - abs(measuredRPM2);

  sumErr2 += err2;

  dErr2 = (err2 - prev_err2);

  control_out2 = kp * err2 + ki * sumErr2 + kd * dErr2;

  if (control_out2 < 0) {
    control_out2 = 0;
  }
  prev_err2 = err2;
  prevTick2 = curTick2;
  control_out2 = constrain(control_out2,0 , 255);

  if (dirs > 0.5) {

    digitalWrite(motor2Pin1, LOW);
    analogWrite(motor2Speed, control_out2);
  }
  else if (dirs < -0.5) {
    digitalWrite(motor2Pin1, HIGH);
    analogWrite(motor2Speed, control_out2);
  }
  else {
    digitalWrite(motor2Pin1, LOW);
    analogWrite(motor2Speed, control_out2);
  }
  //Serial.println(measuredRPM2);

  //Serial.println("Sum error  "+String(sumErr1));

  

}
void computePID3(double control_cmd,
                 long inTick)
{
  int dirs = 1;  //1 for forward direction
  //int dirm = 1;  //1 for forward direction

  curTick3 = inTick;

  setRPM3 = control_cmd;
  dirs = control_cmd / abs(control_cmd);

  diffTick3 = curTick3 - prevTick3;

  measuredRPM3 = (((diffTick3/4.0f) * (PPR/360.0f)) / (0.01f * 6.0f));

  err3 = abs(setRPM3) - abs(measuredRPM3);

  sumErr3 += err3;

  dErr3 = err3 - prev_err3;

  control_out3 = kp * err3 + ki * sumErr3 + kd * dErr3;

  if (control_out3 < 0) {
    control_out3 = 0;
  }
  prev_err3 = err3;
  prevTick3 = curTick3;
  control_out3 = constrain(control_out3,0 , 255);

  if (dirs > 0.5) {

    digitalWrite(motor3Pin1, LOW);
    analogWrite(motor3Speed, control_out3);
  }
  else if (dirs < -0.5) {
    digitalWrite(motor3Pin1, HIGH);
    analogWrite(motor3Speed, control_out3);
  }
  else {
    digitalWrite(motor3Pin1, LOW);
    analogWrite(motor3Speed, 0);
  }
  
  //Serial.println(measuredRPM3);
  //Serial.println("Sum error  "+String(sumErr1));

  

}

void computePID4(double control_cmd,
                 long inTick)
{
  int dirs = 1;  //1 for forward direction
  //int dirm = 1;  //1 for forward direction

  curTick4 = inTick;

  setRPM4 = control_cmd;
  dirs = control_cmd / abs(control_cmd);

  diffTick4 = curTick4 - prevTick4;

  measuredRPM4 = (((diffTick4/4.0f) * (PPR/360.0f)) / (0.01f * 6.0f));

  err4 = abs(setRPM4) - abs(measuredRPM4);

  sumErr4 += err4;

  dErr4 = err4 - prev_err4;

  control_out4 = kp * err4 + ki * sumErr4 + kd * dErr4;

  if (control_out4 < 0) {
    control_out4 = 0;
  }
  prev_err4 = err4;
  prevTick4 = curTick4;
  control_out4 = constrain(control_out4,0 , 255);

  if (dirs > 0.5) {

    digitalWrite(motor4Pin1, LOW);
    analogWrite(motor4Speed, control_out4);
  }
  else if (dirs < -0.5) {
    digitalWrite(motor4Pin1, HIGH);
    analogWrite(motor4Speed, control_out4);
  }
  else {
    digitalWrite(motor4Pin1, LOW);
    analogWrite(motor4Speed, 0);
  }
 //Serial.println(measuredRPM4);

  //Serial.println("Sum error  "+String(sumErr1));

  

}
