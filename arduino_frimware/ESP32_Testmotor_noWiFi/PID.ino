void computePID1(double control_cmd,
                 long inTick)
{
  int dirs = 1;  //1 for forward direction
  //int dirm = 1;  //1 for forward direction

  curTick1 = inTick;
  curTime1 = millis();

  setRPM1 = control_cmd;
  dirs = control_cmd / abs(control_cmd);

  diffTick1 = curTick1 - prevTick1;
  dt1 =  (curTime1 - prevTime1);

  measuredRPM1 = ((diffTick1 / PPR) / (dt1 * 0.001)) * 60;

  err1 = abs(setRPM1) - abs(measuredRPM1);

  sumErr1 += err1 * dt1;

  dErr1 = (err1 - prev_err1) / dt1;

  control_out1 = kp * err1 + ki * sumErr1 + kd * dErr1;

  if (control_out1 < 0) {
    control_out1 = 0;
  }
  prev_err1 = err1;
  prevTick1 = curTick1;
  prevTime1 = curTime1;
  control_out1 = constrain(control_out1,0 , 100);
  if (dirs > 0.5) {
    digitalWrite(motor1Pin1, LOW);
    ledcWrite(1, control_out1);
  }
  else if (dirs < -0.5) {
    digitalWrite(motor1Pin1, HIGH);
    ledcWrite(1, control_out1);
  }
  else {
    digitalWrite(motor1Pin1, LOW);
    ledcWrite(1, 0);
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
  curTime2 = millis();

  setRPM2 = control_cmd;
  dirs = control_cmd / abs(control_cmd);

  diffTick2 = curTick2 - prevTick2;
  dt2 =  (curTime2 - prevTime2);

  measuredRPM2 = ((diffTick2 / PPR) / (dt2 * 0.001)) * 60;

  err2 = abs(setRPM2) - abs(measuredRPM2);

  sumErr2 += err2 * dt2;

  dErr2 = (err2 - prev_err2) / dt2;

  control_out2 = kp * err2 + ki * sumErr2 + kd * dErr2;

  if (control_out2 < 0) {
    control_out2 = 0;
  }
  prev_err2 = err2;
  prevTick2 = curTick2;
  prevTime2 = curTime2;
  control_out2 = constrain(control_out2,0 , 100);

  if (dirs > 0.5) {

    digitalWrite(motor2Pin1, LOW);
    ledcWrite(2, abs(control_out2));
  }
  else if (dirs < -0.5) {
    digitalWrite(motor2Pin1, HIGH);
    ledcWrite(2, abs(control_out2));
  }
  else {
    digitalWrite(motor2Pin1, LOW);
    ledcWrite(2, 0);
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
  curTime3 = millis();

  setRPM3 = control_cmd;
  dirs = control_cmd / abs(control_cmd);

  diffTick3 = curTick3 - prevTick3;
  dt3 =  (curTime3 - prevTime3);

  measuredRPM3 = ((diffTick3 / PPR) / (dt3 * 0.001)) * 60;

  err3 = abs(setRPM3) - abs(measuredRPM3);

  sumErr3 += err3 * dt3;

  dErr3 = (err3 - prev_err3) / dt3;

  control_out3 = kp * err3 + ki * sumErr3 + kd * dErr3;

  if (control_out3 < 0) {
    control_out3 = 0;
  }
  prev_err3 = err3;
  prevTick3 = curTick3;
  prevTime3 = curTime3;
  control_out3 = constrain(control_out3,0 , 100);

  if (dirs > 0.5) {

    digitalWrite(motor3Pin1, LOW);
    ledcWrite(3, abs(control_out3));
  }
  else if (dirs < -0.5) {
    digitalWrite(motor3Pin1, HIGH);
    ledcWrite(3, abs(control_out3));
  }
  else {
    digitalWrite(motor3Pin1, LOW);
    ledcWrite(3, 0);
  }
  //Serial.println(measuredRPM1);

  //Serial.println("Sum error  "+String(sumErr1));

  

}

void computePID4(double control_cmd,
                 long inTick)
{
  int dirs = 1;  //1 for forward direction
  //int dirm = 1;  //1 for forward direction

  curTick4 = inTick;
  curTime4 = millis();

  setRPM4 = control_cmd;
  dirs = control_cmd / abs(control_cmd);

  diffTick4 = curTick4 - prevTick4;
  dt4 =  (curTime4 - prevTime4);

  measuredRPM4 = ((diffTick4 / PPR) / (dt4 * 0.001)) * 60;

  err4 = abs(setRPM4) - abs(measuredRPM4);

  sumErr4 += err4 * dt4;

  dErr4 = (err4 - prev_err4) / dt4;

  control_out4 = kp * err4 + ki * sumErr4 + kd * dErr4;

  if (control_out4 < 0) {
    control_out4 = 0;
  }
  prev_err4 = err4;
  prevTick4 = curTick4;
  prevTime4 = curTime4;
  control_out4 = constrain(control_out4,0 , 100);

  if (dirs > 0.5) {

    digitalWrite(motor4Pin1, LOW);
    ledcWrite(4, abs(control_out4));
  }
  else if (dirs < -0.5) {
    digitalWrite(motor4Pin1, HIGH);
    ledcWrite(4, abs(control_out4));
  }
  else {
    digitalWrite(motor4Pin1, LOW);
    ledcWrite(4, 0);
  }
 // Serial.println(measuredRPM4);

  //Serial.println("Sum error  "+String(sumErr1));

  

}
