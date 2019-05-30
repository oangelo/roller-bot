#define PIN_RIGHTMOTOR1 6 //RED PIN
#define PIN_RIGHTMOTOR2 7 //BROWN PIN
#define PIN_LEFTMOTOR1 8 //YELLOW PIN
#define PIN_LEFTMOTOR2 9 //ORANGE PIN
#define PIN_FRONTSENSOR A0
#define PIN_BACKSENSOR A1

namespace world {

  double rightSetpoint, rightOutput;
  double leftSetpoint, leftOutput;
  double walk_Kp=0.05, walk_Ki=0.002, walk_Kd=0.008;
  double turn_Kp=0.05, turn_Ki=0.002, turn_Kd=0.008;
  double rightEncoderPosition, leftEncoderPosition;
  double newRightEncoderPosition, newLeftEncoderPosition;

  byte collision = 10;
  byte frontDistance, backDistance;

  bool actionCollision = false; // false é pra parar, true é pra proximo passo.

  Encoder rightEncoderMotor(19, 18); //BROWN PIN, WHITE PIN
  Encoder leftEncoderMotor(20, 21); //BROWN PIN, WHITE PIN

  PID rightMotorPID(&rightEncoderPosition, &rightOutput, &rightSetpoint, walk_Kp, walk_Ki, walk_Kd, P_ON_E, DIRECT);
  PID leftMotorPID(&leftEncoderPosition, &leftOutput, &leftSetpoint, walk_Kp, walk_Ki, walk_Kd, P_ON_E, DIRECT);

  SharpIR frontSensor( SharpIR::GP2Y0A41SK0F, PIN_FRONTSENSOR );
  SharpIR backSensor( SharpIR::GP2Y0A41SK0F, PIN_BACKSENSOR );

  void updateEncoder(){
    if (rightEncoderMotor.read() != rightEncoderPosition) rightEncoderPosition = rightEncoderMotor.read();
    if (leftEncoderMotor.read() != leftEncoderPosition) leftEncoderPosition = leftEncoderMotor.read();
  }

  void updateDistance(){
    frontDistance = frontSensor.getDistance();
    backDistance = backSensor.getDistance();
  }

  void  zeroEncoder(){
    leftEncoderMotor.write(0);
    rightEncoderMotor.write(0);
  }

  void walkMoveMotor(){
    if(leftOutput>0){
      analogWrite(PIN_LEFTMOTOR1, fabs(leftOutput));
      analogWrite(PIN_LEFTMOTOR2, LOW);
      analogWrite(PIN_RIGHTMOTOR1, fabs(rightOutput));
      analogWrite(PIN_RIGHTMOTOR2, LOW);
    }
    else{
      analogWrite(PIN_LEFTMOTOR2, fabs(leftOutput));
      analogWrite(PIN_LEFTMOTOR1, LOW);
      analogWrite(PIN_RIGHTMOTOR2, fabs(rightOutput));
      analogWrite(PIN_RIGHTMOTOR1, LOW);
    }
  }

  void turnMoveMotor(){
    leftMotorPID.SetTunings(turn_Kp, turn_Ki, turn_Kd);
    rightMotorPID.SetTunings(turn_Kp, turn_Ki, turn_Kd);
    if(leftOutput>0&&rightOutput<0){
      analogWrite(PIN_LEFTMOTOR1, fabs(leftOutput));
      analogWrite(PIN_LEFTMOTOR2, LOW);
      analogWrite(PIN_RIGHTMOTOR2, fabs(rightOutput));
      analogWrite(PIN_RIGHTMOTOR1, LOW);
    }
    else{
      analogWrite(PIN_LEFTMOTOR2, fabs(leftOutput));
      analogWrite(PIN_LEFTMOTOR1, LOW);
      analogWrite(PIN_RIGHTMOTOR1, fabs(rightOutput));
      analogWrite(PIN_RIGHTMOTOR2, LOW);
    }
  }

  void stopMove(){
    analogWrite(PIN_RIGHTMOTOR1, LOW);
    analogWrite(PIN_RIGHTMOTOR2, LOW);
    analogWrite(PIN_LEFTMOTOR1, LOW);
    analogWrite(PIN_LEFTMOTOR2, LOW);
  }

  void permawalk(boolean direction){
    String result;
    zeroEncoder();
    while(true){
      updateEncoder();
      updateDistance();
      if(Serial.available()>0)
      if(Serial.peek()==31) {
        Serial.read();
        result = "INTERRUPTED";
        break;
      }
      else Serial.read();
      if (direction){
        analogWrite(PIN_LEFTMOTOR2, fabs(255));
        analogWrite(PIN_LEFTMOTOR1, LOW);
        analogWrite(PIN_RIGHTMOTOR1, fabs(255));
        analogWrite(PIN_RIGHTMOTOR2, LOW);
        if(backDistance<=collision) {
          result = "COLLISION";
          break;
        }
      }else{
        analogWrite(PIN_LEFTMOTOR1, fabs(255));
        analogWrite(PIN_LEFTMOTOR2, LOW);
        analogWrite(PIN_RIGHTMOTOR1, fabs(255));
        analogWrite(PIN_RIGHTMOTOR2, LOW);
        if(frontDistance<=collision) {
          result = "COLLISION";
          break;
        }
      }
    }
    if(direction){
      analogWrite(PIN_LEFTMOTOR1, fabs(255));
      analogWrite(PIN_LEFTMOTOR2, LOW);
      analogWrite(PIN_RIGHTMOTOR1, fabs(255));
      analogWrite(PIN_RIGHTMOTOR2, LOW);
    }else{
      analogWrite(PIN_LEFTMOTOR2, fabs(255));
      analogWrite(PIN_LEFTMOTOR1, LOW);
      analogWrite(PIN_RIGHTMOTOR1, fabs(255));
      analogWrite(PIN_RIGHTMOTOR2, LOW);
    }
    delay(100);
    stopMove();
    Serial.print(result);
    Serial.print(", ");
    Serial.print(leftEncoderPosition);
    Serial.print(", ");
    Serial.print(rightEncoderPosition);
  }

  void execute(int action, int steps){
    String result;
    if(action == 0){
      leftMotorPID.SetTunings(walk_Kp, walk_Ki, walk_Kd);
      rightMotorPID.SetTunings(walk_Kp, walk_Ki, walk_Kd);
      if(steps > 0){
        leftMotorPID.SetOutputLimits(30, 255);
        rightMotorPID.SetOutputLimits(30, 255);
      }
      else{
        leftMotorPID.SetOutputLimits(-255, -30);
        rightMotorPID.SetOutputLimits(-255, -30);
      }
      leftSetpoint = steps;
      rightSetpoint = steps;
      zeroEncoder();
      while(fabs(leftEncoderPosition - leftSetpoint) > 20){
        if(Serial.available()>0) if(Serial.peek() == 31){
          Serial.read();
          result = "INTERRUPTED";
          break;
        }
        updateEncoder();
        updateDistance();
        rightMotorPID.Compute();
        leftMotorPID.Compute();
        if(steps > 0 && frontDistance <= collision){
          result = "COLLISION";
          break;
        }
        if(steps < 0 && backDistance <= collision){
          result = "COLLISION";
          break;
        }
        walkMoveMotor();
      }
      stopMove();
      if(result == NULL) result = "SUCCESS";
    }
    if(action == 1){
      leftMotorPID.SetTunings(turn_Kp, turn_Ki, turn_Kd);
      rightMotorPID.SetTunings(turn_Kp, turn_Ki, turn_Kd);
      if(steps > 0){
        leftMotorPID.SetOutputLimits(30, 255);
        rightMotorPID.SetOutputLimits(-255, -30);
      }
      else{
        leftMotorPID.SetOutputLimits(-255, -30);
        rightMotorPID.SetOutputLimits(30, 255);
      }
      leftSetpoint = steps;
      rightSetpoint = -steps;
      zeroEncoder();
      while(fabs(leftEncoderPosition - leftSetpoint) > 20){
        if(Serial.available()>0) if(Serial.peek() == 31){
          Serial.read();
          result = "INTERRUPTED";
          break;
        }
        updateEncoder();
        rightMotorPID.Compute();
        leftMotorPID.Compute();
        turnMoveMotor();
      }
      stopMove();
      if(result == NULL) result = "SUCCESS";
    }
    Serial.print(result);
    Serial.print(", ");
    Serial.print(leftEncoderPosition);
    Serial.print(", ");
    Serial.print(rightEncoderPosition);
  }

}
