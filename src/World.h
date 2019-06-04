#define PIN_RIGHTMOTOR1 6 //RED PIN
#define PIN_RIGHTMOTOR2 7 //BROWN PIN
#define PIN_LEFTMOTOR1 8 //YELLOW PIN
#define PIN_LEFTMOTOR2 9 //ORANGE PIN

namespace world {

  double rightSetpoint, rightOutput;
  double leftSetpoint, leftOutput;
  double walk_Kp=0.05, walk_Ki=0.002, walk_Kd=0.008;
  double turn_Kp=0.05, turn_Ki=0.002, turn_Kd=0.008;
  double rightEncoderPosition, leftEncoderPosition;
  double newRightEncoderPosition, newLeftEncoderPosition;

  long distNow = millis();

  byte collision = 10;
  short frontDistance, backDistance;

  bool actionCollision = false; // false é pra parar, true é pra proximo passo.

  Encoder rightEncoderMotor(19, 18); //BROWN PIN, WHITE PIN
  Encoder leftEncoderMotor(20, 21); //BROWN PIN, WHITE PIN

//  PID rightMotorPID(&rightEncoderPosition, &rightOutput, &rightSetpoint, walk_Kp, walk_Ki, walk_Kd, P_ON_E, DIRECT);
  PID leftMotorPID(&leftEncoderPosition, &leftOutput, &leftSetpoint, walk_Kp, walk_Ki, walk_Kd, P_ON_E, DIRECT);

  Ultrasonic frontSensor(22, 23, 4000UL);
  Ultrasonic backSensor(24, 25, 4000UL);

  void updateEncoder(){
    if (rightEncoderMotor.read() != rightEncoderPosition) rightEncoderPosition = rightEncoderMotor.read();
    if (leftEncoderMotor.read() != leftEncoderPosition) leftEncoderPosition = leftEncoderMotor.read();
  }

  void updateDistance(){
    backDistance = backSensor.read();
    frontDistance = frontSensor.read();
  }

  void  zeroEncoder(){
    leftEncoderMotor.write(0);
    leftEncoderPosition = 0;
    rightEncoderMotor.write(0);
    rightEncoderPosition = 0;
  }

  void moveRightMotor(short output){
    if(output>=0){
      analogWrite(PIN_RIGHTMOTOR1, output);
      analogWrite(PIN_RIGHTMOTOR2, LOW);
    }else{
      analogWrite(PIN_RIGHTMOTOR2, -output);
      analogWrite(PIN_RIGHTMOTOR1, LOW);
    }
  }

  void moveLeftMotor(short output){
    if(output>0){
      analogWrite(PIN_LEFTMOTOR1, output);
      analogWrite(PIN_LEFTMOTOR2, LOW);
    }else{
      analogWrite(PIN_LEFTMOTOR2, -output);
      analogWrite(PIN_LEFTMOTOR1, LOW);
    }
  }

  void walkMoveMotor(){
      moveLeftMotor(leftOutput);
      moveRightMotor(leftOutput); // mudamos pra testar
  }

  void turnMoveMotor(){
    leftMotorPID.SetTunings(turn_Kp, turn_Ki, turn_Kd);
    //rightMotorPID.SetTunings(turn_Kp, turn_Ki, turn_Kd);
    moveLeftMotor(-leftOutput);
    moveRightMotor(leftOutput); //teste
  }

  void stopMove(){
    moveLeftMotor(0);
    moveRightMotor(0);
  }

  void permawalk(boolean direction){
    String result;
    zeroEncoder();
    while(true){
      updateEncoder();
      updateDistance();
      if(Serial.available()>0){
        if(Serial.peek()==31) {
          Serial.read();
          result = "INTERRUPTED";
          break;
        }
        else Serial.read();
      }
      if (direction){
        moveLeftMotor(-255);
        moveRightMotor(-255);
        if(backDistance<=collision) {
          result = "COLLISION";
          break;
        }
      }else{
        moveLeftMotor(255);
        moveRightMotor(255);
        if(frontDistance<=collision) {
          result = "COLLISION";
          break;
        }
      }
    }
    if(direction){
      moveLeftMotor(255);
      moveRightMotor(255);
    }else{
      moveLeftMotor(-255);
      moveRightMotor(-255);
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
    zeroEncoder();
    if(action == 0){
      leftMotorPID.SetTunings(walk_Kp, walk_Ki, walk_Kd);
      //rightMotorPID.SetTunings(walk_Kp, walk_Ki, walk_Kd);
      if(steps > 0){
        leftMotorPID.SetOutputLimits(30, 255);
        //rightMotorPID.SetOutputLimits(30, 255);
      }
      else{
        leftMotorPID.SetOutputLimits(-255, -30);
        //rightMotorPID.SetOutputLimits(-255, -30);
      }
      leftSetpoint = steps;
      //rightSetpoint = steps;
      while(fabs(leftEncoderPosition - leftSetpoint) > 20){
        updateEncoder();
        updateDistance();
        if(Serial.available()>0) if(Serial.peek() == 31){
          Serial.read();
          result = "INTERRUPTED";
          break;
        }
        //rightMotorPID.Compute();
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
      //rightMotorPID.SetTunings(turn_Kp, turn_Ki, turn_Kd);
      if(steps > 0){
        leftMotorPID.SetOutputLimits(30, 255);
        //rightMotorPID.SetOutputLimits(-255, -30);
      }
      else{
        leftMotorPID.SetOutputLimits(-255, -30);
        //rightMotorPID.SetOutputLimits(30, 255);
      }
      leftSetpoint = steps;
      //rightSetpoint = -steps;
      while(fabs(leftEncoderPosition - leftSetpoint) > 20){
        if(Serial.available()>0) if(Serial.peek() == 31){
          Serial.read();
          result = "INTERRUPTED";
          break;
        }
        updateEncoder();
        //rightMotorPID.Compute();
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
