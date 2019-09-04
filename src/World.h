#define PIN_RIGHTMOTOR1 6 //RED PIN
#define PIN_RIGHTMOTOR2 7 //BROWN PIN
#define PIN_LEFTMOTOR1 8 //YELLOW PIN
#define PIN_LEFTMOTOR2 9 //ORANGE PIN


namespace world {

  double Setpoint, Output;
  double walk_Kp=0.02, walk_Ki=0.002, walk_Kd=0.02;
  double turn_Kp=0.02, turn_Ki=0.002, turn_Kd=0.01;
  double rightEncoderPosition, leftEncoderPosition;
  double acceleration;

  byte collision = 0;
  byte frontDistance, backDistance;
  volatile byte leftSpeed, rightSpeed;

  bool actionCollision = false; // false é pra parar, true é pra proximo passo.
  volatile bool leftDirection, rightDirection; //TRUE é para frente

  Encoder rightEncoderMotor(19, 18); //BROWN PIN, WHITE PIN
  Encoder leftEncoderMotor(20, 21); //BROWN PIN, WHITE PIN

  PID MotorPID(&leftEncoderPosition, &Output, &Setpoint, walk_Kp, walk_Ki, walk_Kd, P_ON_E, DIRECT);

  Ultrasonic frontSensor(22, 23, 2000UL);
  Ultrasonic backSensor(24, 25, 2000UL);

  void updateEncoder(bool direction){
    if (rightEncoderMotor.read() != rightEncoderPosition) rightEncoderPosition = ((direction?-1:1)*rightEncoderMotor.read());
    if (leftEncoderMotor.read() != leftEncoderPosition) leftEncoderPosition = ((direction?-1:1)*leftEncoderMotor.read());
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

  void moveMotor(){
    if(leftDirection){
      analogWrite(PIN_LEFTMOTOR1, leftSpeed);
      analogWrite(PIN_LEFTMOTOR2, 0);
    }else{
      analogWrite(PIN_LEFTMOTOR1, 0);
      analogWrite(PIN_LEFTMOTOR2, leftSpeed);
    }
    if(rightDirection){
      analogWrite(PIN_RIGHTMOTOR1, rightSpeed);
      analogWrite(PIN_RIGHTMOTOR2, 0);
    }else{
      analogWrite(PIN_RIGHTMOTOR1, 0);
      analogWrite(PIN_RIGHTMOTOR2, rightSpeed);
    }
  }

  String execute(int action, uint32_t steps, bool direction){
    zeroEncoder();
    if(action == 0){
      MotorPID.SetTunings(walk_Kp, walk_Ki, walk_Kd);
      MotorPID.SetOutputLimits(30, 255);
      Setpoint = steps;
      if(fabs(leftEncoderPosition - Setpoint) > 6000){
        //rampUp(); FUNÇÃO DE ACELERAÇÃO
        MotorPID.Compute();
      }
      Output = 0;
      while(fabs(leftEncoderPosition - Setpoint) > 20){
        updateEncoder(direction);
        updateDistance();
        if(Serial.available()>0) if(Serial.peek() == 31){
          Serial.read();
          return "INTERRUPTED";
        }
        MotorPID.Compute();
        leftSpeed = Output;
        rightSpeed = Output;
        leftDirection = direction;
        rightDirection = direction;
        if(!direction && frontDistance <= collision){
          return "COLLISION";
        }
        if(direction && backDistance <= collision){
          return "COLLISION";
        }
        moveMotor();
      }
      Output = 0;
      return "SUCCESS";
    }

    if(action == 1){
      MotorPID.SetTunings(turn_Kp, turn_Ki, turn_Kd);
      MotorPID.SetOutputLimits(40, 255);
      Setpoint = steps;
      while(fabs(leftEncoderPosition - Setpoint) > 20){
        if(Serial.available()>0) if(Serial.peek() == 31){
          Serial.read();
          return "INTERRUPTED";
        }
        updateEncoder(direction);
        MotorPID.Compute();
        leftSpeed = Output;
        rightSpeed = Output;
        leftDirection = direction;
        rightDirection = !direction;
      }
    Output = 0;
    moveMotor();
    return "SUCCESS";
    }
  }

}
