#define PIN_RIGHTMOTOR1 13 //RED PIN
#define PIN_RIGHTMOTOR2 12 //BROWN PIN
#define PIN_LEFTMOTOR1 25 //YELLOW PIN
#define PIN_LEFTMOTOR2 26 //ORANGE PIN

#define PIN_RIGHTENCODER1 35 //BROWN PIN
#define PIN_RIGHTENCODER2 34 //WHITE PIN
#define PIN_LEFTENCODER1 33 //BROWN PIN
#define PIN_LEFTENCODER2 34 //WHITE PIN

#define PIN_BACKTRIGGER 19 //GREEN PIN
#define PIN_BACKECHO 18 //YELLOW PIN
#define PIN_FRONTTRIGGER 23 //GREEN PIN
#define PIN_FRONTECHO 22 //YELLOW PIN

#define RIGHTCHANNEL1 0
#define RIGHTCHANNEL2 1
#define LEFTCHANNEL1 2
#define LEFTCHANNEL2 3

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

  ESP32Encoder rightEncoderMotor;
  ESP32Encoder leftEncoderMotor;

  PID MotorPID(&leftEncoderPosition, &Output, &Setpoint, walk_Kp, walk_Ki, walk_Kd, P_ON_E, DIRECT);

  Ultrasonic frontSensor(PIN_FRONTTRIGGER, PIN_FRONTECHO, 2000UL);
  Ultrasonic backSensor(PIN_BACKTRIGGER, PIN_BACKECHO, 2000UL);

  void updateEncoder(){
    leftEncoderPosition = leftEncoderMotor.getCount();
    rightEncoderPosition = rightEncoderMotor.getCount();
  }

  void updateDistance(){
    backDistance = backSensor.read();
    frontDistance = frontSensor.read();
  }

  void  zeroEncoder(){
    leftEncoderMotor.setCount(0);
    rightEncoderMotor.setCount(0);
    updateEncoder();
  }

  void moveMotor(){
    if(leftDirection){
      ledcWrite(LEFTCHANNEL1, leftSpeed);
      ledcWrite(LEFTCHANNEL1, 0);
      //analogWrite(PIN_LEFTMOTOR1, leftSpeed);
      //analogWrite(PIN_LEFTMOTOR2, 0);
    }else{
      ledcWrite(LEFTCHANNEL1, 0);
      ledcWrite(LEFTCHANNEL1, leftSpeed);
      //analogWrite(PIN_LEFTMOTOR1, 0);
      //analogWrite(PIN_LEFTMOTOR2, leftSpeed);
    }
    if(rightDirection){
      ledcWrite(RIGHTCHANNEL1, rightSpeed);
      ledcWrite(RIGHTCHANNEL1, 0);
      //analogWrite(PIN_RIGHTMOTOR1, rightSpeed);
      //analogWrite(PIN_RIGHTMOTOR2, 0);
    }else{
      ledcWrite(RIGHTCHANNEL1, 0);
      ledcWrite(RIGHTCHANNEL1, rightSpeed);
      //analogWrite(PIN_RIGHTMOTOR1, 0);
      //analogWrite(PIN_RIGHTMOTOR2, rightSpeed);
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
        updateEncoder();
        moveMotor();
      }
      Output = 0;
      while(fabs(leftEncoderPosition - Setpoint) > 20){
        updateEncoder();
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
        updateEncoder();
        MotorPID.Compute();
        leftSpeed = Output;
        rightSpeed = Output;
        leftDirection = direction;
        rightDirection = !direction;
        moveMotor();
      }
    Output = 0;
    moveMotor();
    return "SUCCESS";
    }
  }

}
