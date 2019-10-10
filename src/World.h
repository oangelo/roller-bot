#define PIN_RIGHTMOTOR1 13 //BROWN PIN
#define PIN_RIGHTMOTOR2 12 //RED PIN
#define PIN_LEFTMOTOR1 25 //ORANGE PIN
#define PIN_LEFTMOTOR2 26 //YELLOW PIN

#define PIN_RIGHTENCODER1 34 //BROWN PIN
#define PIN_RIGHTENCODER2 35 //WHITE PIN
#define PIN_LEFTENCODER1 32 //BROWN PIN
#define PIN_LEFTENCODER2 33 //WHITE PIN

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

  long now = millis();

  byte collision = 5;
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

  void updateDistance(int sensor){
    if(millis()-now>=10){
    if(sensor==0 || sensor==2) backDistance = backSensor.read();
    if(sensor==1 || sensor==2) frontDistance = frontSensor.read();
    now = millis();
    }
  }

  void  zeroEncoder(){
    leftEncoderMotor.clearCount();
    rightEncoderMotor.clearCount();
    updateEncoder();
  }

  void moveMotor(){
      ledcWrite(LEFTCHANNEL1, leftDirection?leftSpeed:0);
      ledcWrite(LEFTCHANNEL2, leftDirection?0:leftSpeed);
      ledcWrite(RIGHTCHANNEL1, rightDirection?rightSpeed:0);
      ledcWrite(RIGHTCHANNEL2, rightDirection?0:rightSpeed);
  }

  String execute(int action, uint32_t steps, bool direction){
    zeroEncoder();
    Serial.printf("Iniciando World.step com %d passos e %s%s\n", steps, action==0?"andando para ":"girando no sentido", action==0?direction?"frente":"tras":direction?"horario":"anti-horario");
    if(action == 0){
      MotorPID.SetTunings(walk_Kp, walk_Ki, walk_Kd);
      MotorPID.SetOutputLimits(40, 255);
      Setpoint = steps;
      leftDirection = direction;
      rightDirection = direction;
      /*if(fabs(leftEncoderPosition - Setpoint) > 6000){
        //rampUp(); FUNÇÃO DE ACELERAÇÃO
        updateEncoder();
        MotorPID.Compute();
        leftSpeed = Output;
        rightSpeed = Output;
        moveMotor();
      }*/
      while(fabs(leftEncoderPosition - Setpoint) > 20){
        updateEncoder();
        updateDistance(direction?1:0);
        if(Serial.available()>0) if(Serial.peek() == 31){
          Serial.read();
          return "INTERRUPTED";
        }
        byte left = leftSpeed;
        MotorPID.Compute();
        leftSpeed = Output;
        rightSpeed = Output;
        if(direction && frontDistance <= collision){Serial.println(frontDistance); return "COLLISION FRENTE";};
        if(!direction && backDistance <= collision){Serial.println(backDistance); return "COLLISION TRAS";};
        if(leftSpeed!=left) moveMotor();
        Serial.print(leftSpeed);
        Serial.print(',');
        Serial.println(leftEncoderPosition);
      }
      return "SUCCESS";
    } //Fim de Walk

    if(action == 1){
      MotorPID.SetTunings(turn_Kp, turn_Ki, turn_Kd);
      MotorPID.SetOutputLimits(40, 255);
      Setpoint = steps;
      leftDirection = direction;
      rightDirection = !direction;
      while(fabs(leftEncoderPosition - Setpoint) > 20){
        if(Serial.available()>0) if(Serial.peek() == 31){
          Serial.read();
          return "INTERRUPTED";
        }
        updateEncoder();
        MotorPID.Compute();
        leftSpeed = Output;
        rightSpeed = Output;
        moveMotor();
      }
      return "SUCCESS";
  } //Fim de Turn

  } //Fim de Execute

}
