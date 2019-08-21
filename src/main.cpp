#define ENCODER_USE_INTERRUPTS

#include <Arduino.h>
#include <Ultrasonic.h>
#include <Encoder.h>
#include <PID_v1.h>
#include <avr/wdt.h>
#include <World.h>

using namespace world;

byte serial[7];

union AtoF{
  float f;
  byte a4[4];
}floatbyte;

union AtoI{
  uint32_t u;
  byte a4[4];
}intbyte;

uint32_t array_to_Int(byte byte0, byte byte1, byte byte2, byte byte3){

  intbyte.a4[3] = byte0;
  intbyte.a4[2] = byte1;
  intbyte.a4[1] = byte2;
  intbyte.a4[0] = byte3;

  return intbyte.u;
}

float array_to_Float(byte byte0, byte byte1, byte byte2, byte byte3){

  floatbyte.a4[3] = byte0;
  floatbyte.a4[2] = byte1;
  floatbyte.a4[1] = byte2;
  floatbyte.a4[0] = byte3;

  return floatbyte.f;
}

void serialReader(){
  while(Serial.available() <= 6);

  while(Serial.available() >= 7){

    while(Serial.peek() != 30) Serial.read();
    Serial.readBytes(serial, 7);
    if(serial[6] != 50) break;

    switch (serial[1]){

      case 100:
      Serial.println("OK");
      break;

      case 101:
      wdt_reset();
      break;

      case 102:
      Serial.print("WALK PID = ");
      Serial.print(walk_Kp, 4);
      Serial.print(", ");
      Serial.print(walk_Ki, 4);
      Serial.print(", ");
      Serial.println(walk_Kd, 4);
      break;

      case 103:
      Serial.print("TURN PID = ");
      Serial.print(turn_Kp, 4);
      Serial.print(", ");
      Serial.print(turn_Ki, 4);
      Serial.print(", ");
      Serial.println(turn_Kd, 4);
      break;

      case 104:
      Serial.print(actionCollision?"NEXT":"STOP");
      Serial.print(", ");
      Serial.println(collision);
      break;

      case 105:
      Serial.print("POSITION = ");
      Serial.print(((direction?-1:1)*leftEncoderPosition));
      Serial.print(", ");
      Serial.println(((direction?-1:1)*rightEncoderPosition));
      break;

      case 106:
      updateDistance();
      Serial.print("DISTANCE = ");
      Serial.print(frontDistance);
      Serial.print(", ");
      Serial.println(backDistance);
      break;

      case 110:
      walk_Kd = array_to_Float(serial[2], serial[3], serial[4], serial[5]);
      leftMotorPID.SetTunings(walk_Kp, walk_Ki, walk_Kd);
      //rightMotorPID.SetTunings(walk_Kp, walk_Ki, walk_Kd);
      Serial.print("WALK_KD = ");
      Serial.println(walk_Kd, 4);
      break;

      case 111:
      walk_Ki = array_to_Float(serial[2], serial[3], serial[4], serial[5]);
      leftMotorPID.SetTunings(walk_Kp, walk_Ki, walk_Kd);
      //rightMotorPID.SetTunings(walk_Kp, walk_Ki, walk_Kd);
      Serial.print("WALK_KI = ");
      Serial.println(walk_Ki, 4);
      break;

      case 112:
      walk_Kp = array_to_Float(serial[2], serial[3], serial[4], serial[5]);
      leftMotorPID.SetTunings(walk_Kp, walk_Ki, walk_Kd);
      //rightMotorPID.SetTunings(walk_Kp, walk_Ki, walk_Kd);
      Serial.print("WALK_KP = ");
      Serial.println(walk_Kp, 4);
      break;

      case 113:
      turn_Kd = array_to_Float(serial[2], serial[3], serial[4], serial[5]);
      leftMotorPID.SetTunings(turn_Kp, turn_Ki, turn_Kd);
      //rightMotorPID.SetTunings(turn_Kp, turn_Ki, turn_Kd);
      Serial.print("TURN_KD = ");
      Serial.println(turn_Kd, 4);
      break;

      case 114:
      turn_Ki = array_to_Float(serial[2], serial[3], serial[4], serial[5]);
      leftMotorPID.SetTunings(turn_Kp, turn_Ki, turn_Kd);
      //rightMotorPID.SetTunings(turn_Kp, turn_Ki, turn_Kd);
      Serial.print("TURN_KI = ");
      Serial.println(turn_Ki, 4);
      break;

      case 115:
      turn_Kp = array_to_Float(serial[2], serial[3], serial[4], serial[5]);
      leftMotorPID.SetTunings(turn_Kp, turn_Ki, turn_Kd);
      //rightMotorPID.SetTunings(turn_Kp, turn_Ki, turn_Kd);
      Serial.print("TURN_KP = ");
      Serial.println(turn_Kp, 4);
      break;

      case 116:
      actionCollision = (serial[4] == 1);
      if(serial[5]>30) serial[5] = 30;
      else if(serial[5]<0) serial[5] = 0;
      collision = serial[5];
      Serial.print("COLLISION = ");
      Serial.print(serial[4]==1?"NEXT":"STOP");
      Serial.print(", ");
      Serial.println(serial[5]);

      break;

      case 117:
      uint32_t steps = array_to_Int(0, serial[3], serial[4], serial[5]);
      int action = ((serial[2]>>1)&1);
      direction = ((serial[2]&1) == 1);
      if(((serial[2]>>2)&1) == 0) execute(action, steps);
      else permawalk();
      Serial.println();
    }
  }
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Tudo Pronto");
  //rightMotorPID.SetMode(AUTOMATIC);
  //rightMotorPID.SetSampleTime(10);
  leftMotorPID.SetMode(AUTOMATIC);
  leftMotorPID.SetSampleTime(10);
}

void loop() {
  serialReader();
}
