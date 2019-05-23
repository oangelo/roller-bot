#define ENCODER_USE_INTERRUPTS

#include <Arduino.h>
#include <Encoder.h>
#include <PID_v1.h>
#include <SharpIR.h>
#include <avr/wdt.h>
#include <World.h>

using namespace world;

byte serial[7];

float array_to_Float(byte byte0, byte byte1, byte byte2, byte byte3){
  float output;
  output = ((byte0<<24)|(byte1<<16)|(byte2<8)|(byte3)); // WE HAVE NO IDEA IF THIS FUNCTION WORKS
  return output;
}

int array_to_Int(byte byte0, byte byte1, byte byte2, byte byte3){
  int output;
  output = ((byte0<<24)|(byte1<<16)|(byte2<8)|(byte3));
  return output;
}

void serialReader(){
  while(Serial.available() <= 6);

  while(Serial.available() >= 7){

    while(Serial.peek() != 30) Serial.read();
    Serial.readBytes(serial, 7);
    if(serial[6] != 50) break;

    switch (serial[1]){

      case 100:
        Serial.print("OK");
      break;

      case 101:
        wdt_reset();
      break;

      case 102:
        Serial.print(walk_Kd);
        Serial.print(", ");
        Serial.print(walk_Ki);
        Serial.print(", ");
        Serial.print(walk_Kp);
        Serial.print(", ");
      break;

      case 103:
        Serial.print(turn_Kd);
        Serial.print(", ");
        Serial.print(turn_Ki);
        Serial.print(", ");
        Serial.print(turn_Kp);
      break;

      case 104:
        Serial.print(actionCollision?"NEXT":"STOP");
        Serial.print(", ");
        Serial.print(collision);
      break;

      case 105:
        Serial.print(leftEncoderPosition);
        Serial.print(", ");
        Serial.print(rightEncoderPosition);
      break;

      case 106:
        Serial.print(frontDistance);
        Serial.print(", ");
        Serial.print(backDistance);
      break;

      case 110:
        walk_Kd = array_to_Float(serial[2], serial[3], serial[4], serial[5]);
        leftMotorPID.SetTunings(walk_Kp, walk_Ki, walk_Kd);
        rightMotorPID.SetTunings(walk_Kp, walk_Ki, walk_Kd);
      break;

      case 111:
        walk_Ki = array_to_Float(serial[2], serial[3], serial[4], serial[5]);
        leftMotorPID.SetTunings(walk_Kp, walk_Ki, walk_Kd);
        rightMotorPID.SetTunings(walk_Kp, walk_Ki, walk_Kd);
      break;

      case 112:
        walk_Kp = array_to_Float(serial[2], serial[3], serial[4], serial[5]);
        leftMotorPID.SetTunings(walk_Kp, walk_Ki, walk_Kd);
        rightMotorPID.SetTunings(walk_Kp, walk_Ki, walk_Kd);
      break;

      case 113:
        turn_Kd = array_to_Float(serial[2], serial[3], serial[4], serial[5]);
        leftMotorPID.SetTunings(turn_Kp, turn_Ki, turn_Kd);
        rightMotorPID.SetTunings(turn_Kp, turn_Ki, turn_Kd);
      break;

      case 114:
        turn_Ki = array_to_Float(serial[2], serial[3], serial[4], serial[5]);
        leftMotorPID.SetTunings(turn_Kp, turn_Ki, turn_Kd);
        rightMotorPID.SetTunings(turn_Kp, turn_Ki, turn_Kd);
      break;

      case 115:
        turn_Kp = array_to_Float(serial[2], serial[3], serial[4], serial[5]);
        leftMotorPID.SetTunings(turn_Kp, turn_Ki, turn_Kd);
        rightMotorPID.SetTunings(turn_Kp, turn_Ki, turn_Kd);
      break;

      case 116:
        actionCollision = (serial[4] == 1);
        collision = serial[5];
      break;

      case 117:
        int steps = array_to_Int(0, serial[3], serial[4], serial[5]);
        int action = ((serial[2]>>1)&1);
        if((serial[2]&1) == 1) steps *= -1;
        execute(action, steps);
  }
}
}

void setup() {
  delay(1000);
  Serial.begin(115200);
  rightMotorPID.SetMode(AUTOMATIC);
  rightMotorPID.SetSampleTime(10);

  leftMotorPID.SetMode(AUTOMATIC);
  leftMotorPID.SetSampleTime(10);
}

void loop() {
  serialReader();
}
