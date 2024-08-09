#include "MPU.h"
#include "pidcontroler.h"
#include "ppm.h"
#include  "Servo.h"

pid controller;

const float loopTime = .1;
MPU6050 mpu(0x68);

#define LF_Motor 3
#define LB_Motor 5
#define RF_Motor 6
#define RB_Motor 9

Servo LF_motor;
Servo LB_motor;
Servo RF_motor;
Servo RB_motor;


void setup() {
  Serial.begin(9600);
  Serial.println(" ---Configuration---");
  LF_motor.attach(LF_Motor);
  LB_motor.attach(LB_Motor);
  RF_motor.attach(RF_Motor);
  RB_motor.attach(RB_Motor);

  LF_motor.writeMicroseconds(1000);
  LB_motor.writeMicroseconds(1000);
  RF_motor.writeMicroseconds(1000);
  RB_motor.writeMicroseconds(1000);

  mpu.begin();

  ppm.begin(A0, false);
  delay(1000);
  timePrev = millis();
  controller.reset();
  

}

void updateElapsedTime() {
  time = millis();
  elapsedTime = (time - timePrev) / 1000.0;
  timePrev = time;
}

void loop(){
  int th = ppm.read_channel(3);
  updateElapsedTime();
  float inclinations[3];
  mpu.readInclination(inclinations);

  float actualRoll =  inclinations[0]; 
  float actualPitch = inclinations[1]; 
  float actualYaw =   inclinations[2]; 

  controller.MapAngles(ppm.read_channel(1),ppm.read_channel(2),ppm.read_channel(4));

  controller.error(actualRoll, actualPitch, actualYaw);
  controller.compute();

  controller.motorControl(th);

  LF_motor.writeMicroseconds(pwm_L_F);
  LB_motor.writeMicroseconds(pwm_L_B);
  RF_motor.writeMicroseconds(pwm_R_F);
  RB_motor.writeMicroseconds(pwm_R_B);
  Serial.println( "||moteur1 :" +String(pwm_L_F) + "||moteur2 :" + String(pwm_L_B) + "||moteur3:"+String(pwm_R_F) + "||moteur4:" +String(pwm_R_B));


  delay(10);
}