/*
|| @author         Brett Hagman <bhagman@wiring.org.co>
|| @url            http://wiring.org.co/
||
|| @description
|| |
|| | This is an example sketch using the ControlledServo library to control a TinkerKit Braccio
|| | (servo controlled robotic arm).
|| |
|| #
||
|| @license Please see the accompanying LICENSE.txt file for this project.
||
*/

#include "ControlledServo.h"
#include <Servo.h>

Servo s_base;
Servo s_shoulder;
Servo s_elbow;
Servo s_wrist;
Servo s_rotate;
Servo s_gripper;

const int basePin = 11;
const int shoulderPin = 10;
const int elbowPin = 9;
const int wristPin = 6;
const int rotatePin = 5;
const int gripperPin = 3;

const int angleRate = 20; // in msPerDegree

ControlledServo base;
ControlledServo shoulder;
ControlledServo elbow;
ControlledServo wrist;
ControlledServo rotate;
ControlledServo gripper;


/*
|| Set all servos to a given angle.  All parameters required.
*/
void setAll(uint8_t baseAngle,
            uint8_t shoulderAngle,
            uint8_t elbowAngle,
            uint8_t wristAngle,
            uint8_t rotateAngle,
            uint8_t gripperAngle)
{
  base.moveTo(baseAngle);
  shoulder.moveTo(shoulderAngle);
  elbow.moveTo(elbowAngle);
  wrist.moveTo(wristAngle);
  rotate.moveTo(rotateAngle);
  gripper.moveTo(gripperAngle);
}


/*
|| Checks if any joint is moving.
*/
boolean moving()
{
  boolean result;

  result = base.moving();
  result |= shoulder.moving();
  result |= elbow.moving();
  result |= wrist.moving();
  result |= rotate.moving();
  result |= gripper.moving();

  return result;
}


/*
|| Updates all servos. Can be used to asynchronously control the servos.
*/
bool update()
{
  bool processing = false;

  processing = base.update();
  processing |= shoulder.update();
  processing |= elbow.update();
  processing |= wrist.update();
  processing |= rotate.update();
  processing |= gripper.update();

  return processing;
}


void setup()
{
  Serial.begin(9600);
  Serial.println(F("Braccio Example"));

  s_base.attach(basePin);
  s_shoulder.attach(shoulderPin);
  s_elbow.attach(elbowPin);
  s_wrist.attach(wristPin);
  s_rotate.attach(rotatePin);
  s_gripper.attach(gripperPin);

  base.setServo(s_base);
  shoulder.setServo(s_shoulder);
  elbow.setServo(s_elbow);
  wrist.setServo(s_wrist);
  rotate.setServo(s_rotate);
  gripper.setServo(s_gripper);

  // Set angle change rates
  base.setRate(angleRate);
  shoulder.setRate(angleRate);
  elbow.setRate(angleRate/2);  // we want this joint to move twice as fast as the shoulder and wrist.
  wrist.setRate(angleRate);
  rotate.setRate(angleRate);
  gripper.setRate(angleRate);

  Serial.print(F("Going to start position..."));
  setAll(90, 90, 90, 90, 90, 45);
  while (moving())
    update();
  Serial.println(F("Done"));
}


void loop()
{
  Serial.print(F("Position 1..."));
  setAll(90, 135, 0, 135, 90, 45);
  while (moving())
    update();
  Serial.print(F("Position 2..."));
  setAll(90, 45, 180, 45, 90, 45);
  while (moving())
    update();
}

