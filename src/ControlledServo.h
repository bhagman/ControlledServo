/*
||
|| @author         Brett Hagman <bhagman@wiring.org.co>
|| @url            http://wiring.org.co/
|| @url            http://roguerobotics.com/
||
|| @description
|| |
|| | A class to control the movement of a servo.
|| |
|| #
||
|| @license
|| |
|| | Copyright (c) 2016 - Brett Hagman
|| |
|| | Permission is hereby granted, free of charge, to any person obtaining a copy of
|| | this software and associated documentation files (the "Software"), to deal in
|| | the Software without restriction, including without limitation the rights to
|| | use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
|| | the Software, and to permit persons to whom the Software is furnished to do so,
|| | subject to the following conditions:
|| |
|| | The above copyright notice and this permission notice shall be included in all
|| | copies or substantial portions of the Software.
|| |
|| | THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
|| | IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
|| | FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
|| | COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
|| | IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
|| | CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
|| #
||
|| @notes
|| |
|| #
||
|| @todo
|| |
|| | - add radians/sec or degrees/sec rate of change for update() -- simple math to get msPerDegree
|| | - make methods chainable
|| | - rate ramping - ease in/out for movement
|| | - sequencing? set x number of waypoints and specified rates, and execute
|| |
|| #
||
*/

#ifndef CONTROLLEDSERVO_H
#define CONTROLLEDSERVO_H

#include <Arduino.h>
#include <Servo.h>

class ControlledServo
{
  public:
    ControlledServo();
    ControlledServo(Servo &servo);

    // Prepares the ControlledServo for subsequent actions.
    void begin(uint8_t angle = 90);
    // Executes a move.
    void move(boolean blocking = false);
    void moveNow() { move(true); }
    // Sets the next angle to move to, then executes.
    void moveTo(uint8_t angle, boolean blocking = false);
    void moveToNow(uint8_t angle) { moveTo(angle, true); }
    // Checks if the servo is still moving.
    boolean moving();
    // Stops an asynchronous move().  Sets the targetAngle to the currentAngle.
    void stop();
    // For asynchronous operation, this is called to update the servo position.
    bool update();

    // Sets the servo.
    void setServo(Servo &s) { _servo = &s; }
    // Sets the target angle.
    void setAngle(uint8_t angle);
    // Gets the current angle that the servo is set to.
    uint8_t getAngle() { return _currentAngle; }
    // Gets the current target angle.
    uint8_t getTargetAngle() { return _targetAngle; }
    // Sets the rate at which the servo turns (in milliseconds per degree)
    void setRate(uint16_t rate) { _msPerDegree = rate > 0 ? rate : 1; }
    // Gets the current rate.
    uint16_t getRate() { return _msPerDegree; }
    // Sets the limits for the servo (servo range).
    void setMin(uint8_t minAngle) { _minAngle = constrain(minAngle, 0, 180); }
    void setMax(uint8_t maxAngle) { _maxAngle = constrain(maxAngle, 0, 180); }
    // Gets the limis for the servo.
    uint8_t getMin() { return _minAngle; }
    uint8_t getMax() { return _maxAngle; }
    // Sets the trim/offset for the servo.
    void setTrim(int8_t trim) { _trim = trim; }
    int8_t getTrim() { return _trim; }

  private:
    Servo *_servo;
    uint8_t _currentAngle;
    uint8_t _targetAngle;
    uint16_t _msPerDegree;
    uint8_t _minAngle;
    uint8_t _maxAngle;
    int8_t _trim;
    uint32_t _lastUpdateTime;
};

#endif

