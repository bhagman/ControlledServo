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
|| | See ControlledServo.h for notes.
|| |
|| #
||
*/

#include <ControlledServo.h>

ControlledServo::ControlledServo()
{
  _servo = NULL;
  _currentAngle = 90;
  _targetAngle = 90;
  _msPerDegree = 1;
  _minAngle = 0;
  _maxAngle = 180;
  _trim = 0;
  _lastUpdateTime = 0;
}

ControlledServo::ControlledServo(Servo &servo)
{
  ControlledServo();
  _servo = &servo;
}


/*
|| Prepares the ControlledServo for subsequent actions.
*/
void ControlledServo::begin(uint8_t angle)
{
  // Sets the starting angle.
  _currentAngle = constrain(angle, 0, 180);
  // Resets the target angle.
  stop();
}


/*
|| Executes a move.
*/
void ControlledServo::move(boolean blocking)
{
  _lastUpdateTime = millis();

  // If blocking is true, let's wait here until the servo is at the target.
  if (blocking)
    while (update());
}

/*
|| Sets the next angle to move to, then executes.
*/
void ControlledServo::moveTo(uint8_t angle, boolean blocking)
{
  setAngle(angle);
  move(blocking);
}


/*
|| Checks if the servo is still moving.
*/
boolean ControlledServo::moving()
{
  return _currentAngle != _targetAngle;
}


/*
|| Stops an asynchronous move().  Sets the targetAngle to the currentAngle.
*/
void ControlledServo::stop(void)
{
  // Sets targetAngle to currentAngle, to stop any more updates.
  _targetAngle = _currentAngle;
}


/*
|| For asynchronous operation, this is called to update the servo position.
*/
bool ControlledServo::update()
{
  // This is our processing state - i.e. do we still need to move the servo to the target?
  bool processing = false;
  uint32_t currentTime = millis();

  if (_currentAngle != _targetAngle)
  {
    // Don't bother processing unless we are ready to move at least 1 degree (i.e. _msPerDegree ms has passed).
    if (currentTime < (_lastUpdateTime + _msPerDegree))
    {
      return true;
    }

    int16_t angleDifference = (int16_t)_targetAngle - (int16_t)_currentAngle;
    int8_t direction = angleDifference < 0 ? -1 : 1;
    uint8_t newAngle;
    newAngle = _currentAngle + ((currentTime - _lastUpdateTime) / _msPerDegree) * direction;

//Serial.print(angleDifference);
//Serial.print(" : ");
//Serial.print(direction);
//Serial.print(" : ");
//Serial.print(currentTime);
//Serial.print(" : ");
//Serial.print(_lastUpdateTime);
//Serial.print(" : ");
//Serial.print(_msPerDegree);

    if (direction < 0)
    {
      if (newAngle < _targetAngle)
        newAngle = _targetAngle;
    }
    else
    {
      if (newAngle > _targetAngle)
        newAngle = _targetAngle;
    }

    if (_servo)
    {
      int16_t outAngle = (int16_t)newAngle + _trim);
      _servo->write(constrain(outAngle, 0, 180));
    }
    _currentAngle = newAngle;

//Serial.print(" >> ");
//Serial.print(newAngle);
//Serial.print(" : ");
//Serial.print(_currentAngle);
//Serial.print(" : ");
//Serial.println(_targetAngle);
    processing = true;
  }

//Serial.print("Good -- ");
//Serial.print(_lastUpdateTime);
//Serial.print(" : ");
//Serial.println(currentTime);
  _lastUpdateTime = currentTime;

  return processing;
}


/*
|| Sets the target angle.
*/
void ControlledServo::setAngle(uint8_t angle)
{
  _targetAngle = constrain(angle, _minAngle, _maxAngle);
}

