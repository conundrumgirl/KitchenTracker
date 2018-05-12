/*
 * This example shows how to use a servo motor with the RedBear Duo by slowly oscillating the motor
 * back and forth
 *
 * For CSE590, we are using the Tower PRo SG92R micro servo that can rotate
 * approximately 180 degrees (90 in each direction). See: https://www.adafruit.com/product/169
 *
 * By Jon Froehlich for CSE590
 * http://makeabilitylab.io
 *
 * Other resources:
 *  - https://www.arduino.cc/en/Tutorial/Sweep
 *  - https://learn.adafruit.com/adafruit-arduino-lesson-14-servo-motors
 */

/*
 * IMPORTANT: When working with the RedBear Duo, you must have this line of
 * code at the top of your program. The default state is SYSTEM_MODE(AUTOMATIC);
 * however, this puts the RedBear Duo in a special cloud-based mode that we
 * are not using. For our purposes, set SYSTEM_MODE(SEMI_AUTOMATIC) or
 * SYSTEM_MODE(MANUAL). See https://docs.particle.io/reference/firmware/photon/#system-modes
 */
SYSTEM_MODE(MANUAL);

const int SERVO_OUTPUT_PIN = D2;
const int INPUT_PIN = A0;
const int DELAY_MS = 50;

int _stepAmount = 1; // the amount to change the angle of servo on each pass
int _minAngle = 0;
int _maxAngle = 180;
int _curAngle = 0;
Servo _servo;



//smoothing
const int SMOOTHING_WINDOW_SIZE = 5;
volatile int _readings[SMOOTHING_WINDOW_SIZE]; // the readings from the analog input
volatile int _readIndex = 0;                   // the index of the current reading
volatile int _total = 0;                       // the running total
volatile int _average = 0;

volatile int _oldReading = 0; // the average - used to check if we are reading phone or potentiometer

//sensor

// Pins
const int TRIG_PIN = D8;
const int ECHO_PIN = D9;

// Anything over 400 cm (23200 us pulse) is "out of range"
const unsigned int MAX_DIST = 23200;




void setup()
{
  _servo.attach(SERVO_OUTPUT_PIN);

  // us sensor

    pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  Serial.begin(9600);

  //smoothing
  for (int i = 0; i < SMOOTHING_WINDOW_SIZE; i++)
  {
    _readings[i] = 0;
  }
}


void loop()
{
  // set the current angle
  _servo.write(_curAngle);

  //int x = analogRead(INPUT_PIN);
  /*int x = getSmoothedPotReading(analogRead(INPUT_PIN));
  write_("pot reading:", x);
  _curAngle = x / 23;              // 0 to 180-ish
  write_("angle:",  _curAngle);
  _servo.write(_curAngle);*/


  // update the angle for next time the loop
  _curAngle += _stepAmount;

  // reverse the direction of the angle (as necessary)
  if (_curAngle <= _minAngle || _curAngle >= _maxAngle) {
    _stepAmount = -_stepAmount;
  }

  // wait for 30 milliseconds to see the dimming effect
  delay(DELAY_MS);
}

void write_ (char* msg, int val) {
  Serial.print(msg);
  Serial.println(val);
}

/* From Jon's example */
int getSmoothedPotReading(int curReading)
{
  _total = _total - _readings[_readIndex];
  _readings[_readIndex] = curReading;
  _total = _total + _readings[_readIndex];
  _readIndex = _readIndex + 1;
  // if we're at the end of the array...
  if (_readIndex >= SMOOTHING_WINDOW_SIZE)
  {
    _readIndex = 0;
  }

  _average = _total / SMOOTHING_WINDOW_SIZE;
  return _average;
}
