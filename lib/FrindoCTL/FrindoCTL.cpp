#include "FrindoCTL.h"

#include <math.h>

// define the pins used by the robot shield
int speedA = 6;          // pin 6 sets the speed of motor A (this is a PWM output)
int speedB = 9;          // pin 9 sets the speed of motor B (this is a PWM output)
int dirA = 8;            // pin 8 sets the direction of motor A
int dirB = 7;            // pin 7 sets the direction of motor B

// define the direction of motor rotation - this allows you change the  direction without effecting the hardware
int fwdA  =  HIGH;       // this sketch assumes that motor A is the right-hand motor of your robot (looking from the back of your robot)
int revA  =  LOW;        // (note this should ALWAYS be opposite the fwdA)
int fwdB  =  HIGH;
int revB  =  LOW;        // (note this should ALWAYS be opposite the fwdB)

// define the variables used by the FrindoCTL
int dist;
int angle;
int vel;

#define FORWARD HIGH    // defines the voltage of forward motion
#define REVERSE LOW     // defines the voltage of reverse motion

#define MAX_VELOCITY 1.0f
#define MIN_VELOCITY -1.0f

#define MIN_SENSOR_DIST 0
#define MAX_SENSOR_DIST 0

#define MAX_VOLTAGE 128

#define PI 3.14159265f
#define DEGREES (PI / 180.0f)

// clamps a value of type T in the interval min-max
template <typename T>
const T clamp(const T v, const T min, const T max)
{
    return max(min, min(v, max));
}

const int vel_2_volt(const float v)
{
    return (v >= 0) ? v * HIGH : v * LOW;
}

float raw2cm(int raw)
{
    return raw * 2.0;
}

FrindoCTL::FrindoCTL()                           // sets up the pinModes for the pins we are using
{
    // initialize sensors
    front = new Sensor(0, &raw2cm);
    right = new Sensor(1, &raw2cm);
    left  = new Sensor(2, &raw2cm);

    // initialize internal
    wheel = Vector();
    velocity = 0.0;

    // initialize pins
    pinMode(dirA, OUTPUT);
    pinMode(dirB, OUTPUT);
    pinMode(speedA, OUTPUT);
    pinMode(speedB, OUTPUT);
}

FrindoCTL::~FrindoCTL()
{
    // deallocate sensors
    delete front;
    delete right;
    delete left;
}

// low-level API
void FrindoCTL::set_motor_r(int vel)
{
    digitalWrite (dirA, fwdA);
    analogWrite (speedA, vel);
}

void FrindoCTL::set_motor_l(int vel)
{
    digitalWrite (dirB, fwdB);
    analogWrite (speedB, vel);
}

// high-level API
void FrindoCTL::go(void)
{
    (wheel.getX() < 0) ? digitalWrite(dirA, revA) : digitalWrite(dirA, fwdA);
    (wheel.getY() < 0) ? digitalWrite(dirB, revB) : digitalWrite(dirB, fwdB);
    analogWrite (speedA, wheel.getY() * MAX_VOLTAGE);
    analogWrite (speedB, wheel.getX() * MAX_VOLTAGE);
}

void FrindoCTL::stop(void)
{
    digitalWrite (speedA, LOW);
    digitalWrite (speedB, LOW);

    velocity = 0.0f;
}

float FrindoCTL::readFront(void)
{
    return front->read();
}

float FrindoCTL::readLeft(void)
{
    return left->read();
}

float FrindoCTL::readRight(void)
{
    return right->read();
}

void FrindoCTL::setSpeed(const float s)
{
    velocity = clamp<float>(s, MIN_VELOCITY, MAX_VELOCITY);
}

void FrindoCTL::setAngle(const float theta)
{
    this->setPolar(Vector(velocity, theta));
}

void FrindoCTL::setPolar(const Vector& v)
{
    float x = velocity * cos(v.getY() * DEGREES);
    float y = velocity * sin(v.getY() * DEGREES);

    this->setSpeed(v.getX());
    this->setDirection(Vector(x, y));
}

void FrindoCTL::setDirection(const Vector& v)
{
    direction = Vector::normalize(v);

    // convert to wheel coordinate system
    float x = direction.getX();
    float y = direction.getY();
    float theta = -45.0 * DEGREES;

    float s = x * cos(theta) - y * sin(theta);
    float t = x * sin(theta) + y * cos(theta);

    this->setWheel(Vector(s,t));
}

void FrindoCTL::setWheel(const Vector& v)
{
    wheel = Vector::normalize(v) * velocity;
}


/*** ORIGINAL API ***/
/*
void FrindoCTL::forward(int dist, int vel)       // forward: both motors set to forward direction
{
  digitalWrite (dirA, fwdA);
  digitalWrite (dirB, fwdB);
  analogWrite (speedA, vel);                        // both motors set to same speed
  analogWrite (speedB, vel);
  delay (dist);                                     // wait for a while (dist in mSeconds)

  digitalWrite (speedA, LOW); 						// stop motors
  digitalWrite (speedB, LOW);
 }


void FrindoCTL::reverse(int dist, int vel)       // reverse: both motors set to reverse direction
{
  digitalWrite (dirA, revA);
  digitalWrite (dirB, revB);
  analogWrite (speedA, vel);                        // both motors set to same speed
  analogWrite (speedB, vel);
  delay (dist);  									// wait for a while (dist in mSeconds)

  digitalWrite (speedA, LOW); 						// stop motors
  digitalWrite (speedB, LOW);
}


void FrindoCTL::rot_cw (int angle, int vel)      // rotate clock-wise: right-hand motor reversed, left-hand motor forward
{
  digitalWrite (dirA, revA);
  digitalWrite (dirB, fwdB);
  analogWrite (speedA, vel);                        // both motors set to same speed
  analogWrite (speedB, vel);
  delay (angle);                                    // wait for a while (angle in mSeconds)

  digitalWrite (speedA, LOW); 						// stop motors
  digitalWrite (speedB, LOW);
}


void FrindoCTL::rot_ccw (int angle, int vel)     // rotate counter-clock-wise: right-hand motor forward, left-hand motor reversed
{
  digitalWrite (dirA, fwdA);
  digitalWrite (dirB, revB);
  analogWrite (speedA, vel);                        // both motors set to same speed
  analogWrite (speedB, vel);
  delay (angle);                                    // wait for a while (angle in mSeconds)

  digitalWrite (speedA, LOW); 						// stop motors
  digitalWrite (speedB, LOW);
}


void FrindoCTL::turn_right (int angle, int vel)  // turn right: right-hand motor stopped, left-hand motor forward
{
  digitalWrite (dirA, revA);
  digitalWrite (dirB, fwdB);
  digitalWrite (speedA, LOW);                       // right-hand motor speed set to zero
  analogWrite (speedB, vel);
  delay (angle);                                    // wait for a while (angle in mSeconds)

  digitalWrite (speedA, LOW); 						// stop motors
  digitalWrite (speedB, LOW);
}


void FrindoCTL::turn_left (int angle, int vel)   // turn left: left-hand motor stopped, right-hand motor forward
{
  digitalWrite (dirA, fwdA);
  digitalWrite (dirB, revB);
  analogWrite (speedA, vel);
  digitalWrite (speedB, LOW);                       // left-hand motor speed set to zero
  delay (angle);                                    // wait for a while (angle in mSeconds)

  digitalWrite (speedA, LOW); 						// stop motors
  digitalWrite (speedB, LOW);
}

void FrindoCTL::go_back(int vel)           // reverse: both motors set to reverse direction
{
  digitalWrite (dirA, revA);
  digitalWrite (dirB, revB);
  analogWrite (speedA, vel);   				// both motors set to same speed
  analogWrite (speedB, vel);
}

void FrindoCTL::go_cw (int vel)          // rotate clock-wise: right-hand motor reversed, left-hand motor forward
{
  digitalWrite (dirA, revA);
  digitalWrite (dirB, fwdB);
  analogWrite (speedA, vel);   				// both motors set to same speed
  analogWrite (speedB, vel);
}

void FrindoCTL::go_ccw (int vel)         // rotate counter-clock-wise: right-hand motor forward, left-hand motor reversed
{
  digitalWrite (dirA, fwdA);
  digitalWrite (dirB, revB);
  analogWrite (speedA, vel);   				// both motors set to same speed
  analogWrite (speedB, vel);
}
*/
