#ifndef FrindoCTL_h
#define FrindoCTL_h

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "Vector.h"

typedef struct {
    int front, left, right;
    int reserved_0, reserved_1, reserved_2;
} Sensor;

class FrindoCTL
{
    public:
    FrindoCTL();

    // low-level API
    void set_motor_l(int vel);
    void set_motor_r(int vel);

    // high-level API
    void go(void);
    void stop(void);
    void read(void);

    // getters
    Sensor getSensors(void);

    // setters
    void setSpeed(const float s);
    void setAngle(const float theta);
    void setPolar(const Vector& v);
    void setDirection(const Vector& v);

    void setWheel(const Vector& v);

    private:
    float velocity;
    Vector direction;
    Vector wheel;

    Sensors sensors;

    /* obsolete
        //void stop();
        void forward(int dist, int vel);
        void reverse(int dist, int vel);
        void rot_cw(int dist, int vel);
        void rot_ccw(int dist, int vel);
        void turn_right(int dist, int vel);
        void turn_left(int dist, int vel);

        // new in Version 1.1 2 December 2011
        //void go(int vel);
        void go_back(int vel);
        void go_cw(int vel);
        void go_ccw(int vel);
    */
};

#endif
