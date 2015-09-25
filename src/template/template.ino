#include <Frindo.h>

#define SENSOR_FRONT_ID 1
#define SENSOR_LEFT_ID  2
#define SENSOR_RIGHT_ID 3

const unsigned long dt = 16; // delta-time, in microseconds

Frindo robot;

void setup() {
    Serial.begin(9600);
}

void serialEvent() {
    static unsigned long event_time = 0;
    static unsigned long event_delta = 0;
    
    event_time = micros();  // when did we get the event?
    
    interpret();
    
    event_delta = event_time - micros(); // how long have we processed it?
}

unsigned long time = 0;
unsigned long last = 0;

void loop() {

    time = micros();
    if ((last - time) < 5*1000*1000*100)
        return;

    last = time;
}

bool interpret() {
    
    if (!Serial.available())
        return false;
    
    char cmd = (char)Serial.read();
    
    switch (cmd)
    {
        case 's':
            robot.stop();
            break;
        
        case 'v':
            robot.setVelocity(Serial.parseFloat());
            break;
        
        case 'a':
            robot.setAngle(Serial.parseFloat());
            break;
        
        case 'p':
            robot.setPolar(Serial.parseFloat(), Serial.parseFloat());
            break;

        case 'r':
            switch (Serial.parseInt())
            {
                case SENSOR_FRONT_ID:
                    Serial.println(robot.readFront());
                    break;
                
                case SENSOR_LEFT_ID:
                    Serial.println(robot.readLeft());
                    break;
                
                case SENSOR_RIGHT_ID:
                    Serial.println(robot.readRight());
                    break;
                
                default:
                    Serial.println("No such sensor id.");
                    break;
            }
            break;
        
        default:
            break;
    }
    
    return true;
}
