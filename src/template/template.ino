#include <Frindo.h>

#define SENSOR_FRONT_ID 1
#define SENSOR_LEFT_ID  2
#define SENSOR_RIGHT_ID 3

unsigned long time;
unsigned long dt; // delta-time

Frindo robot;

void setup() {
    Serial.begin(9600);
}
 
void serialEvent() {
    interpret();
}

void loop() {
    // handle precise timing here
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
            Serial.println("Unknown command");
            break;
    }
    
    return true;
}
