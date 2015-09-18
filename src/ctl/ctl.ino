//
// REX 2015, Arduino Controller
//

#include <Frindo.h>

Frindo rs;

// define variables used
int Response;
int motor_speed = 128;
int turn_speed = 128;
int step_time = 1000; //mS
int turn_time = 250; //mS

void setup() {
    Serial.begin(9600);
}

void loop() {
    if (Serial.available() > 0)
    {
        Serial.println(rs.readFront());
        Serial.println(rs.readLeft());
        Serial.println(rs.readRight());

        int incoming = Serial.read();

        if ((char)incoming == 'v') {
            rs.setPolar(1.0, 0.0);
            Serial.println("Branch v");
            Serial.println((char)incoming);
        }
    }
} 
