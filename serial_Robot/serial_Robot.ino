//  serial_Robot     version 1.0      J.Luke     30/08/2013
//
//  This sketch converts an Arduino with a RobotShield into a serial robot controller.
//
//  The sketch provides basic motion control as well as access to Arduinos analog inputs over the
//  serial or USB port for robotics applications. The simple commands mean the controller can be
//  interfaced to anything with a serial or USB port including a PC, Raspberry Pi, microcontroller 
//  or another Arduino.
//
//  This sketch uses the RobotShield library which can be downloaded from www.frindo.com and provides
//  a mix of stepped commands, continuous commands and analog measurements. The movement of your robot 
//  is controlled by sending a single character command over the serial port.
//  
//  Stepped commands make your robot move in a specific way for a fixed period of time. The length of time
//  is set using the "step_time" and "turn_time" variables. All times are in mS (milliseconds).
//
//    f = Forward, b = Backwards, l = rotate Left, r = rotate Right 
//
//  Continuous commands make you robot move in the same way continuously until you send a stop or other command.  
//
//    s = Stop, g = Go, v = Reverse, n = rotate Left, m = rotate Right 
//
//  The speed of the motors during all movements is set by the "motor_speed" and "turn_speed". Motor speeds 
//  range from 0 to 255; where 0 = 0% of full speed and 255 = 100% of full speed. 
//
//  Analog measurements are requested by sending a single numeric character over the serial port, the character 
//  from 0 to 5 represents the Arduino port number. The Arduino makes an analog measurement (0 to 5 volts) and 
//  returns a digital value (0 to 1023) where 0 = 0 volts and 1023 = 5 volts.
//
//    0 = A0, 1 = A1, 2 = A2, 3 = A3, 4 = A4, 5 = A5
//
//  For support or suggestions, please use the forums at www.frindo.com
//
//  This code is open source so share, change, improve and contribute to the robot building community!


#include <Frindo.h> // include the RobotShield library

Frindo rs;          // create an instance of the RobotShield class called "rs"

// define variables used

void setup() {       
              
    Serial.begin(9600);             // set up serial port
}     
            
void loop() {
   
    if (Serial.available() > 0)            // if something has been received 
      {
        int incoming = Serial.read();
        
        if ((char)incoming == 'p')
        {
          float velocity = Serial.parseFloat();
          float angle = Serial.parseFloat();
          Vector v(velocity, angle);
          rs.setPolar(v);
        }

        else if ((char)incoming == 'v')
        {
          float velocity = Serial.parseFloat();
          float angle = Serial.parseFloat();
          Vector v(velocity, angle);
          rs.setPolar(v);
        }

        else if ((char)incoming == 'r')
        {
          int sensor = Serial.parseInt();  
          if (sensor == 1)
          {
            Serial.println(rs.readFront());
          }
          else if (sensor == 2)
          {
            Serial.println(rs.readLeft());
          }
          else if (sensor == 3)
          {
            Serial.println(rs.readRight());
          }
        }
        
        else if ((char)incoming == 's')
        {
          rs.stop();
        }
        
        else if ((char)incoming == 'd')
        {
          float x = Serial.parseFloat();
          float y = Serial.parseFloat();
          Vector v(x,y);
          rs.setDirection(v);
        }
        
        else if ((char)incoming == 'a')
        {
          float angle = Serial.parseFloat();
          rs.setAngle(angle);
        }
    }        
} 
