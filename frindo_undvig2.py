import time
from time import sleep

import serial
port = '/dev/ttyACM0'

serialRead = serial.Serial(port,9600, timeout=1)

# Wait if serial port is not open yet
while not serialRead.isOpen():
    sleep(1)

print("Waiting for serial port connection ...")
sleep(2)

print("Running ...")

while (True):
    # foward
    msg='p(1.0,0.0)\n'
    serialRead.write(msg.encode('ascii'))
    
    # request to read analog sensor 0 (Front IR sensor)
    msg='r1\n'
    serialRead.write(msg.encode('ascii'))
    front_val=serialRead.readline()
    print "Front sensor = ", front_val
    if(float(front_val) > 200.0):
        msg='s\n'
        serialRead.write(msg.encode('ascii'))
        sleep(1)
        print("obstical ahead")
        
        
        msg='r3\n'
        serialRead.write(msg.encode('ascii'))
        right_val = serialRead.readline()
        print "Right sensor = ", right_val
        
        msg='r2\n'
        serialRead.write(msg.encode('ascii'))
        left_val = serialRead.readline()
        
        print "Left sensor = ", left_val
        if (float(right_val) > float(left_val)):
            msg='p(1.0, -90.0)\n'
            
        else:
            msg='p(1.0, 90.0)\n'
        while(float(front_val) > 200.0):
            serialRead.write(msg.encode('ascii'))
            sleep(0.05)
            msg='r1\n'
            serialRead.write(msg.encode('ascii'))
            front_val = serialRead.readline()
        msg='s\n'
        serialRead.write(msg.encode('ascii'))
        sleep(1)
msg='s\n'
serialRead.write(msg.encode('ascii'))
str_val=serialRead.readline()
print str_val
