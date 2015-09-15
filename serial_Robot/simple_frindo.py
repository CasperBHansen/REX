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

# send a go command
msg='g\n'
serialRead.write(msg.encode('ascii'))
str_val=serialRead.readline()
print str_val

# Wait a bit while robot moves forward
sleep(1)

# request to read analog sensor 0 (Front IR sensor)
msg='0\n'
serialRead.write(msg.encode('ascii'))
str_val=serialRead.readline()
print "Front sensor = ", str_val

# request to read analog sensor 1 (Right IR sensor)
msg='1\n'
serialRead.write(msg.encode('ascii'))
str_val=serialRead.readline()
print "Right sensor = ", str_val

# request to read analog sensor 2 (Left IR sensor)
msg='2\n'
serialRead.write(msg.encode('ascii'))
str_val=serialRead.readline()
print "Left sensor = ", str_val

        
# send a stop command
msg='s\n'
serialRead.write(msg.encode('ascii'))
str_val=serialRead.readline()
print str_val



print("Finished")
