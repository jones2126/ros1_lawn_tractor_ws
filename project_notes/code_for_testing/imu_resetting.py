#!/usr/bin/env python3
import serial

# Open serial port at 9600 baud rate
ser = serial.Serial('/dev/rosimu', 115200)  

# Reset sensor heading direction
reset_cmd = '$MIB,RESET*87\n'.encode()
ser.write(reset_cmd)

# Close serial port
ser.close()

'''
This code uses the serial module to open a serial connection with the 
IMU sensor at 9600 baud rate. Then, it sends the reset command to the 
sensor to reset the heading direction to 0. Finally, it closes the serial connection.
ref: https://web.archive.org/web/20220123053002/https://docs.chiprobotics.com/sensors/imu-sensor/api

'''