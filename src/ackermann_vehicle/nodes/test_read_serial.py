#!/usr/bin/env python3
import serial
import time

# Open the serial port
ser = serial.Serial('/dev/ttgo_main', 115200)

try:
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            print("from Tractor: ", line)
        else:
            # Sleep for a short period to prevent this loop from running too fast
            time.sleep(0.01)
except KeyboardInterrupt:
    # User has pressed Ctrl+C, exit the program
    print("Exiting program...")
except Exception as e:
    # Some other error occurred, print it out and exit
    print("An error occurred: ", str(e))
finally:
    # Close the serial port
    ser.close()
   