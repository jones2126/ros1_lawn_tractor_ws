import serial
import time

# Open the serial port
#ser = serial.Serial('/dev/ttgo_main', 57600)  # Adjust the port and baud rate as per your setup /dev/ttyACM0
ser = serial.Serial('/dev/ttyACM0', 57600)  # Adjust the port and baud rate as per your setup /dev/ttyACM0

# Send a message to the serial port once per second
while True:
    message = "Hello, serial port!"
    ser.write(message.encode())
    print("Sent:", message)
    time.sleep(1)

# Close the serial port
ser.close()