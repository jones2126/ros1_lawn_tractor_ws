import serial

# Open the serial port
ser = serial.Serial('/dev/ttgo_main', 57600)  # Adjust the port and baud rate as per your setup

# Send a message to the serial port
message = "Hello, serial port!"
ser.write(message.encode())

# Close the serial port
ser.close()
