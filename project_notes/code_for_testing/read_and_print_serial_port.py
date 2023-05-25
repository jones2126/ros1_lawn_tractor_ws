import serial

# Open the serial port
ser = serial.Serial('/dev/ttyACM0', 115200)  # Adjust the port and baud rate as per your setup

# Discard initial read
#ser.readline()

# Read and print data from the serial port
while True:
    if ser.in_waiting > 0:
        data = ser.readline().decode('utf-8').strip()
        print("Received:", data)

