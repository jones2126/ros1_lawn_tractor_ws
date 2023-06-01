import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
ser.flush()

voltage = 0.0
linear_x = 0.0
angular_z = 0.0
previous_time_send = time.time()
previous_time_receive = time.time()
previous_time_display = time.time()
voltage_count = 0

while True:
    current_time = time.time()

    # Receiving voltage data at 5Hz
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').rstrip()
        voltage = float(line)
        #print(f"Voltage: {voltage}")
        voltage_count += 1

    # Sending linear_x and angular_z data at 10Hz
    if (current_time - previous_time_send) >= 0.1:
        previous_time_send = current_time
        ser.write(f"{linear_x},{angular_z}\n".encode('utf-8'))

    # For testing, we can simulate linear_x and angular_z changes
    if (current_time - previous_time_receive) >= 1.0:
        previous_time_receive = current_time
        linear_x += 0.1
        angular_z += 0.2  # Adjust the increment to suit your needs

    # Displaying the voltage count every 10 seconds
    if (current_time - previous_time_display) >= 10.0:
        previous_time_display = current_time
        print(f"Number of voltage messages received: {voltage_count}, incoming Hz: {(voltage_count/10)}")
        voltage_count = 0  # reset counter        
