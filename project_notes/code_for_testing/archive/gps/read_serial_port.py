# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/gps/read_serial_port.py
import serial
import time

# Replace '/dev/ttyUSB0' with your serial port if different
serial_port = '/dev/ttyUSB0'
baud_rate = 115200  # In baud rate, adjust as necessary

def read_gps():
    try:
        with serial.Serial(serial_port, baud_rate, timeout=1) as ser:
            while True:
                line = ser.readline().decode('ascii', errors='replace').strip()
                if line:
                    print(line)
                time.sleep(0.1)  # Adjust this if you need
    except serial.SerialException as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    read_gps()
