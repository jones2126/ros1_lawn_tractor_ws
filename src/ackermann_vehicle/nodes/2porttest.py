import serial
import threading
import time

# Add a global variable to signal threads to stop
running = True

def read_ttgo_main():
    while running:
        if ser1.in_waiting > 0:
            line = ser1.readline()
            print("Read from /dev/ttgo_main:", line.decode('utf-8').strip())
        else:
            time.sleep(0.1)

def write_USB2TTL():
    while running:
        velocity_str = '1,' + ','.join([
            str(1.99),
            str(2.98),
            str(3.3),
            str(4.4),
            str(7),
            str(1)
        ])
        ser2.write((velocity_str + '\n').encode())  
        print("Written to /dev/USB2TTL:", velocity_str)
        time.sleep(1)

try:
    ser1 = serial.Serial('/dev/ttgo_main', 115200)
    ser2 = serial.Serial('/dev/USB2TTL', 115200)

    threading.Thread(target=read_ttgo_main).start()
    threading.Thread(target=write_USB2TTL).start()

    while running:
        time.sleep(0.1) 

except KeyboardInterrupt:
    print("Exiting program due to Ctrl+C...")
    running = False  # signal threads to stop
finally:
    time.sleep(1)  # give threads time to notice the stop signal
    ser1.close()
    ser2.close()
