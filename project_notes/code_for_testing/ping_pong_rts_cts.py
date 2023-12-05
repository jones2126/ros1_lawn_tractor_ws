import serial
import time

import time

def send_with_handshake(ser, message):
    print("in handshake")
    max_retries = 3
    retries = 0

    while retries < max_retries:
        print("RTS")
        #ser.reset_input_buffer()
        start_time = time.time()
        cts_timeout = 2  # 2 seconds
        cts_received = False

        while not cts_received and (time.time() - start_time < cts_timeout):
            if ser.in_waiting > 0:
                data = ser.readline().decode().strip()
                '''
                print(f"data trimmed ({data})")
                print("hex (", end="")

                # Print each character in the string in hexadecimal format
                for c in data:
                    print(f"{ord(c):X}:", end="")
                print(")")
                '''
                if data == "CTS":
                    # Handle the message here
                    print(message)
                    cts_received = True
            else:
                print("RTS")
            time.sleep(0.1)  # delay of 100ms

        if cts_received:
            print("cts received if statement")
            #ser.reset_input_buffer()
            return
        else:
            print("retrying")
            retries += 1

    print("timeout error")


def tx_rx(ser):
    # Check if there's data to read
    if ser.in_waiting > 0:
        # Read incoming message
        incoming_msg = ser.readline().decode().strip()
        print("read from ESP32 (" + incoming_msg + ")")
        # If incoming message is "RTS", send "CTS"
        if incoming_msg == "RTS":
            ser.reset_input_buffer()
            ser.write("CTS".encode())
            print("Received: RTS, Sent: CTS")
        elif incoming_msg == "ping":
            send_with_handshake(ser, "pong")
            print("Received: ping, Sent: pong")
    else:
        print("no data avail from ESP32")

def main():
    # Open the serial port
    ser = serial.Serial('/dev/ttyACM0', 115200)  # Adjust the port and baud rate as per your setup

    while True:
        tx_rx(ser)
        time.sleep(0.5)  # 2 Hz / 1/2 second delay

# Call the main function
if __name__ == "__main__":
    main()

