import serial
from serial.serialutil import SerialException

def main():
    # Define the maximum number of attempts
    max_attempts = 3
    attempts = 0

    while attempts < max_attempts:
        try:
            # Open the serial port
            ser = serial.Serial('/dev/ttyACM0', 115200)  # Adjust the port and baud rate as per your setup

            # Read and print data from the serial port
            while True:
                if ser.in_waiting > 0:
                    data = ser.readline().decode('utf-8').strip()
                    print("Received:", data)

        except SerialException as e:
            print(f"SerialException: {e}. Attempt {attempts+1} of {max_attempts}.")
            attempts += 1
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            break  # If an unexpected error occurred, break the loop
        finally:
            # Safely close the serial port
            if 'ser' in locals() and isinstance(ser, serial.Serial) and ser.is_open:
                ser.close()
                print("Serial port closed.")
        if attempts == max_attempts:
            print("Failed to read from serial port after maximum attempts.")
            
if __name__ == "__main__":
    main()
