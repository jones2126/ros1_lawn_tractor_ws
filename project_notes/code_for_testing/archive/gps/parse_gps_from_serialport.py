# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/gps/parse_gps_from_serialport.py
import serial
import time

class GPGGAData:
    def __init__(self, sentence):
        parts = sentence.split(',')
        self.timestamp = parts[1]
        self.latitude = self.parse_latitude(parts[2], parts[3])
        self.longitude = self.parse_longitude(parts[4], parts[5])
        self.fix_quality = parts[6]
        self.num_satellites = parts[7]
        self.horizontal_dilution = parts[8]
        self.altitude = parts[9]

    def parse_latitude(self, lat, ns_indicator):
        if not lat:
            return None
        lat_deg = float(lat[:2]) + float(lat[2:]) / 60
        return lat_deg if ns_indicator == 'N' else -lat_deg

    def parse_longitude(self, lon, ew_indicator):
        if not lon:
            return None
        lon_deg = float(lon[:3]) + float(lon[3:]) / 60
        return lon_deg if ew_indicator == 'E' else -lon_deg

    def fix_quality_description(self):
        descriptions = {
            "0": "Invalid",
            "1": "GPS fix (SPS)",
            "2": "DGPS fix",
            "3": "PPS fix",
            "4": "Real Time Kinematic",
            "5": "Float RTK",
            "6": "Estimated (dead reckoning)",
            "7": "Manual input mode",
            "8": "Simulation mode"
        }
        return descriptions.get(self.fix_quality, "Unknown")

    def __str__(self):
        return (f"Time: {self.timestamp}, Latitude: {self.latitude}, Longitude: {self.longitude}, "
                f"Fix Quality: {self.fix_quality_description()}, Number of Satellites: {self.num_satellites}, "
                f"HDOP: {self.horizontal_dilution}, Altitude: {self.altitude}m")


class GNRMCData:
    def __init__(self, sentence):
        parts = sentence.split(',')
        self.timestamp = parts[1]
        self.status = parts[2]  # A=Active, V=Void
        self.latitude = self.parse_latitude(parts[3], parts[4])
        self.longitude = self.parse_longitude(parts[5], parts[6])
        self.speed_over_ground = parts[7]
        self.track_angle = parts[8]
        self.date = parts[9]
        self.magnetic_variation = parts[10]

    def parse_latitude(self, lat, ns_indicator):
        if not lat:
            return None
        lat_deg = float(lat[:2]) + float(lat[2:]) / 60
        return lat_deg if ns_indicator == 'N' else -lat_deg

    def parse_longitude(self, lon, ew_indicator):
        if not lon:
            return None
        lon_deg = float(lon[:3]) + float(lon[3:]) / 60
        return lon_deg if ew_indicator == 'E' else -lon_deg

    def __str__(self):
        return f"Time: {self.timestamp}, Status: {self.status}, Latitude: {self.latitude}, Longitude: {self.longitude}, Speed: {self.speed_over_ground}, Track angle: {self.track_angle}, Date: {self.date}, Magnetic Variation: {self.magnetic_variation}"


def read_gps():
    try:
        with serial.Serial(serial_port, baud_rate, timeout=1) as ser:
            while True:
                line = ser.readline().decode('ascii', errors='replace').strip()
                if line.startswith('$GPGGA'):
                    gpgga_data = GPGGAData(line)
                    print(gpgga_data)
                elif line.startswith('$GNRMC'):
                    # gnrmc_data = GNRMCData(line)
                    # print(gnrmc_data)
                    pass  # Implement similar to GPGGA
                time.sleep(0.05)
    except serial.SerialException as e:
        print(f"Error: {e}")

serial_port = '/dev/ttyUSB0'
baud_rate = 115200  # Adjust as necessary

if __name__ == '__main__':
    read_gps()
