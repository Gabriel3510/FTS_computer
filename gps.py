import serial
import time

def parse_nmea(sentence):
    try:
        parts = sentence.split(',')
        if parts[0] == '$GNGGA':  # GGA sentence contains position data
            raw_lat = parts[2]
            lat_dir = parts[3]
            raw_lon = parts[4]
            lon_dir = parts[5]

            if not raw_lat or not raw_lon:
                return None, None  # Invalid data

            latitude = float(raw_lat[:2]) + float(raw_lat[2:]) / 60.0
            if lat_dir == 'S':
                latitude = -latitude

            longitude = float(raw_lon[:3]) + float(raw_lon[3:]) / 60.0
            if lon_dir == 'W':
                longitude = -longitude

            return latitude, longitude
    except (IndexError, ValueError):
        pass
    return None, None


def read_gps(port, baudrate=38400):
    try:
        with serial.Serial(port, baudrate=baudrate, timeout=1) as ser:
            print(f"Reading GPS data from port {port} with baudrate {baudrate}...")
            while True:
                line = ser.readline().decode('ascii', errors='ignore').strip()
                if line.startswith('$GNGGA'):
                    lat, lon = parse_nmea(line)
                    if lat and lon:
                        print(f"Latitude: {lat}, Longitude: {lon}")
                    else:
                        print("No satellite fix.")
    except serial.SerialException as e:
        print(f"Serial port error: {e}")
    except KeyboardInterrupt:
        print("Reading interrupted by user.")


if __name__ == "__main__":
    gps_port = "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00"
    read_gps(gps_port, baudrate=38400)
