import serial
import time
import subprocess
import RPi.GPIO as GPIO

# GSM module serial port configuration
serial_port = "/dev/ttyS0"  # Adjust this to the correct port
baud_rate = 115200  # Baud rate for serial communication
target_number = "+5516991623916" 

# GPS serial port configuration
gps_port = "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00"  
gps_baudrate = 38400


fire_command_received = False

# Function to send AT commands and read the response
def send_command(ser, command, read_timeout=1):
    print(f"Sending command: {command}")
    ser.write((command + "\r").encode())
    time.sleep(1)
    response = ser.read_all().decode(errors='ignore')
    print(f"Response: {response}")
    return response.strip()

# Function to set up the GSM module
def setup_gsm(ser):
    print("Setting up GSM module...")
    send_command(ser, 'ATE0')  # Disable echo
    send_command(ser, 'AT+CMGF=1')  # Set SMS text mode
    send_command(ser, 'AT+CNMI=2,1,0,0,0')  # Configure real-time SMS notification

    print("Deleting all stored messages...")
    send_command(ser, "AT+CMGD=1,4")  # Delete all SMS

    send_command(ser, "AT+CSQ")  # Check signal quality

# Function to send SMS
def send_sms(ser, phone_number, message):
    print(f"Sending SMS to {phone_number}...")
    response = send_command(ser, f'AT+CMGS="{phone_number}"')
    if ">" in response:
        print("Ready to send the message.")
        ser.write((message + chr(26)).encode())  
        time.sleep(2)
        response = ser.read_all().decode(errors='ignore')
        print("Response after sending SMS:", response)

# Function to parse GPS NMEA sentence and extract latitude and longitude
def parse_nmea(sentence):
    try:
        parts = sentence.split(',')
        if parts[0] == '$GNGGA':
            raw_lat = parts[2]
            lat_dir = parts[3]
            raw_lon = parts[4]
            lon_dir = parts[5]

            if not raw_lat or not raw_lon:
                return None, None

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

# Function to read GPS data and return latitude and longitude
def get_gps_data():
    with serial.Serial(gps_port, baudrate=gps_baudrate, timeout=1) as ser:
        while True:
            line = ser.readline().decode('ascii', errors='ignore').strip()
            if line.startswith('$GNGGA'):
                lat, lon = parse_nmea(line)
                if lat and lon:
                    return lat, lon
                else:
                    print("No satellite fix.")
            time.sleep(1)

# Function to monitor incoming SMS and forward commands to the external script
def monitor_sms(ser, external_serial):
    global fire_command_received

    print("Monitoring SMS in real-time...")

    while True:
        response = send_command(ser, "AT+CMGL=\"REC UNREAD\"")  
        if "+CMGL" in response:
            lines = response.split("\r\n")
            for i in range(len(lines)):
                if "+CMGL" in lines[i]:
                    if i + 1 < len(lines):
                        message_content = lines[i + 1].strip()
                        print(f"Message content: {message_content}")

                        if message_content.upper() == "ARM":
                            external_serial.write(b"<ARM>\r")
                            send_sms(ser, target_number, "PARAZERO ARM")
                        elif message_content.upper() == "DISARM":
                            external_serial.write(b"<DISARM>\r")
                            send_sms(ser, target_number, "PARAZERO DISARM")
                        elif message_content.upper() == "FIRE":
                            fire_command_received = True
                            external_serial.write(b"<FIRE>\r")
                            send_sms(ser, target_number, "PARAZERO FIRE")
                            print("Fire command received. Sending GPS data...")
                            while fire_command_received:
                                lat, lon = get_gps_data()
                                if lat and lon:
                                    gps_message = f"www.google.com/maps?q={lat},{lon}"
                                    send_sms(ser, target_number, gps_message)
                                    print(gps_message)
                                time.sleep(2)  
                                # Check if the stop command has been received
                                response = send_command(ser, "AT+CMGL=\"REC UNREAD\"")
                                if "+CMGL" in response:
                                    lines = response.split("\r\n")
                                    for i in range(len(lines)):
                                        if "+CMGL" in lines[i]:
                                            if i + 1 < len(lines):
                                                message_content = lines[i + 1].strip()
                                                if message_content.upper() == "STOP":
                                                    fire_command_received = False
                                                    send_sms(ser, target_number, "PARAZERO STOP")
                                                    print("Stop command received. Stopping GPS data transmission.")
                                                    break
                        elif message_content.upper() == "PARE":
                            fire_command_received = False
                            send_sms(ser, target_number, "PARAZERO STOP")
                            print("Stop command received. Stopping GPS data transmission.")
        else:
            print("No new messages received.")
        time.sleep(2)


# Function to execute the sudo poff command
def execute_poff():
    print("Executing 'sudo poff'...")
    try:
        subprocess.run(["sudo", "poff"], check=True)
        print("'sudo poff' executed successfully.")
    except subprocess.CalledProcessError as e:
        print(f"Error executing 'sudo poff': {e}")

# Main function
def main():
    try:
        execute_poff()  # Ensure previous connections are terminated

        gsm_serial = serial.Serial(serial_port, baudrate=baud_rate, timeout=1)
        external_serial_port = "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0"  
        external_serial = serial.Serial(external_serial_port, baudrate=57600, timeout=1)

        response = send_command(gsm_serial, 'AT')
        print("Response to AT command:", response)

        setup_gsm(gsm_serial)  
        monitor_sms(gsm_serial, external_serial)  

    except serial.SerialException as e:
        print(f"Error accessing the serial port: {e}")

    except KeyboardInterrupt:
        print("\nProgram interrupted.")
    finally:
        if 'gsm_serial' in locals() and gsm_serial.is_open:
            gsm_serial.close()
        if 'external_serial' in locals() and external_serial.is_open:
            external_serial.close()
        GPIO.cleanup()  # Clean up GPIO pins

if __name__ == "__main__":
    main()
