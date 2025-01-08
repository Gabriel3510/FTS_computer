import serial
import time
import subprocess
from queue import Queue
import threading
import socket

feedback_queue = Queue()

# GSM Settings
serial_port = "/dev/ttyS0"  
baud_rate = 115200  

# Test phone number
target_number = "+5516991623916"

# GPS Settings
gps_port = "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00"
gps_baudrate = 38400

fire_command_received = False

# Communication with nvidia
usb_serial_port = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"

# Socket
HOST = '127.0.0.1'  
PORT = 65432        

# -------------------------
# Functions for ParaZero
# -------------------------

def start_parazero_process():
    process = subprocess.Popen(
        ["python", "parazero.py", "usb"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    print("Started parazero.py process.")
    return process

def send_to_parazero(command):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((HOST, PORT))
            s.sendall(command.encode())
            feedback = s.recv(1024).decode()
            print(f"ParaZero feedback: {feedback}")
            return feedback
    except Exception as e:
        print(f"Failed to communicate with ParaZero: {e}")
        return None

def parse_parazero_data(data):
    try:
        parsed_data = data.split(" ") 
        return parsed_data
    except Exception as e:
        return None

def monitor_parazero(serial_port, gsm_serial, usb_serial_port):
    current_state = None
    
    try:
        usb_serial = serial.Serial(usb_serial_port, baudrate=115200, timeout=1)
        
        while True:
            try:
                # Read data from ParaZero
                data = serial_port.readline().decode('utf-8', errors='ignore').strip()
                if data:
                    parsed_data = parse_parazero_data(data)
                    if parsed_data:
                        print(f"Parsed data: {parsed_data}")

                        if usb_serial.isOpen():
                            usb_serial.write(f"Parsed data: {parsed_data}\n".encode('utf-8'))
                        else:
                            print("USB serial not open.")
                        
                        new_state = parsed_data[0]  
                        if new_state != current_state:
                            current_state = new_state
                            if new_state == "<1,0,0,0,72>":
                                send_sms(gsm_serial, target_number, "ParaZero DISARM")
                            elif new_state == "<1,1,0,0,71>":
                                send_sms(gsm_serial, target_number, "ParaZero ARM")
                            elif "MOTOR_CUT" in data:
                                send_sms(gsm_serial, target_number, "ParaZero FIRE")
                            else:
                                send_sms(gsm_serial, target_number, f"ParaZero UNKNOWN STATE: {new_state}")
                        
                        feedback_queue.put(parsed_data)
                    else:
                        print("Invalid data received.")
                else:
                    print("No data received.")
            except serial.SerialException as e:
                print(f"Serial error: {e}")
                break
            time.sleep(1) 
    except Exception as e:
        print(f"Error in monitor_parazero: {e}")
        time.sleep(0.5)


# -------------------------
# Functions for GSM/SMS
# -------------------------

def send_command(ser, command, read_timeout=1):
    ser.write((command + "\r").encode())
    time.sleep(1)
    response = ser.read_all().decode(errors='ignore').strip()
    if not response:
        print(f"No response for command: {command}")
    return response.strip()

def setup_gsm(ser):
    print("Setting up GSM module...")
    send_command(ser, 'AT')  
    send_command(ser, 'ATE0')  # Disable echo
    send_command(ser, 'AT+CMGF=1')  # Set SMS text mode
    send_command(ser, 'AT+CNMI=2,1,0,0,0')  # Configure real-time SMS notification
    print("Deleting all stored messages...")
    send_command(ser, "AT+CMGD=1,4")  # Delete all SMS
    send_command(ser, "AT+CSQ")  # Check signal quality

def send_sms(ser, phone_number, message):
    response = send_command(ser, f'AT+CMGS="{phone_number}"')
    if ">" in response:
        ser.write((message + chr(26)).encode())
        time.sleep(2)

def monitor_sms(ser, external_serial, serial_port, gsm_serial):
    global fire_command_received

    while True:
        print("Checking for unread SMS...")
        response = send_command(ser, "AT+CMGL=\"REC UNREAD\"")
        print(f"Response from AT+CMGL: {response}")

        if "+CMGL" in response:
            lines = response.split("\r\n")
            for i in range(len(lines)):
                if "+CMGL" in lines[i]:
                    if i + 1 < len(lines):
                        message_content = lines[i + 1].strip()

                        if message_content.upper() == "ARM":
                            external_serial.write(b"<ARM>\r")
                            time.sleep(2)
                            monitor_parazero(serial_port, gsm_serial, usb_serial_port)

                        elif message_content.upper() == "DISARM":
                            external_serial.write(b"<DISARM>\r")
                            time.sleep(2)
                            monitor_parazero(serial_port, gsm_serial, usb_serial_port)

                        elif message_content.upper() == "FIRE":
                            fire_command_received = True
                            external_serial.write(b"<FIRE>\r")
                    
                            time.sleep(2)
                            monitor_parazero(serial_port, gsm_serial, usb_serial_port)
                            while fire_command_received:
                                lat, lon = get_gps_data()
                                if lat and lon:
                                    gps_message = f"www.google.com/maps?q={lat},{lon}"
                                    send_sms(ser, target_number, gps_message)
                                time.sleep(2)

                            

                        elif message_content.upper() == "PARE":
                            fire_command_received = False
                            send_sms(ser, target_number, "PARAZERO STOP")
        else:
            print("No unread messages.")

        time.sleep(2)

# -------------------------
# GPS functions
# -------------------------

def parse_nmea(sentence):
    try:
        parts = sentence.split(',')
        if parts[0] == '$GNGGA':
            raw_lat = parts[2]
            lat_dir = parts[3]
            raw_lon = parts[4]
            lon_dir = parts[5]

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

# -------------------------
# Auxiliary functions
# -------------------------

def execute_poff():
    print("Executing 'sudo poff'...")
    try:
        subprocess.run(["sudo", "poff"], check=True)
        print("'sudo poff' executed successfully.")
    except subprocess.CalledProcessError as e:
        print(f"Error executing 'sudo poff': {e}")

def start_socket_server():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((HOST, PORT))
            s.listen()
            print(f"Socket server listening on {HOST}:{PORT}")

            while True:
                conn, addr = s.accept()
                with conn:
                    print(f"Connected by {addr}")
                    while True:
                        data = conn.recv(1024)
                        if not data:
                            break
                        print(f"Received data: {data}")
                        conn.sendall(data)
    except Exception as e:
        print(f"Error occurred while processing data: {e}")


def main():
    try:
        execute_poff()

        process = start_parazero_process()

        external_serial_port = "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0"
        external_serial = serial.Serial(external_serial_port, baudrate=57600, timeout=1)
        print(f"External serial {external_serial_port} opened successfully.")

        gsm_serial = serial.Serial(serial_port, baudrate=baud_rate, timeout=1)
        print(f"GSM serial {serial_port} opened successfully.")

        setup_gsm(gsm_serial)
        print("GSM module setup completed successfully.")

        sms_thread = threading.Thread(target=monitor_sms, args=(gsm_serial, external_serial, serial_port, gsm_serial), daemon=True)
        sms_thread.start()
        print("SMS monitoring thread started.")

        socket_thread = threading.Thread(target=start_socket_server, daemon=True)
        socket_thread.start()
        print("Socket server thread started.")

        parazero_thread = threading.Thread(target=monitor_parazero, args=(external_serial, gsm_serial, usb_serial_port ), daemon=True)
        parazero_thread.start()
        print("Parazero monitoring thread started.")

        while True:
            time.sleep(1)

    except Exception as e:
        print(f"Error occurred: {e}")
        if process:
            process.terminate()
            print("Parazero process terminated.")

if __name__ == "__main__":
    main()