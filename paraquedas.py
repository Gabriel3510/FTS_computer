import serial
import time
import logging


logging.basicConfig(level=logging.DEBUG, format="%(asctime)s %(levelname)s %(message)s")
logger = logging.getLogger()

# Defina a porta serial USB e o baud rate
SERIAL_PORT = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'  
BAUD_RATE = 115200  

def listen_to_serial():
    try:
        serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Conectado à porta serial {SERIAL_PORT} com baud rate {BAUD_RATE}")
        
        while True:
            if serial_conn.in_waiting > 0:
                data = serial_conn.readline().decode('utf-8').strip()  
                if data:
                    logger.debug(f"{data}")  
            time.sleep(0.1)  

    except serial.SerialException as e:
        print(f"Erro ao acessar a porta serial {SERIAL_PORT}: {str(e)}")
        logger.error(f"Erro ao acessar a porta serial {SERIAL_PORT}: {str(e)}")

    except KeyboardInterrupt:
        print("\nPrograma interrompido pelo usuário.")
        logger.info("Programa interrompido pelo usuário.")
    
if __name__ == "__main__":
    listen_to_serial()
