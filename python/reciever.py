import serial
import struct
import time

# Configuración del puerto - AJUSTA EL COM SEGÚN TU PC
SERIAL_PORT = 'COM11'  # En Linux/Mac suele ser '/dev/ttyACM0'
BAUD_RATE = 115200    # Asegúrate de que coincida con tu .ioc

def run_telemetry():
    try:
        # Abrimos el puerto serial
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Conectado a {SERIAL_PORT} a {BAUD_RATE} baudios.")
        
        # El paquete esperado es: 
        # [0xAA, 0xBB] (2 bytes) + [FLOAT] (4 bytes) + [0xFF] (1 byte) = 7 bytes
        packet_size = 7
        
        while True:
            # 1. Buscamos el primer byte del Header (0xAA)
            if ser.read(1) == b'\xaa':
                # 2. Verificamos el segundo byte (0xBB)
                if ser.read(1) == b'\xbb':
                    # 3. Leemos los 4 bytes del float + el byte de footer
                    payload = ser.read(packet_size - 2)
                    
                    if len(payload) == 5: # 4 bytes de data + 1 de footer
                        data_bytes = payload[:4]
                        footer = payload[4]
                        
                        if footer == 0xff:
                            # 4. Convertimos los 4 bytes a float (formato Little Endian '<f')
                            # Usamos struct.unpack para interpretar los bytes binarios
                            y_out = struct.unpack('<f', data_bytes)[0]
                            
                            print(f"Filtro Out: {y_out:.4f}")
                        else:
                            print("Error: Footer incorrecto, paquete corrupto.")
                            
    except serial.SerialException as e:
        print(f"Error de conexión: {e}")
    except KeyboardInterrupt:
        print("\nPrograma detenido por el usuario.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Puerto cerrado.")

if __name__ == "__main__":
    run_telemetry()