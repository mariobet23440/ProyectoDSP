import serial
import struct
import time

# Constantes del protocolo (ASCII)
CMD_START      = b'a'
CMD_VECTOR_SEL = b'b'
CMD_INDEX      = b'c'
CMD_DATA_0     = b'd'
CMD_DATA_1     = b'e'
CMD_DATA_2     = b'f'
CMD_DATA_3     = b'g'
CMD_STOP       = b'h'
CMD_INSPECT    = b'i' # Comando de reporte

VECTOR_A = 0 
VECTOR_B = 1 

class FilterController:
    def __init__(self, port, baud=115200):
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1) # Timeout bajo para no trabar
            print(f"Conectado a {port}. Esperando inicialización...")
            time.sleep(2) 
        except Exception as e:
            print(f"Error al abrir puerto: {e}")

    def _send_packet(self, data_byte, cmd_char):
        packet = bytes([data_byte]) + cmd_char
        self.ser.write(packet)
        # Esperar el ACK del MCU
        ack = self.ser.read(1)
        if ack != cmd_char:
            # Si falla, intentamos leer una vez más por si hubo lag
            ack = self.ser.read(1)
            if ack != cmd_char:
                print(f"ERROR: Fallo de sincronía en {cmd_char}. Recibido: {ack}")

    def update_coefficient(self, vector_type, index, value):
        print(f"--- Actualizando {'A' if vector_type==0 else 'B'}[{index}] -> {value} ---")
        
        # 0. LIMPIEZA CRÍTICA: Vaciar telemetría antes de empezar
        self.ser.reset_input_buffer()
        time.sleep(0.01)

        # 1. Iniciar secuencia
        self._send_packet(0, CMD_START)
        self._send_packet(vector_type, CMD_VECTOR_SEL)
        self._send_packet(index, CMD_INDEX)
        
        # 2. Float bytes
        float_bytes = struct.pack('<f', value)
        self._send_packet(float_bytes[0], CMD_DATA_0)
        self._send_packet(float_bytes[1], CMD_DATA_1)
        self._send_packet(float_bytes[2], CMD_DATA_2)
        self._send_packet(float_bytes[3], CMD_DATA_3)
        
        # 3. Finalizar
        self._send_packet(0, CMD_STOP)
        print("Envío completado.\n")

    def request_coefs_report(self):
        print("Solicitando reporte de coeficientes...")
        self.ser.reset_input_buffer()
        
        # Enviar comando de inspección
        self.ser.write(bytes([0]) + CMD_INSPECT)
        time.sleep(0.4) 
        
        if self.ser.in_waiting > 0:
            # LEER los datos del buffer
            raw_bytes = self.ser.read_all()
            raw_data = raw_bytes.decode('ascii', errors='ignore')
            
            start_idx = raw_data.find("--- Coeficientes")
            end_idx = raw_data.find("-----------------------------")
            
            if start_idx != -1 and end_idx != -1:
                clean_report = raw_data[start_idx : end_idx + 29] 
                print("\n" + clean_report + "\n")
            else:
                print("Reporte incompleto. Probablemente mucha telemetría en medio.")
        else:
            print("El MCU no respondió al comando de inspección.")

# --- PRUEBA DE CAMBIO ---
if __name__ == "__main__":
    mcu = FilterController('COM11', 115200)

    # 1. Ver qué tiene el MCU ahorita
    mcu.request_coefs_report()

    # 2. Cambiar a Pasa-Todo (A0=1.0, B1=0.0, B2=0.0)
    #mcu.update_coefficient(VECTOR_A, 0, 1.0)
    #mcu.update_coefficient(VECTOR_B, 1, 0.0)
    #mcu.update_coefficient(VECTOR_B, 2, 0.0)

    # 3. Verificar si cambiaron
    mcu.request_coefs_report()

    mcu.ser.close()