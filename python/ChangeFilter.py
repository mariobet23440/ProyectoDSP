import serial
import struct
import time

# --- CONSTANTES DEL PROTOCOLO ---
CMD_START      = b'a'
CMD_VECTOR_SEL = b'b'
CMD_INDEX      = b'c'
CMD_DATA_0     = b'd'
CMD_DATA_1     = b'e'
CMD_DATA_2     = b'f'
CMD_DATA_3     = b'g'
CMD_STOP       = b'h'
CMD_INSPECT    = b'i'
CMD_FREQ_LOW   = b'j'
CMD_FREQ_HIGH  = b'k'
CMD_RESUME     = b'r'

VECTOR_A = 0 
VECTOR_B = 1 

class FilterController:
    def __init__(self, port, baud=115200):
        self.ser = None
        try:
            # Intentamos abrir el puerto
            self.ser = serial.Serial(port, baud, timeout=0.2)
            print(f"Conectado a {port}. Esperando inicialización...")
            time.sleep(2) # Tiempo para que el MCU reinicie si es necesario
        except Exception as e:
            print(f"\n--- ERROR CRÍTICO ---")
            print(f"No se pudo abrir el puerto {port}: {e}")
            print("Asegúrate de cerrar el Monitor Serie de Arduino o cualquier otra Terminal.")
            exit()

    def _send_packet(self, data_byte, cmd_char):
        """ Envía un paquete y espera el ACK, limpiando basura previa """
        # Limpiar cualquier byte de telemetría que haya quedado en el buffer de entrada
        if self.ser.in_waiting > 0:
            self.ser.read_all()
            
        packet = bytes([data_byte]) + cmd_char
        self.ser.write(packet)
        
        # Esperar el ACK (el MCU devuelve el mismo cmd_char)
        ack = self.ser.read(1)
        if ack != cmd_char:
            # Reintento rápido por si hubo lag
            ack_retry = self.ser.read(1)
            if ack_retry != cmd_char:
                print(f"ERROR: Fallo de sincronía en {cmd_char}. Recibido: {ack.hex()}")
                return False
        return True

    def update_coefficient(self, vector_type, index, value):
        """ Envía un nuevo coeficiente float al MCU """
        print(f"--- Actualizando {'A' if vector_type==0 else 'B'}[{index}] -> {value} ---")
        
        # 1. Iniciar secuencia (esto pone config_mode = 1 en el MCU)
        self._send_packet(0, CMD_START)
        self._send_packet(vector_type, CMD_VECTOR_SEL)
        self._send_packet(index, CMD_INDEX)
        
        # 2. Empaquetar float (Little Endian)
        float_bytes = struct.pack('<f', value)
        self._send_packet(float_bytes[0], CMD_DATA_0)
        self._send_packet(float_bytes[1], CMD_DATA_1)
        self._send_packet(float_bytes[2], CMD_DATA_2)
        self._send_packet(float_bytes[3], CMD_DATA_3)
        
        # 3. Finalizar envío
        self._send_packet(0, CMD_STOP)
        print("Coeficiente enviado con éxito.\n")

    def set_sampling_frequency(self, freq_hz):
        """ Cambia la frecuencia de muestreo (uint16_t) """
        if not (0 < freq_hz <= 65535):
            print("Frecuencia fuera de rango (1 - 65535 Hz)")
            return

        print(f"--- Cambiando Frecuencia a {freq_hz} Hz ---")
        low_byte = freq_hz & 0xFF
        high_byte = (freq_hz >> 8) & 0xFF
        
        self._send_packet(low_byte, CMD_FREQ_LOW)
        self._send_packet(high_byte, CMD_FREQ_HIGH)
        print("Frecuencia actualizada.\n")

    def request_coefs_report(self):
        """ Solicita y muestra el reporte ASCII de los coeficientes """
        print("Solicitando reporte de coeficientes...")
        self.ser.reset_input_buffer()
        
        # Enviar comando de inspección
        self.ser.write(bytes([0]) + CMD_INSPECT)
        
        # Esperar a que el MCU termine de transmitir el texto
        time.sleep(0.6) 
        
        if self.ser.in_waiting > 0:
            raw_bytes = self.ser.read_all()
            # Decodificamos ignorando los bytes binarios de telemetría que pudieran quedar
            raw_data = raw_bytes.decode('ascii', errors='ignore')
            
            start_tag = "--- Coeficientes"
            end_tag = "-----------------------------"
            
            start_idx = raw_data.find(start_tag)
            end_idx = raw_data.find(end_tag)
            
            if start_idx != -1 and end_idx != -1:
                clean_report = raw_data[start_idx : end_idx + len(end_tag)]
                print("\n" + clean_report + "\n")
            else:
                print("DEBUG: Reporte incompleto. Intenta aumentando el time.sleep().")
        else:
            print("El MCU no respondió al comando de inspección.")

    def resume_telemetry(self):
        """ Le dice al MCU que salga del modo config y vuelva a enviar telemetría """
        print("Reanudando flujo de telemetría binaria...")
        self._send_packet(0, CMD_RESUME)

# --- FLUJO PRINCIPAL ---
if __name__ == "__main__":
    # Ajusta el COM según tu Administrador de Dispositivos
    mcu = FilterController('COM11', 115200)

    try:
        # 1. Ver qué coeficientes tiene el MCU ahorita
        mcu.request_coefs_report()
        
        # 2. Ejemplo: Cambiar la frecuencia a 15kHz
        mcu.set_sampling_frequency(30000)
        
        # 3. Ejemplo: Actualizar un coeficiente del vector B
        # update_coefficient(vector, indice, valor)

        # 4. Ver el reporte final para confirmar cambios

        # 5. REANUDAR TELEMETRÍA (Para que el Serial Plotter de Python funcione)
        mcu.resume_telemetry()

    except KeyboardInterrupt:
        print("\nCerrando programa...")
    finally:
        if mcu.ser:
            mcu.ser.close()
            print("Puerto cerrado.")