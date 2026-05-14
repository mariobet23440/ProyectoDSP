import serial
import struct
import collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from datetime import datetime

# --- CONFIGURACIÓN ---
SERIAL_PORT = 'COM10'  # Ajusta según tu PC
BAUD_RATE = 115200    # Si es muy lento para 10kHz, sube a 921600 en STM32 y aquí
MAX_POINTS = 200      # Cuántos puntos mostrar en pantalla a la vez
PACKET_SIZE = 7       # [0xAA, 0xBB, float(4), 0xFF]

# Configuración de la gráfica
fig, ax = plt.subplots()
x_data = collections.deque(maxlen=MAX_POINTS)
y_data = collections.deque(maxlen=MAX_POINTS)
line, = ax.plot([], [], lw=2, color='cyan')

# Estética de la gráfica
ax.set_facecolor('#202020')
fig.patch.set_facecolor('#101010')
ax.set_ylim(-10, 4100) # Rango para DAC de 12 bits (0-4095)
ax.set_xlim(0, MAX_POINTS)
ax.grid(True, color='#404040')
ax.set_title("STM32 DSP - Salida del Filtro en Tiempo Real", color='white')

# Abrir puerto serial
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)

def init():
    line.set_data([], [])
    return line,

def update(frame):
    # Intentamos leer varios paquetes por frame para no quedar rezagados
    while ser.in_waiting >= PACKET_SIZE:
        if ser.read(1) == b'\xaa':
            if ser.read(1) == b'\xbb':
                payload = ser.read(5)
                if len(payload) == 5 and payload[4] == 0xff:
                    raw_val = struct.unpack('<f', payload[:4])[0]
                    
                    # Agregar dato y timestamp para consola
                    y_data.append(raw_val)
                    x_data.append(len(y_data))
                    
                    # Timestamp opcional en consola para debug
                    # ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                    # print(f"[{ts}] Val: {raw_val:.2f}")

    # Actualizar datos de la línea
    line.set_data(range(len(y_data)), list(y_data))
    return line,

# Crear animación
# blit=True para mayor velocidad de renderizado
ani = animation.FuncAnimation(fig, update, init_func=init, interval=20, blit=True)

plt.show()
ser.close()