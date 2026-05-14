%% EJEMPLO DE STM32_SetFilterCoefficients

% 1. Configuración del puerto (Ajusta el nombre del COM)
puerto = serialport("COM10", 115200, "Timeout", 0.5);

% 2. Diseño del filtro (ejemplo: pasa-bajos, orden 2)
fs = 10000; % Frecuencia de muestreo
fc = 1000;  % Frecuencia de corte
[b, a] = cheby1(2, 0.5, fc/(fs/2));

% 3. Enviar a la STM32
DSP_FilterSetCoefficients(puerto, a, b);

% 4. Cerrar puerto al finalizar
clear puerto;