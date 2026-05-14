function STM32_SetFilterCoefficients(s, a, b)
% DSP_FilterSetCoefficients Envía coeficientes de filtro a la STM32
% s: Objeto serial de MATLAB (ya abierto)
% a: Vector de coeficientes 'a' (del diseño del filtro)
% b: Vector de coeficientes 'b' (del diseño del filtro)

    % --- CONSTANTES DEL PROTOCOLO ---
    CMD_START      = uint8('a');
    CMD_VECTOR_SEL = uint8('b');
    CMD_INDEX      = uint8('c');
    CMD_DATA_0     = uint8('d');
    CMD_DATA_1     = uint8('e');
    CMD_DATA_2     = uint8('f');
    CMD_DATA_3     = uint8('g');
    CMD_STOP       = uint8('h');
    CMD_RESUME     = uint8('r');

    VECTOR_A = 0; 
    VECTOR_B = 1;

    fprintf('Iniciando transferencia de coeficientes...\n');

    % --- ENVIAR VECTOR A ---
    for i = 1:length(a)
        send_single_coefficient(s, VECTOR_A, i-1, a(i));
    end

    % --- ENVIAR VECTOR B ---
    for i = 1:length(b)
        send_single_coefficient(s, VECTOR_B, i-1, b(i));
    end

    % Salir del modo configuración en la STM32
    write(s, [0, CMD_RESUME], "uint8");
    fprintf('¡Transferencia finalizada con éxito!\n');

    % --- FUNCIÓN ANIDADA PARA ENVIAR CADA COEFICIENTE ---
    function send_single_coefficient(s, vector_type, index, value)
        % 1. Convertir double de MATLAB a float (32 bits)
        val_float = single(value);
        
        % 2. Convertir a 4 bytes (Little Endian)
        % typecast convierte el valor single a una representación de 4 uint8
        float_bytes = typecast(val_float, 'uint8');
        
        % 3. Protocolo de envío (Dato, Comando)
        % Cada write envía [byte_de_datos, byte_de_comando]
        
        % Inicio de paquete para este coeficiente
        write_and_ack(s, 0, CMD_START);
        write_and_ack(s, vector_type, CMD_VECTOR_SEL);
        write_and_ack(s, index, CMD_INDEX);
        
        % Envío de los 4 bytes del float
        write_and_ack(s, float_bytes(1), CMD_DATA_0);
        write_and_ack(s, float_bytes(2), CMD_DATA_1);
        write_and_ack(s, float_bytes(3), CMD_DATA_2);
        write_and_ack(s, float_bytes(4), CMD_DATA_3);
        
        % Fin de paquete
        write_and_ack(s, 0, CMD_STOP);
        
        fprintf('Enviado: Vector %d [%d] = %f\n', vector_type, index, value);
    end

    % --- FUNCIÓN PARA MANEJAR EL ACK ---
    function write_and_ack(s, data, cmd)
        % Limpiar buffer de entrada por si hay telemetría vieja
        if s.NumBytesAvailable > 0
            read(s, s.NumBytesAvailable, "uint8");
        end
        
        % Enviar paquete
        write(s, [data, cmd], "uint8");
        
        % Esperar el ACK (el MCU devuelve el CMD)
        ack = read(s, 1, "uint8");
        if isempty(ack) || ack ~= cmd
             warning('Fallo de sincronía en comando: %c', char(cmd));
        end
    end
end