/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float *buffer;
    uint16_t size;
    uint16_t head;
} CircularBuffer;

typedef union {
    float f;
    uint32_t u32;
    uint8_t bytes[4];
} BinaryData;

// Estructura de paquete para evitar desincronización en la PC
typedef struct {
    uint8_t header[2]; // Ejemplo: 0xAA, 0xBB
    uint8_t data[4];
    uint8_t footer;    // Ejemplo: 0xFF
} TelemetryPacket;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART2_BUFFER_SIZE 100
#define DSP_BUFFER_SIZE 100

// Protocolo de comunicaciones USART
#define CMD_START 			'a'
#define CMD_VECTOR_SEL		'b'
#define CMD_INDEX			'c'
#define CMD_DATA_0			'd'
#define CMD_DATA_1			'e'
#define CMD_DATA_2			'f'
#define CMD_DATA_3			'g'
#define CMD_STOP			'h'
#define CMD_INSPECT_COEFS    'i'  // Nuevo comando para reporte ASCII
#define CMD_FREQ_LOW  'j'
#define CMD_FREQ_HIGH 'k'
#define CMD_RESUME    'r'

#define VECTOR_SEL_A		0x00
#define VECTOR_SEL_B		0x01

#define FCLK1				1000000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
volatile uint8_t adc_ready = 0;
volatile uint16_t adc_raw_value = 0; // Cambiado de puntero a variable real

// Buffers físicos
float x_data[3] = {0};
float y_data[3] = {0};

// Estructuras de control
CircularBuffer cb_x = {x_data, 3, 0};
CircularBuffer cb_y = {y_data, 3, 0};

// Coeficientes
float a_coefs[3] = {1.0f-0.9f};
float b_coefs[3] = {1.0f, -0.9f}; // Ajustados para estabilidad (ejemplo)

// Buffer para entrada de UART
uint8_t RX_buffer[UART2_BUFFER_SIZE];

// Variables para bytes para conversión de byte a float
uint8_t rx_byte_0 = 0;
uint8_t rx_byte_1 = 0;
uint8_t rx_byte_2 = 0;
uint8_t rx_byte_3 = 0;

uint8_t rx_vector_sel = 0;
uint8_t rx_index = 0;

TelemetryPacket tx_packet = {{0xAA, 0xBB}, {0}, 0xFF};
BinaryData dsp_output;

volatile uint8_t config_mode = 0;

uint16_t target_frequency = 10000; // Valor por defecto
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_DAC_Init(void);
/* USER CODE BEGIN PFP */
void SetSamplingRate(uint16_t frequency);
float DigitalFilter(float x_n, CircularBuffer *cX, CircularBuffer *cY, float a[], float b[]);
void ProcessInputUART(float* a, float* b, uint8_t input[]);
float Convert4BytesToFloat(uint8_t b3, uint8_t b2, uint8_t b1, uint8_t b0);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Función para insertar: Sobrescribe el más viejo y avanza el head
void CB_Push(CircularBuffer *cb, float value) {
    cb->buffer[cb->head] = value;
    cb->head = (cb->head + 1) % cb->size;
}

// Función para leer: lookback 0 es el más reciente, 1 el anterior...
float CB_Read(CircularBuffer *cb, uint16_t lookback) {
    // Si head es 2, el más reciente (lookback 0) está en index 1
    int16_t index = (int16_t)cb->head - 1 - (int16_t)lookback;
    while (index < 0) index += cb->size;
    return cb->buffer[index];
}

// Cambiar la frecuencia de meuestreo
void SetSamplingRate(uint16_t frequency)
{
    if (frequency == 0) return;

    // Usar FCLK1 que es la que definiste arriba
    uint32_t new_period = (FCLK1 / frequency) - 1;

    __HAL_TIM_SET_AUTORELOAD(&htim2, new_period);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
}

// Filtro Digital usando la lógica de índices móviles
float DigitalFilter(float x_n, CircularBuffer *cX, CircularBuffer *cY, float a[], float b[]) {
    float y_acc = 0;

    // 1. Guardamos la entrada actual en el buffer circular
    CB_Push(cX, x_n);

    // 2. Feedforward: y[n] = a0*x[n] + a1*x[n-1] + ...
    for(uint16_t i = 0; i < cX->size; i++) {
        y_acc += a[i] * CB_Read(cX, i);
    }

    // 3. Feedback: - (b1*y[n-1] + b2*y[n-2]...)
    // Nota: Empezamos en j=1 porque b[0] suele ser 1
    for(uint16_t j = 1; j < cY->size; j++) {
        y_acc -= b[j] * CB_Read(cY, j - 1);
    }

    // 4. Guardamos la salida calculada para el siguiente ciclo
    CB_Push(cY, y_acc);

    return y_acc;
}


// Comandos UART
void ProcessInputUART(float* a, float* b, uint8_t input[])
{
    uint8_t data = input[0];
    uint8_t command = input[1];

    switch(command)
    {
        case(CMD_START):
            config_mode = 1;
            HAL_UART_AbortTransmit(&huart2); // Detener telemetría de inmediato
            rx_byte_0 = 0; rx_byte_1 = 0; rx_byte_2 = 0; rx_byte_3 = 0;
            break;

        case(CMD_VECTOR_SEL): rx_vector_sel = data; break;
        case(CMD_INDEX):      rx_index = data;      break;

        case(CMD_DATA_0):     rx_byte_0 = data;     break;
        case(CMD_DATA_1):     rx_byte_1 = data;     break;
        case(CMD_DATA_2):     rx_byte_2 = data;     break;
        case(CMD_DATA_3):     rx_byte_3 = data;     break;

        case(CMD_STOP):
            {
                BinaryData temp;
                temp.bytes[0] = rx_byte_0; // Little Endian (LSB primero)
                temp.bytes[1] = rx_byte_1;
                temp.bytes[2] = rx_byte_2;
                temp.bytes[3] = rx_byte_3;

                if (rx_vector_sel == VECTOR_SEL_A) a[rx_index] = temp.f;
                else if (rx_vector_sel == VECTOR_SEL_B) b[rx_index] = temp.f;

                HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            }
            break;

        case(CMD_INSPECT_COEFS):
            config_mode = 1;
            HAL_UART_AbortTransmit(&huart2);

            char msg[64];
            sprintf(msg, "\r\n--- Coeficientes Actuales ---\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

            for(int i=0; i<3; i++) {
                sprintf(msg, "a[%d]: %.6f\r\n", i, a[i]);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
            }
            for(int i=0; i<3; i++) {
                sprintf(msg, "b[%d]: %.6f\r\n", i, b[i]);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
            }
            sprintf(msg, "-----------------------------\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

            //config_mode = 0;
		break;

        case CMD_FREQ_LOW:
			// Guardamos los 8 bits menos significativos
			target_frequency = (target_frequency & 0xFF00) | data;
		break;

		case CMD_FREQ_HIGH:
			// Guardamos los 8 bits más significativos y actualizamos el Timer
			target_frequency = (target_frequency & 0x00FF) | (data << 8);

			// Evitamos frecuencias de 0 para no romper el cálculo del Timer
			if (target_frequency > 0) {
				SetSamplingRate(target_frequency);
			}
		break;

		case CMD_RESUME:
			config_mode = 0; // Salir del modo configuración y reanudar DMA
		break;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_DAC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);    // Disparador de frecuencia de muestreo
  HAL_ADC_Start_IT(&hadc1);      // Iniciar ADC por interrupción

  // Iniciamos el DAC de forma normal (sin DMA).
  // Actualizaremos el valor manualmente después de filtrar.
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

  SetSamplingRate(target_frequency);        // 10 kHz
  HAL_UART_Receive_IT(&huart2, RX_buffer, 2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if(adc_ready)
	{
		adc_ready = 0;

		float x = (float)adc_raw_value;
		float y_out = DigitalFilter(x, &cb_x, &cb_y, a_coefs, b_coefs);

		int32_t dac_val = (int32_t)y_out;
		if(dac_val > 4095) dac_val = 4095;
		if(dac_val < 0)    dac_val = 0;

		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)dac_val);

		// SOLO ENVIAR SI NO ESTAMOS EN MODO CONFIGURACIÓN
		if (config_mode == 0) {
			UART_SendBinaryFloat(y_out);
		}
	}
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 90-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// INTERRUPCIONES

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
    if(hadc->Instance == ADC1) {
        adc_raw_value = (uint16_t)HAL_ADC_GetValue(hadc); // Correcto: asignación directa
        adc_ready = 1;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if (huart->Instance == USART2) {
        // 1. Procesar los 2 bytes recibidos
        ProcessInputUART(a_coefs, b_coefs, RX_buffer);

        // 2. ENVIAR ACK: Devolvemos el comando recibido como confirmación
        uint8_t ack = RX_buffer[1];
        HAL_UART_Transmit(&huart2, &ack, 1, 10);

        // 3. RE-ARMAR la recepción para los PRÓXIMOS 2 BYTES únicamente
        HAL_UART_Receive_IT(&huart2, RX_buffer, 2);
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
