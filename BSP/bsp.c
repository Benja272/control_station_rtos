/*
 /*
 * bsp.c
 *
 *  Created on: 28 abr. 2021
 *      Author: agust
 */
#include "stm32f411e_discovery.h"
#include "mk_dht11.h"
#include "stm32f4xx.h"
#include "bsp.h"
#include "led.h"
#include "stdio.h"
#include "string.h"

/* Tamaño del buffer rx de wifi */
#define BUFFER_SIZE 200

extern void *ledRed;
extern void *ledBlue;
extern void *ledOrange;
extern void *ledGreen;

GPIOLED_TypeDef gpio_ledRed = {GPIOD, GPIO_PIN_14};
GPIOLED_TypeDef gpio_ledBlue = {GPIOD, GPIO_PIN_15};
GPIOLED_TypeDef gpio_ledOrange = {GPIOD, GPIO_PIN_13};
GPIOLED_TypeDef gpio_ledGreen = {GPIOD, GPIO_PIN_12};

TIM_HandleTypeDef htim2;
ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart1;

/* Handlers necesarios */
ADC_HandleTypeDef 	hadc1;
TIM_HandleTypeDef 	htim3;
UART_HandleTypeDef 	huart1;
UART_HandleTypeDef 	huart2;
dht11_t 			dht;

/* Buffer de datos wifi */
uint8_t rx_data;					// Byte de destino
uint8_t rx_buffer[BUFFER_SIZE];		// Buffer de destino
uint8_t init_wifi = 0;				// Flag de control de inicializacion
uint8_t check_ok  = 0;				// Flag de control de comando correcto



void *ledBlinky;
uint16_t timesBlinky, tonBlinky, toffBlinky;
uint16_t adcValue;
uint8_t RxData;


void 		LED_Init(void);
void 		BSP_ADC1_Init(void);
void 		BSP_LUZ_Init(void);
void 		BSP_DHT11_Init(void);
void 		BSP_TIM2_Init(void);
void 		BSP_TIM3_Init(void);
void 		BSP_USART1_Init(void);
void 		BSP_USART2_Init(void);
void 		HAL_UART_RxCpltCallback ( UART_HandleTypeDef *huart);
void		BSP_WIFI_send_msg(char *msg, uint8_t string_len);
void 		BSP_WIFI_connect(void);
uint8_t     BSP_WIFI_status(void);
void BSP_WIFI_transmit_it(char * msg);
void SystemClock_Config(void);
void Error_Handler(void);





void BSP_Init(void){

	SystemInit();
	HAL_Init();

	SystemClock_Config();

	LED_Init();
	/* Inicializamos el sensor de luz */
	BSP_LUZ_Init();
	/* Inicializamos el conversor ADC */
	BSP_ADC1_Init();
	/* Inicializamos timers */
	BSP_TIM2_Init();
	BSP_TIM3_Init();

	/* Inicializamos usart */
	BSP_USART1_Init();
	BSP_USART2_Init();

	/* Inicializamos el sensor de temperatura y humedad DHT11 */
	BSP_DHT11_Init();

	/* Inicializacion del modulo wifi */
	BSP_WIFI_Init();
	BSP_Delay(1000);
}

/******************************************************************************
 * 				     	   INICIALIZACION DE SENSORES 					      *
 *****************************************************************************/

void BSP_DHT11_Init(){
	init_dht11(&dht, &htim3, DHT11_USART_PORT, DHT11_USART_Tx_PIN);
}

void BSP_LUZ_Init(){
	/* Inicializamos el clock del puerto del sensor */
	 __HAL_RCC_GPIOC_CLK_ENABLE();
	 __HAL_RCC_GPIOA_CLK_ENABLE();
	 GPIO_InitTypeDef GPIO_InitStruct = {0};
	 /* Configuracion GPIO del sensor */
	 GPIO_InitStruct.Pin = SENSOR_LUZ_PIN;
	 GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	 GPIO_InitStruct.Pull = GPIO_NOPULL;
	 HAL_GPIO_Init(SENSOR_LUZ_PORT, &GPIO_InitStruct);
}




void BSP_ADC1_Init(){
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
}


void BSP_TIM3_Init(){
	  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};

	  htim3.Instance = TIM3;
	  htim3.Init.Prescaler = 48;
	  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim3.Init.Period = 65535;
	  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
	  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
	  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}


void BSP_USART1_Init(){
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 38400;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
	  Error_Handler();
	}
}

void BSP_USART2_Init(){
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 38400;
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
}

void BSP_WIFI_Init(){
	uint8_t command[8];
	sprintf((char *)command, "AT\r\n");
	HAL_UART_Receive_IT(&huart2, &rx_data, 1);
	HAL_UART_Transmit(&huart2, command, 4, 100);
	/* Iniciamos la secuencia de comandos AT */
	init_wifi = 1;
}



/******************************************************************************
 * 				     	   MANIPULACION DE SENSORES 					      *
 *****************************************************************************/

/**
 * @brief	Obtiene una lectura del sensor de temperatura de la placa
 * @retval	Temp: Temperatura en Celsius de la placa
 */
float BSP_BOARD_GetTemp(void){
	ADC_ChannelConfTypeDef sConfig = {0};
	uint32_t ADCValue;
	float Vsense, Temp;

	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	sConfig.Rank    = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			Error_Handler();
	}

	HAL_ADC_Start(&hadc1);

	if(HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK){
		return 0;
	}

	ADCValue = HAL_ADC_GetValue(&hadc1);
	Vsense   = (float)ADCValue * 3000 / ((1<<12) - 1);
	Temp     = (Vsense - 760) / 2.5 + 25;
	return Temp;
}

/**
 * @brief	Obtiene una lectura del sensor de humedad del suelo
 * @retval	Hum: Devuelve la humedad del suelo medida.
 */
uint32_t BSP_SUELO_GetHum(void){
	ADC_ChannelConfTypeDef sConfig = {0};
	float ADCValue, Hum;
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank    = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

	if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){
		Error_Handler();
	}

	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK){
			return 0;
	}
	ADCValue = HAL_ADC_GetValue(&hadc1);
	Hum = 1 - ADCValue/4095;
	Hum = (Hum - 0.25) * 400;
	if (Hum < 0)
		return 0;
	else if (Hum < 100)
		return Hum;
	else
		return 100;
}


uint8_t res[2];
/**
 * @brief	Obtiene una lectura del sensor DHT11.
 * @retval	res[0]: Temperatura medida con el sensor.
 * @retval  res[1]: Humedad del ambiente medida con el sensor.
 */
uint8_t *BSP_DHT11_Read(){
	readDHT11(&dht);
	res[0] = dht.temperature;
	res[1] = dht.humidty;
	return res;
}


/**
 * @brief	Obtiene una lectura del sensor de luz
 * @retval	luz_state: Devuelve el estado del sensor de luz
 */
uint32_t BSP_LUZ_GetState(){
	uint32_t luz_state;
	luz_state = HAL_GPIO_ReadPin(SENSOR_LUZ_PORT, SENSOR_LUZ_PIN);
	return luz_state;
}

/**
 * @brief	Envia un comando a el modulo wifi
 */

void BSP_WIFI_send_msg(char *msg, uint8_t string_len){
	uint8_t n=string_len/5;
	uint8_t mod=string_len%5;
	uint8_t msg_len;
	char dest[6], comm_end[3], comm[50];
	strcpy(comm_end, "\r\n");
	if(mod>0){
		n++;
	}
	for (uint8_t i=0;i<n;i++){
		strncpy(dest, &msg[5*i], sizeof(dest)-1);
		dest[5]='\0';
		msg_len = strlen(dest);
		sprintf((char *)comm, "ATPT=%d,1:", msg_len);
		strcat((char *)comm, dest);
		strcat((char *)comm, comm_end);
		BSP_WIFI_transmit_it((char *)comm);
		BSP_Delay(100);
	}
}

uint8_t BSP_WIFI_status(void){
	return (init_wifi==4);
}

void BSP_WIFI_transmit_it(char * msg){
	HAL_StatusTypeDef res = HAL_BUSY;
	uint8_t command[50] = {'\0'};
	uint8_t len = strlen(msg);
	strcpy((char *)command, msg);
	while (res != HAL_OK){
		res = HAL_UART_Transmit_IT(&huart2, command, len);
		BSP_Delay(2000);
	}
}

void BSP_WIFI_connect(){
	/* Checkeamos si hay que inicializar el Access Point */
	for (;;){
		if(BSP_WIFI_status()){
			break;
		}
		if(check_ok == 2){
			check_ok = 0;

			switch (init_wifi){
			case 1:
				BSP_WIFI_transmit_it("ATPW=1\r\n");
				init_wifi++;
				break;
			case 2:
				BSP_WIFI_transmit_it("ATPN=MICRO2023");
				BSP_WIFI_transmit_it(",MICRO2023\r\n");
				/* Pasamos a la siguiente etapa*/
				init_wifi++;
				break;
			case 3:
				BSP_Delay(500);
				BSP_WIFI_transmit_it("ATPC=0,192.168.1");
				BSP_WIFI_transmit_it("37.133,30");
				BSP_WIFI_transmit_it("00\r\n");

				/* Pasamos a la siguiente etapa*/
				init_wifi = 4;
				break;
			default:
				break;
			}
		}
	}
}

/******************************************************************************
 * 				     	CALLBACKS DE INTERRUPCIONES 						  *
 *****************************************************************************/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART2){
		/* Shift de bytes */
		for(uint8_t i=BUFFER_SIZE - 1; i>0; i--){
			rx_buffer[i] = rx_buffer[i-1];
		}
		rx_buffer[0] = rx_data;

		/* Verificamos si llego un OK */
		if(rx_data == 79 && check_ok == 0)
			check_ok = 1;
		else if (rx_data == 75 && check_ok == 1)
			check_ok = 2;


		/* Recibimos el siguiente byte */
		HAL_UART_Receive_IT(&huart2, &rx_data, 1);
	}
}

/*************************************************************************************/

void CONSOLE_SendMsg(uint8_t *pData, uint16_t Size){
	HAL_UART_Transmit(&huart1, pData, Size, 500);
}

//HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);




void LED_on(void *led){

	GPIOLED_TypeDef *gpioLed;

	gpioLed = (GPIOLED_TypeDef*)led;
	HAL_GPIO_WritePin(gpioLed->port, gpioLed->pin, GPIO_PIN_SET);
}

void LED_off(void *led){

	GPIOLED_TypeDef *gpioLed;

	gpioLed = (GPIOLED_TypeDef*)led;
	HAL_GPIO_WritePin(gpioLed->port, gpioLed->pin, GPIO_PIN_RESET);
}

void LED_toggle(void *led){
	GPIOLED_TypeDef *gpioLed;

	gpioLed = (GPIOLED_TypeDef*)led;
	HAL_GPIO_TogglePin(gpioLed->port, gpioLed->pin);
}

void LED_blinky(void *led, uint16_t ton, uint16_t toff, uint16_t times){

	ledBlinky = led;
	timesBlinky = times;
	tonBlinky = ton;
	toffBlinky = toff;
}

void LED_blinkyIRQ(void){
	static uint16_t ton = 1;
	static uint16_t toff = 1;

	if(timesBlinky){
		if(ton){
			ton--;
			if(!ton){
				LED_off(ledBlinky);
			}
		} else if(toff){
			toff--;
			if(!toff){
				timesBlinky--;
				if(timesBlinky){
					ton = tonBlinky;
					toff = toffBlinky;
					LED_on(ledBlinky);
				}
			}
		}
	}
}


void LED_Init(void){

	GPIO_InitTypeDef  GPIO_InitStruct;

	__HAL_RCC_GPIOD_CLK_ENABLE();

	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

	GPIO_InitStruct.Pin = gpio_ledRed.pin;
	HAL_GPIO_Init(gpio_ledRed.port, &GPIO_InitStruct);
	ledRed = (void*)&gpio_ledRed;

	GPIO_InitStruct.Pin = gpio_ledBlue.pin;
	HAL_GPIO_Init(gpio_ledBlue.port, &GPIO_InitStruct);
	ledBlue = (void*)&gpio_ledBlue;

	GPIO_InitStruct.Pin = gpio_ledOrange.pin;
	HAL_GPIO_Init(gpio_ledOrange.port, &GPIO_InitStruct);
	ledOrange = (void*)&gpio_ledOrange;

	GPIO_InitStruct.Pin = gpio_ledGreen.pin;
	HAL_GPIO_Init(gpio_ledGreen.port, &GPIO_InitStruct);
	ledGreen = (void*)&gpio_ledGreen;
}

float SENSTEMP_getTemperature(void){

	float temp, admVolt;
	uint32_t adResult;

	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, 500) != HAL_OK)
		return (0);
	adResult = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	admVolt = (float)adResult * 3000 / 4095;
	temp = ((admVolt - 760) / 2.5) + 25;

	return temp;
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  /*new*/
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  /*new*/
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ =  8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
    Error_Handler();
  }
}

void BSP_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 160;
  htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END TIM2_Init 2 */

}

/******************************************************************************
 * 				    FUNCIONES DE INICIALIZACION (MSP) 					      *
 *****************************************************************************/

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
  if(tim_baseHandle->Instance==TIM3)
  {
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* TIM3 interrupt Init */
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /* TIM2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
  if(tim_baseHandle->Instance==TIM3)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();

    /* TIM3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
  }
}



void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1) {
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /*
     * ADC1 GPIO Configuration
     * PA1 ------> ADC1_IN1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}


void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle){
  if(adcHandle->Instance==ADC1){
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();
    /*
     * ADC1 GPIO Configuration
     * PA1 ------> ADC1_IN1
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);
  }
}



void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /*
    USART1 GPIO Configuration
    PA15  ------> USART1_TX
    PB7   ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  }
  else if(uartHandle->Instance==USART2) {
    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /*
     * *USART2 GPIO Configuration
    	PA2  ------> USART2_TX
    	PA3  ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle) {
  if(uartHandle->Instance==USART1) {
	  /* Peripheral clock disable */
	  __HAL_RCC_USART1_CLK_DISABLE();
	  /*
    	USART1 GPIO Configuration
    	PA15  ------> USART1_TX
    	PB7   ------> USART1_RX
	   */
	  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);
	  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

	  /* USART1 interrupt DeInit */
	  HAL_NVIC_DisableIRQ(USART1_IRQn);
  }
  else if(uartHandle->Instance==USART2) {
	  /* Peripheral clock disable */
	  __HAL_RCC_USART2_CLK_DISABLE();
	  /*
	   * *USART2 GPIO Configuration
  	  	  PA2     ------> USART2_TX
  	  	  PA3     ------> USART2_RX
	   */
	  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);
	  /* USART2 interrupt DeInit */
	  HAL_NVIC_DisableIRQ(USART2_IRQn);
  }
}

/******************************************************************************
 * 				     	FUNCIONES DE MISCELANEAS   					          *
 *****************************************************************************/

/**
 * @brief	Delay bloqueante
 * @param	ms: Indica la cantidad en ms del delay
 */
void BSP_Delay(uint32_t ms){
	HAL_Delay(ms);
}


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
