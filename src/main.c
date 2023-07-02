/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "main.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "Task.h"
#include "Queue.h"

void *ledRed;
void *ledBlue;
void *ledOrange;
void *ledGreen;


TaskHandle_t xTask2Handle = NULL;
QueueHandle_t xQueue;

/* Variables de sensores */
volatile uint8_t *dht11_measures;
volatile float 	  temperatura_board;
volatile float    temperatura_dht11;
volatile float 	  humedad_suelo;
volatile float    humedad_dht11;
volatile float 	  sensor_inputs[4];
static TaskHandle_t xEnviar = NULL;
static TaskHandle_t xLeer = NULL;
void create_msg(float * sensor_inputs, char * msg);


/**************************************** TASKS ****************************************/

static void vConectarWifi( void *pvParameters ){
	for(;;)
	{
		/* Conectamos el modulo wifi a la red */
		BSP_WIFI_connect();
		/* Esperamos a que se conecte el modulo */
		//vTaskDelay(pdMS_TO_TICKS(1000));
		xTaskNotifyGive( xLeer );
		break;
	}
	/* Borramos la task ya que cumplio su proposito */
	vTaskDelete(NULL);
}

static void vLeerDatos( void *pvParameters ){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xPeriod = pdMS_TO_TICKS( 100*1000 );

	ulTaskNotifyTake( pdFALSE, portMAX_DELAY );
	for(;;)
	{
		temperatura_board = BSP_BOARD_GetTemp();
		humedad_suelo     = BSP_SUELO_GetHum();
		dht11_measures    = BSP_DHT11_Read();
		temperatura_dht11 = dht11_measures[0];
		humedad_dht11     = dht11_measures[1];

		/* Permitimos al modulo wifi enviar los datos */
		xTaskNotifyGive( xEnviar );

		/* Establecemos periodicidad */
		vTaskDelayUntil(&xLastWakeTime, xPeriod);

	}
	//vTaskDelete(NULL);
}

static void vEnviarDatos( void *pvParameters )
{
	float sensor_inputs[4];
	char msg[150];
	for(;;){
		ulTaskNotifyTake( pdFALSE, portMAX_DELAY );

		/* Creamos el mensaje */
		sensor_inputs[0] = temperatura_board;
		sensor_inputs[1] = humedad_suelo;
		sensor_inputs[2] = temperatura_dht11;
		sensor_inputs[3] = humedad_dht11;
		create_msg(sensor_inputs, msg);

		/* Enviamos el mensaje */
		BSP_WIFI_send_msg(msg, strlen(msg));
	}
	//vTaskDelete(NULL);
}

/**************************************** AUXILIARES ****************************************/

void create_msg(float * sensor_inputs, char * msg){
	int main, fractional;
	char temp[50];
	char inputs[4][6] = {"\0"};
	for (uint8_t i=0; i<4; i++){
		main = (int)(sensor_inputs[i]);
		fractional = (int)(sensor_inputs[i]-main)*100;
		sprintf(&inputs[i][0], "%d,%d", main, fractional);
	}
	sprintf(msg, "Temp placa %s °C \n ", inputs[0]);
	sprintf(temp, "Hum suelo %s \n ", inputs[1]);
	strcat(msg, temp);
	sprintf(temp, "Temp Ambiente %s °C, ", inputs[2]);
	strcat(msg, temp);
	sprintf(temp, "Hum Ambiente %s \n", inputs[3]);
	strcat(msg, temp);
}


/*******************************************************************************************/


int main(void)
{

	/*Inicializacion de los recursos*/
	BSP_Init();

	/* Instanciacion de tasks */
	BaseType_t res = xTaskCreate(vConectarWifi, "Task conectar wifi"   , 500, NULL, 7, NULL);
	if (res != pdPASS){for(;;){}}
	res = xTaskCreate(vLeerDatos   , "Task lectura de datos", 500, NULL, 6, &xLeer);
	if (res != pdPASS){for(;;){}}
	res = xTaskCreate(vEnviarDatos , "Task envio de datos"  , 500, NULL, 6, &xEnviar);
	if (res != pdPASS){for(;;){}}

	vTaskStartScheduler();
	/* Execution will only reach here if there was insufficient heap to
	start the scheduler. */

	for(;;);
}


