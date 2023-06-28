/*
 * bsp.h
 *
 *  Created on: 28 abr. 2021
 *      Author: agust
 */

#ifndef BSP_H_
#define BSP_H_

#include <stdint.h>

void BSP_Init(void);

void CONSOLE_SendMsg(uint8_t *pData, uint16_t Size);

void 		LED_on(void *led);
void 		LED_off(void *led);
void 		LED_toggle(void *led);
void 		LED_blinky(void *led, uint16_t ton, uint16_t toff, uint16_t times);
float 		BSP_BOARD_GetTemp(void);
void		BSP_Delay(uint32_t ms);
uint8_t*	BSP_DHT11_Read(void);
void 		BSP_Init(void);
uint32_t    BSP_LUZ_GetState(void);
uint32_t    BSP_SUELO_GetHum(void);
void 		BSP_WIFI_Init(void);
void 		BSP_WIFI_send_msg(char *msg, uint8_t string_len);
void 		BSP_WIFI_connect(void);
uint8_t		BSP_WIFI_status(void);

float SENSTEMP_getTemperature(void);



#endif /* BSP_H_ */
