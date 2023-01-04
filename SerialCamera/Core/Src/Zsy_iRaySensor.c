/*
 * Zsy_iRaySensor.c
 *
 *  Created on: Dec 13, 2022
 *      Author: Administrator
 */
#include "Zsy_iRaySensor.h"
#include "stm32l4xx_hal.h"
//115200bps, 8N1, LSB first.
//USART1 was connected on PCB Board.
extern UART_HandleTypeDef huart1;
void
Zsy_iRaySensorInit (void)
{
  //read FPA temperature.
  uint8_t cmd[] =
    { 0xaa, 0x04, 0x01, 0xc3, 0x00, 0x72, 0xeb, 0xaa };
  uint8_t rxData[32];
  //block tx with 5000ms.
  if(HAL_UART_Transmit (&huart1, (uint8_t*) cmd, sizeof(cmd), 5000)==HAL_OK)
    {
      //block rx with 2000ms.
      HAL_UART_Receive(&huart1, rxData, sizeof(rxData), 5000);
    }
  if(rxData[0]==0x55 && rxData[8]==0xAA)
    {

    }
}
