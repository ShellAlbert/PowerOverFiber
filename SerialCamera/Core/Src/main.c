/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include <stdarg.h>
#include "ov2640.h"
#include "Zsy_iRaySensor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//#define ZDBG_MSG_EN 1
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
static void
MX_DCMI_Init_OV2640 (void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void
SystemClock_Config (void);
static void
MX_GPIO_Init (void);
static void
MX_DMA_Init (void);
static void
MX_DCMI_Init (void);
static void
MX_I2C1_Init (void);
static void
MX_I2C2_Init (void);
static void
MX_TIM1_Init (void);
static void
MX_USART1_UART_Init (void);
static void
MX_USART2_UART_Init (void);
static void
MX_USART3_UART_Init (void);
static void
MX_FMC_Init (void);
/* USER CODE BEGIN PFP */
/* USER CODE BEGIN 0 */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//the maximum linear address space is 65535 in word unit (32bits) for DMA.
//If the image size in words does not exceed 65535, the stream can be configured in normal mode.
//In general, one frame of JPEG compressed image can be 50KBytes~80KBytes.
//So here we use a 128KByte buffer.
#define JPEG_BUFFER_SIZE  (1024*31)  //128KB buffer size= 1024*31 INT bytes.
uint32_t iJpegBufferSize = JPEG_BUFFER_SIZE; //1024*128.
//SoC RAM.
//if we define this array as uint8_t, it will be failed. why?
//uint32_t iJpegBuffer[JPEG_BUFFER_SIZE];

//External RAM.
//Since DMA DONOT support write External RAM directly,
//so we must use DMA interrupt to copy data from SoC RAM to External RAM.
uint32_t iJpegBuffer[JPEG_BUFFER_SIZE] __attribute__ ((section(".ExtRAM")));
uint32_t iITFrameCnt = 0;
uint8_t g_iFSMFlag = 0;

/* frame buffer for infrared image buffer */
/* Yantai iRay Infrared Camera : 14-bit one pixel, resolution is 614*512 */
/* So 640*152*16bit/8bit= 655360bytes, /4bytes, sizeof(int)=163840 INT(32bits) */
/* 163840 Exceed 65535 !!!!!*/
#define INFRARED_IMGBUF_INT		163840 //(640*512*16/8/4)
uint32_t frameBufferExt[INFRARED_IMGBUF_INT] __attribute__ ((section(".ExtRAM")));

//SoC RAM Frame Buffer.
//640*512*16-bits
//640/2=320,512/2=256.
//The size of 1/4 part of image is calculated by 320*256*16-bits/8-bits=163840bytes,/4bytes=40960words.
#define SIZE_1_OF_4_WORD	(40960)
uint32_t frameBufferSoC[SIZE_1_OF_4_WORD + 128];

void
vprint (const char *fmt, va_list argp)
{
  char string[128];
  if (0 < vsprintf (string, fmt, argp)) // build string
    {
      // send message via UART3
      HAL_UART_Transmit (&huart3, (uint8_t*) string, strlen (string), 0xFFFF);
    }
}

void
ZUART_Printf (const char *fmt, ...) // custom printf() function
{
#ifdef ZDBG_MSG_EN
  va_list argp;
  va_start(argp, fmt);
  vprint (fmt, argp);
  va_end(argp);
#endif
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int
main (void)
{
  /* USER CODE BEGIN 1 */
  char msg_buffer[128];
  uint32_t i;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init ();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config ();

  /* USER CODE BEGIN SysInit */

  //make sure MX_DCMI_Init() must be called after MX_DMA_Init().
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  //MX_USART1_UART_Init ();
  MX_USART2_UART_Init ();
  //MX_USART3_UART_Init ();
  MX_GPIO_Init ();
  MX_DMA_Init ();
  MX_DCMI_Init ();
  MX_I2C1_Init ();
  MX_I2C2_Init ();
  //MX_TIM1_Init ();
  //MX_FMC_Init ();
  /* USER CODE BEGIN 2 */
  //ZUART_Printf("SerialCAM built on %s %s\r\n", __DATE__, __TIME__);
  /* External SRAM write-read test successfully!*/
#if 0
  //clear RAM to zero.
  for (i = 0; i < INFRARED_IMGBUF_INT; i++)
    {
      *(frameBufferExt + i) = 0;
    }
  //write data.
  for (i = 0; i < INFRARED_IMGBUF_INT; i++)
    {
      *(frameBufferExt + i) = i + 3;
    }
  //Read back and check data.
  for (i = 0; i < INFRARED_IMGBUF_INT; i++)
    {
      if ((i + 3) != frameBufferExt[i])
	{
	  sprintf (msg_buffer, "External RAM Test Error, %d!=%d, Code break down!\r\n", i + 3, frameBufferExt[i]);
	  HAL_UART_Transmit (&huart3, (uint8_t*) msg_buffer, strlen (msg_buffer), 200);
	  while (1)
	    {
	      //LED1 & LED2 OFF.
	      HAL_GPIO_WritePin (GPIOF, LED1_Pin | LED2_Pin, GPIO_PIN_RESET);
	      HAL_Delay (250);

	      //LED1 & LED2 ON.
	      HAL_GPIO_WritePin (GPIOF, LED1_Pin | LED2_Pin, GPIO_PIN_SET);
	      HAL_Delay (250);
	    }
	}
    }
#ifdef ZDBG_MSG_EN
  sprintf (msg_buffer, "External RAM Test Passed!\r\n");
  HAL_UART_Transmit (&huart3, (uint8_t*) msg_buffer, strlen (msg_buffer), 200);
#endif
#endif
  //Two LEDs OFF.
  HAL_GPIO_WritePin (GPIOF, LED1_Pin | LED2_Pin, GPIO_PIN_SET);

  //read DAY_NIGHT signal.
  //HAL_Delay (50);
  //if (HAL_GPIO_ReadPin (DAY_NIGHT_GPIO_Port, DAY_NIGHT_Pin))
  if (1)
    {
      //ZUART_Printf("Day - Visible\r\n");
      MX_DCMI_Init_OV2640 ();

      //Enable CAM1 power.
      HAL_GPIO_WritePin (GPIOG, CAM1_PWR_EN_Pin, GPIO_PIN_SET);
      //SYNC_SWITCH=0, CAM1 Signal Pass.
      //BUS_SWITCH=0, CAM1 Signal Pass.
      HAL_GPIO_WritePin (GPIOB, SYNC_SWITCH_Pin | BUS_SWITCH_Pin, GPIO_PIN_RESET);
      HAL_Delay (10);

      //Initial OV2640 registers via I2C.
      OV2640_Init (&hi2c1, &hdcmi);
      //must delay X seconds to bypass white-balance images to get normal images.
      HAL_Delay (2500);
      //OV2640_ResolutionOptions (imgRes);
      //HAL_Delay (10);

      /* Disable unwanted HSYNC (IT_LINE) / VSYNC (IT_VSYNC) interrupts */
      __HAL_DCMI_DISABLE_IT(&hdcmi, DCMI_IT_LINE | DCMI_IT_VSYNC|DCMI_IT_ERR|DCMI_IT_OVR);
      __HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_FRAME);

      while (1)
	{
	  uint8_t *pJpegData = (uint8_t*) (frameBufferSoC);
	  uint8_t iHeadFound = 0;
	  uint8_t iTailFound = 0;
	  uint32_t iValidEndPosition = 0;
	  int recv_len;
	  switch (g_iFSMFlag)
	    {
	    case 0:
	      //ZUART_Printf ("1-Start DMA\r\n");
	      //memset (frameBufferSoC, 0, SIZE_1_OF_4_WORD * 4);
	      //LED1 on.
	      //HAL_GPIO_WritePin (GPIOF, LED1_Pin, GPIO_PIN_RESET);
	      HAL_DCMI_Start_DMA (&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t) frameBufferSoC, SIZE_1_OF_4_WORD);
	      g_iFSMFlag = 1;
	      break;

	    case 1:
	      //ZUART_Printf("2-Waiting IT_FRAME...\r\n");
	      //HAL_Delay (100);
	      break;

	    case 2:
	      //ZUART_Printf("3-Get FRAME\r\n");
	      //Disable CAM1 power.
	      HAL_GPIO_WritePin (GPIOG, CAM1_PWR_EN_Pin, GPIO_PIN_RESET);
	      g_iFSMFlag = 3;
	      break;

	    case 3:
	      recv_len = SIZE_1_OF_4_WORD - __HAL_DMA_GET_COUNTER(hdcmi.DMA_Handle);
	      //ZUART_Printf("4-Get IT_FRAME %d,DMA %d\r\n", iITFrameCnt, recv_len);

	      //dump first 10 bytes data to UART3.
	      //ZUART_Printf("DATA-%08x-%08x\r\n", frameBufferSoC[0], frameBufferSoC[1]);

	      //frame head FF D8, frame end FF D9.
	      //loop to find the frame end to skip the last ZEROs.
	      for (i = 0; i < recv_len * 4; i++)
		{
		  if (pJpegData[i + 0] == 0xFF && pJpegData[i + 1] == 0xD8)
		    {
		      iHeadFound = 1;
		      break;
		    }
		}
	      for (i = recv_len * 4; i > 0; i--)
		{
		  if (pJpegData[i - 1] == 0xD9 && pJpegData[i - 2] == 0xFF)
		    {
		      uint32_t bypass = recv_len * 4 - i;
		      iTailFound = 1;
		      iValidEndPosition = recv_len * 4 - bypass;
		      break;
		    }
		}

	      //dump all data to UART2, transmit via Laser Diode.
	      if (iHeadFound && iTailFound)
		{
		  //Laser Diode Power up before Transmitting.
		  //0 = Laser Diode Power ON.
		  //1 = Laser Diode Power OFF.
		  HAL_GPIO_WritePin (LD_PWR_EN_GPIO_Port, LD_PWR_EN_Pin, GPIO_PIN_RESET);
		  HAL_Delay (5);

		  //Dump data to UART3 for debug.
		  //frame structure: package size+package data.
		  //HAL_UART_Transmit (&huart3, (uint8_t*) &iValidEndPosition, sizeof(iValidEndPosition) , 0xFFFFFFFF); //package size.
		  for (i = 0; i < iValidEndPosition; i++)
		    {
		      //UART2 is Laser Diode.
		      HAL_UART_Transmit (&huart2, (uint8_t*) &pJpegData[i], 1, 0xFFFFFFFF);
		      //UART3 is Debug Message Output.
		      //HAL_UART_Transmit (&huart3, (uint8_t*) &pJpegData[i], 1, 0xFFFFFFFF); //package data.
		    }
		  //Laser Diode Power down to save energy.
		  //0 = Laser Diode Power ON.
		  //1 = Laser Diode Power OFF.
		  HAL_GPIO_WritePin (LD_PWR_EN_GPIO_Port, LD_PWR_EN_Pin, GPIO_PIN_SET);
		}
	      g_iFSMFlag = 4;
	      break;

	    case 4:
	      if (1)
		{
		  //Configuration register pointer=0x02, 16-bits.
		  //0x1000: bit[12]=1, Temperature and Humidity are acquired in sequence, Temperature First.
		  uint8_t HDC_config[3] =
		    { 0x02, 0x10, 0x00 };
		  uint8_t HDC_trig = 0x00;
		  uint8_t HDC_data[4];
		  float temperature, humidity;
		  uint32_t tmpData;
		  uint32_t dataSize;

		  //initial I2C Slave.
		  HAL_I2C_Master_Transmit (&hi2c2, 0x80, HDC_config, 3, 0xFFFFFFFF);

		  //Trigger the measurements by executing a pointer write transaction with the address pointer set to 0x00.
		  HAL_I2C_Master_Transmit (&hi2c2, 0x80, &HDC_trig, 1, 0xFFFFFFFF);

		  //Wait for the measurements to complete.
		  while (HAL_GPIO_ReadPin (HT_DRDYn_GPIO_Port, HT_DRDYn_Pin) == 1)
		    {
		    }
		  //receive data.
		  HAL_I2C_Master_Receive (&hi2c2, 0x80, HDC_data, 4, 0xFFFFFFFF);

		  //Laser Diode Power up before Transmitting.
		  //0 = Laser Diode Power ON.
		  //1 = Laser Diode Power OFF.
		  HAL_GPIO_WritePin (LD_PWR_EN_GPIO_Port, LD_PWR_EN_Pin, GPIO_PIN_RESET);
		  HAL_Delay (5);

		  //dump all data to UART2, transmit via Laser Diode.
		  HAL_UART_Transmit (&huart2, HDC_data, 4, 0xFFFFFFFF);

		  //Laser Diode Power down to save energy.
		  //0 = Laser Diode Power ON.
		  //1 = Laser Diode Power OFF.
		  HAL_GPIO_WritePin (LD_PWR_EN_GPIO_Port, LD_PWR_EN_Pin, GPIO_PIN_SET);

		  //dump data to UART3, frame structure: package size+package data.
//		  dataSize=4;  //package size=4 bytes.
//		  HAL_UART_Transmit (&huart3, (const uint8_t*)&dataSize, 4, 0xffffffff);
//		  //package data.
//		  HAL_UART_Transmit (&huart3, HDC_data, 4, 0xffffffff);
#if 0 //Disable to Save Energy.
		  //ZUART_Printf("T: %02x%02x H:%02x%02x\r\n", HDC_data[0], HDC_data[1], HDC_data[2], HDC_data[3]);
		  tmpData = (uint32_t) HDC_data[0] << 8 | HDC_data[1];
		  temperature = tmpData / 65536.0;
		  temperature = temperature * 165.0f - 40.0f;
		  tmpData = (uint32_t) HDC_data[2] << 8 | HDC_data[3];
		  humidity = tmpData / 65536.0 * 100.0;
		  //ZUART_Printf ("Temperature:%.2f, Humidity:%.2f%%\r\n", temperature, humidity);
#endif
		  //LED1 off.
		  //HAL_GPIO_WritePin (GPIOF, LED1_Pin, GPIO_PIN_SET);
		}
	      //ZUART_Printf ("5-Done\r\n");
	      //HAL_PWR_EnterSTANDBYMode();
	      //dead loop to consume power till voltage drops down below 2.489V to reset MCU.
	      while (1)
		{
		  HAL_GPIO_WritePin (GPIOF, LED1_Pin, GPIO_PIN_RESET);
		  HAL_Delay (100);
		  HAL_GPIO_WritePin (GPIOF, LED1_Pin, GPIO_PIN_SET);
		  HAL_Delay (100);
		}
	      g_iFSMFlag = 0;
	      break;

	    default:
	      g_iFSMFlag = 0;
	      break;
	    }
	}
    }
  else
    {
      uint32_t i;
      uint8_t iPieces = 0;

      ZUART_Printf ("Night - Infrared\r\n");

      //Enable CAM2 Power.
      HAL_GPIO_WritePin (GPIOG, CAM2_PWR_EN_Pin, GPIO_PIN_SET);
      HAL_Delay (6000);

      //SYNC_SWITCH=1, CAM2 Signal Pass.
      //BUS_SWITCH=1, CAM2 Signal Pass.
      HAL_GPIO_WritePin (GPIOB, SYNC_SWITCH_Pin | BUS_SWITCH_Pin, GPIO_PIN_SET);
      HAL_Delay (500);

      //Set more than Once to ensure command was received successfully by Infrared Sensor.
      for (i = 0; i < 2; i++)
	{
	  uint8_t rxData[32];

	  //read FPA temperature.
	  uint8_t cmd_FPA_Temperature[] =
	    { 0xAA, 0x04, 0x01, 0xC3, 0x00, 0x72, 0xEB, 0xAA };

	  //read Chip Temperature.
	  uint8_t cmd_Chip_Temperature[] =
	    { 0xAA, 0x04, 0x01, 0x7C, 0x00, 0x2B, 0xEB, 0xAA };

	  //set LVCMOS interface.
	  uint8_t cmd_LVCMOS[] =
	    { 0xAA, 0x06, 0x01, 0x5D, 0x02, 0x02, 0x00, 0x12, 0xEB, 0xAA };

	  //Digital Video Source Selection=NUC, 14-bits.
	  uint8_t cmd_NUCSource[] =
	    { 0xAA, 0x05, 0x01, 0x5C, 0x01, 0x01, 0x0E, 0xEB, 0xAA };

	  //Digital Video Source Selection=DRC, 8-bits.
	  uint8_t cmd_DRCSource[] =
	    { 0xAA, 0x05, 0x01, 0x5C, 0x01, 0x02, 0x0F, 0xEB, 0xAA };

	  //Stop NUC.
	  uint8_t cmd_NUCStop[] =
	    { 0xAA, 0x05, 0x01, 0x01, 0x01, 0x00, 0xB2, 0xEB, 0xAA };

	  //LED2 on.
	  HAL_GPIO_WritePin (GPIOF, LED2_Pin, GPIO_PIN_RESET);
	  //read FPA Temperature.
	  if (HAL_UART_Transmit (&huart1, (uint8_t*) cmd_FPA_Temperature, sizeof(cmd_FPA_Temperature), 200) == HAL_OK)
	    {
	      memset (rxData, 0, sizeof(rxData));
	      //block rx with 200ms.
	      HAL_UART_Receive (&huart1, rxData, sizeof(rxData), 200);
	      ZUART_Printf ("FPA_Temperature: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \r\n", ///<
		  rxData[0], rxData[1], rxData[2], rxData[3], rxData[4], ///<
		  rxData[5], rxData[6], rxData[7], rxData[8], rxData[9]);
	      HAL_Delay (100);
	    }

	  //read Chip Temperature.
	  if (HAL_UART_Transmit (&huart1, (uint8_t*) cmd_Chip_Temperature, sizeof(cmd_Chip_Temperature), 200) == HAL_OK)
	    {
	      memset (rxData, 0, sizeof(rxData));
	      //block rx with 200ms.
	      HAL_UART_Receive (&huart1, rxData, sizeof(rxData), 200);
	      ZUART_Printf ("Chip_Temperature: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \r\n", ///<
		  rxData[0], rxData[1], rxData[2], rxData[3], rxData[4], ///<
		  rxData[5], rxData[6], rxData[7], rxData[8], rxData[9]);
	      HAL_Delay (100);
	    }

	  //select NUC as Digital Video Source.
	  if (HAL_UART_Transmit (&huart1, (uint8_t*) cmd_NUCSource, sizeof(cmd_NUCSource), 200) == HAL_OK)
	  //if (HAL_UART_Transmit (&huart1, (uint8_t*) cmd_DRCSource, sizeof(cmd_DRCSource), 200) == HAL_OK)
	    {
	      memset (rxData, 0, sizeof(rxData));
	      //block rx with 200ms.
	      HAL_UART_Receive (&huart1, rxData, sizeof(rxData), 200);
	      ZUART_Printf ("dvSource: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \r\n", ///<
		  rxData[0], rxData[1], rxData[2], rxData[3], rxData[4], ///<
		  rxData[5], rxData[6], rxData[7], rxData[8], rxData[9]);
	      HAL_Delay (100);
	    }

	  //set LVCMOS interface.
	  if (HAL_UART_Transmit (&huart1, (uint8_t*) cmd_LVCMOS, sizeof(cmd_LVCMOS), 200) == HAL_OK)
	    {
	      memset (rxData, 0, sizeof(rxData));
	      //block rx with 200ms.
	      HAL_UART_Receive (&huart1, rxData, sizeof(rxData), 200);
	      ZUART_Printf ("LVCMOS: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \r\n", ///<
		  rxData[0], rxData[1], rxData[2], rxData[3], rxData[4], ///<
		  rxData[5], rxData[6], rxData[7], rxData[8], rxData[9]);
	      HAL_Delay (100);
	    }

	  //set NUC calibration.
	  if (HAL_UART_Transmit (&huart1, (uint8_t*) cmd_NUCStop, sizeof(cmd_NUCStop), 200) == HAL_OK)
	    {
	      memset (rxData, 0, sizeof(rxData));
	      //block rx with 200ms.
	      HAL_UART_Receive (&huart1, rxData, sizeof(rxData), 200);
	      ZUART_Printf ("NUC_Stop: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \r\n", ///<
		  rxData[0], rxData[1], rxData[2], rxData[3], rxData[4], ///<
		  rxData[5], rxData[6], rxData[7], rxData[8], rxData[9]);
	      HAL_Delay (100);
	    }

	  //LED2 off.
	  HAL_GPIO_WritePin (GPIOF, LED2_Pin, GPIO_PIN_SET);
	  HAL_Delay (200);
	}
      /* Disable unwanted HSYNC (IT_LINE) / VSYNC (IT_VSYNC) interrupts */
      __HAL_DCMI_DISABLE_IT(&hdcmi, DCMI_IT_LINE | DCMI_IT_VSYNC|DCMI_IT_ERR|DCMI_IT_OVR);
      __HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_FRAME|DCMI_IT_ERR);

      while (1)
	{
	  int remain_len;
	  int recv_len;
	  switch (g_iFSMFlag)
	    {
	    case 0:
	      ZUART_Printf ("1-Start DMA\r\n");
	      // 65535 (0xFFFF is the DMA maximum transfer length).
	      //(640*512*16bits)/8bits=655360bytes=//4bytes(int)=163840(int)
	      //163840(int)/65535(int)=2.5 times.
	      //163840/4=40960 << 65535
	      //Since DMA is in Circle mode, so it can not generate Complete Interrupt.
	      //But we can use Frame Interrupt.
	      // HAL_DCMI_Start_DMA (&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t) frameBufferInfrared, 40960/*INFRARED_IMGBUF_INT*/);

	      //why can't DMA store data to External RAM?
	      //if change to SoC RAM, we can get IT_FRAME interrupt!
	      //why?????
	      //HAL_DCMI_Start_DMA (&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t) frameBufferInfrared, 1024*64);

	      /*
	       * @param  DCMI_Mode DCMI capture mode snapshot or continuous grab.
	       * @param  pData     The destination memory buffer address.
	       * @param  Length    The length of capture to be transferred (in 32-bit words).
	       * @note  In case of length larger than 65535 (0xFFFF is the DMA maximum transfer length),
	       *        the API uses the end of the destination buffer as a work area: HAL_DCMI_Start_DMA()
	       *        initiates a circular DMA transfer from DCMI DR to the ad-hoc work buffer and each
	       *        half and complete transfer interrupt triggers a copy from the work buffer to
	       *        the final destination pData through a second DMA channel.
	       */
	      memset (frameBufferSoC, 0, sizeof(frameBufferSoC));
	      //okay, we can get data from Infrared Sensor.
	      //HAL_DCMI_Start_DMA (&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t) frameBufferSoC, 1024*64/2);
	      //HAL_DCMI_Start_DMA (&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t) frameBufferSoC, 65535);

	      //Since we have no enough SoC RAM to hold one complete frame,
	      //so we use crop features to cut one frame into 4 pieces.
	      //We get 1/4 part each time and after 4 times, we get one complete frame.
	      //First, we store 1/4 part data into SoC RAM, then we move them into External RAM to generate a complete frame.
	      //640*512*16-bits
	      //640/2=320,512/2=256
	      //So, cut into 4 pieces, one part size = 320*256*16-bits/8-bits=163840bytes,/4bytes=40960(words)

	      //X0: DCMI window crop window X offset (number of pixels clocks to count before the capture).
	      //HOFFCNT[13:0]: Horizontal offset count
	      //This value gives the number of pixel clocks to count before starting a capture.

	      //Y0: DCMI window crop window Y offset (image capture starts with this line number, previous line data are ignored).
	      //VST[12:0]: Vertical start line count
	      //The image capture starts with this line number. Previous line data are ignored.

	      //XSize DCMI crop window horizontal size (in number of pixels per line).
	      //CAPCNT[13:0]: Capture count
	      //This value gives the number of pixel clocks to be captured from the starting point on the same line.
	      //It value must corresponds to word-aligned data for different widths of parallel interfaces.
	      //0x0000 => 1 pixel
	      //0x0001 => 2 pixels
	      //0x0002 => 3 pixels

	      //YSize DCMI crop window vertical size (in lines count).
	      //VLINE[13:0]: Vertical line count
	      //This value gives the number of lines to be captured from the starting point.
	      //0x0000: 1 line
	      //0x0001: 2 lines
	      //0x0002: 3 lines
	      //HAL_DCMI_ConfigCROP(&DCMIHandle, x*2, y, w*2-1, h-1);
	      switch (iPieces)
		{
		case 0: //(0,0)
		  HAL_DCMI_ConfigCrop (&hdcmi, 0, 0, 640 * 2 - 1, 128 - 1);
		  break;
		case 1: //(0,1)
		  HAL_DCMI_ConfigCrop (&hdcmi, 0, 128, 640 * 2 - 1, 128 - 1);
		  break;
		case 2: //(1,0)
		  HAL_DCMI_ConfigCrop (&hdcmi, 0, 256, 640 * 2 - 1, 128 - 1);
		  break;
		case 3: //(1,1)
		  HAL_DCMI_ConfigCrop (&hdcmi, 0, 384, 640 * 2 - 1, 128 - 1);
		  break;
		default:
		  HAL_DCMI_ConfigCrop (&hdcmi, 0, 0, 640 * 2 - 1, 128 - 1);
		  break;
		}
	      HAL_DCMI_EnableCrop (&hdcmi);
	      //the Frame Buffer Size must be a little greater than Expected Size.
	      //Otherwise __HAL_DMA_GET_COUNTER() will not return correct result!!!
	      HAL_DCMI_Start_DMA (&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t) frameBufferSoC, SIZE_1_OF_4_WORD + 128);

	      //error, why doesn't DMA store data to External RAM?
	      //Blocked at 2-Waiting IT_FRAME.
	      //HAL_DCMI_Start_DMA (&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t) frameBufferInfrared, 65535/*INFRARE_IMGBUF_INT*/);

	      //error, we get all 0x00 data from Infrared Sensor!!!!!!!!
	      //HAL_DCMI_Start_DMA (&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t) frameBufferSoC, 1024*64);

	      g_iFSMFlag = 1;
	      break;

	    case 1:
	      ZUART_Printf ("2-Waiting IT_FRAME...\r\n");
	      HAL_Delay (100);
	      break;

	    case 2:
	      ZUART_Printf ("3-Get FRAME\r\n");
	      g_iFSMFlag = 3;
	      break;

	    case 3:
	      remain_len = __HAL_DMA_GET_COUNTER(hdcmi.DMA_Handle);
	      recv_len = (SIZE_1_OF_4_WORD + 128) - remain_len;
	      ZUART_Printf ("4-Get IT_FRAME %d, DMA Transfered:%d\r\n", iITFrameCnt, recv_len);
	      //LED2 on.
	      HAL_GPIO_WritePin (GPIOF, LED2_Pin, GPIO_PIN_RESET);
	      //dump first 10 bytes data to UART3.
	      ZUART_Printf ("DATA-%08x-%08x\r\n", frameBufferSoC[0], frameBufferSoC[1]);

	      //move data from SoC RAM to External RAM.
	      switch (iPieces)
		{
		case 0:
		  memcpy ((void*) (frameBufferExt + 0), (void*) frameBufferSoC, SIZE_1_OF_4_WORD * 4); //copy by bytes.
//		  for (i = 0; i < SIZE_1_OF_4_WORD; i++)
//		    {
//		      HAL_UART_Transmit (&huart3, (const uint8_t*) &frameBufferSoC[i], 4, 5000);
//		    }
//		  HAL_Delay (200);
		  break;
		case 1:
		  memcpy ((void*) (frameBufferExt + SIZE_1_OF_4_WORD * 1), (void*) frameBufferSoC, SIZE_1_OF_4_WORD * 4); //copy by bytes.
//		  for (i = 0; i < SIZE_1_OF_4_WORD; i++)
//		    {
//		      HAL_UART_Transmit (&huart3, (const uint8_t*) &frameBufferSoC[i], 4, 5000);
//		    }
//		  HAL_Delay (200);
		  break;
		case 2:
		  memcpy ((void*) (frameBufferExt + SIZE_1_OF_4_WORD * 2), (void*) frameBufferSoC, SIZE_1_OF_4_WORD * 4); //copy by bytes.
//		  for (i = 0; i < SIZE_1_OF_4_WORD; i++)
//		    {
//		      HAL_UART_Transmit (&huart3, (const uint8_t*) &frameBufferSoC[i], 4, 5000);
//		    }
//		  HAL_Delay (200);
		  break;
		case 3:
		  memcpy ((void*) (frameBufferExt + SIZE_1_OF_4_WORD * 3), (void*) frameBufferSoC, SIZE_1_OF_4_WORD * 4); //copy by bytes.
//		  for (i = 0; i < SIZE_1_OF_4_WORD; i++)
//		    {
//		      HAL_UART_Transmit (&huart3, (const uint8_t*) &frameBufferSoC[i], 4, 5000);
//		    }
//		  HAL_Delay (200);
		  break;
		default:
		  break;
		}

	      if (iPieces >= 3)
		{
		  iPieces = 0;
		  g_iFSMFlag = 4; //all 4 parts captured done.
		}
	      else
		{
		  iPieces++;
		  g_iFSMFlag = 0; //continue to capture others parts.
		}

	      //LED2 off.
	      HAL_GPIO_WritePin (GPIOF, LED2_Pin, GPIO_PIN_SET);
	      break;

	    case 4:
	      ZUART_Printf ("5-Done\r\n");
#if   1
	      //dump all data to UART2, transmit via Laser Diode.
	      for (i = 0; i < SIZE_1_OF_4_WORD; i++)
		{
		  HAL_UART_Transmit (&huart3, (const uint8_t*) &frameBufferExt[i], 4, 2000);
		}
	      HAL_Delay (2000);
	      for (i = 0; i < SIZE_1_OF_4_WORD; i++)
		{
		  HAL_UART_Transmit (&huart3, (const uint8_t*) &frameBufferExt[SIZE_1_OF_4_WORD * 1 + i], 4, 2000);
		}
	      HAL_Delay (2000);
	      for (i = 0; i < SIZE_1_OF_4_WORD; i++)
		{
		  HAL_UART_Transmit (&huart3, (const uint8_t*) &frameBufferExt[SIZE_1_OF_4_WORD * 2 + i], 4, 2000);
		}
	      HAL_Delay (2000);
	      for (i = 0; i < SIZE_1_OF_4_WORD; i++)
		{
		  HAL_UART_Transmit (&huart3, (const uint8_t*) &frameBufferExt[SIZE_1_OF_4_WORD * 3 + i], 4, 2000);
		}
	      HAL_Delay (2000);
#endif
	      g_iFSMFlag = 0;
	      break;

	    default:
	      g_iFSMFlag = 0;
	      break;
	    }
	}
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
      ZUART_Printf ("App Overflow Here!\r\n");

      //LED1 and LED2 on.
      HAL_GPIO_WritePin (GPIOF, LED1_Pin | LED2_Pin, GPIO_PIN_RESET);
      HAL_Delay (500);
      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */
      //LED1 and LED2 off.
      HAL_GPIO_WritePin (GPIOF, LED1_Pin | LED2_Pin, GPIO_PIN_SET);
      HAL_Delay (500);
    }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void
SystemClock_Config (void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct =
    { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct =
    { 0 };

  /** Configure the main internal regulator output voltage
   */
  if (HAL_PWREx_ControlVoltageScaling (PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
      Error_Handler ();
    }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig (&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler ();
    }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
      Error_Handler ();
    }
}

/**
 * @brief DCMI Initialization Function
 * @param None
 * @retval None
 */
static void
MX_DCMI_Init (void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_LOW;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_14B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
  hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
  hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  if (HAL_DCMI_Init (&hdcmi) != HAL_OK)
    {
      Error_Handler ();
    }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void
MX_I2C1_Init (void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00303D5B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init (&hi2c1) != HAL_OK)
    {
      Error_Handler ();
    }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter (&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
      Error_Handler ();
    }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter (&hi2c1, 0) != HAL_OK)
    {
      Error_Handler ();
    }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void
MX_I2C2_Init (void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00303D5B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init (&hi2c2) != HAL_OK)
    {
      Error_Handler ();
    }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter (&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
      Error_Handler ();
    }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter (&hi2c2, 0) != HAL_OK)
    {
      Error_Handler ();
    }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void
MX_TIM1_Init (void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig =
    { 0 };
  TIM_MasterConfigTypeDef sMasterConfig =
    { 0 };

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 800 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init (&htim1) != HAL_OK)
    {
      Error_Handler ();
    }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource (&htim1, &sClockSourceConfig) != HAL_OK)
    {
      Error_Handler ();
    }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization (&htim1, &sMasterConfig) != HAL_OK)
    {
      Error_Handler ();
    }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void
MX_USART1_UART_Init (void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT | UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart1.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init (&huart1) != HAL_OK)
    {
      Error_Handler ();
    }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void
MX_USART2_UART_Init (void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 460800;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_8;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init (&huart2) != HAL_OK)
    {
      Error_Handler ();
    }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void
MX_USART3_UART_Init (void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 500000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init (&huart3) != HAL_OK)
    {
      Error_Handler ();
    }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void
MX_DMA_Init (void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority (DMA2_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ (DMA2_Channel5_IRQn);

}

/* FMC initialization function */
static void
MX_FMC_Init (void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_NORSRAM_TimingTypeDef Timing =
    { 0 };

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
   */
  hsram1.Instance = FMC_NORSRAM_DEVICE;
  hsram1.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FMC_MEMORY_TYPE_PSRAM;
  hsram1.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
  hsram1.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hsram1.Init.WriteFifo = FMC_WRITE_FIFO_ENABLE;
  hsram1.Init.PageSize = FMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 4;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 2;
  Timing.BusTurnAroundDuration = 2;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init (&hsram1, &Timing, NULL) != HAL_OK)
    {
      Error_Handler ();
    }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void
MX_GPIO_Init (void)
{
  GPIO_InitTypeDef GPIO_InitStruct =
    { 0 };

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2 ();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin (GPIOF, CAM1_PWDN_Pin | LED1_Pin | LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin (CAM1_RST_GPIO_Port, CAM1_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin (LD_PWR_EN_GPIO_Port, LD_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin (GPIOB, SYNC_SWITCH_Pin | BUS_SWITCH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin (GPIOG, CAM2_PWR_EN_Pin | CAM1_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CAM1_PWDN_Pin LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = CAM1_PWDN_Pin | LED1_Pin | LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : CAM1_RST_Pin */
  GPIO_InitStruct.Pin = CAM1_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (CAM1_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD_PWR_EN_Pin */
  GPIO_InitStruct.Pin = LD_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (LD_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SYNC_SWITCH_Pin BUS_SWITCH_Pin */
  GPIO_InitStruct.Pin = SYNC_SWITCH_Pin | BUS_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : HT_DRDYn_Pin VER2_Pin */
  GPIO_InitStruct.Pin = HT_DRDYn_Pin | VER2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DAY_NIGHT_Pin VER1_Pin */
  GPIO_InitStruct.Pin = DAY_NIGHT_Pin | VER1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CAM2_PWR_EN_Pin CAM1_PWR_EN_Pin */
  GPIO_InitStruct.Pin = CAM2_PWR_EN_Pin | CAM1_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void
HAL_DCMI_FrameEventCallback (DCMI_HandleTypeDef *hdcmi)
{
  if (hdcmi->Instance == DCMI)
    {
      HAL_DCMI_Suspend (hdcmi);
      HAL_DCMI_Stop (hdcmi);
      iITFrameCnt++;
      g_iFSMFlag = 2;
    }
}
//https://community.st.com/s/question/0D50X0000AU3ZLtSQN/stm32f4-dcmi-difference-between-itframe-vs-itvsync-interrupt
//STM32F4 DCMI - difference between IT_FRAME vs IT_VSYNC interrupt ?
//VSYNC interrupt occurs always upon VSYNC changing from inactive to active,
//regardless of whether there's a capture ongoing or not.
//FRAME interrupt occurs only when capture achieves end of frame,
//either indicated by VSYNC or before that if crop is set,
//but certainly only if capture is going on.
void
HAL_DCMI_VsyncEventCallback (DCMI_HandleTypeDef *hdcmi)
{
  if (0)
    {
      if (hdcmi->Instance == DCMI)
	{
	  ZUART_Printf ("VSYNCCallback\r\n");
	}
    }
}
void
HAL_DCMI_LineEventCallback (DCMI_HandleTypeDef *hdcmi)
{
  if (0)
    {
      if (hdcmi->Instance == DCMI)
	{
	  ZUART_Printf ("HSYNCCallback\r\n");
	}
    }
}
void
HAL_DCMI_ErrorCallback (DCMI_HandleTypeDef *hdcmi)
{
  if (0)
    {
      if (hdcmi->Instance == DCMI)
	{
	  ZUART_Printf ("DCMI_ErrorCallback\r\n");
	}
    }
}
/**
 * @brief DCMI Initialization Function
 * @param None
 * @retval None
 */
static void
MX_DCMI_Init_OV2640 (void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_LOW;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_ENABLE;
  hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
  hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
  hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  if (HAL_DCMI_Init (&hdcmi) != HAL_OK)
    {
      Error_Handler ();
    }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void
Error_Handler (void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq ();
  while (1)
    {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
