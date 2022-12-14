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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//the maximum linear address space is 65535 in word unit (32bits) for DMA.
//If the image size in words does not exceed 65535, the stream can be configured in normal mode.
#define JPEG_BUFFER_SIZE  (1024*31)  //31kB buffer size.
uint32_t iJpegBufferSize = JPEG_BUFFER_SIZE;
uint32_t iJpegBuffer[JPEG_BUFFER_SIZE];
uint32_t iVsyncCnt = 0;
uint32_t iFps = 0;

//#define RES160X120
#define RES320X240
//#define RES640X480
//#define RES800x600
//#define RES1024x768
//#define RES1280x960

#ifdef RES160X120
enum imageResolution imgRes=RES_160X120;
uint8_t frameBuffer[RES_160X120] = { 0 };
#endif

#ifdef RES320X240
enum imageResolution imgRes = RES_320X240;
uint8_t frameBuffer[RES_320X240] =
  { 0 };
#endif

#ifdef RES640X480
enum imageResolution imgRes=RES_640X480;
uint8_t frameBuffer[RES_640X480] = { 0 };
#endif

#ifdef RES800x600
enum imageResolution imgRes=RES_800x600;
uint8_t frameBuffer[RES_800x600] = { 0 };
#endif

#ifdef RES1024x768
enum imageResolution imgRes=RES_1024x768;
uint8_t frameBuffer[RES_1024x768] = { 0 };
#endif

#ifdef RES1280x960
enum imageResolution imgRes = RES_1280x960;
uint8_t frameBuffer[RES_1280x960] ={ 0 };
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t g_iVSYNCFlag = 0;
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
MX_USART1_UART_Init (void);
static void
MX_I2C3_Init (void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void
vprint (const char *fmt, va_list argp)
{
  char string[200];
  if (0 < vsprintf (string, fmt, argp)) // build string
    {
      HAL_UART_Transmit (&huart1, (uint8_t*) string, strlen (string), 0xffffff); // send message via UART
    }
}

void
uart_printf (const char *fmt, ...) // custom printf() function
{
#ifdef UART_DEBUG
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
  uint16_t i;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init ();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config ();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init ();
  MX_DMA_Init ();
  MX_DCMI_Init ();
  MX_USART1_UART_Init ();
  MX_I2C3_Init ();
  /* USER CODE BEGIN 2 */

  //1. Turn on the power switch to power camera module.
  HAL_GPIO_WritePin (CAM_PWR_EN_GPIO_Port, CAM_PWR_EN_Pin, GPIO_PIN_SET);
  HAL_Delay (1000);

  //2. Initial OV2640 registers via I2C.
  OV2640_Init (&hi2c3, &hdcmi);
  HAL_Delay (100);
  //OV2640_ResolutionOptions (imgRes);
  HAL_Delay (100);

  /* Disable unwanted HSYNC (IT_LINE) / VSYNC (IT_VSYNC) interrupts */
  __HAL_DCMI_DISABLE_IT(&hdcmi, DCMI_IT_LINE | DCMI_IT_VSYNC);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
      int recv_len;
      switch (g_iVSYNCFlag)
	{
	case 0:
	  uart_printf ("start DMA\r\n");
	  memset (iJpegBuffer, 0, iJpegBufferSize);
	  HAL_DCMI_Start_DMA (&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t) iJpegBuffer, iJpegBufferSize);
	  g_iVSYNCFlag = 1;
	  break;

	case 1:
	  uart_printf ("waiting IT_FRAME...\r\n");
	  HAL_Delay (100);
	  break;

	case 2:
	  uart_printf ("get Frame\r\n");
	  g_iVSYNCFlag = 3;
	  break;

	case 3:
	  recv_len = iJpegBufferSize - __HAL_DMA_GET_COUNTER(hdcmi.DMA_Handle);
	  uart_printf ("get VSYNC %d,Frame:%d DMA:%d\r\n", iVsyncCnt, iFps, recv_len);
	  //dump first 10 bytes data to uart.
	  for(i=0;i<recv_len;i++)
	   {
	      HAL_UART_Transmit (&huart1, (uint8_t*)& iJpegBuffer[i], 4, 0xffffff);
	   }

	  g_iVSYNCFlag = 4;
	  break;

	case 4:
	  //uart_printf ("Stop\r\n");
	  HAL_Delay (1000);
	  g_iVSYNCFlag = 0;
	  break;

	default:
	  break;
	}
      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */
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

/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void
MX_I2C3_Init (void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10909CEC;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init (&hi2c3) != HAL_OK)
    {
      Error_Handler ();
    }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter (&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
      Error_Handler ();
    }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter (&hi2c3, 0) != HAL_OK)
    {
      Error_Handler ();
    }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  huart1.Init.BaudRate = 460800;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init (&huart1) != HAL_OK)
    {
      Error_Handler ();
    }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin (DCMI_RESET_GPIO_Port, DCMI_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin (CAM_PWR_EN_GPIO_Port, CAM_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DCMI_RESET_Pin */
  GPIO_InitStruct.Pin = DCMI_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (DCMI_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMI_PWDN_Pin */
  GPIO_InitStruct.Pin = DCMI_PWDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init (DCMI_PWDN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CAM_PWR_EN_Pin */
  GPIO_InitStruct.Pin = CAM_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (CAM_PWR_EN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void
HAL_DCMI_VsyncEventCallback (DCMI_HandleTypeDef *hdcmi)
{

}
void
HAL_DCMI_FrameEventCallback (DCMI_HandleTypeDef *hdcmi)
{
  if (hdcmi->Instance == DCMI)
    {
      HAL_DCMI_Suspend (hdcmi);
      HAL_DCMI_Stop (hdcmi);
      iFps++;
      g_iVSYNCFlag = 2;
    }
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
