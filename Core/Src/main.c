/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"
#include "dma.h"
#include "fdcan.h"
#include "memorymap.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mycan.h"
#include "pid.h"
#include "disk.h"
#include "route.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Computer_Vision.h"
#include "Teaching_Pendant.h"
#include "shangceng.h"
#include "A1_Motor.h"
#include "ANO_TC.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
uint8_t Can_1_Data[16];
uint8_t Can_2_Data[8];
uint8_t Can_3_Data[8];
// extern Coordinate_Speed_Struct Robot_Coordinate_System_V;
extern Route_STU Route_Status;
extern float Line_Angle;
extern Coordinate_Position_Struct World_Coordinate_System_NowPos;
extern Disk_Encoder_Struct Disk_Encoder;
Coordinate_Speed_Struct i;
uint8_t Fire_Start_Check = 0; // 光电门标志位，当置1时表示发射装置初始化完成
float A1_Angle_I_Want = 0;    // A1角度。全局变量
float testaaa = 0;
extern osThreadId_t RouteHandle;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_FDCAN1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();  
  MX_FDCAN2_Init();
  MX_FDCAN3_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  MX_USART10_UART_Init();
  MX_TIM3_Init();
  MX_UART7_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  FDCAN_Config_Start(&hfdcan1);
  FDCAN_Config_Start(&hfdcan2);
  FDCAN_Config_Start(&hfdcan3);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  Disk_Encoder_Restart();     // 瀵??閸??鐖滈惄妯硅?閸欙絾甯撮弨?
  Computer_Vision_Restart();  // 瀵??閸???鐟欏?瑕嗛崣锝嗗复閺??
  Teaching_Pendant_Restart(); // 瀵??閸??澧滈弻鍕??閸欙絾甯撮弨?
  A1Motor_Restart();          // 瀵??閸??A1閺傚洣瑕嗛崣锝嗗复閺??
  Route_Init();
  //ShangCeng_Init(); // 涓婂眰鍒濆?鍖?
  Enhanced_Teaching_Pendant_Init();
  
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 67;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 4096;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_1) // Teaching Pendant
  {
    if (!Fire_Start_Check)
    {
      Fire_Start_Check = 1; // 光电门标志位，当置1时表示发射装置初始化完成
      g_a1motor.command.velocity = 0;
      A1_feedback.Position_Sum = 0;
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  uint32_t status_value;
  status_value = taskENTER_CRITICAL_FROM_ISR();
  if (htim->Instance == TIM2)
  {
    Motor_DJI_Angle_PID_Output_Calculate(&ShangCeng_motor[0], Dribble_Motor_Angle);
    Motor_DJI_Angle_PID_Output_Calculate(&ShangCeng_motor[1], -Dribble_Motor_Angle);
    Motor_DJI_Angle_PID_Output_Calculate(&ShangCeng_motor[2], Camera_Angle);
    Motor_DJI_Angle_PID_Output_Calculate(&ShangCeng_motor[3], Trigger_Angle);
    Trigger_Angle = testaaa;
    CAN_Send_Data(&hfdcan2, 0x200, Can_3_Data, ShangCeng_motor, 8);
    //FDCAN_Send_Data(&hfdcan1, 0x03F, Can_1_Data, &i, &Order_To_Subcontroller);
    FDCAN_Send_Data(&hfdcan1, 0x03F, Can_1_Data, &Route_Status.Coordinate_System.Robot_Coordinate_System_V, &Order_To_Subcontroller);
  }

  if (htim->Instance == TIM3)
  {
    // 这里是A1电机计算和发送定时器,0.25毫秒进一次，用于抓数
    //  g_a1motor.command.position = A1_Angle_Splitting(5000, testagles);
    if (Fire_Start_Check)
    {
      A1Motor_Smooth_Position_Control(&g_a1motor, A1_Angle_I_Want, 3000, 3, 25);
    }
    A1Motor_SendCommand(&huart10, &g_a1motor.command, GPIOE, GPIO_PIN_4);
  }

  if (htim->Instance == TIM4)
  {
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  taskEXIT_CRITICAL_FROM_ISR(status_value);
  /* USER CODE END Callback 1 */
}

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
