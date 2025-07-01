/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "route.h"
#include "main.h"
#include "mycan.h"
#include "Teaching_Pendant.h"
#include "Computer_Vision.h"
#include "math.h"
#include "shangceng.h"
#include "usart.h"
#include "ANO_TC.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int a[4] = {0, 0, 0, 0};
#define Basket_Hight 2553;                   // 我们场地篮框的高度+篮球半径
#define CHANGE_TO_RADIAN (0.01745329251994f) // 角度制转换为弧度制系数
#define CHANGE_TO_ANGLE (57.29577951308232f) // 弧度制转换为角度制系数
// uint8_t Last_Route = 0;                                           // 这个是用于Reset时储存上次打的是哪个点
uint8_t Vision_Point_Flag = 0;                          // 到达视觉识别点1时置为1，到达视觉识别点2时置为2，其余时间都是0
uint8_t Start_Pass_Ball_From_Dribble_To_Shoot_Flag = 0; // 开始将球从运球装置转移到投球装置上标志位，置为1时执行
uint8_t Ball_On_Shoot_Structure_Flag = 0;               // 球在发射装置上标志位，置为1时球在发射装置上
uint8_t Dribble_Structure_In_Medium_Flag = 0;           // 运球装置在中间位置
uint8_t Reload_Flag = 0;                                // 换弹标志位，置为1时执行换弹
uint8_t Finish_Reload_Flag = 0;                         // 换弹完成标志位，置为1时换弹完成
uint8_t Finish_Fire_Flag = 0;                           // 开火完成标志位，置为1时投球完成
uint8_t Adjusting_Fire_Angle_Flag = 0;                  // 微调发射丝杠位置标志位，置为1时微调发射丝杠位置
float data2send[8] = {0};
uint8_t testbbb = 0;
Competition_Mode_ENUM Competition_Mode = Competition_Mode_None; // 竞赛模式,这是代码大和谐的关键
extern uint8_t Can_1_Data[16];
extern Coordinate_Speed_Struct i;
extern Route_STU Route_Status;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Route */
osThreadId_t RouteHandle;
const osThreadAttr_t Route_attributes = {
  .name = "Route",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Shoot */
osThreadId_t ShootHandle;
const osThreadAttr_t Shoot_attributes = {
  .name = "Shoot",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Reload */
osThreadId_t ReloadHandle;
const osThreadAttr_t Reload_attributes = {
  .name = "Reload",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void RoutTask(void *argument);
void ShootTask(void *argument);
void ReloadTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Route */
  RouteHandle = osThreadNew(RoutTask, NULL, &Route_attributes);

  /* creation of Shoot */
  ShootHandle = osThreadNew(ShootTask, NULL, &Shoot_attributes);

  /* creation of Reload */
  ReloadHandle = osThreadNew(ReloadTask, NULL, &Reload_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;)
  {
    if (Competition_Mode == Competition_Mode_Dribble_Preliminary)
    {
      // 运球预选赛
    }
    else if (Competition_Mode == Competition_Mode_Shoot_Preliminary)
    {
      // 投球预选赛
    }
    else if (Competition_Mode == Competition_Mode_Final)
    {
      // 正赛
    }
    // /*===================================================================================================================
    //                 车身自瞄，车身自瞄期间，车的朝向只能能处于[-90, 90]度之间（即只能朝向对面半场）
    // =====================================================================================================================*/
    // if (Teaching_Pendant.Automatic_Aiming_Switch == 1 && Teaching_Pendant.Automatic_Switch == 0)
    // {
    //   //只有当手动路径时才能开启自瞄，防止自动路径时自瞄导致车身角度不对
    //   if (Computer_Vision_Data.Camera.Kinect.Z != 0)
    //   {
    //     // 通过摄像机实现自瞄（摄像机越往右面像素点值越大）
    //     PID_Calculate_Positional(&Automatic_Aiming_PID, Computer_Vision_Data.Camera.Kinect.X, 0);
    //     Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vw = Automatic_Aiming_PID.Output;
    //   }
    //   else
    //   {
    //     // 通过坐标位置信息来实现自瞄
    //     Data_For_Automatic_Aiming.X = Basket_Position.X - World_Coordinate_System_NowPos.X;
    //     Data_For_Automatic_Aiming.Y = Basket_Position.Y - World_Coordinate_System_NowPos.Y;
    //     Data_For_Automatic_Aiming.W = atanf(Data_For_Automatic_Aiming.X / Data_For_Automatic_Aiming.Y);
    //     Data_For_Automatic_Aiming.W = Data_For_Automatic_Aiming.W * CHANGE_TO_ANGLE;
    //     PID_Calculate_Positional(&Automatic_Aiming_PID, World_Coordinate_System_NowPos.W, -Data_For_Automatic_Aiming.W);
    //     Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vw = Automatic_Aiming_PID.Output;
    //   }
    // }
    // else
    // {
    //   // 如果自瞄关了，车身角度控制由手柄接管
    //   Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vw = Teaching_Pendant.W;
    // }
    /*===================================================================================================================
                                                  运球预选赛相关
    =====================================================================================================================*/
    osDelay(2);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_RoutTask */
/**
 * @brief Function implementing the Route thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_RoutTask */
void RoutTask(void *argument)
{
  /* USER CODE BEGIN RoutTask */
  /* Infinite loop */
  for (;;)
  {
    labiao(2);
    // 手动路径相关
    if (Teaching_Pendant_Data.Automatic_Switch == 1)
    {
      Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vx = Teaching_Pendant_Data.Joystick_V.Vx;
      Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vy = Teaching_Pendant_Data.Joystick_V.Vy;
      Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vw = Teaching_Pendant_Data.Joystick_V.Vw;
    }
    // 检查是否到两个视觉识别点的函数，需要一直跑来检测
    Check_Near_Vision_Points(&Vision_Point_Flag, 20);
    // Dribble_Pre_Competition();
    Dribble_Twice();
    //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET); // 气泵停止吸气
    /*===================================================================================================================
                                                  运球预选赛相关
    =====================================================================================================================*/
    if (Competition_Mode == Competition_Mode_Dribble_Preliminary)
    {
      Dribble_Pre_Competition();
    }
    osDelay(2000);
  }
  /* USER CODE END RoutTask */
}

/* USER CODE BEGIN Header_ShootTask */
/**
 * @brief Function implementing the Shoot thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ShootTask */
void ShootTask(void *argument)
{
  /* USER CODE BEGIN ShootTask */
  /* Infinite loop */
  for (;;)
  {
    // data2send[0] = Speed_Data_From_Teaching_Pendant.Vy;
    // data2send[1] = Teaching_Pendant_Data.Joystick_V.Vy;
    // data2send[2] = Teaching_Pendant_Data.Joystick_V.Vw;
    // data2send[3] = Trigger_Angle;
    // Usart_Send_To_Show32(&huart7, data2send);
    if (Start_Pass_Ball_From_Dribble_To_Shoot_Flag)
    {
      // 将球从运球装置转移到投球装置上
      // Dribble_Structure_In_Medium_Flag = Pass_Ball_From_Dribble_To_Shoot(&Ball_On_Shoot_Structure_Flag);
      Pass_Ball_From_Dribble_To_Shoot(&Ball_On_Shoot_Structure_Flag);
      Start_Pass_Ball_From_Dribble_To_Shoot_Flag = 0;
    }
    osDelay(2);
  }
  /* USER CODE END ShootTask */
}

/* USER CODE BEGIN Header_ReloadTask */
/**
 * @brief Function implementing the Reload thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ReloadTask */
void ReloadTask(void *argument)
{
  /* USER CODE BEGIN ReloadTask */
  /* Infinite loop */
  for (;;)
  {
    // 这个任务里面写发射装置的相关机构
    if (Reload_Flag)
    {
      // 在换弹时A1拉丝杠到最上面位置，然后拉到三分线投球的位置
      // 这个函数会一直卡在这里直到换弹完成
      Finish_Reload_Flag = Reload();
      Reload_Flag = 0;
    }
    // Adjusting_Fire_Angle_Flag为1的时候丝杠的控制权交给操手
    //  else if(!Reload_Flag && !Adjusting_Fire_Angle_Flag)
    //  {
    //    //不换弹的其他时间不断计算，A1时刻在跑
    //    Calculate_Fire_Position(Competition_Mode_Shoot_Preliminary); // 计算A1拉丝杠到的位置
    //  }
    //  在程序里面自动开火或者操作手根据情况手动开火，两种有一个置为1直接开火
    if (Teaching_Pendant.Fire == -1 || Teaching_Pendant_Data.Fire == -1)
    {
      Finish_Fire_Flag = 0; // 开火前将开火完成标志位置为0
      Finish_Fire_Flag = Fire();
      Teaching_Pendant.Fire = 0;      // 开火后将开火标志位置为0
      Teaching_Pendant_Data.Fire = 0; // 开火后将开火标志位置为0
    }
    osDelay(2);
  }
  /* USER CODE END ReloadTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

