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
Competition_Mode_ENUM Competition_Mode = Competition_Mode_Dribble_Preliminary; // 竞赛模式，默认运球预选赛
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
    // if (a[0] == 1)
    // {
    //   // Chassis_Line_Route(a[1],a[2],a[3],1500,5000,2000,50,2000,0.01,0.4,100);//娑擃厽顔屾径褎顩?4000瀹革箑褰哥捄婵堫瀲
    //   // Chassis_Line_Route(a[1],a[2],a[3],1500,14000,2000,1500,2000,0.01,0.4,100);//妤傛﹢?鐔兼毐濞堜絻鐭惧??7000瀹革箑褰哥捄婵堫瀲
    //   // a[0]=0;
    // }
    /*===================================================================================================================
                                                  路径规划相关
    =====================================================================================================================*/
    // if (Teaching_Pendant.Automatic_Switch == 1)
    // {
    //   // 自动路径相关
    //   if(Teaching_Pendant.Route_Type == Route_Type1)
    //   {
    //     Chassis_Line_Route(2000,2000,0,1500,5000,2000,50,2000,0.01,0.4,100);
    //   }
    // }
    // else
    // {
    // 手动路径相关
    Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vx = Teaching_Pendant_Data.Joystick_V.Vx;
    Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vy = Teaching_Pendant_Data.Joystick_V.Vy;
    Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vw = Teaching_Pendant_Data.Joystick_V.Vw;
    // }
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
    /*if (Teaching_Pendant_Data.Automatic_Switch && Teaching_Pendant.Automatic_Switch)
    {
      // 在运球预选赛，打开自动路径开关即开始跑全自动流程
      if (Teaching_Pendant.Route_Type == Route_Type_0 && !Teaching_Pendant_Data.Reset_Confirm)
      {
        // 起始点到S&E点，长路径
        Chassis_Line_Route(2487, -2157, 0, 1500, 11000, 500, 50, 2000, 0.01, 0.5, 100);
        Dribble_Twice();                            // 开启运球
        Teaching_Pendant.Route_Type = Route_Type_1; // 运球后自动切换到第二个点
      }
      if (Teaching_Pendant.Route_Type == Route_Type_1 && !Teaching_Pendant_Data.Reset_Confirm)
      {
        // S&E点到第二个点，短路径
        Chassis_Line_Route(1758, -2981, 0, 1500, 4000, 800, 50, 2000, 0.01, 0.4, 100);
        Dribble_Twice();                            // 开启运球
        Teaching_Pendant.Route_Type = Route_Type_2; // 运球后自动切换到第三个点
      }
      if (Teaching_Pendant.Route_Type == Route_Type_2 && !Teaching_Pendant_Data.Reset_Confirm)
      {
        // 第二个点到第三个点，中路径
        Chassis_Line_Route(1783, -4610, 0, 1500, 6000, 500, 50, 2000, 0.01, 0.4, 100);
        Dribble_Twice();                            // 开启运球
        Teaching_Pendant.Route_Type = Route_Type_3; // 运球后自动切换到第四个点
      }
      if (Teaching_Pendant.Route_Type == Route_Type_3 && !Teaching_Pendant_Data.Reset_Confirm)
      {
        // 第三个点到第四个点，中路径
        Chassis_Line_Route(3758, -4582, 0, 1500, 7000, 500, 50, 2000, 0.01, 0.4, 100);
        Dribble_Twice();                            // 开启运球
        Teaching_Pendant.Route_Type = Route_Type_4; // 运球后自动切换到第五个点
      }
      if (Teaching_Pendant.Route_Type == Route_Type_4 && !Teaching_Pendant_Data.Reset_Confirm)
      {
        // 第四个点到第五个点，中路径
        Chassis_Line_Route(5778, -4574, 0, 1500, 7000, 500, 50, 2000, 0.01, 0.4, 100);
        Dribble_Twice();                            // 开启运球
        Teaching_Pendant.Route_Type = Route_Type_5; // 运球后自动切换到第六个点
      }
      if (Teaching_Pendant.Route_Type == Route_Type_5 && !Teaching_Pendant_Data.Reset_Confirm)
      {
        // 第五个点到第六个点，中路径
        Chassis_Line_Route(5821, -2887, 0, 1500, 6000, 500, 50, 2000, 0.01, 0.4, 100);
        Dribble_Twice();                            // 开启运球
        Teaching_Pendant.Route_Type = Route_Type_6; // 运球后自动切换到第七个点
      }
      if (Teaching_Pendant.Route_Type == Route_Type_6 && !Teaching_Pendant_Data.Reset_Confirm)
      {
        // 第六个点到第七个点，短路径
        Chassis_Line_Route(5005, -2119, 0, 1500, 4000, 800, 50, 2000, 0.01, 0.4, 100);
        Dribble_Twice();                            // 开启运球
        Teaching_Pendant.Route_Type = Route_Type_7;
      }
      if (Teaching_Pendant.Route_Type == Route_Type_7 && !Teaching_Pendant_Data.Reset_Confirm)
      {
        // 第七个点到S&E点，直接冲过去，最后一个点不运球
        Chassis_Line_Route(1000, -2157, 0, 1500, 7000, 500, 50, 2000, 0.01, 0.4, 100);
        Teaching_Pendant.Route_Type = Route_Type_8;
      }
      if (Teaching_Pendant.Route_Type == Route_Type_8 && !Teaching_Pendant_Data.Reset_Confirm)
      {
        // S&E点左侧到出发点，此时已完赛，回出发点方便调试
        Chassis_Line_Route(200, -200, 0, 1500, 5000, 1500, 50, 2000, 0.01, 0.4, 100);
        Teaching_Pendant.Route_Type = Route_Type_0;
        Teaching_Pendant.Automatic_Switch = 0;//跳出循环
      }

      if (Teaching_Pendant.Reset_Confirm && Teaching_Pendant_Data.Reset_Confirm)
      {
        Last_Route = Teaching_Pendant.Route_Type;//这里把Last_Route赋值为上一次的路径常量点
        Teaching_Pendant.Route_Type = Route_Type_Reset;
      }
      // 如果运球运丢了，直接回reset点，长路径,操作手在看到车往回走后即可松手
      if (Teaching_Pendant.Route_Type == Route_Type_Reset)
      {
        Teaching_Pendant.Reset_Confirm = 0;
        Chassis_Line_Route(291, -2129, -90, 1500, 7000, 500, 50, 2000, 0.01, 0.4, 100);//这里回那两个视觉点
        Teaching_Pendant.Route_Type = Last_Route;//这里从这两个视觉点回失误点前一个点时，那个参数要用Nearest_Vision_Point_Route的参数
        Teaching_Pendant.Reset_Confirm = 1;
      }
    }*/
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
    // 检查是否到两个视觉识别点的函数，需要一直跑来检测
    Check_Near_Vision_Points(&Vision_Point_Flag, 20);
    //Dribble_Pre_Competition();
    // Dribble_Twice()
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET); // 气泵停止吸气
    //Chassis_Line_Route(2458, -2204, -90, 1500, 3000, 1700, 50, 500, 0.01f, 0.6f, 100, 25, 10);
    // FDCAN_Send_Data(&hfdcan1, 0x03F, Can_2_Data, &i, &Order_To_Subcontroller);
    /*===================================================================================================================
                                                  投球预选赛相关
    =====================================================================================================================*/
    osDelay(2);
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

