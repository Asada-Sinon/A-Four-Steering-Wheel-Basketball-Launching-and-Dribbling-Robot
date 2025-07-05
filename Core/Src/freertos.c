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
#include "A1_Motor.h"
#include "Filter.h"
#include "disk.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
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
float route_Test[6] = {0, 0, 0, 0};                                    // 用于测试的数组
Coordinate_Speed_Struct Word_Coordinate_Speed_For_Gamepad = {0, 0, 0}; // 世界坐标系下零速度
Coordinate_Speed_Struct mingsang_Coordinate_Speed = {0, 0, 0};         // 马铭泽要的坐标系速度
uint8_t Dribble_Pre_Next_point_Flag = 0;
Robot_Route_From_Teaching_Pendant_ENUM Dribble_Route_Type = Route_Type_0;
Robot_Route_From_Teaching_Pendant_ENUM Last_Dribble_Route_Type = Route_Type_0; // 上次运球赛路径类型
uint8_t Route_Executed_Flag[10] = {0};                                         // 索引0不用，1-8对应Route_Type_1到Route_Type_8，9对应Route_Type_Reset
Competition_Mode_ENUM Competition_Mode = Competition_Mode_None;                // 竞赛模式,这是代码大和谐的关键
extern uint8_t Can_1_Data[16];
extern Coordinate_Speed_Struct i;
extern Route_STU Route_Status;

extern float Basketball_Angle;
extern Coordinate_Position_Struct Now_World_Positon;

// 投篮时的手柄状态
Robot_Shooting_From_Teaching_Pendant_ENUM Shooting_Type = Shooting_Catching;
extern uint8_t Fire_Start_Check; // 光电门标志位，当置1时表示发射装置初始化完成
extern float A1_Angle_I_Want;
extern A1Motor g_a1motor;

extern KalmanFilter kr_X; // 卡尔曼滤波器实例
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
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for Route */
osThreadId_t RouteHandle;
const osThreadAttr_t Route_attributes = {
    .name = "Route",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for Shoot */
osThreadId_t ShootHandle;
const osThreadAttr_t Shoot_attributes = {
    .name = "Shoot",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for Reload */
osThreadId_t ReloadHandle;
const osThreadAttr_t Reload_attributes = {
    .name = "Reload",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for Test */
osThreadId_t TestHandle;
const osThreadAttr_t Test_attributes = {
    .name = "Test",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void RoutTask(void *argument);
void ShootTask(void *argument);
void ReloadTask(void *argument);
void TestTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
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

  /* creation of Test */
  TestHandle = osThreadNew(TestTask, NULL, &Test_attributes);

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
    Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vx = Teaching_Pendant_Data.Joystick_V.Vx;
    Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vy = Teaching_Pendant_Data.Joystick_V.Vy;
    // 手柄和标志位任务
    if (Competition_Mode == Competition_Mode_Dribble_Preliminary)
    {
      // 运球预选赛
      if (Dribble_Pre_Next_point_Flag == 0)
      {
        // 上面这个标志位是为了防止拨杆向下时，数字一直加
        if (Teaching_Pendant_Data.Fire == 1)
        {
          // 最左侧拨杆向下，0是无路径，从一开始
          Dribble_Pre_Next_point_Flag = 1;
          if (Dribble_Route_Type != Route_Type_Reset)
          {
            // 这里是保护，正常来讲不会跑到这里，这里防止操作手作死完赛还瞎按，保护操作手
            if (Dribble_Route_Type < Route_Type_8)
            {
              Dribble_Route_Type++;
            }
            else
            {
              Dribble_Route_Type = Route_Type_0; // 回到起始点
            }
          }
          else
          {
            Dribble_Pre_Next_point_Flag = 1;
            Route_Executed_Flag[Last_Dribble_Route_Type] = 0; // 重置上次运球赛路径类型的执行标志位
            Dribble_Route_Type = Last_Dribble_Route_Type;     // 回到上一点
            Route_Executed_Flag[Route_Type_Reset] = 0;        // 也对Reset点进行保护
          }
        }
        if (Teaching_Pendant_Data.Fire == -1)
        {
          // 最左侧拨杆向上，进入reset逻辑
          if (Dribble_Route_Type != Route_Type_Reset)
          {
            Dribble_Pre_Next_point_Flag = 1;
            Dribble_Route_Type = Route_Type_Reset;
          }
          else if (Dribble_Route_Type == Route_Type_Reset)
          {
            Dribble_Pre_Next_point_Flag = 1;
            Route_Executed_Flag[Last_Dribble_Route_Type] = 0; // 重置上次运球赛路径类型的执行标志位
            Dribble_Route_Type = Last_Dribble_Route_Type;     // 回到上一点
            Route_Executed_Flag[Route_Type_Reset] = 0;        // 也对Reset点进行保护
          }
        }
      }
      if (Dribble_Pre_Next_point_Flag == 1)
      {
        if (Teaching_Pendant_Data.Fire == 0)
        {
          Dribble_Pre_Next_point_Flag = 0;
        }
      }
    }
    else if (Competition_Mode == Competition_Mode_Shoot_Preliminary)
    {
      // 投球预选赛
    }
    else if (Competition_Mode == Competition_Mode_Final)
    {
      // 正赛
    }
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
    // 路径任务
    if (Competition_Mode == Competition_Mode_Dribble_Preliminary)
    {
      // 运球预选赛
      if (Dribble_Route_Type == Route_Type_Reset)
      {
        if (Route_Executed_Flag[Route_Type_Reset] == 0)
        {
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET); // 保证reset时气泵一直吸气
          Dribble_Pre_Check_Near_Reset_Points_And_Go();
          Route_Executed_Flag[Route_Type_Reset] = 1;
        }
      }
      if (Dribble_Route_Type == Route_Type_1)
      {
        if (Route_Executed_Flag[Route_Type_1] == 0)
        {
          Dribble_Motor_Angle = DRIBBLE_MOTOR_ANGLE_OUT; // 运球装置拉到最外面
          Keep_Position_Speed(2458, -2204, 0, 10000);
          Last_Dribble_Route_Type = Route_Type_1; // 储存上次运球赛路径类型
          Order_To_Subcontroller.Wheel_Break = 1;
          Dribble_Twice(); // 运球赛运球
          Order_To_Subcontroller.Wheel_Break = 0;
          osDelay(400); // 等待运球完成
          Route_Executed_Flag[Route_Type_1] = 1;
        }
      }
      if (Dribble_Route_Type == Route_Type_2)
      {
        if (Route_Executed_Flag[Route_Type_2] == 0)
        {
          Keep_Position_Speed(1730, -2976, 0, 15000);
          Last_Dribble_Route_Type = Route_Type_2; // 储存上次运球赛路径类型
          Order_To_Subcontroller.Wheel_Break = 1;
          Dribble_Twice(); // 运球赛运球
          Order_To_Subcontroller.Wheel_Break = 0;
          osDelay(400); // 等待运球完成
          Route_Executed_Flag[Route_Type_2] = 1;
        }
      }
      if (Dribble_Route_Type == Route_Type_3)
      {
        if (Route_Executed_Flag[Route_Type_3] == 0)
        {
          Keep_Position_Speed(1741, -4705, 0, 15000);
          Last_Dribble_Route_Type = Route_Type_3; // 储存上次运球赛路径类型
          Order_To_Subcontroller.Wheel_Break = 1;
          Dribble_Twice(); // 运球赛运球
          Order_To_Subcontroller.Wheel_Break = 0;
          osDelay(400); // 等待运球完成
          Route_Executed_Flag[Route_Type_3] = 1;
        }
      }
      if (Dribble_Route_Type == Route_Type_4)
      {
        if (Route_Executed_Flag[Route_Type_4] == 0)
        {
          Keep_Position_Speed(3765, -4666, 0, 12000);
          Last_Dribble_Route_Type = Route_Type_4; // 储存上次运球赛路径类型
          Order_To_Subcontroller.Wheel_Break = 1;
          Dribble_Twice(); // 运球赛运球
          Order_To_Subcontroller.Wheel_Break = 0;
          osDelay(400); // 等待运球完成
          Route_Executed_Flag[Route_Type_4] = 1;
        }
      }
      if (Dribble_Route_Type == Route_Type_5)
      {
        if (Route_Executed_Flag[Route_Type_5] == 0)
        {
          Keep_Position_Speed(5777, -4648, 0, 12000);
          Last_Dribble_Route_Type = Route_Type_5; // 储存上次运球赛路径类型
          Order_To_Subcontroller.Wheel_Break = 1;
          Dribble_Twice(); // 运球赛运球
          Order_To_Subcontroller.Wheel_Break = 0;
          osDelay(400); // 等待运球完成
          Route_Executed_Flag[Route_Type_5] = 1;
        }
      }
      if (Dribble_Route_Type == Route_Type_6)
      {
        if (Route_Executed_Flag[Route_Type_6] == 0)
        {
          Keep_Position_Speed(5745, -2920, 0, 15000);
          Last_Dribble_Route_Type = Route_Type_6; // 储存上次运球赛路径类型
          Order_To_Subcontroller.Wheel_Break = 1;
          Dribble_Twice(); // 运球赛运球
          Order_To_Subcontroller.Wheel_Break = 0;
          osDelay(400); // 等待运球完成
          Route_Executed_Flag[Route_Type_6] = 1;
        }
      }
      if (Dribble_Route_Type == Route_Type_7)
      {
        if (Route_Executed_Flag[Route_Type_7] == 0)
        {
          Keep_Position_Speed(4972, -2175, 0, 15000);
          Last_Dribble_Route_Type = Route_Type_7; // 储存上次运球赛路径类型
          Order_To_Subcontroller.Wheel_Break = 1;
          Dribble_Twice(); // 运球赛运球
          Order_To_Subcontroller.Wheel_Break = 0;
          osDelay(400); // 等待运球完成
          Route_Executed_Flag[Route_Type_7] = 1;
        }
      }
      if (Dribble_Route_Type == Route_Type_8)
      {
        if (Route_Executed_Flag[Route_Type_8] == 0)
        {
          Keep_Position_Speed(1000, -2175, 0, 15000);
          Last_Dribble_Route_Type = Route_Type_8; // 储存上次运球赛路径类型
          Order_To_Subcontroller.Wheel_Break = 1;
          Order_To_Subcontroller.Wheel_Break = 0;
          Route_Executed_Flag[Route_Type_8] = 1;
          Keep_Position_Speed(100, -100, 0, 10000);
        }
      }
    }
    else if (Competition_Mode == Competition_Mode_Shoot_Preliminary)
    {
      // 投球预选赛
      // 手动路径相关
      if (Teaching_Pendant_Data.Automatic_Switch == 1)
      {
        // 直接赋值车身坐标系速度
        // 投球预选赛
        // 手动路径相关
        // 直接赋值车身坐标系速度
        Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vx = Teaching_Pendant_Data.Joystick_V.Vx;
        Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vy = Teaching_Pendant_Data.Joystick_V.Vy;
        //        Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vw = Teaching_Pendant_Data.Joystick_V.Vw;
        // 当手操速度比较小的时候再进行角速度的计算，这里小于50,当R1位置距离出发点有一段距离后再进行角速度的计算，防止撞墙，这里为300mm
        if (fabs(Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vx) < 50 &&
            fabs(Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vy) < 50 &&
            fabs(Final_Now_Pos.X) > 300 && fabs(Final_Now_Pos.Y) > 300)
        {
          Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vw = Automatic_Aiming_W_Calculate(2, 0, 0);
        }
        else
        {
          Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vw = Teaching_Pendant_Data.Joystick_V.Vw;
        }
        // 车身坐标系速度转化为马铭泽要的坐标系速度（车的开始位置有讲究）
        /*---------------------------------------------------------------------------
        |                                                                            |
        |                               半场地图                                      |
        |                                                                            |
        |                                                                            |
        |                                 篮筐                           车（车头朝下）|
        |----------------------------------------------------------------------------*/
        /* 操作手在这里*/
        //      Word_Coordinate_Speed_For_Gamepad = Speed_Coordinate_Transformation(&Teaching_Pendant_Data.Joystick_V, &Word_Coordinate_Speed_For_Gamepad,-Computer_Vision_Data.LiDAR.W);
        //      mingsang_Coordinate_Speed.Vx = -Word_Coordinate_Speed_For_Gamepad.Vx;
        //      mingsang_Coordinate_Speed.Vy = -Word_Coordinate_Speed_For_Gamepad.Vy;
        //      Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vx = mingsang_Coordinate_Speed.Vx;
        //      Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vy = mingsang_Coordinate_Speed.Vy;
        //      Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vw = Automatic_Aiming_W_Calculate(Competition_Mode_Shoot_Preliminary, 0, 0);
      }
    }
    else if (Competition_Mode == Competition_Mode_Final)
    {
      // 正赛
    }
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
    // 如果当前是发射模式
    if (Competition_Mode == Competition_Mode_Shoot_Preliminary)
    {
      // 操作只有最左边拨杆，向下是拉到特定位置，向上是开火
      //  如果发射装置初始化完成
      if (Fire_Start_Check == 1)
      {
        // 如果是勾住模式
        if (Shooting_Type == Shooting_Catching)
        {
          // 钩子锁死，AI上升
          Trigger_Angle = Trigger_ANGLE_LOCK;
          A1_Angle_I_Want = -15;

          // 如果A1电机上升到达了位置, 改变为装弹模式
          if (fabsf(A1_Angle_I_Want - g_a1motor.feedback.Position_Sum) < 0.5f)
          {
            Shooting_Type = Shooting_Reloading;
          }
        }
        // 如果现在是装弹模式,并且角度已经拉到位置
        if (Shooting_Type == Shooting_Reloading && fabsf(A1_Angle_I_Want - g_a1motor.feedback.Position_Sum) < 0.5f)
        {
          // 如果手柄最左边的拨杆向下拨动
          if (Teaching_Pendant_Data.Fire == 1)
          {
            // A1电机向下拉动,并且把更改投篮状态
            A1_Angle_I_Want = testbbb; // 这里是写的那个参数拟合
            Shooting_Type = Shooting_Launching;
          }
        }
        // 如果现在是发射模式，并且角度已经拉到位置
        if (Shooting_Type == Shooting_Launching && fabsf(A1_Angle_I_Want - g_a1motor.feedback.Position_Sum) < 0.5f)
        {
          // 如果手柄最左边的拨杆向上拨动
          if (Teaching_Pendant_Data.Fire == -1)
          {
            // 松开锁死装置，使得篮球发射出去
            Trigger_Angle = Trigger_ANGLE_FIRE;
            Shooting_Type = Shooting_Catching;
            // 等待500ms,会自动进入勾住模式，勾住模式会使得A1上升，钩子锁死
            osDelay(500);
          }
        }
      }
    }
    // if (Start_Pass_Ball_From_Dribble_To_Shoot_Flag)
    // {
    //   // 将球从运球装置转移到投球装置上
    //   // Dribble_Structure_In_Medium_Flag = Pass_Ball_From_Dribble_To_Shoot(&Ball_On_Shoot_Structure_Flag);
    //   Pass_Ball_From_Dribble_To_Shoot(&Ball_On_Shoot_Structure_Flag);
    //   Start_Pass_Ball_From_Dribble_To_Shoot_Flag = 0;
    // }
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
    osDelay(2);
  }
  /* USER CODE END ReloadTask */
}

/* USER CODE BEGIN Header_TestTask */
/**
 * @brief Function implementing the Test thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_TestTask */
void TestTask(void *argument)
{
  /* USER CODE BEGIN TestTask */
  /* Infinite loop */
  for (;;)
  {
    // HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET); // 气泵停止吸气
    // if (route_Test[0] == 1)
    // {
    //   Keep_Position_Speed(route_Test[1], route_Test[2], route_Test[3], 12500);
    //   route_Test[0] = 0; // 重置标志位
    // }
    // labiao(2);
    // 检查是否到两个视觉识别点的函数，需要一直跑来检测
    // Check_Near_Vision_Points(&Vision_Point_Flag, 20);
    // Dribble_Pre_Competition();
    Dribble_Twice();
    // HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET); // 气泵停止吸气
    //    data2send[0] = VESC_Data_From_Subcontroller[0].RPM_From_Subcontroller;
    //     data2send[1] = VESC_Data_From_Subcontroller[1].RPM_From_Subcontroller;
    //     data2send[2] = VESC_Data_From_Subcontroller[2].RPM_From_Subcontroller;
    //     data2send[3] = VESC_Data_From_Subcontroller[3].RPM_From_Subcontroller;
    //     data2send[4] = Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vy;
    //     data2send[5] = Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vw;
    data2send[0] = Computer_Vision_Data.LiDAR.X;
    data2send[1] = Disk_Encoder.Cod.Chassis_Position_From_Disk.X;
    data2send[2] = kr_X.x;
    Usart_Send_To_Show32(&huart7, data2send);
    //     Keep_Position_Speed(100, -400, 0, 15000);
    //     Keep_Position_Speed(600, -400, 0, 15000);
    //     Keep_Position_Speed(600, 100, 0, 15000);
    //     Keep_Position_Speed(100, 100, 0, 15000);
    if (testbbb == 1)
    {
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);    // 小气缸推球
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET); // 气泵停止吸气
    }
    osDelay(2000);
  }
  /* USER CODE END TestTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
