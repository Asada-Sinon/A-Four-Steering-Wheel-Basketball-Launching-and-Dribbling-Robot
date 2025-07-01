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
#define Basket_Hight 2553;                   // ���ǳ�������ĸ߶�+����뾶
#define CHANGE_TO_RADIAN (0.01745329251994f) // �Ƕ���ת��Ϊ������ϵ��
#define CHANGE_TO_ANGLE (57.29577951308232f) // ������ת��Ϊ�Ƕ���ϵ��
// uint8_t Last_Route = 0;                                           // ���������Resetʱ�����ϴδ�����ĸ���
uint8_t Vision_Point_Flag = 0;                          // �����Ӿ�ʶ���1ʱ��Ϊ1�������Ӿ�ʶ���2ʱ��Ϊ2������ʱ�䶼��0
uint8_t Start_Pass_Ball_From_Dribble_To_Shoot_Flag = 0; // ��ʼ���������װ��ת�Ƶ�Ͷ��װ���ϱ�־λ����Ϊ1ʱִ��
uint8_t Ball_On_Shoot_Structure_Flag = 0;               // ���ڷ���װ���ϱ�־λ����Ϊ1ʱ���ڷ���װ����
uint8_t Dribble_Structure_In_Medium_Flag = 0;           // ����װ�����м�λ��
uint8_t Reload_Flag = 0;                                // ������־λ����Ϊ1ʱִ�л���
uint8_t Finish_Reload_Flag = 0;                         // ������ɱ�־λ����Ϊ1ʱ�������
uint8_t Finish_Fire_Flag = 0;                           // ������ɱ�־λ����Ϊ1ʱͶ�����
uint8_t Adjusting_Fire_Angle_Flag = 0;                  // ΢������˿��λ�ñ�־λ����Ϊ1ʱ΢������˿��λ��
float data2send[8] = {0};
uint8_t testbbb = 0;
Competition_Mode_ENUM Competition_Mode = Competition_Mode_None; // ����ģʽ,���Ǵ�����г�Ĺؼ�
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
      // ����Ԥѡ��
    }
    else if (Competition_Mode == Competition_Mode_Shoot_Preliminary)
    {
      // Ͷ��Ԥѡ��
    }
    else if (Competition_Mode == Competition_Mode_Final)
    {
      // ����
    }
    // /*===================================================================================================================
    //                 �������飬���������ڼ䣬���ĳ���ֻ���ܴ���[-90, 90]��֮�䣨��ֻ�ܳ������볡��
    // =====================================================================================================================*/
    // if (Teaching_Pendant.Automatic_Aiming_Switch == 1 && Teaching_Pendant.Automatic_Switch == 0)
    // {
    //   //ֻ�е��ֶ�·��ʱ���ܿ������飬��ֹ�Զ�·��ʱ���鵼�³���ǶȲ���
    //   if (Computer_Vision_Data.Camera.Kinect.Z != 0)
    //   {
    //     // ͨ�������ʵ�����飨�����Խ���������ص�ֵԽ��
    //     PID_Calculate_Positional(&Automatic_Aiming_PID, Computer_Vision_Data.Camera.Kinect.X, 0);
    //     Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vw = Automatic_Aiming_PID.Output;
    //   }
    //   else
    //   {
    //     // ͨ������λ����Ϣ��ʵ������
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
    //   // ���������ˣ�����Ƕȿ������ֱ��ӹ�
    //   Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vw = Teaching_Pendant.W;
    // }
    /*===================================================================================================================
                                                  ����Ԥѡ�����
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
    // �ֶ�·�����
    if (Teaching_Pendant_Data.Automatic_Switch == 1)
    {
      Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vx = Teaching_Pendant_Data.Joystick_V.Vx;
      Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vy = Teaching_Pendant_Data.Joystick_V.Vy;
      Route_Status.Coordinate_System.Robot_Coordinate_System_V.Vw = Teaching_Pendant_Data.Joystick_V.Vw;
    }
    // ����Ƿ������Ӿ�ʶ���ĺ�������Ҫһֱ�������
    Check_Near_Vision_Points(&Vision_Point_Flag, 20);
    // Dribble_Pre_Competition();
    Dribble_Twice();
    //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET); // ����ֹͣ����
    /*===================================================================================================================
                                                  ����Ԥѡ�����
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
      // ���������װ��ת�Ƶ�Ͷ��װ����
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
    // �����������д����װ�õ���ػ���
    if (Reload_Flag)
    {
      // �ڻ���ʱA1��˿�ܵ�������λ�ã�Ȼ������������Ͷ���λ��
      // ���������һֱ��������ֱ���������
      Finish_Reload_Flag = Reload();
      Reload_Flag = 0;
    }
    // Adjusting_Fire_Angle_FlagΪ1��ʱ��˿�ܵĿ���Ȩ��������
    //  else if(!Reload_Flag && !Adjusting_Fire_Angle_Flag)
    //  {
    //    //������������ʱ�䲻�ϼ��㣬A1ʱ������
    //    Calculate_Fire_Position(Competition_Mode_Shoot_Preliminary); // ����A1��˿�ܵ���λ��
    //  }
    //  �ڳ��������Զ�������߲����ָ�������ֶ�����������һ����Ϊ1ֱ�ӿ���
    if (Teaching_Pendant.Fire == -1 || Teaching_Pendant_Data.Fire == -1)
    {
      Finish_Fire_Flag = 0; // ����ǰ��������ɱ�־λ��Ϊ0
      Finish_Fire_Flag = Fire();
      Teaching_Pendant.Fire = 0;      // ����󽫿����־λ��Ϊ0
      Teaching_Pendant_Data.Fire = 0; // ����󽫿����־λ��Ϊ0
    }
    osDelay(2);
  }
  /* USER CODE END ReloadTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

