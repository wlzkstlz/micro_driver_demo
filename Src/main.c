/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/***********************【Encoder】**************************/
#define _ENCODER_CYCLE_ 60000
#define _ENCODER_RESET_VALUE_ 30000
uint32_t g_Encoder = _ENCODER_RESET_VALUE_;
/***********************【Encoder end】**************************/

/***********************【Speed Sense】**************************/
uint32_t g_TCount_Repeats = 0;
double g_Wsense = 0.0f; //radian unit
/***********************【Speed Sense end】**************************/

/***********************【PID Controller】**************************/
uint8_t gb_Release_Motor = 1;
uint8_t g_PWM_Dir = 0; //0 stands for anticlockwise
uint32_t g_PWM_Out = 0;
#define _PWM_CYCLE_FULL_ 2000 //修改定时器周期时，记得同步修改这里！
#define _PWM_CYCLE_SAFE_ (_PWM_CYCLE_FULL_ - 100)
#define _PWM_CYCLE_OFFSET_ 200 //400

#define _PWM_BASE_SCALE_ (_PWM_CYCLE_FULL_ * (1.0 / (2.0 * 3.1416 * 4500.0 * 7.2 / 60.0)))
double g_Vset = 0.0; //-123.0; //对应15000rpm

double g_Err_Pre = 0;
float g_PWM = 0;
const float g_Kp = 1.0 * _PWM_BASE_SCALE_;
const float g_Ki = 0.05 * g_Kp;
const float g_Tscale = 1.0;

/***********************【PID Controller end】**************************/

/***********************【Block Boost】**************************/
uint8_t gb_Blocked = 0;
uint16_t gn_Block_Boost_Count = 0;
uint8_t gn_Block_Boost_Offset = 0;
#define _BOOST_COUNTS_ (((uint32_t)(4 * 9 * 1) / (uint32_t)6) * 6) //要求?????6的???数

//TODO : 将拖动电压设置为可以pwm调节,以缓解启动发热问?????

/***********************【Block Boost End】**************************/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/***********************【Debug Info】**************************/
void send_debug_info_pwm_w(int16_t pwm, int32_t encoder, float w)
{
  const size_t data_length = 1 + sizeof(int16_t) + sizeof(int32_t) + sizeof(float) + 2;
  uint8_t data[data_length];
  data[0] = 0xa5;
  data[data_length - 2] = 0;
  data[data_length - 1] = 0x5a;

  memcpy(data + 1, (uint8_t *)(&pwm), sizeof(int16_t));
  memcpy(data + 1 + sizeof(int16_t), (uint8_t *)(&encoder), sizeof(int32_t));
  memcpy(data + 1 + sizeof(int16_t) + sizeof(int32_t), (uint8_t *)(&w), sizeof(float));

  for (size_t i = 1; i < data_length - 2; i++)
  {
    data[data_length - 2] += data[i];
  }

  HAL_UART_Transmit(&huart1, data, data_length, 0xFFFF);
}

/***************************【can bus】********************************/
CAN_TxHeaderTypeDef canTxMsg;
uint32_t tx_mail_box;
uint8_t can_data[8] = {'c', 'a', 'n', ' ', 't', 'e', 's', 't'};
void send_can_bus()
{
  canTxMsg.StdId = 0x123;
  canTxMsg.RTR = CAN_RTR_DATA;
  canTxMsg.IDE = CAN_ID_STD;
  canTxMsg.DLC = 8;

  HAL_StatusTypeDef ret = HAL_CAN_AddTxMessage(&hcan, &canTxMsg, can_data, &tx_mail_box);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  //尽可能快速地将mosfet驱动 gpio设置低电平！
  GPIOA->BSRR = (uint32_t)(PWM_AP_Pin | PWM_BP_Pin | PWM_CP_Pin) << 16u;
  GPIOB->BSRR = (uint32_t)(PWM_AN_Pin | PWM_BN_Pin | PWM_CN_Pin) << 16u;
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
  MX_CAN_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  //CAN_User_Init(&hcan);
  HAL_StatusTypeDef HAL_Status = HAL_CAN_Start(&hcan); //开启CAN
  if (HAL_Status != HAL_OK)
  {
    int i = 9;
    i++;
    //printf("开启CAN失败\r\n");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //debug step1 !!!!!
  gb_Release_Motor = 1; //0;
  g_Vset = 0;           //150;

  int i = 0;
  while (1)
  {
    HAL_Delay(2);
    i++;

    if (i % 10 == 0)
    {
      //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_SET);
    }
    else if (i % 10 == 5)
    {
      //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_RESET);
    }

    //Send Debug Info
    g_Encoder = __HAL_TIM_GET_COUNTER(&htim2);
    //send_debug_info_pwm_w(g_PWM_Out, g_Encoder, g_Wsense);

    //debug can bus
    //HAL_CAN_Transmit(&hcan, 10);
    send_can_bus();
    //CANx_SendNormalData(&hcan, 0x012, can_data, 8);
//debug step2 !!!!!
#if 0
    static uint8_t pingpong = 0;
    if (pingpong == 0)
    {
      if (g_Encoder < 1000 + _ENCODER_RESET_VALUE_)
      {
        gb_Release_Motor = 0;
        g_Vset = 150;
      }
      else
      {
        pingpong = 1;
      }
    }
    else if (pingpong == 1)
    {
      if (g_Encoder > _ENCODER_RESET_VALUE_)
      {
        gb_Release_Motor = 0;
        g_Vset = -150;
      }
      else
      {
        pingpong = 0;
      }
    }
#endif

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//pwm 占空比高电平结束回调函数，在这里清零电机驱动
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == (&htim1))
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      if (gn_Block_Boost_Count > 0)
      {
        return;
      }

      HAL_GPIO_WritePin(GPIOA, PWM_AP_Pin | PWM_BP_Pin | PWM_CP_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, PWM_AN_Pin | PWM_BN_Pin | PWM_CN_Pin, GPIO_PIN_RESET);
    }
  }
}

static const uint16_t table_set_PortA[16] =
    {
        PWM_BP_Pin,
        PWM_BP_Pin,
        //---------------------------------
        0,
        //---------------------------------//
        PWM_AP_Pin,
        PWM_CP_Pin,
        //---------------------------------
        0,
        //---------------------------------//
        PWM_CP_Pin,
        PWM_AP_Pin,
        ///////////////////////////////////////////////
        PWM_AP_Pin,
        PWM_CP_Pin,
        //---------------------------------
        0,
        //---------------------------------//
        PWM_CP_Pin,
        PWM_AP_Pin,
        //---------------------------------
        0,
        //---------------------------------//
        PWM_BP_Pin,
        PWM_BP_Pin};

static const uint16_t table_set_PortB[16] =
    {
        PWM_AN_Pin,
        PWM_CN_Pin,
        //---------------------------------
        0,
        //---------------------------------//
        PWM_CN_Pin,
        PWM_AN_Pin,
        //---------------------------------
        0,
        //---------------------------------//
        PWM_BN_Pin,
        PWM_BN_Pin,
        ///////////////////////////////////////////////
        PWM_BN_Pin,
        PWM_BN_Pin,
        //---------------------------------
        0,
        //---------------------------------//
        PWM_AN_Pin,
        PWM_CN_Pin,
        //---------------------------------
        0,
        //---------------------------------//
        PWM_CN_Pin,
        PWM_AN_Pin};

static const uint16_t table_reset_PortA[16] =
    {
        PWM_AP_Pin | PWM_CP_Pin,
        PWM_AP_Pin | PWM_CP_Pin,
        //---------------------------------
        PWM_AP_Pin | PWM_BP_Pin | PWM_CP_Pin,
        //---------------------------------//
        PWM_BP_Pin | PWM_CP_Pin,
        PWM_AP_Pin | PWM_BP_Pin,
        //---------------------------------
        PWM_AP_Pin | PWM_BP_Pin | PWM_CP_Pin,
        //---------------------------------//
        PWM_AP_Pin | PWM_BP_Pin,
        PWM_BP_Pin | PWM_CP_Pin,
        ///////////////////////////////////////////////////////////
        PWM_BP_Pin | PWM_CP_Pin,
        PWM_AP_Pin | PWM_BP_Pin,
        //---------------------------------
        PWM_AP_Pin | PWM_BP_Pin | PWM_CP_Pin,
        //---------------------------------//
        PWM_AP_Pin | PWM_BP_Pin,
        PWM_BP_Pin | PWM_CP_Pin,
        //---------------------------------
        PWM_AP_Pin | PWM_BP_Pin | PWM_CP_Pin,
        //---------------------------------//
        PWM_AP_Pin | PWM_CP_Pin,
        PWM_AP_Pin | PWM_CP_Pin};
static const uint16_t table_reset_PortB[16] =
    {
        PWM_BN_Pin | PWM_CN_Pin,
        PWM_AN_Pin | PWM_BN_Pin,
        //---------------------------------
        PWM_AN_Pin | PWM_BN_Pin | PWM_CN_Pin,
        //---------------------------------//
        PWM_AN_Pin | PWM_BN_Pin,
        PWM_BN_Pin | PWM_CN_Pin,
        //---------------------------------
        PWM_AN_Pin | PWM_BN_Pin | PWM_CN_Pin,
        //---------------------------------//
        PWM_AN_Pin | PWM_CN_Pin,
        PWM_AN_Pin | PWM_CN_Pin,
        ///////////////////////////////////////////////////////////
        PWM_AN_Pin | PWM_CN_Pin,
        PWM_AN_Pin | PWM_CN_Pin,
        //---------------------------------
        PWM_AN_Pin | PWM_BN_Pin | PWM_CN_Pin,
        //---------------------------------//
        PWM_BN_Pin | PWM_CN_Pin,
        PWM_AN_Pin | PWM_BN_Pin,
        //---------------------------------
        PWM_AN_Pin | PWM_BN_Pin | PWM_CN_Pin,
        //---------------------------------//
        PWM_AN_Pin | PWM_BN_Pin,
        PWM_BN_Pin | PWM_CN_Pin};

static const uint8_t boost_table_offset[8] =
    {
        //红绿?????
        5, //0 0 0
        4, //0 0 1
        0, //xxx
        3, //0 1 1
        0, //1 0 0
        0, //xxx
        1, //1 1 0
        2  //1 1 1
};

static const uint16_t boost_table_set_PortA[6] =
    {
        PWM_BP_Pin, //红x ?????1 ?????0
        PWM_BP_Pin, //?????0 ?????1 蓝x
        PWM_CP_Pin, //?????0 绿x ?????1
        PWM_CP_Pin, //红x ?????0 ?????1
        PWM_AP_Pin, //?????1 ?????0 蓝x
        PWM_AP_Pin, //?????1 绿x ?????0
};

static const uint16_t boost_table_set_PortB[6] =
    {
        PWM_CN_Pin, //红x ?????1 ?????0
        PWM_AN_Pin, //?????0 ?????1 蓝x
        PWM_AN_Pin, //?????0 绿x ?????1
        PWM_BN_Pin, //红x ?????0 ?????1
        PWM_BN_Pin, //?????1 ?????0 蓝x
        PWM_CN_Pin, //?????1 绿x ?????0
};

static const uint16_t boost_table_reset_PortA[6] =
    {
        PWM_AP_Pin | PWM_CP_Pin, //红x ?????1 ?????0
        PWM_AP_Pin | PWM_CP_Pin, //?????0 ?????1 蓝x
        PWM_AP_Pin | PWM_BP_Pin, //?????0 绿x ?????1
        PWM_AP_Pin | PWM_BP_Pin, //红x ?????0 ?????1
        PWM_BP_Pin | PWM_CP_Pin, //?????1 ?????0 蓝x
        PWM_BP_Pin | PWM_CP_Pin, //?????1 绿x ?????0
};

static const uint16_t boost_table_reset_PortB[6] =
    {
        PWM_AN_Pin | PWM_BN_Pin, //红x ?????1 ?????0
        PWM_BN_Pin | PWM_CN_Pin, //?????0 ?????1 蓝x
        PWM_BN_Pin | PWM_CN_Pin, //?????0 绿x ?????1
        PWM_AN_Pin | PWM_CN_Pin, //红x ?????0 ?????1
        PWM_AN_Pin | PWM_CN_Pin, //?????1 ?????0 蓝x
        PWM_AN_Pin | PWM_BN_Pin, //?????1 绿x ?????0
};

uint8_t GetHallState()
{
  GPIO_PinState sa = 1 - HAL_GPIO_ReadPin(HSENSOR_A_GPIO_Port, HSENSOR_A_Pin);
  GPIO_PinState sb = 1 - HAL_GPIO_ReadPin(HSENSOR_B_GPIO_Port, HSENSOR_B_Pin);
  GPIO_PinState sc = 1 - HAL_GPIO_ReadPin(HSENSOR_C_GPIO_Port, HSENSOR_C_Pin);
  uint8_t s_state = (((uint8_t)sa) << 2) | (((uint8_t)sb) << 1) | (((uint8_t)sc) << 0);
  return s_state;
}

//64MHz时钟条件下，约等于100ns
void DelayDeadTime()
{
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();

  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
}
//TODO 定时器计数溢出中断,htim1在这里根据位置传感器决定驱动电流方向,
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == (&htim1))
  {
    if (gn_Block_Boost_Count > 0 || gb_Release_Motor > 0)
    {
      return;
    }

    //查表输出
    const uint8_t s_state = GetHallState() + g_PWM_Dir * 8;

    //先reset，再set!
    HAL_GPIO_WritePin(GPIOA, table_reset_PortA[s_state], GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, table_reset_PortB[s_state], GPIO_PIN_RESET);

    DelayDeadTime();

    HAL_GPIO_WritePin(GPIOA, table_set_PortA[s_state], GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, table_set_PortB[s_state], GPIO_PIN_SET);
  }

  else if (htim == (&htim3)) //1k hz
  {
    //0.0 release motor
    if (gb_Release_Motor > 0)
      return;
    //1.0 block proccess
    if (gb_Blocked >= 1 && gn_Block_Boost_Count == 0)
    {
      //gn_Block_Boost_Count = _BOOST_COUNTS_; // START A BOOST PROCCESS...
      uint8_t s_state = GetHallState();
      gn_Block_Boost_Offset = boost_table_offset[s_state];
    }

    if (gn_Block_Boost_Count > 0)
    {
      uint8_t idx;
      if (g_Vset > 0)
        idx = (gn_Block_Boost_Offset + _BOOST_COUNTS_ - gn_Block_Boost_Count) % 6;
      else
        idx = (gn_Block_Boost_Offset + gn_Block_Boost_Count) % 6;
      HAL_GPIO_WritePin(GPIOA, boost_table_reset_PortA[idx], GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, boost_table_reset_PortB[idx], GPIO_PIN_RESET);
      DelayDeadTime();
      HAL_GPIO_WritePin(GPIOA, boost_table_set_PortA[idx], GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, boost_table_set_PortB[idx], GPIO_PIN_SET);

      gn_Block_Boost_Count--;
      return;
    }

    //2.0 pid control
    double Err_Cur = g_Vset - g_Wsense;
    float d_pwm = g_Tscale * (g_Kp * (Err_Cur - g_Err_Pre) + g_Ki * Err_Cur);
    g_PWM += d_pwm;

    float pwm_base = _PWM_BASE_SCALE_ * Err_Cur;
    float pwm_cur = g_PWM + pwm_base;

    if (pwm_cur >= _PWM_CYCLE_SAFE_ - _PWM_CYCLE_OFFSET_)
    {
      pwm_cur = _PWM_CYCLE_SAFE_ - _PWM_CYCLE_OFFSET_ - 1;
    }
    if (pwm_cur <= -(_PWM_CYCLE_SAFE_ - _PWM_CYCLE_OFFSET_))
    {
      pwm_cur = -(_PWM_CYCLE_SAFE_ - _PWM_CYCLE_OFFSET_ - 1);
    }
    g_PWM = pwm_cur - pwm_base;

    g_PWM_Out = abs((int32_t)pwm_cur) + _PWM_CYCLE_OFFSET_;
    g_PWM_Dir = pwm_cur > 0 ? 0 : 1;

    g_Err_Pre = Err_Cur;

    //TODO test
    g_PWM_Dir = 0;
    g_PWM_Out = 8000;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, g_PWM_Out);
  }
  else if (htim == (&htim4)) //about 10Hz
  {
    g_TCount_Repeats++;
    static uint16_t pre_encoder = 0;
    uint16_t cur_encoder = __HAL_TIM_GET_COUNTER(&htim2);
    if (cur_encoder == pre_encoder)
    {
      g_Wsense = 0;

      if (g_Vset > 1e-3 || g_Vset < -1e-3)
      {
        gb_Blocked = 1;
      }
    }
    else
    {
      gb_Blocked = 0;
    }
    pre_encoder = cur_encoder;
  }
}

//这里主要用于计算角速度
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == HSENSOR_C_Pin)
  {
    uint16_t timer_count = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim4));
    uint32_t timer_repeat = g_TCount_Repeats;
    uint8_t cur_polar = HAL_GPIO_ReadPin(HSENSOR_B_GPIO_Port, HSENSOR_B_Pin); //逆时针旋转时，???应?????????1
    static uint16_t pre_timer_count = 0;
    static uint32_t pre_timer_repeat = 0;
    static uint8_t pre_polar = 0;

    uint32_t delta_repeat = timer_repeat - pre_timer_repeat;
    if (delta_repeat > 10) //时间跨度大于1??????
    {
      g_Wsense = 0;
    }
    else
    {
      if (delta_repeat > 1)
      {
        delta_repeat = delta_repeat - 1; //delta_repeat == 1 的情况，已经被uint16_t 的补数机制覆盖到！
      }
      else
      {
        delta_repeat = 0;
      }

      uint16_t delta_count = timer_count - pre_timer_count;
      double delta_t = ((double)delta_count + delta_repeat * 65536) / 640000.0f; //因为预分频为100
      double delta_angel = 0;

      const double angel_p = 2.0 * 3.1415926 / 6.0;
      if ((pre_polar == 1) && (cur_polar == 1))
      {
        delta_angel = angel_p;
      }
      else if ((pre_polar == 0) && (cur_polar == 0))
      {
        delta_angel = -angel_p;
      }
      else
      {
        delta_angel = 0;
      }

      g_Wsense = delta_angel / delta_t;
    }

    pre_timer_count = timer_count;
    pre_timer_repeat = timer_repeat;
    pre_polar = cur_polar;
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
