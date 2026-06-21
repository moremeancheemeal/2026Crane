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
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/******************/
/********姣忔cubemx鐢熸垚鍚庨兘闇€瑕佸湪stm32f4xx_it.c閲岄潰鎶奌AL_UART_IRQHandler(&huart1)娉ㄩ噴鎺?******/
/******************/
#include "RC_Task.h"
#include "mytype.h"
#include "CanBus_Task.h"
#include "PID.h"
#include "Paraminit.h"
#include "ControlTask.h" 
#include "MotorControlTask.h"
#include "flow_decode.h"
#include "wit_c_sdk.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "uart_comm.h"
//#include "kalman.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*********閬ユ帶DR16*************/
  uint8_t TIM3_flag=0;
  uint8_t c_flag=0;
	uint8_t ctrl_c_flag=0;
	uint8_t v_flag=0;
	uint8_t ctrl_v_flag=0;
	uint8_t e_flag=0;
	uint8_t f_flag,g_flag=0; 
	uint8_t gimbal_view_chassis_flag=0;
	
	
/********IMU闄€铻轰华**************/
#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
const uint32_t c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);

float fAcc[3], fGyro[3], fAngle[3];
int i;
float fAngle_tmp = 0;

enum{
    
    Angle_Roll = 0,
    Angle_Pitch = 1, 
    Angle_Yaw = 2, 
};
uint8_t IMU_Rx_Buffer = 0;

/*******鍏夋祦璁℃帴鏀?********/
#define MINCOOR 3
uint8_t Flow_Rx_Buffer = 0;
int ret;
uint8_t limit_control_flag = 0;
/*******涓婁綅鏈烘帴鏀?********/
uint8_t uart_rx_buffer[UART_BUFFER_SIZE];
//float speedBuffer[4];
uint8_t stopflag = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t updatePosFlag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void imu_getData()
{
//	CmdProcess();
		if(s_cDataUpdate)
		{
//			for(i = 0; i < 3; i++)
//			{
//				fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
//				fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
//			}
			if(sReg[TEMP] - sReg[Roll] > 10 | sReg[Roll] - sReg[TEMP] > 10)
			{
				fAngle[0] = sReg[Roll] / 32768.0f * 180.0f;
				fAngle_tmp = fAngle[0];
			}
			else   fAngle[0] = fAngle_tmp;
			
			for(i = 1; i<3;i++)
			{
				fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
			}
		   coor.angle = fAngle[Angle_Yaw];
//			if(s_cDataUpdate & ACC_UPDATE)
//			{
//				printf("acc:%.3f %.3f %.3f\r\n", fAcc[0], fAcc[1], fAcc[2]);
//				s_cDataUpdate &= ~ACC_UPDATE;
//			}
//			if(s_cDataUpdate & GYRO_UPDATE)
//			{
//				printf("gyro:%.3f %.3f %.3f\r\n", fGyro[0], fGyro[1], fGyro[2]);
//				s_cDataUpdate &= ~GYRO_UPDATE;
//			}
			if(s_cDataUpdate & ANGLE_UPDATE)
			{
//				printf("angle:%.3f %.3f %.3f\r\n", fAngle[0], fAngle[1], fAngle[2]);
				s_cDataUpdate &= ~ANGLE_UPDATE;
			}
//			if(s_cDataUpdate & MAG_UPDATE)
//			{
//				printf("mag:%d %d %d\r\n", sReg[HX], sReg[HY], sReg[HZ]);
//				s_cDataUpdate &= ~MAG_UPDATE;
//			}
		}
}




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
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_SPI5_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  /* USER CODE BEGIN 2 */
 	CANFilterInit();//锟斤拷锟斤拷CAN通锟斤拷

	ParamInit();//pid param init
	ABoardKey_Init();//DJI A board key init for motor closed-loop test
	
	WitInit(WIT_PROTOCOL_NORMAL, 0x50);//姝ｅ父妯″紡
	WitSerialWriteRegister(SensorUartSend);//娉ㄥ唽涓插彛鍙戦€佸嚱鏁?
	WitRegisterCallBack(SensorDataUpdata);//娉ㄥ唽鏁版嵁鏇存柊鍥炶皟鍑芥暟
	WitDelayMsRegister(Delayms);//娉ㄥ唽寤舵椂鍑芥暟
	// AutoScanSensor();
	HAL_UART_Receive_IT_IDLE(&huart1,UART_Buffer,100); //閬ユ帶鍣ㄤ覆鍙ｅ垵濮嬪寲
	HAL_UART_Receive_IT(&huart6, &Flow_Rx_Buffer ,1);//鍏夋祦璁℃帴鏀朵腑鏂?
	HAL_UART_Receive_IT(&huart7, &IMU_Rx_Buffer, 1);//IMU鎺ユ敹涓柇
	HAL_UART_Receive_IT(&huart8, &uart_rx_buffer[buffer_index], 1);//涓婁綅鏈烘帴鏀朵腑鏂?
	HAL_UART_Transmit_IT(&huart8, &stopflag, 1);//涓婁綅鏈哄彂閫佷腑鏂?

//	kalman_Init();
//	while (!(s_cDataUpdate & ANGLE_UPDATE))
//	{
//		imu_getData();
//	}
	HAL_TIM_Base_Start_IT(&htim3);//1ms瀹氭椂

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		speedBuffer[0] = (float)Chassis.fr_motor_rpm_201;
//	  	speedBuffer[1] = (float)Chassis.fl_motor_rpm_202;
//		speedBuffer[2] = (float)Chassis.rl_motor_rpm_203;
//		speedBuffer[3] = (float)Chassis.rr_motor_rpm_204;

//	  if( (-5 < Chassis.fr_motor_rpm_201 && Chassis.fr_motor_rpm_201 < 5) && 
//		  (-5 < Chassis.fl_motor_rpm_202 && Chassis.fl_motor_rpm_202 < 5) &&
//	      (-5 < Chassis.rl_motor_rpm_203 &&Chassis.rl_motor_rpm_203 < 5) &&
//		  (-5 <Chassis.rr_motor_rpm_204 &&Chassis.rr_motor_rpm_204 < 5))
//		  stopflag = 1;//杞﹀凡缁忛潤姝?
//	 if( (Chassis.fr_motor_rpm_201 == 0) && 
//		  (Chassis.fl_motor_rpm_202==0) &&
//	      (Chassis.rl_motor_rpm_203==0) &&
//		  (Chassis.rr_motor_rpm_204==0))
//		  stopflag = 1;//杞﹀凡缁忛潤姝?
//	 else 
//		  stopflag = 0;
//	  imu_getData();
	  if (coor.flow_x - coor.target_x < 500  &&
			  coor.flow_x - coor.target_x > -500 &&
			  coor.flow_y - coor.target_y < 500  &&
			  coor.flow_y - coor.target_y > -500 &&
			  coor.angle - coor.target_angle < 0.08f&&
			  coor.angle - coor.target_angle > -0.08f)
		  {
			  stopflag = 1;//杞﹀凡缁忛潤姝?
		  }
		  else
		  {
			  stopflag = 0;
		  }
	  	if(TIM3_flag ==1)
		{
			TIM3_flag =0;
			AllTask();
//			 if(RC_UPPER_RIGHT_SW_DOWN)
//			 {
//				 c_flag=1;
//				 LED2_OFF;
			 
//			 }
//			 else 
//			 {
//					c_flag=0;
//					LED2_ON;
//			 }
//			 sen_can_comu_Board2(&hcan2, c_flag,ctrl_c_flag,v_flag,ctrl_v_flag,e_flag,f_flag,g_flag, gimbal_view_chassis_flag);
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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint16_t ms1 = 0,ms2 = 0,ms3=0;
    if(htim->Instance == TIM3)
    {
        
            ms1 = 0;
            TIM3_flag = 1;

	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART6)
	{
		ret = upnotof_parse_char(Flow_Rx_Buffer);
			//302GS浣跨敤鐨刄PIXELS绾厜娴佸崗璁?
		if(!ret)
		{			
			if (ABS(up_flow_data.flow_x_integral) > MINCOOR )
			{	
				 
//				coor.flow_x	+= Kalman_Update(&kf_x, up_flow_data.flow_x_integral);
				coor.flow_x	+= up_flow_data.flow_x_integral;

			}
			if (ABS(up_flow_data.flow_y_integral) > MINCOOR)
			{
					coor.flow_y	+= up_flow_data.flow_y_integral;
			}
		}
		HAL_UART_Receive_IT(&huart6, &Flow_Rx_Buffer, 1);

	}
	if (huart->Instance == UART7)
	{
			WitSerialDataIn(IMU_Rx_Buffer);
			HAL_UART_Receive_IT(&huart7, &IMU_Rx_Buffer, 1);

	} 
	if (huart->Instance == UART8)
	{
		if (uart_rx_buffer[buffer_index] == '!')  // 妫€娴嬪埌甯х粨鏉熺
		{
			uart_rx_buffer[buffer_index] = '\0'; 
			buffer_index = 0;// 閲嶇疆缂撳啿鍖虹储寮?
			UART_ParseCommand(uart_rx_buffer);
		}
		else
		{
			buffer_index++;
			if (buffer_index >= UART_BUFFER_SIZE)
			{
				buffer_index = 0;
			}
		}
			HAL_UART_Receive_IT(&huart8, &uart_rx_buffer[buffer_index], 1);

	}
	
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance  == UART8)
	{
	HAL_UART_Transmit_IT(&huart8, &stopflag, 1);//涓婁綅鏈哄彂閫佷腑鏂?
	}
}

void CopeCmdData(unsigned char ucData)
{
	static unsigned char s_ucData[50], s_ucRxCnt = 0;
	
	s_ucData[s_ucRxCnt++] = ucData;
	if(s_ucRxCnt<3)return;										//Less than three data returned
	if(s_ucRxCnt >= 50) s_ucRxCnt = 0;
	if(s_ucRxCnt >= 3)
	{
		if((s_ucData[1] == '\r') && (s_ucData[2] == '\n'))
		{
			s_cCmd = s_ucData[0];
			memset(s_ucData,0,50);//
			s_ucRxCnt = 0;
		}
		else 
		{
			s_ucData[0] = s_ucData[1];
			s_ucData[1] = s_ucData[2];
			s_ucRxCnt = 2;
		}
	}
}


static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
	HAL_UART_Transmit(&huart6 ,p_data, uiSize, HAL_MAX_DELAY);
}

static void Delayms(uint16_t ucMs)
{
	HAL_Delay(ucMs);
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
//				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
//				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
//				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
//				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
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

