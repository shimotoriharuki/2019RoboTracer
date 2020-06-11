/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "robotre_lib.h"

#include "Macros.h"

#include "G_variable.h"
#include "AQM0802.h"
#include "ICM_20648.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t FatFsCnt = 0;
volatile uint8_t Timer1, Timer2;

void SDTimer_Handler(void){
	if(Timer1 > 0)
		Timer1--;

	if(Timer2 > 0)
		Timer2--;
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	FatFsCnt++;
	if(FatFsCnt >= 10){
		FatFsCnt = 0;
		SDTimer_Handler();
	}
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt and DAC1, DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

	static short ad_timer = 0;
	static uint8_t cnt;
	int16_t SumPulse;
	static double SumVelo[2];

	increment_mytimer();

	angle_measurement(flag.angle_enable);

	flag.error = error_check(&error_number);

	updata_ADval();

	read_gyro_data();
	read_accel_data();
	updata_imu_data_lowpassed();
	CalcIMUoffset(flag.IMU_offset);

	store_log_data(flag.log_store);
	//RotationTest(flag.rotation);
	BanquetArt(flag.art);

	TargetOmega = getTargetOmega();

	choice_following_mode(flag.following_start);
	sensor_following(flag.following);
	angle_ctrl(flag.angle, 2000);

	Line_trace(flag.line_trace);

	//speed_ctrl(flag.speed_ctrl_enable, speed_L, speed_R, robot_speed, &speed_val);//

	hand_push_trace(flag.hand_push);

	updata_enc_cnt(&total_encL, &total_encR, &total_encL_memory, &total_encR_memory, &total_encL_distance, &total_encR_distance, &total_encL_reset, &total_encR_reset);

	cnt++;
	SumPulse = (getEncorder_L() + getEncorder_R()) / 2;
	SumVelo[0] += MM_PER_PULS * SumPulse;	//[m/s]
	SumVelo[1] += omega_z_l;	//[rad/s]
	//SumVelo[0] += 10;	//[m/s]
	//SumVelo[1] += 1;	//[rad/s]

	if(cnt >= 10){	//10ms
		static uint16_t access;

		CurrentVelo[0] = SumVelo[0] / cnt;	//[m/s]
		CurrentVelo[1] = SumVelo[1] / cnt;	//[rad/s]
		//SumPulse = (getEncorder_L() + getEncorder_R()) / 2;
		//CurrentVelo[0] = MM_PER_PULS * SumPulse;	//[m/s]
		//CurrentVelo[1] = omega_z_l;


		if(flag.GSL_enable == 1){

			GetPosition1(PreDR_Position, DR_Position, CurrentVelo, flag.GSL_enable);

			GetPosition2(PreRobotPosition, MeaRobotPosition, CurrentVelo, flag.GSL_enable);
			main_GetSelfLocation(MeaRobotPosition, CurrentVelo, RobotPosition);

			various_memory1[access] = DR_Position[0];
			various_memory2[access] = DR_Position[1];
			various_memory3[access] = DR_Position[2];

			various_memory4[access] = RobotPosition[0];
			various_memory5[access] = RobotPosition[1];
			various_memory6[access] = RobotPosition[2];

			PreDR_Position[0] = DR_Position[0];
			PreDR_Position[1] = DR_Position[1];
			PreDR_Position[2] = DR_Position[2];

			PreRobotPosition[0] = RobotPosition[0];
			PreRobotPosition[1] = RobotPosition[1];
			PreRobotPosition[2] = RobotPosition[2];

			//MeaRobotPosition[0] = 0;
			//MeaRobotPosition[1] = 0;
			//MeaRobotPosition[2] = 0;


			access++;
			if(access >= MEMORY_ARRAY_SIZE_2 - 1) access = MEMORY_ARRAY_SIZE_2 - 1;

		}

		SumVelo[0] = 0;
		SumVelo[1] = 0;
		SumPulse = 0;
		cnt = 0;
	}

	enc_reset();	//	初期値にする

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
#define AVE_NUM 10
	static int sum_pot = 0;
	static int sum_sens1 = 0, sum_sens2 = 0, sum_sens3 = 0, sum_sens4 = 0;
	static short ful_flag = 0;

	timer.check_01ms++;

	//sum_pot += (2048 - ad1[6]);
	sum_pot += (2048 - (ad1[6] >> 0));
/*
	sum_sens1 += ad1[0];
	sum_sens2 += ad1[1];
	sum_sens3 += ad1[2];
	sum_sens4 += ad1[3];
*/
	ful_flag++;

	if(ful_flag >= AVE_NUM){
		average_pot = sum_pot / AVE_NUM;
/*
		average_sens1 = sum_sens1 / AVE_NUM;
		average_sens2 = sum_sens2 / AVE_NUM;
		average_sens3 = sum_sens3 / AVE_NUM;
		average_sens4 = sum_sens4 / AVE_NUM;
*/
		sum_sens1 = 0;
		sum_sens2 = 0;
		sum_sens3 = 0;
		sum_sens4 = 0;

		sum_pot = 0;

		ful_flag = 0;
	}
  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
