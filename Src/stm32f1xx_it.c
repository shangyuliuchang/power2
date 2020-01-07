/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
//#define CAP_A_HIGH HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET)
//#define CAP_A_LOW HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET)
//#define CAP_B_HIGH HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)
//#define CAP_B_LOW HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)
//#define OUT_A_HIGH HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET)
//#define OUT_A_LOW HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET)
//#define OUT_B_HIGH HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET)
//#define OUT_B_LOW HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET)

#define CAP_A_HIGH GPIOA->BSRR=0x00000001<<(3+16)
#define CAP_A_LOW GPIOA->BSRR=0x00000001<<(3)
#define CAP_B_HIGH GPIOA->BSRR=0x00000001<<(4+16)
#define CAP_B_LOW GPIOA->BSRR=0x00000001<<(4)
#define OUT_A_HIGH GPIOA->BSRR=0x00000001<<(5+16)
#define OUT_A_LOW GPIOA->BSRR=0x00000001<<(5)
#define OUT_B_HIGH GPIOA->BSRR=0x00000001<<(6+16)
#define OUT_B_LOW GPIOA->BSRR=0x00000001<<(6)

#define CHARGE 1
#define UP 1

int cnt=0;
int turn=0;
int emergency=0;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */ 
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
  * @brief This function handles Prefetch fault, memory access fault.
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

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	if(emergency==0){
		if(cnt==0){
			//turn=1-turn;
			if(turn==0){
				//		cap_volt_tmp=0;
	//		for(int i=0;i<11;i++){
	//			cap_volt_tmp+=adcData[i*3];
	//		}
			cap_volt_tmp=adcData[0]+adcData[3]+adcData[6]+adcData[9]+adcData[12]+adcData[15]+adcData[18]+adcData[21]+adcData[24]+adcData[27]+adcData[30];
			//cap_volt=cap_volt_tmp/12;
			cap_ratio=243-(cap_volt_tmp>>7);//243->26V
			//if(cap_volt<1800){
				//emergency=1;
			//}
	//		out_volt_tmp=0;
	//		for(int i=0;i<11;i++){
	//			out_volt_tmp+=adcData[i*3+1];
	//		}
			out_volt_tmp=adcData[1]+adcData[4]+adcData[7]+adcData[10]+adcData[13]+adcData[16]+adcData[19]+adcData[22]+adcData[25]+adcData[28]+adcData[31];
			//out_volt=out_volt_tmp/12;
			out_ratio=243-(out_volt_tmp>>7);//243->26V
			
			I_tmp=adcData[2]+adcData[5]+adcData[8]+adcData[11]+adcData[14]+adcData[17]+adcData[20]+adcData[23]+adcData[26]+adcData[29]+adcData[32];
			I_ratio=85-(I_tmp>>7);//85->4A
//			
//			cap_ratio=pid_calculate(0,2600.0f,cap_volt);
//			out_ratio=pid_calculate(1,2600.0f,out_volt);
//			I_ratio=pid_calculate(2,4*2730,I_tmp); //I=4A;
			//cap_ratio=(2600-cap_volt)/10;
			if(cap_ratio<0){
				cap_ratio=0;
			}else if(cap_ratio>5){
				cap_ratio=5;
			}
			
			//out_ratio=(2600-out_volt)/10;
			if(out_ratio<0){
				out_ratio=0;
			}else if(out_ratio>5){
				out_ratio=5;
			}
			
			//I_ratio=(10920-I_tmp)/100; //10920=2730*4=4A
			if(I_ratio<0){
				I_ratio=0;
			}else if(I_ratio>5){
				I_ratio=5;
			}
			
			if(I_ratio<cap_ratio){
				cap_ratio=I_ratio;
			}
	//			
	//		cap_ratio=pid_calculate_special(0,2600,cap_volt);
	//		out_ratio=pid_calculate_special(1,2600,out_volt);
			}
		}
		if(CHARGE && cap_ratio>cnt){
			if(turn==0){
				CAP_A_HIGH;
				CAP_B_LOW;
			}else{
				CAP_A_LOW;
				CAP_B_HIGH;
			}
		}else{
			CAP_A_LOW;
			CAP_B_LOW;
		}
		if(UP && out_ratio>cnt){
			if(turn==0){
				OUT_A_HIGH;
				OUT_B_LOW;
			}else{
				OUT_A_LOW;
				OUT_B_HIGH;
			}
		}else{
			OUT_A_LOW;
			OUT_B_LOW;
		}
		
		turn=1-turn;
		if(turn==0){
			cnt=(cnt+1)%10;
		}
		
	}else{
		CAP_A_LOW;
		CAP_B_LOW;
		OUT_A_LOW;
		OUT_B_LOW;
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
	}
	
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
