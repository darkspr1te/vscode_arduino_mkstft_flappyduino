/*
 *******************************************************************************
 * Copyright (c) 2018, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************
 */
#include "pins_arduino.h"
#include "stm32f1xx_hal.h"
#include "stdio.h"
#ifdef __cplusplus
extern "C" {
#endif

// Pin number
const PinName digitalPin[] = {
  // Right side
  PB_11, //D0
  PB_10, //D1
  PB_2,  //D2
  PB_0,  //D3
  PA_7,  //D4
  PA_6,  //D5
  PA_5,  //D6
  PA_4,  //D7
  PA_3,  //D8
  PA_2,  //D9
  PA_1,  //D10
  PA_0,  //D11
  PC_15, //D12
  PC_14, //D13
  PC_13, //D14
  // Left side
  PB_7,  //D15
  PB_6,  //D16
  PB_5,  //D17
  PB_4,  //D18
  PB_3,  //D19
  PA_15, //D20
  PA_14, //D21 - SWCLK
  PA_13, //D22 - SWDI0
  PA_12, //D23 - USB DP
  PA_11, //D24 - USB DM
  PA_10, //D25
  PA_9,  //D26
  PA_8,  //D27
  PB_15, //D28
  PB_14, //D29
  PB_13, //D30
  PB_12, //D31
  // Other
  PB_8,  //D32 - BOOT0 - User buttons
  PB_1,  //D33 - LED
  PB_9,  //D34 - USB DISC
  PD_14,//D35 LCD Backlight
  PD_15,//D36 LCD RD
  PC_8,//D37 LCD CS
  PC_5,//D38 Touch input IRQ
  PD_11,//D39 SDCARD chip select 
  PC_12,//D40 unknown yet
  PE_0,//D41 Port E Bit 0
  PE_1,//D42 Port E Bit 1
  PE_2,//D43 Port E Bit 2
  PE_3,//D44 Port E Bit 3
  PE_4,//D45 Port E Bit 4
  PE_5,//D46 Port E Bit 5
  PE_6,//D47 Port E Bit 6
  PE_7,//D48 Port E Bit 7
  PD_13,//D49 LCD_RS
  PC_9,//D50 TOUCH_CS
  PC_10,//D51 
  PC_11,//D52
  PD_8,//D53
  PD_9,//D54
  PD_10,//D55
  PD_5,//D56
  PD_6,//D57

  // Duplicated pins to avoid issue with analogRead
  // A0 have to be greater than NUM_ANALOG_INPUTS
  PB_0,  //D58/A0 = D3
  PA_7,  //D59/A1 = D4
  PA_6,  //D60/A2 = D5
  PA_5,  //D61/A3 = D6
  PA_4,  //D62/A4 = D7
  PA_3,  //D63/A5 = D8
  PA_2,  //D64/A6 = D9
  PA_1,  //D65/A7 = D10
  PA_0   //D66/A8 = D11
};

#ifdef __cplusplus
}
#endif

// ----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            PLL_Source                     = HSE
  *            PLL_Mul                        = 9
  *            Flash Latency(WS)              = 2
  *            ADC Prescaler                  = 6
  *            USB Prescaler                  = 1.5
  * @param  None
  * @retval None
  */

#if defined(STM32F107xC) && defined(MKS_TFT)
//extern HCD_HandleTypeDef hhcd_USB_OTG_FS;
//extern TIM_HandleTypeDef htim2;
//extern UART_HandleTypeDef huart3;
#elif defined(STM32F103xE) && defined(CZMINI)
extern SD_HandleTypeDef hsd;
#endif

extern DMA_HandleTypeDef hdma_usart2_rx;
extern TIM_HandleTypeDef htim7;
//extern UART_HandleTypeDef huart2;

  

void initVariant(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, 0xffff, GPIO_PIN_RESET);

  /*Configure GPIO pins : Pin1_Pin Pin2_Pin */
  GPIO_InitStruct.Pin = 0xffff;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);	
}


/**
  * @brief  Update SystemCoreClock variable according to Clock Register Values.
  *         The SystemCoreClock variable contains the core clock (HCLK), it can
  *         be used by the user application to setup the SysTick timer or configure
  *         other parameters.
  *
  * @note   Each time the core clock (HCLK) changes, this function must be called
  *         to update SystemCoreClock variable value. Otherwise, any configuration
  *         based on this variable will be incorrect.
  *
  * @note   - The system frequency computed by this function is not the real
  *           frequency in the chip. It is calculated based on the predefined
  *           constant and the selected clock source:
  *
  *           - If SYSCLK source is HSI, SystemCoreClock will contain the HSI_VALUE(*)
  *
  *           - If SYSCLK source is HSE, SystemCoreClock will contain the HSE_VALUE(**)
  *
  *           - If SYSCLK source is PLL, SystemCoreClock will contain the HSE_VALUE(**)
  *             or HSI_VALUE(*) multiplied by the PLL factors.
  *
  *         (*) HSI_VALUE is a constant defined in stm32f1xx.h file (default value
  *             8 MHz) but the real value may vary depending on the variations
  *             in voltage and temperature.
  *
  *         (**) HSE_VALUE is a constant defined in stm32f1xx.h file (default value
  *              8 MHz or 25 MHz, depending on the product used), user has to ensure
  *              that HSE_VALUE is same as the real frequency of the crystal used.
  *              Otherwise, this function may have wrong result.
  *
  *         - The result of this function could be not correct when using fractional
  *           value for HSE crystal.
  * @param  None
  * @retval None
  */
  


void SystemClock_Config(void)
{
	WWDG->SR = 0;
	
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
#if defined(STM32F107xC) && defined(MKS_TFT)
    RCC_PeriphCLKInitTypeDef PeriphClkInit;
#endif
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
#if defined(STM32F107xC) && defined(MKS_TFT)
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
    RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
    RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
    RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
    RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
#endif
#if defined(STM32F103xE) && defined(CZMINI)
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
#endif
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
            |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
        Error_Handler();

#if defined(STM32F107xC) && defined(MKS_TFT)
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV3;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
        Error_Handler();
#endif

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
#if defined(STM32F107xC) && defined(MKS_TFT)
    __HAL_RCC_PLLI2S_ENABLE();
#endif
    HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}



void WWDG_IRQHandler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  //while (1)
 // {
 // }
__HAL_RCC_CLEAR_RESET_FLAGS(); 
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

#if defined(STM32F107xC) && defined(MKS_TFT)
/**
* @brief This function handles EXTI line0 interrupt.
*/
void WWDG_IRQHandler_exp(void)
{
   
    //WWDG_ClearFlag();    /*Remove pre wakeup interrupt flag*/
    __HAL_RCC_CLEAR_RESET_FLAGS(); 
    
    //LED1 = ~LED1;         /*LED state turnover */
}

void hard_fault_handler_c (unsigned int * hardfault_args)
{
unsigned int stacked_r0;
  unsigned int stacked_r1;
  unsigned int stacked_r2;
  unsigned int stacked_r3;
  unsigned int stacked_r12;
  unsigned int stacked_lr;
  unsigned int stacked_pc;
  unsigned int stacked_psr;
 
  stacked_r0 = ((unsigned long) hardfault_args[0]);
  stacked_r1 = ((unsigned long) hardfault_args[1]);
  stacked_r2 = ((unsigned long) hardfault_args[2]);
  stacked_r3 = ((unsigned long) hardfault_args[3]);
 
  stacked_r12 = ((unsigned long) hardfault_args[4]);
  stacked_lr = ((unsigned long) hardfault_args[5]);
  stacked_pc = ((unsigned long) hardfault_args[6]);
  stacked_psr = ((unsigned long) hardfault_args[7]);
  
  printf ("\n\n[Hard fault handler - all numbers in hex]\r\n");
  printf ("R0 = %x\r\n", stacked_r0);
  printf ("R1 = %x\r\n", stacked_r1);
  printf ("R2 = %x\r\n", stacked_r2);
  printf ("R3 = %x\r\n", stacked_r3);
  printf ("R12 = %x\r\n", stacked_r12);
  printf ("LR [R14] = %x  subroutine call return address\r\n", stacked_lr);
  printf ("PC [R15] = %x  program counter\r\n", stacked_pc);
  printf ("PSR = %x\r\n", stacked_psr);
  printf ("BFAR = %x\r\n", (*((volatile unsigned long *)(0xE000ED38))));
  printf ("CFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED28))));
  printf ("HFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED2C))));
  printf ("DFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED30))));
  printf ("AFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED3C))));
  printf ("SCB_SHCSR = %x\r\n", SCB->SHCSR);
  
  while (1);
}

//intial save fault handler then branch to  hard_fault_handler_c to dump to uart 1 
void HardFault_Handler(void)
{
	
	__asm volatile (
		"tst LR, #4       \n"
		"ite EQ       \n"
		"mrseq R0,MSP       \n"
		"mrsne R0, PSP       \n"
		"b hard_fault_handler_c \n"
		);
	
  __asm volatile (
    " movs r0,#4       \n"
    " movs r1, lr      \n"
    " tst r0, r1       \n"
    " beq _MSP         \n"
    " mrs r0, psp      \n"
    " b _HALT          \n"
  "_MSP:               \n"
    " mrs r0, msp      \n"
  "_HALT:              \n"
    " ldr r1,[r0,#20]  \n"
    " bkpt #0          \n"
  );
}

//restore original irq vectors 
#if defined(VECTOR)
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

 void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
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
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
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
* @brief This function handles System tick timer.
*/
void TIM1_TRG_COM_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */

  //HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

void PendSV_Handler(void)
{
}
void PVD_IRQHandler(void)
{
	
}
void OTG_FS_WKUP_IRQHandler(void)
{
}

void TAMPER_IRQHandler(void)
{
	
}

void SVC_Handler (void)
{
}

void SPI3_IRQHandler(void)
{
}

void SPI2_IRQHandler(void)
{
}
void SPI1_IRQHandler(void)
{
}
void RTC_IRQHandler (void)
{
}
void RCC_IRQHandler(void)
{
}

WEAK void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}
/*
void TIM1_CC_IRQHandler(void)
{
	
}
*/
void TIM1_BRK_IRQHandler(void)
{
	
}

/**
* @brief This function handles EXTI line1 interrupt.
*/
WEAK void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}


/**
* @brief This function handles USB OTG FS global interrupt.
*/
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  //HAL_HCD_IRQHandler(&hhcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}
#endif
#endif
#ifdef __cplusplus
}
#endif
