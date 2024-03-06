/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name         : bluenrg_lp_it.c
* Author            : RF Application team
* Version           : 1.0.0
* Date              : 27-October-2021
* Description       : File generated by the IO Mapper tool
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
#include "system_BlueNRG_LP.h"
#include "bluenrg_lp_hw_config.h"
#include "rf_driver_hal_vtimer.h"
#include "bluenrg_lp_stack.h"
#include "hal_miscutil.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Public function prototypes ------------------------------------------------*/

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */ 
/******************************************************************************/

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_IRQHandler(void)
{  
  while (1)
  {
  }
}


/**
* @brief  This function handles SysTick Handler.
* @param  None
* @retval None
*/
void SysTick_IRQHandler(void)
{
	  SysCount_Handler();
}

/******************************************************************************/
/* Peripheral Interrupt Handlers                                              */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file.                                          */
/******************************************************************************/


#if defined(ENABLE_ADC_IRQ) && (ENABLE_ADC_IRQ == 1)
void ADC_IRQHandler(void)
{

}
#endif

#if defined(ENABLE_DMA1_IRQ) && (ENABLE_DMA1_IRQ == 1)
void DMA_IRQHandler(void)
{

}

#endif

#if defined(ENABLE_GPIOA_IRQ) && (ENABLE_GPIOA_IRQ == 1)
void GPIOA_IRQHandler(void)
{

}
#endif

#if defined(ENABLE_GPIOB_IRQ) && (ENABLE_GPIOB_IRQ == 1)
void GPIOB_IRQHandler(void)
{

}
#endif

#if defined(ENABLE_I2C1_IRQ) && (ENABLE_I2C1_IRQ == 1)
void I2C1_IRQHandler(void)
{

}
#endif

#if defined(ENABLE_I2C2_IRQ) && (ENABLE_I2C2_IRQ == 1)
void I2C2_IRQHandler(void)
{

}
#endif

#if defined(ENABLE_LPUART1_IRQ) && (ENABLE_LPUART1_IRQ == 1)
void LPUART1_IRQHandler(void)
{

}
#endif

#if defined(ENABLE_RTC_IRQ) && (ENABLE_RTC_IRQ == 1)
void RTC_IRQHandler(void)
{

}
#endif

#if defined(ENABLE_SPI1_IRQ) && (ENABLE_SPI1_IRQ == 1)
void SPI1_IRQHandler(void)
{

}
#endif

#if defined(ENABLE_SPI2_IRQ) && (ENABLE_SPI2_IRQ == 1)
void SPI2_IRQHandler(void)
{

}
#endif

#if defined(ENABLE_SPI3_IRQ) && (ENABLE_SPI3_IRQ == 1)
void SPI3_IRQHandler(void)
{

}
#endif

#if defined(ENABLE_TIM1_IRQ) && (ENABLE_TIM1_IRQ == 1)
void TIM1_IRQHandler(void)
{

}
#endif

#if defined(ENABLE_USART1_IRQ) && (ENABLE_USART1_IRQ == 1)
void USART1_IRQHandler(void)
{

}
#endif

#if defined(ENABLE_PVD_IRQ) && (ENABLE_PVD_IRQ == 1)
void PVD_IRQHandler(void)
{

}
#endif

void BLE_WKUP_IRQHandler(void)
{
  HAL_VTIMER_WakeUpCallback();
}

void CPU_WKUP_IRQHandler(void)
{
  HAL_VTIMER_TimeoutCallback();
}


void BLE_ERROR_IRQHandler(void)
{
  volatile uint32_t debug_cmd;

  BLUE->DEBUGCMDREG |= 1;

  /* If the device is configured with
     System clock = 64 MHz and BLE clock = 16 MHz
     a register read is necessary to end fine
     the clear interrupt register operation,
     due the AHB down converter latency */
  debug_cmd = BLUE->DEBUGCMDREG;
}

void BLE_TX_RX_IRQHandler(void)
{
  uint32_t blue_status = BLUE->STATUSREG;
  uint32_t blue_interrupt = BLUE->INTERRUPT1REG;

  /** clear all pending interrupts */
  BLUE->INTERRUPT1REG = blue_interrupt;

  HAL_VTIMER_EndOfRadioActivityIsr();
  BLE_STACK_RadioHandler(blue_status|blue_interrupt);
  HAL_VTIMER_RadioTimerIsr();

  /* If the device is configured with
     System clock = 64 MHz and BLE clock = 16 MHz
     a register read is necessary to end fine
     the clear interrupt register operation,
     due the AHB down converter latency */
  blue_interrupt = BLUE->INTERRUPT1REG;
}

void BLE_RXTX_SEQ_IRQHandler(void)
{
  HAL_RXTX_SEQ_IRQHandler();
}

void NMI_IRQHandler(void)
{
  while (1)
  {
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/