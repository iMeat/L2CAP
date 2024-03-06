/*
  ******************************************************************************
  * @file    user.h 
  * @author  AMS - RF Application Team
  * @date    27 - 04 - 2021
  * @brief   Application Header functions
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2021 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */ 

#ifndef _user_H_
#define _user_H_

#include "steval_idb011V1_config.h"

extern uint8_t l2capConnectedFlag;

//LEDS
//BLUE - PA6
#define LED_BLUE_OFF() LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS6, LL_GPIO_OUTPUT_HIGH)
#define LED_BLUE_ON() LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS6, LL_GPIO_OUTPUT_LOW)
//GREEN - PB8
#define LED_GREEN_OFF()		LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS8, LL_GPIO_OUTPUT_HIGH)	
#define LED_GREEN_ON()		LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS8, LL_GPIO_OUTPUT_LOW)	
//RED -  PB9
#define LED_RED_OFF()		LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS9, LL_GPIO_OUTPUT_HIGH)	
#define LED_RED_ON()		LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS9, LL_GPIO_OUTPUT_LOW)		 
//PB0
#define PB0_ON()	LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS0, LL_GPIO_OUTPUT_HIGH)	
#define PB0_OFF()		LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS0, LL_GPIO_OUTPUT_LOW)
//PA13
#define PA13_ON()	LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS13, LL_GPIO_OUTPUT_HIGH)	
#define PA13_OFF()		LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS13, LL_GPIO_OUTPUT_LOW)
//PA1
#define PA1_ON()	LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS1, LL_GPIO_OUTPUT_HIGH)	
#define PA1_OFF()		LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS1, LL_GPIO_OUTPUT_LOW)
//PA5
#define PA5_ON()	LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS5, LL_GPIO_OUTPUT_HIGH)	
#define PA5_OFF()		LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS5, LL_GPIO_OUTPUT_LOW)


//PHY parameters



typedef enum
{
	IDLE = 0,
	READY_TO_SEND,
	START_SEND_DATA,
	SENDING_DATA,
	DATA_SENT
}state_t;

typedef struct
{
	state_t state;
	uint16_t connectionHandle;
	uint16_t localCID;
	uint16_t remoteCID;
	//commands
	uint8_t sendDataCmd;
	//flags
	uint8_t sentDataFlag;
	
	//enables
	uint8_t sendDataEn;//general enable
}l2capConnectionServer_t;
		
		
/**
  * @brief  This function initializes the BLE GATT & GAP layers and it sets the TX power level 
  * @param  None
  * @retval None
  */
void device_initialization(void);


void ComPortRead(uint8_t *pRxDataBuffer, uint16_t DataSize);
void L2capDataHandler(l2capConnectionServer_t *l2capConenctionServerPtr);
void BufferInit(uint8_t *bufferPtr, uint16_t bufferSize, uint8_t initValue);
void L2capConnectionServerInit(l2capConnectionServer_t *l2capConnectionServerPtr);

/**
  * @brief  User Application tick 
  * @param  None
  * @retval None
  */
void APP_Tick(void);



#endif /* _user_H_ */
/** \endcond 
*/
