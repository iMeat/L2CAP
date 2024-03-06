/*
  ******************************************************************************
  * @file    user.c 
  * @author  AMS - RF Application Team
  * @date    27 - 04 - 2021
  * @brief   Application functions
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
 
/* Includes-----------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "bluenrg_lp_it.h"
#include "ble_const.h"
#include "bluenrg_lp_stack.h"
#include "rf_driver_hal_power_manager.h"
#include "rf_driver_hal_vtimer.h"
#include "bluenrg_lp_evb_com.h"
#include "gatt_profile.h"
#include "gap_profile.h"
#include "user.h"
 
#ifndef DEBUG
#define DEBUG 1
#endif

uint8_t l2capConnectedFlag;
extern uint8_t txBuffer[32000+2];		
uint16_t sizeCounter = 210;

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define DEVICE_ADDRESS       0xa2, 0x00, 0x00, 0xE1, 0x80, 0x03

volatile uint8_t set_discoverable = TRUE;
//volatile uint8_t set_discoverable = FALSE;
//static volatile uint8_t set_discoverable=FALSE;
static Advertising_Set_Parameters_t Advertising_Set_Parameters[1];


/**
 * @brief  Init a BlueNRG device
 * @param  None.
 * @retval None.
*/
//=============================================================================
void device_initialization(void)
{
  uint8_t bdaddr[] = {DEVICE_ADDRESS};
  uint8_t device_name[]={'N', 'o', 'd', 'e'};
  uint8_t ret;
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  static uint8_t adv_data[] = {0x02,AD_TYPE_FLAGS, FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE|FLAG_BIT_BR_EDR_NOT_SUPPORTED,
                               5, AD_TYPE_COMPLETE_LOCAL_NAME,'N', 'o', 'd', 'e'};
  /* Configure Public address */
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);
  if(ret != BLE_STATUS_SUCCESS) 
	{
    PRINTF("aci_hal_write_config_data() failed: 0x%02x\r\n", ret);
  }
  /* Set the TX power to 8 dBm */
  aci_hal_set_tx_power_level(1, 32);
  /* GATT Init */
  ret = aci_gatt_srv_init();    
  if(ret){
    PRINTF("aci_gatt_srv_init() failed: 0x%02x\r\n", ret);
  }
  /* GAP Init */
  ret = aci_gap_init(GAP_PERIPHERAL_ROLE,
										 0,
										 0x07,
										 0x00,
										 &service_handle,
										 &dev_name_char_handle,
										 &appearance_char_handle);
  if(ret){
    PRINTF("aci_gap_Init() failed: 0x%02x\r\n", ret);
  }
    /* Update device name */
  Gap_profile_set_dev_name(0, sizeof(device_name), device_name);
  ret = aci_gap_set_advertising_configuration(0, GAP_MODE_GENERAL_DISCOVERABLE,
                                              ADV_PROP_CONNECTABLE|ADV_PROP_SCANNABLE|ADV_PROP_LEGACY,
                                              100, 
                                              100,
                                              ADV_CH_ALL,
                                              PUBLIC_ADDR,NULL,
                                              ADV_NO_WHITE_LIST_USE,
                                              0, /* 0 dBm */
                                              LE_1M_PHY, /* Primary advertising PHY */
                                              0, /* 0 skips */
                                              LE_1M_PHY, /* Secondary advertising PHY. Not used with legacy advertising. */
                                              0, /* SID */
                                              0 /* No scan request notifications */);
	printf("====================================\r\n");	
	printf("Server: aci_gap_set_advertising_configuaration(r\n");
  PRINTF("Advertising configuration %02X\n", ret);
  ret = aci_gap_set_advertising_data(0, ADV_COMPLETE_DATA, sizeof(adv_data), adv_data);
  PRINTF("Set advertising data %02X\n", ret);  
	//PB0 --- start send data
	LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB);
	LL_GPIO_SetPinMode(GPIOB, GPIO_BSRR_BS0, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinSpeed(GPIOB, GPIO_BSRR_BS0, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOB, GPIO_BSRR_BS0, LL_GPIO_OUTPUT_PUSHPULL);

//PA13 --- complete TX data
	LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOA);
	LL_GPIO_SetPinMode(GPIOA, GPIO_BSRR_BS13, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinSpeed(GPIOA, GPIO_BSRR_BS13, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOA, GPIO_BSRR_BS13, LL_GPIO_OUTPUT_PUSHPULL);
	
//PA1 --- BLE stack tick
	LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOA);
	LL_GPIO_SetPinMode(GPIOA, GPIO_BSRR_BS1, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinSpeed(GPIOA, GPIO_BSRR_BS1, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOA, GPIO_BSRR_BS1, LL_GPIO_OUTPUT_PUSHPULL);

//PA5 ---
	LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOA);	
	LL_GPIO_SetPinMode(GPIOA, GPIO_BSRR_BS5, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinSpeed(GPIOA, GPIO_BSRR_BS5, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOA, GPIO_BSRR_BS5, LL_GPIO_OUTPUT_PUSHPULL);	

//LED		
//PA6 --- BLUE
	LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOA);
	LL_GPIO_SetPinMode(GPIOA, GPIO_BSRR_BS6, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinSpeed(GPIOA, GPIO_BSRR_BS6, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOA, GPIO_BSRR_BS6, LL_GPIO_OUTPUT_PUSHPULL);	
	LL_GPIO_WriteOutputPin(GPIOA, GPIO_BSRR_BS6, LL_GPIO_OUTPUT_HIGH);
	
//PB8 --- GREEN
	LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB);
	LL_GPIO_SetPinMode(GPIOB, GPIO_BSRR_BS8, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinSpeed(GPIOB, GPIO_BSRR_BS8, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOB, GPIO_BSRR_BS8, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS8, LL_GPIO_OUTPUT_HIGH);
	
//PB9 --- RED
	LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB);
	LL_GPIO_SetPinMode(GPIOB, GPIO_BSRR_BS9, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinSpeed(GPIOB, GPIO_BSRR_BS9, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOB, GPIO_BSRR_BS9, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_WriteOutputPin(GPIOB, GPIO_BSRR_BS9, LL_GPIO_OUTPUT_HIGH);
	
	LED_BLUE_OFF();
	LED_GREEN_OFF();
	LED_RED_OFF();
}
//=============================================================================
/**
 * @brief  Put a device in discoverable mode
 * @param  None.
 * @retval None.
*/
void set_device_discoverable(void)
{  
  uint8_t ret;    
  Advertising_Set_Parameters[0].Advertising_Handle = 0;
  Advertising_Set_Parameters[0].Duration = 0;
  Advertising_Set_Parameters[0].Max_Extended_Advertising_Events = 0;
  ret = aci_gap_set_advertising_enable(ENABLE, 1, Advertising_Set_Parameters); 
  if (ret != BLE_STATUS_SUCCESS)
    PRINTF ("Error in aci_gap_set_advertising_enable(): 0x%02x\r\n", ret);
  else
    PRINTF ("aci_gap_set_advertising_enable() --> SUCCESS\r\n");
  PRINTF("Start Advertising \r\n");
}
//=============================================================================
/**
 * @brief  Device Demo state machine.
 * @param  None.
 * @retval None.
*/
void APP_Tick(void)
{
  //USER ACTION IS NEEDED
  /* Make the device discoverable */
  if(set_discoverable)
	{
    set_device_discoverable();
    set_discoverable = FALSE;
  } 
}
//=============================================================================
void ComPortRead(uint8_t *pRxDataBuffer, uint16_t DataSize)
{
	uint8_t rxne = 0;
	uint8_t temp = 0;
	if(LL_USART_IsActiveFlag_RXNE(BSP_UART))
	{
		temp = *pRxDataBuffer;
	}
	return;
}
//=============================================================================
void TestClockSys(void)
{
	//prepare pin PA11 - AF0, output, mode = 0x10, otype = 0x00, speed = 0x11
	//enable clocking port A
	RCC->AHBENR |= (RCC_AHBENR_GPIOAEN);
	//mode =0x10
	GPIOA->MODER |= (GPIO_MODER_MODE11_1);
	//otype = 0x00
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT11);
	//speed = 0x11
	GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED11_1 | GPIO_OSPEEDR_OSPEED11_0);
	//pullup-down reg = 0x00
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD11_1 | GPIO_PUPDR_PUPD11_0);
	// alternate function = 0x00
	GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL11_0 | GPIO_AFRH_AFSEL11_1); 
	//set output system clock signal deiveder to 1
	RCC->CFGR &= ~(0x7 << 28);
	//output the system clock signal to the MSO pin 
	RCC->CFGR |= (RCC_CFGR_MCOSEL_0);  
}
//=============================================================================
void L2capDataHandler(l2capConnectionServer_t *l2capConnectionServerPtr)
{
	uint8_t status;
	if(l2capConnectionServerPtr->sendDataEn == 0)
	{
		l2capConnectionServerPtr->state = IDLE;
	}
	//=============================
	switch(l2capConnectionServerPtr->state)
	{
	//=============================
	case IDLE:
	{
		if(l2capConnectionServerPtr->sendDataEn == 1)
		{
			l2capConnectionServerPtr->state = READY_TO_SEND;
		}
		break;
	}
	//=============================	
	case READY_TO_SEND:
	{
		if(l2capConnectionServerPtr->sendDataCmd == 1)
		{
			l2capConnectionServerPtr->sendDataCmd = 0;
			l2capConnectionServerPtr->state = START_SEND_DATA;
		}
		break;
	}
	//=============================	
	case START_SEND_DATA:
	{
/*			if(sizeCounter < 16000)
			{
				sizeCounter += 1;
			} 
			
			status = aci_l2cap_transmit_sdu_data(l2capConnectionServerPtr->connectionHandle,
																	l2capConnectionServerPtr->localCID,
																	sizeCounter, //data size
																	txBuffer); //buffer pointer	
*/			
		PB0_ON();
		status = aci_l2cap_transmit_sdu_data(l2capConnectionServerPtr->connectionHandle,
																	l2capConnectionServerPtr->localCID,
																	12000,//data size
																	txBuffer); //buffer pointer			
		PB0_OFF();
			
#ifdef PRINT_DEBUG_INFO
		printf("====================================\r\n");
		printf("Server: aci_l2cap_transmit_sdu_data()");
#endif //PRINT_DEBUG_INFO			
		if(status == BLE_STATUS_SUCCESS)
		{
#ifdef PRINT_DEBUG_INFO				
			printf(" OK\r\n");
#endif //PRINT_DEBUG_INFO					
			l2capConnectionServerPtr->state = SENDING_DATA;	
		}
		else
		{
#ifdef PRINT_DEBUG_INFO				
			printf("Error code: 0x%02x\r\n", status);
#endif //PRINT_DEBUG_INFO
		}
		break;
	}
	//=============================	
	case SENDING_DATA:
	{
		if(l2capConnectionServerPtr->sentDataFlag == 1)
		{
			l2capConnectionServerPtr->sentDataFlag = 0;
			l2capConnectionServerPtr->state = DATA_SENT; 
		}
		break;
	}
	//=============================	
	case DATA_SENT:
	{
		l2capConnectionServerPtr->state = READY_TO_SEND;
		break;
	}
	//=============================	
	default:
	{
		printf("Unaccepted value\r\n");
	}
}
}
//=============================================================================
void BufferInit(uint8_t *bufferPtr, uint16_t bufferSize, uint8_t initValue)
{
	for(uint16_t i = 0; i < bufferSize; i++)
	{
		bufferPtr[i] = initValue;
	}
}
//=============================================================================
void L2capConnectionServerInit(l2capConnectionServer_t *l2capConnectionServerPtr)
{
	l2capConnectionServerPtr->state = IDLE;
	l2capConnectionServerPtr->connectionHandle = 0;
	l2capConnectionServerPtr->localCID = 0;
	l2capConnectionServerPtr->remoteCID = 0;
	l2capConnectionServerPtr->sendDataCmd = 0;
	l2capConnectionServerPtr->sentDataFlag = 0;
	l2capConnectionServerPtr->sendDataEn = 0;
}