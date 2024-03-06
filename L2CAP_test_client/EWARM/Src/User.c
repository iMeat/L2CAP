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
//#include <stdio.h>
//#include <string.h>
//#include <stdlib.h>
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


#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define DEVICE_ADDRESS       0xa2, 0x00, 0x00, 0xE1, 0x80, 0x02

static volatile uint8_t set_discoverable = TRUE; 
static Advertising_Set_Parameters_t Advertising_Set_Parameters[1];

uint8_t rxBuffer[32000 + 2];

uint8_t rxDataExtracted[12000];


//=============================================================================
/**
 * @brief  Init a BlueNRG device
 * @param  None.
 * @retval None.
*/
void device_initialization(void)
{
  uint8_t bdaddr[] = {DEVICE_ADDRESS};
  uint8_t device_name[] = {'N', 'o', 'd', 'e'};
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
  /* Set the TX power to 0 dBm */
  aci_hal_set_tx_power_level(1, 32);
  /* GATT Init */
  ret = aci_gatt_srv_init();    
  if(ret)
	{
    PRINTF("aci_gatt_srv_init() failed: 0x%02x\r\n", ret);
  }
      
  /* GAP Init */
#if GAP_CONNECTION == 1	
  ret = aci_gap_init(GAP_CENTRAL_ROLE, 0, 0x07,  0x00, &service_handle, &dev_name_char_handle, &appearance_char_handle);
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
	printf("Client: aci_gap_set_advertising_configuration()\r\n");
  PRINTF("Advertising configuration %02X\n", ret);
  ret = aci_gap_set_advertising_data(0, ADV_COMPLETE_DATA, sizeof(adv_data), adv_data);
  PRINTF("Set advertising data %02X\n", ret);  
#endif //GAP_CONNECTION    	
	
//PB0 --- data Rx event
	LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB);
	LL_GPIO_SetPinMode(GPIOB, GPIO_BSRR_BS0, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinSpeed(GPIOB, GPIO_BSRR_BS0, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOB, GPIO_BSRR_BS0, LL_GPIO_OUTPUT_PUSHPULL);

//PA13 --- data extract
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

/**
 * @brief  Device Demo state machine.
 * @param  None.
 * @retval None.
*/
//=============================================================================
void APP_Tick(void)
{

  //USER ACTION IS NEEDED
  
  /* Make the device discoverable */
  if(set_discoverable) {
//    set_device_discoverable();
    set_discoverable = FALSE;
  } 
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
//BLUETOOTH CONNECTION	
//=============================================================================	
void BluetoothConnectionHandler(bluetoothConnection_t *bleConnectionPtr)
{
	uint8_t status = 0;
	//check first entry to state
	if(bleConnectionPtr->prevState == bleConnectionPtr->state)
	{
		bleConnectionPtr->firstEntryToState = 0;
	}
	else
	{
		bleConnectionPtr->firstEntryToState = 1;
	}
	bleConnectionPtr->prevState = bleConnectionPtr->state;
	
	
	switch(bleConnectionPtr->state)
	{
	//=============================	
	case BT_IDLE:
	{
		if(bleConnectionPtr->startGapConnectionEn == 1)
		{
			bleConnectionPtr->state = BT_START_GAP_CONNECTION;
		}
		else if(bleConnectionPtr->startLLConnectionEn == 1)
		{
			bleConnectionPtr->state = BT_START_LL_CONNECTION;
		}
		else
		{
			return;
		}
		break;	
	}
	//=============================
	case BT_START_GAP_CONNECTION:
	{
		status = aci_gap_create_connection(bleConnectionPtr->connectionParameters.PHY,
																				bleConnectionPtr->connectionParameters.addressType,
																				bleConnectionPtr->connectionParameters.peerAddress);
		if(status == BLE_STATUS_SUCCESS)
		{
#ifdef PRINT_DEBUG_INFO
			printf("====================================\r\n");
			printf("Client: aci_gap_create_connection() OK\r\n");
#endif // PRINT_DEBUG_INFO			
			bleConnectionPtr->state = BT_CONNECTING;
		}
		else
		{
			bleConnectionPtr->state = BT_IDLE;
#ifdef PRINT_DEBUG_INFO
			printf("====================================\r\n");
			printf("Client: aci_gap_create_connection() Error code: 0x%02x\r\n", status);
#endif // PRINT_DEBUG_INFO	
		}
		break;
	}
	//=============================
	case BT_START_LL_CONNECTION:
		{
			if(bleConnectionPtr->firstEntryToState == 1)
			{
				uint16_t llscanInterval = (uint16_t)(100.00/0.625);	//2.50ms...10240ms
				uint16_t llScanWindow = (uint16_t)(50.00/0.625);	//2.50ms...10240ms
				uint8_t llInitiatorFilterPolicy = 0x00;//white lis is not used
				//address type from connectionPtr
				//peer address from connectionPtr
				uint8_t ownAddressType = 0x00;// own public address
				status = hci_le_create_connection(llscanInterval,
																				llScanWindow,
																				llInitiatorFilterPolicy,
																				bleConnectionPtr->connectionParameters.addressType,
																				bleConnectionPtr->connectionParameters.peerAddress,
																				ownAddressType,
																				CONN_INTERVAL_MIN,
																				CONN_INTERVAL_MAX,
																				SLAVE_LATENCY,
																				SUPERVISION_TIMEOUT,
																				CE_LENGTH_MIN,
																				CE_LENGTH_MAX);
#ifdef PRINT_DEBUG_INFO
				printf("====================================\r\n");	
				printf("Client: hci_le_create_connection()\r\n");
#endif //PRINTF_DEBU_INFO			
				if(status == BLE_STATUS_SUCCESS)
				{
					bleConnectionPtr->state = BT_CONNECTING;
#ifdef PRINT_DEBUG_INFO							
					printf("OK");
#endif //PRINTF_DEBU_INFO						
				}
				else
				{
#ifdef PRINT_DEBUG_INFO					
					printf("Error code: 0x%02x", status);
#endif //PRINTF_DEBU_INFO				
				}
			}
			break;
		}
	//=============================
	case BT_CONNECTING:
	{
		if(bleConnectionPtr->connectedFlag == 1)
		{
			bleConnectionPtr->connectedFlag = 0;
			bleConnectionPtr->state = BT_READ_FEATURES;			
		}
		else
		{
			return;
		}
		break;
	}		
	//=============================
	case BT_READ_FEATURES:
	{
		status = hci_le_read_remote_used_features(bleConnectionPtr->connectionParameters.connectionHandle);
#ifdef PRINT_DEBUG_INFO
		printf("====================================\r\n");				
		printf("Client: hce_le_read_remote_used_features()\r\n");
#endif // PRINT_DEBUG_INFO
		if(status == 0)
		{
#ifdef PRINT_DEBUG_INFO				
			printf("OK\r\n");
#endif	// PRINT_DEBUG_INFO									
			bleConnectionPtr->state = BT_SET_PAYLOAD_SIZE;		
		}
		else
		{
#ifdef PRINT_DEBUG_INFO					
			printf("Error code: 0x%02x\r\n", status);
#endif // PRINT_DEBUG_INFO	
		}					
		if(bleConnectionPtr->remoteFeaturesReadDoneFlag == 1)
		{
			bleConnectionPtr->remoteFeaturesReadDoneFlag = 0;	
		}	
		break;
		}
	//=============================
	case BT_SET_PAYLOAD_SIZE:
	{
		status = hci_le_set_data_length(bleConnectionPtr->connectionParameters.connectionHandle,
																		HCI_LE_TX_OCTETS,//tx octets - max value is 251
																		HCI_LE_TX_TIME);//1 us for bit * 8 * 251
#ifdef PRINT_DEBUG_INFO
		printf("====================================\r\n");
		printf("Client: hci_le_set_data_length()\r\n");
#endif	//PRINT_DEBUG_INFO				
		if(status == BLE_STATUS_SUCCESS)
		{
#ifdef PRINT_DEBUG_INFO					
			printf("OK\r\n");
			uint16_t hciLeTxOctets = HCI_LE_TX_OCTETS;
			uint16_t hciLeTxTime = HCI_LE_TX_TIME;
			printf("Set hci le Tx octets for controller: %03d\r\n", hciLeTxOctets);
			printf("set hci le Tx time for controller: %04d\r\n", hciLeTxTime);	
#endif	//PRINT_DEBUG_INFO						
			bleConnectionPtr->state = BT_CONNECTED;	
		}
		else if(status == 0x3A)
		{
#ifdef PRINT_DEBUG_INFO			
			printf("Error code: 0x%02x\r\n (controller busy), repeat request\r\n", status);
#endif //PRINT_DEBUG_INFO					
		}
		else
		{
#ifdef PRINT_DEBUG_INFO							
			printf("Error code: 0x%02x\r\n", status);
#endif	//PRINT_DEBUG_INFO					
		}
		//check the complition
		if(bleConnectionPtr->payloadSizeSetDoneFlag == 1)
		{
			bleConnectionPtr->payloadSizeSetDoneFlag = 0;
			bleConnectionPtr->state = BT_CONNECTED;				
		}
		break;
	}
	//=============================		
	case BT_READ_PAYLOAD_SIZE:
	{
		if(bleConnectionPtr->payloadSizeReadDoneFlag == 1)
		{
			bleConnectionPtr->payloadSizeReadDoneFlag = 0;
			bleConnectionPtr->state = BT_CONNECTED;
		}
		break;
	}
	//=============================	
	case BT_CONNECTED:
	{
		if(bleConnectionPtr->l2capConnectionStartPerm != 1)
		{
			bleConnectionPtr->l2capConnectionStartPerm = 1;//permission for enable L2CAPHandler
		}
		if(bleConnectionPtr->disconectCmd == 1)
		{
			bleConnectionPtr->disconectCmd = 0;
			bleConnectionPtr->state = BT_START_DISCONNECTION;
		}
		if(bleConnectionPtr->disconnectedFlag == 1)
		{
			bleConnectionPtr->disconnectedFlag = 0;
			bleConnectionPtr->state = BT_DISCONNECTED;
		}
		break;
	}
	//=============================	
	case BT_START_DISCONNECTION:
	{
		//wait until L2CAP connection will be disconnected
		status = hci_disconnect(bleConnectionPtr->connectionParameters.connectionHandle, 0x15);
		if(status == BLE_STATUS_SUCCESS)
		{
#ifdef PRINT_DEBUG_INFO
			printf("====================================\r\n");	
			printf("Client: hci_disconnect() successfuly disconnect\r\n");
#endif //PRINT_DEBUG_INFO
			bleConnectionPtr->state = BT_DISCONNECTING;
		}
		else
		{
#ifdef PRINT_DEBUG_INFO			
			printf("====================================\r\n");	
			printf("Client: hci_disconnect() successfuly disconnect\r\n");	
#endif //PRINT_DEBUG_INFO	
		}
		break;
	}	
	//=============================
	case BT_DISCONNECTING:
	{
		if(bleConnectionPtr->disconnectedFlag == 1)
		{
			bleConnectionPtr->disconnectedFlag = 0;
			bleConnectionPtr->state = BT_DISCONNECTED; 
		}
		else
		{
			return;
		}
		break;
	}		
	//=============================	
	case BT_AUTOCONNECT:
	{
		bleConnectionPtr->state = BT_START_GAP_CONNECTION;
		break;
	}
	//=============================	
	case BT_DISCONNECTED:
	{
		bleConnection.l2capConnectionStartPerm = 0;
		if(bleConnectionPtr->autoconnectEn == 1)
		{
			bleConnectionPtr->state = BT_AUTOCONNECT;
		}
		break;
			
	//=============================		
	default:
		printf("bleConnectionPtr: invalid value\r\n");
		break;
	}		
	//=============================		
	}
}
//=============================================================================
//L2CAP CONNECTON
void L2capClientConnectionHandler(l2capConnection_t *l2capConnectionPtr)
{
	if(l2capConnectionPtr->StartConnectionEn == 0)
	{
		l2capConnectionPtr->state = L2CAP_IDLE;
		L2capConnectionInit(l2capConnectionPtr);
	}	
	if(l2capConnectionPtr->bluetoothConnectionPtr->l2capConnectionStartPerm == 0)
	{
		l2capConnectionPtr->state = L2CAP_READY_TO_CONNECTION;
	}
	switch(l2capConnectionPtr->state)
	{
	//=============================		
	case L2CAP_IDLE:
	{
		if(l2capConnectionPtr->StartConnectionEn == 1)
		{
			l2capConnectionPtr->state = L2CAP_READY_TO_CONNECTION;
		}
		else
		{
			return;
		}
		break;
	}
	//=============================
	case L2CAP_READY_TO_CONNECTION:
	{
		if(l2capConnectionPtr->bluetoothConnectionPtr->l2capConnectionStartPerm == 1)
		{
			l2capConnectionPtr->state = L2CAP_SEND_REQUEST_TO_CONNECTION;
		}
		break;
	}
	//=============================
	case L2CAP_SEND_REQUEST_TO_CONNECTION:
	{		
		uint8_t status = aci_l2cap_cfc_connection_req(l2capConnectionPtr->bluetoothConnectionPtr->connectionParameters.connectionHandle,
																										l2capConnectionPtr->connectionParameters.SPSM,//dynamic SPSM
																										l2capConnectionPtr->connectionParameters.CID,//Chanel ID
																										l2capConnectionPtr->connectionParameters.MTU,//MTU size
																										l2capConnectionPtr->connectionParameters.MPS,//MPS size
																										l2capConnectionPtr->connectionParameters.CFCPolicy,//automatic credit control
																										l2capConnectionPtr->connectionParameters.RXSDUbufferSize,
																										l2capConnectionPtr->connectionParameters.RXSDUBufferPtr);				
#ifdef PRINT_DEBUG_INFO				
		printf("====================================\r\n");
		printf("Client: aci_l2cap_cfc_connection_req()");
#endif //PRINT_DEBUG_INFO			
		if(status == BLE_STATUS_SUCCESS)
		{
#ifdef PRINT_DEBUG_INFO
			printf(" OK\r\n");
#endif //PRINT_DEBUG_INFO
			l2capConnectionPtr->state = L2CAP_WAIT_RESPONCE_CONNECTION;
		}
		else
		{
#ifdef PRINT_DEBUG_INFO
			printf(" Error code: 0x%02x\r\n", status);	
#endif //PRINT_DEBUG_INFO				
		}
		break;
	}		
	//=============================
	case L2CAP_SEND_RESPONCE_TO_CONNECTION:
	{	
		break;
	}
	//=============================
	case L2CAP_WAIT_RESPONCE_CONNECTION:
	{
		if(l2capConnectionPtr->ConnectedFlag == 1)
		{
			l2capConnectionPtr->ConnectedFlag = 0;
			l2capConnectionPtr->state = L2CAP_CONNECTED;
		}
		break;
	}
	//=============================
	case L2CAP_SEND_CREDITS:
	{
		uint16_t curretCredits;
		uint8_t status = aci_l2cap_send_flow_control_credits(l2capConnectionPtr->bluetoothConnectionPtr->connectionParameters.connectionHandle,
																													 l2capConnectionPtr->connectionParameters.CID,
																													 32000,//credits
																													 0x01,//automatic credit control
																													 &curretCredits);
#ifdef PRINT_DEBUG_INFO				
		printf("====================================\r\n");
		printf("Client: aci_l2cap_send_flow_control_credits()");
#endif //PRINT_DEBUG_INFO	
		if(status == BLE_STATUS_SUCCESS)
		{
#ifdef PRINT_DEBUG_INFO
			printf(" OK\r\n");	
#endif //PRINT_DEBUG_INFO
			l2capConnectionPtr->state = L2CAP_CONNECTED;
		}
		else
		{
#ifdef PRINT_DEBUG_INFO
			printf(" Error code: 0x%02x\r\n", status);	
#endif //PRINT_DEBUG_INFO		
		}
#ifdef PRINT_DEBUG_INFO
		printf("Current credit: %06d\r\n\r\n", curretCredits);	
#endif //PRINT_DEBUG_INFO						
		break;
	}      
	//=============================                
	case L2CAP_CONNECTED:
	{				
		if(l2capConnectionPtr->disconectCmd == 1)
		{
			l2capConnectionPtr->disconectCmd = 0;
			l2capConnectionPtr->state = L2CAP_START_DISCONNECT;}
			break;
		}
	//=============================
	case L2CAP_START_DISCONNECT:
	{
		uint8_t status = aci_l2cap_disconnect(l2capConnectionPtr->bluetoothConnectionPtr->connectionParameters.connectionHandle,
													 l2capConnectionPtr->connectionParameters.CID);
#ifdef PRINT_DEBUG_INFO
		printf("====================================\r\n");
		printf("Client: aci_l2cap_disconnect()");
#endif// PRINT_DEBUG_INFO			
		if(status == BLE_STATUS_SUCCESS)
		{
#ifdef PRINT_DEBUG_INFO				
			printf(" OK\r\n");	
#endif //PRINT_DEBUG_INFO					
			l2capConnectionPtr->state = L2CAP_DISCONNECTED;	
		}
		else
		{
#ifdef PRINT_DEBUG_INFO
			printf(" Error code: 0x%02x", status);	
#endif //PRINT_DEBUG_INFO					
		}
		break;
	}
	//=============================		
	case L2CAP_DISCONNECTING:
	{
		if(l2capConnectionPtr->disconnectedFlag == 1)
		{
			l2capConnectionPtr->disconnectedFlag = 0;
			l2capConnectionPtr->state = L2CAP_DISCONNECTED;
		}
		break;
	}
	//=============================
	case L2CAP_DISCONNECTED:
	{
		l2capConnectionPtr->state = L2CAP_READY_TO_CONNECTION;
		break;
	}
	//=============================
	}
}
//=============================================================================
/*
	IDLE_EXCHANGE = 0,
	//transmit data states
	READY_TO_TRANSMIT,
	TRANSMITTING,
	TRANSMIT_DONE,
	//receive data states
	READY_TO_RECEIVE,
	RECIEVING_DONE
*/
void L2capDataExchangeHandler(l2capDataExchangeState_t l2capDataExchangeState)
{

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
/*
uint16_t connection handle
uint16_t SPSM = 80 (fixed in the throughput example)
uint16_t CID = 40 (in the throughput example)
uint16 MTU = 0x7D00 (32000)
uint16 MPU = 1024 (in the throughput example)
uint8_t CID_Count = 550 (in the throughput example)

*/
void L2CAPMakeConnection(void)
{
//	aci_l2cap_cfc_connection_req(
}
//=============================================================================
void BleConnectionInit(bluetoothConnection_t *bleConnectionPtr)
{
	bleConnectionPtr->autoconnectEn = 0;
	bleConnectionPtr->connectedFlag = 0;
	bleConnectionPtr->disconectCmd = 0;
	bleConnectionPtr->connectedFlag = 0;
	bleConnectionPtr->startGapConnectionEn = 0;
	bleConnectionPtr->startLLConnectionEn = 0;
	bleConnectionPtr->state = BT_IDLE;
	bleConnectionPtr->prevState = 255;
	bleConnectionPtr->payloadSizeSetDoneFlag = 0;	
	bleConnectionPtr->payloadSizeReadDoneFlag = 0;
	bleConnectionPtr->remoteFeaturesReadDoneFlag = 0;
}
//=============================================================================
void L2capConnectionInit(l2capConnection_t *l2capConnectionPtr)
{	
	l2capConnectionPtr->StartConnectionEn = 0;
	l2capConnectionPtr->state = L2CAP_IDLE;
	l2capConnectionPtr->ConnectedFlag = 0;
}
//=============================================================================
//=============================================================================