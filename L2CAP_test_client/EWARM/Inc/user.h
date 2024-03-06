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

	 
#define HCI_LE_TX_OCTETS		251	//set txOctets for controller
#define HCI_LE_TX_TIME 			((HCI_LE_TX_OCTETS + 14) * 8)	//set txTime for controller	 

//Connecton configuration
#define CONN_INTERVAL_MIN   ((uint16_t)(10.00/1.25))		// ms for 1M
#define CONN_INTERVAL_MAX   ((uint16_t)(10.00/1.25))		// ms for 1M
#define CE_LENGTH_MIN				((uint16_t)(10.00/0.625))	// ms
#define CE_LENGTH_MAX				((uint16_t)(10.00/0.625))	// ms
#define SUPERVISION_TIMEOUT ((uint16_t)(100/10))			// ms
#define SLAVE_LATENCY				0	 
	 
	 
typedef enum
{
	BT_IDLE = 0,
	BT_START_GAP_CONNECTION,
	BT_START_LL_CONNECTION,
	BT_CONNECTING,
	BT_READ_FEATURES,
	BT_SET_PAYLOAD_SIZE,
	BT_READ_PAYLOAD_SIZE,
	BT_CONNECTED,
	BT_START_DISCONNECTION,
	BT_DISCONNECTING,
	BT_DISCONNECTED,
	BT_AUTOCONNECT
}bluetoothConnectionState_t;

typedef struct
{
	uint8_t PHY; //0x01 = LE_1M_PHY_BIT, 0x02 = LE_2M_PHY_BIT, 0x04 = LE_CODED_PHY_BIT
	uint8_t addressType;
	uint8_t peerAddress[6];
	uint16_t connectionHandle;
}
bluetoothConnectonParameters_t;

typedef struct
{
	bluetoothConnectonParameters_t connectionParameters;
	bluetoothConnectionState_t state;
	bluetoothConnectionState_t prevState;
	uint8_t firstEntryToState;
	//enables
	uint8_t autoconnectEn;
	uint8_t startGapConnectionEn;
	uint8_t startLLConnectionEn;
	//flags
	uint8_t connectedFlag;
	uint8_t disconnectedFlag;
	uint8_t remoteFeaturesReadDoneFlag;
	uint8_t payloadSizeSetDoneFlag;
	uint8_t payloadSizeReadDoneFlag;

	//commands
	uint8_t disconectCmd;
	//permissions
	uint8_t l2capConnectionStartPerm;	
}bluetoothConnection_t;

typedef struct
{
//	uint16_t connectionHandle;//value is taken from ble connection
	uint16_t SPSM;// Simplified Protocol/Service Multiplexer 
								//0x0001...0x007F = static
								//0x007F...0x00FF = dynamic
	uint16_t CID;//endpoint value = 0x0001...0x00FF
	uint16_t MTU;//maximum SDU size = 23...65535
	uint16_t MPS;// maximum PDU zise = 23...1024
	uint8_t CFCPolicy;//flow control policy
										//L2CAP_CFC_MANUAL = 0x00
										//L2CAP_AUTO = 0x01
	uint16_t RXSDUbufferSize;
	uint8_t *RXSDUBufferPtr;


}l2capconnectionParameters_t;

typedef enum
{
	L2CAP_IDLE = 0,
	L2CAP_READY_TO_CONNECTION,
	L2CAP_SEND_REQUEST_TO_CONNECTION,
	L2CAP_SEND_RESPONCE_TO_CONNECTION,
	L2CAP_WAIT_RESPONCE_CONNECTION,
  L2CAP_SEND_CREDITS,
	L2CAP_CONNECTED,
	L2CAP_START_DISCONNECT,
	L2CAP_DISCONNECTING,
	L2CAP_DISCONNECTED,
	L2CAP_AUTOCONNECT
} l2capConnectionState_t;

typedef struct
{
	l2capconnectionParameters_t connectionParameters;
	l2capConnectionState_t state;
	const bluetoothConnection_t *bluetoothConnectionPtr;
	//enables
	uint8_t StartConnectionEn;
	//flags
	uint8_t ConnectedFlag;
	uint8_t disconnectedFlag;
	uint8_t disconectCmd;


}l2capConnection_t;


typedef enum
{
	IDLE_EXCHANGE = 0,
	//transmit data states
	READY_TO_TRANSMIT,
	TRANSMITTING,
	TRANSMIT_DONE,
	//receive data states
	READY_TO_RECEIVE,
	RECIEVING_DONE
} l2capDataExchangeState_t;


extern bluetoothConnection_t bleConnection;
extern l2capConnection_t l2capConnection;
extern uint8_t rxBuffer[32000 + 2];
extern uint8_t l2capConnectedFlag;

/**
  * @brief  This function initializes the BLE GATT & GAP layers and it sets the TX power level 
  * @param  None
  * @retval None
  */
void device_initialization(void);


/**
  * @brief  User Application tick 
  * @param  None
  * @retval None
  */
void APP_Tick(void);

void TestClockSys(void);
void BluetoothConnectionHandler(bluetoothConnection_t *bleConnectionPtr);
void L2capClientConnectionHandler(l2capConnection_t *l2capConnectionPtr);
void L2capDataExchangeHandler(l2capDataExchangeState_t l2capDataExchangeState);
void ComPortRead(uint8_t * pRxDataBuffer, uint16_t DataSize);
void BleConnectionInit(bluetoothConnection_t *bleConnectionPtr);
void L2capConnectionInit(l2capConnection_t *l2capConnectionPtr);

#endif /* _user_H_ */
/** \endcond 
*/
