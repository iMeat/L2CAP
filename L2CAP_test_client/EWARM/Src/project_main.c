/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name         : project_main.c
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
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "bluenrg_lp_it.h"
#include "ble_const.h"

#include "rf_driver_hal_power_manager.h"
#include "rf_driver_hal_vtimer.h"
#include "bluenrg_lp_stack.h"
#include "bluenrg_lp_evb_com.h"
#include "bleplat.h"
#include "nvm_db.h"
#include "osal.h"
#include "user.h"
#include "pka_manager.h"
#include "rng_manager.h"
#include "aes_manager.h"
#include "ble_controller.h"
#include "User_config.h"
#include "bluenrg_lp_hw_config.h"
#include "clock.h"

#ifndef DEBUG
#define DEBUG 0
#endif

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BLE_USER_VERSION_STRING /"1.0.0/"
/* Private macro -------------------------------------------------------------*/
NO_INIT(uint32_t dyn_alloc_a[DYNAMIC_MEMORY_SIZE>>2]);
/* Private variables ---------------------------------------------------------*/
WakeupSourceConfig_TypeDef WakeupSourceConfig = {0};
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

//Advertizing settings
#define SCAN_INTERVAL_MS 		100
#define	SCAN_WINDOW_MS			100

//L2CAP settings
#define L2CAP_MPU						1024	//23...1024
#define L2CAP_MTU						32000	//23...65553
//
#define	PDU_SUGGESTED_MAX_TX_OCTETS	(uint16_t) 251 // max value is 251
#define	PDU_SUGGESTED_MAX_TX_TIME (uint16_t)((PDU_SUGGESTED_MAX_TX_OCTETS + 14) * 8)
//
#define GAP_CONNECTION			0
#define LL_CONNECTION				1

#if GAP_CONNECTION == 1
#undef LL_CONNECTION
#endif	//GAP_CONNECTION

bluetoothConnection_t bleConnection;
l2capConnection_t l2capConnection;
uint16_t inputDataSize;

//	bluetoothConnection_t bleConnection = {LE_1M_PHY, 0x00, {0xA2, 0x00, 0x00, 0xE1, 0x80, 0x03}, BT_IDLE};
//	bluetoothConnection_t bleConnection = {LE_1M_PHY, 0x00, {0x03, 0x80, 0xE1, 0x00, 0x00, 0xA2}, BT_IDLE};	
//=============================================================================
void ModulesInit(void)
{
  uint8_t ret;

  BLE_STACK_InitTypeDef BLE_STACK_InitParams = BLE_STACK_INIT_PARAMETERS;
  
  LL_AHB_EnableClock(LL_AHB_PERIPH_PKA|LL_AHB_PERIPH_RNG);
 
  /* BlueNRG-LP stack init */
  ret = BLE_STACK_Init(&BLE_STACK_InitParams);
	
	
  if (ret != BLE_STATUS_SUCCESS)
  {
    printf("Error in BLE_STACK_Init() 0x%02x\r\n", ret);
    while(1);
  }
	else
	{
#ifdef PRINT_DEBUG_INFO
		printf("====================================\r\n");	 
		printf("CLIENT BLE STACK SETTINGS\r\n");
		printf("TotalBuffersize: \t\t%06d\r\n",BLE_STACK_InitParams.TotalBufferSize);
		printf("NumAttrRecords: \t\t%06d\r\n",BLE_STACK_InitParams.NumAttrRecords);
		printf("MaxNumOfClientProcs: \t\t%06d\r\n",BLE_STACK_InitParams.MaxNumOfClientProcs);
		printf("NumOfLinks: \t\t%06d\r\n",BLE_STACK_InitParams.NumOfLinks);
		printf("NumOfEATTChannels: \t\t%06d\r\n",BLE_STACK_InitParams.NumOfEATTChannels);
		printf("NumBlockCount: \t\t%06d\r\n",BLE_STACK_InitParams.NumBlockCount);
		printf("ATT_MTU: \t\t\t%06d\r\n",BLE_STACK_InitParams.ATT_MTU);
		printf("MaxConnEventLength: \t\t%08x\r\n",BLE_STACK_InitParams.MaxConnEventLength);
		printf("SleepClockAccuracy: \t\t%06d\r\n",BLE_STACK_InitParams.SleepClockAccuracy);
		printf("NumOfAdvDataSet: \t\t%06d\r\n",BLE_STACK_InitParams.NumOfAdvDataSet);
		printf("NumOfAuxScanSlots: \t\t%06d\r\n",BLE_STACK_InitParams.NumOfAuxScanSlots);
		printf("WhiteListSizeLog2: \t\t%06d\r\n",BLE_STACK_InitParams.WhiteListSizeLog2);
		printf("L2CAP_MPS: \t\t\t%06d\r\n",BLE_STACK_InitParams.L2CAP_MPS);
		printf("L2CAP_NumChannels: \t\t%06d\r\n",BLE_STACK_InitParams.L2CAP_NumChannels);	
		printf("NumOfSyncSlots: \t\t%06d\r\n",BLE_STACK_InitParams.NumOfSyncSlots);
		printf("CTE_MaxNumAntennaIDs: \t\t%06d\r\n",BLE_STACK_InitParams.CTE_MaxNumAntennaIDs);
		printf("CTE_MaxNumIQSamples: \t\t%06d\r\n",BLE_STACK_InitParams.CTE_MaxNumIQSamples);
		printf("isr0_fifo_size: \t\t%06d\r\n",BLE_STACK_InitParams.isr0_fifo_size);
		printf("isr1_fifo_size: \t\t%06d\r\n",BLE_STACK_InitParams.isr1_fifo_size);
		printf("user_fifo_size: \t\t%06d\r\n",BLE_STACK_InitParams.user_fifo_size);
#endif	//PRINT_DEBUG_INFO
	}
	BLECNTR_InitGlobal();
	HAL_VTIMER_InitType VTIMER_InitStruct = {HS_STARTUP_TIME, INITIAL_CALIBRATION, CALIBRATION_INTERVAL};
	HAL_VTIMER_Init(&VTIMER_InitStruct);
	BLEPLAT_Init();  
	if (PKAMGR_Init() == PKAMGR_ERROR)
	{
		while(1);
	}
	if (RNGMGR_Init() != RNGMGR_SUCCESS)
  {
	while(1);
	}
    /* Init the AES block */
	AESMGR_Init();
}
//=============================================================================
void ModulesTick(void)
{
  /* Timer tick */
  HAL_VTIMER_Tick();
  /* Bluetooth stack tick */
  BLE_STACK_Tick();
  /* NVM manager tick */
  NVMDB_Tick();
}
//=============================================================================
/* User callback if an interrupt is associated to the wakeup source */
void HAL_PWR_MNGR_WakeupIOCallback(uint32_t source)
{
}
//=============================================================================
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{	
//  PowerSaveLevels stopLevel;//ZAO disable
  /* System initialization function */
	if(SystemInit(SYSCLK_64M, BLE_SYSCLK_32M) != SUCCESS)
	{
    /* Error during system clock configuration take appropriate action */
		while(1);
  }
  
	//bluetooth setting
	bleConnection.connectionParameters.PHY = LE_1M_PHY;
	bleConnection.connectionParameters.addressType = 0x00;//public address
	bleConnection.connectionParameters.peerAddress[0] = 0xA2;
	bleConnection.connectionParameters.peerAddress[1] = 0x00;
	bleConnection.connectionParameters.peerAddress[2] = 0x00;
	bleConnection.connectionParameters.peerAddress[3] = 0xE1;
	bleConnection.connectionParameters.peerAddress[4] = 0x80;
	bleConnection.connectionParameters.peerAddress[5] = 0x03;
	//init
	BleConnectionInit(&bleConnection);
	bleConnection.autoconnectEn = 1;
//L2CAP settings
	l2capConnection.bluetoothConnectionPtr = &bleConnection;
	l2capConnection.connectionParameters.SPSM = 0x0080;
	l2capConnection.connectionParameters.CID = 0x0040;
	l2capConnection.connectionParameters.MPS = L2CAP_MPU;	//23...1024
	l2capConnection.connectionParameters.MTU = L2CAP_MTU; //23...65553
	l2capConnection.connectionParameters.CFCPolicy = 0x01;
	l2capConnection.connectionParameters.RXSDUbufferSize = 32000 + 2;
	l2capConnection.connectionParameters.RXSDUBufferPtr = rxBuffer;

	L2capConnectionInit(&l2capConnection);
	l2capConnection.StartConnectionEn = 1;
  bluenrg_lp_initialization();
//   WakeupSourceConfig.RTC_enable = 0;
//   WakeupSourceConfig.LPU_enable = 0;
   /* Init BLE stack, HAL virtual timer and NVM modules */\
	BSP_COM_Init(&ComPortRead);
	ModulesInit(); 
	Clock_Init();
  /* Init the Bluetooth LE stack layers */
	device_initialization();
	TestClockSys();
	uint8_t scan_filter = SCAN_ACCEPT_ALL;
	uint8_t status;
#if GAP_CONNECTION == 1
	//settings GAP connection configuration
	status = aci_gap_set_scan_configuration(DUPLICATE_FILTER_DISABLED, 
																					scan_filter, 
																					LE_1M_PHY_BIT, 
																					PASSIVE_SCAN,
																					SCAN_INTERVAL_MS*1000/625,
																					SCAN_WINDOW_MS*1000/625);
	

	if(status != BLE_STATUS_SUCCESS)
	{printf("====Client: aci_gap_set_scan_configuration() failed:0x%02x\r\n", status);
	}else
	{printf("====Client: aci_gap_set_scan_configuration() success\r\n");
	}
	
	status = aci_gap_set_connection_configuration(LE_1M_PHY_BIT,
																								CONN_INTERVAL_MIN,
																								CONN_INTERVAL_MAX,
																								SLAVE_LATENCY,
																								SUPERVISION_TIMEOUT,
																								CE_LENGTH_MIN,
																								CE_LENGTH_MAX);	
	
		bleConnection.startGapConnectionEn = 1;//enable
#endif	//GAP_CONNECTINO	
		
#ifdef LL_CONNECTION		
	bleConnection.startLLConnectionEn = 1;
#endif //LL_CONNECTION
		
	//set PHY
	status = hci_le_set_default_phy(0x00, LE_2M_PHY, LE_2M_PHY);
#ifdef PRINT_DEBUG_INFO
	printf("====================================\r\n");	
	printf("Client: hci_le_set_default_phy()\r\n");
	if(status == BLE_STATUS_SUCCESS)
	{
		printf("OK");
	}
	else
	{
		printf("Error code: 0x%02x\r\n", status);
	}	
#endif //PRINT_DEBUG_INFO		
		
	//read controller buffer size
	uint16_t hcLeAclDataPacketLength = 0;
	uint8_t	hcTotalNumLeAclDataPackets = 0;
	status = hci_le_read_buffer_size(&hcLeAclDataPacketLength, &hcTotalNumLeAclDataPackets);
#ifdef PRINT_DEBUG_INFO
	printf("====================================\r\n");	 
	printf("Client: hci_le_read_buffer_size()\r\n");
	if(status == BLE_STATUS_SUCCESS)
	{
		printf("HC LE ACL data packet length: %06d\r\n", hcLeAclDataPacketLength);
		printf("HC total num of data packets: %03d\r\n", hcTotalNumLeAclDataPackets);
	}
	else
	{
		printf("Error code: 0x%02x\r\n", status);
	}
	//set payload size for controller
//	status = hci_le_set_data_length
	// read payload size from controller
#endif //PRINT_DEBUG_INFO	
	
#ifdef PRINT_DEBUG_INFO
	//read PDU size
	uint16_t suggestedMaxTxOctets;
	uint16_t suggestedMaxTxTime;
	status = hci_le_read_suggested_default_data_length(&suggestedMaxTxOctets,
																									&suggestedMaxTxTime);
	printf("====================================\r\n");	
	printf("Client: hci_le_read_suggested_default_data_length()\r\n");
	if(status == BLE_STATUS_SUCCESS)
	{
		printf("OK\r\n");
		printf("Read suggestedMaxTxOctets: %04d\r\n", suggestedMaxTxOctets);
		printf("Read suggestedMaxTxTime: %04d\r\n", suggestedMaxTxTime);
	}
	else
	{
		printf("Error code: 0x%02x", status);
	}
#endif	//PRINT_DEBUG_INFO
	//write PDU size
#ifdef PRINT_DEBUG_INFO
	status = hci_le_write_suggested_default_data_length(PDU_SUGGESTED_MAX_TX_OCTETS,
																									PDU_SUGGESTED_MAX_TX_TIME);
	printf("====================================\r\n");	
	printf("Client: hci_le_write_suggested_default_data_length()\r\n");
	if(status == BLE_STATUS_SUCCESS)
	{
		printf("OK\r\n");
		printf("Set suggestedMaxTxOctets: %04d\r\n", PDU_SUGGESTED_MAX_TX_OCTETS);
		printf("Set suggestedMaxTxTime: %04d\r\n", PDU_SUGGESTED_MAX_TX_TIME);
	}
	else
	{
		printf("Error code: 0x%02x", status);
	}
#endif	//PRINT_DEBUG_INFO
	
#ifdef PRINT_DEBUG_INFO
	//repeat read PDU size
	status = hci_le_read_suggested_default_data_length(&suggestedMaxTxOctets,
																									&suggestedMaxTxTime);
	printf("====================================\r\n");	
	printf("Client: hci_le_read_suggested_default_data_length()\r\n");
	if(status == BLE_STATUS_SUCCESS)
	{
		printf("OK\r\n");
		printf("Read suggestedMaxTxOctets: %04d\r\n", suggestedMaxTxOctets);
		printf("Read suggestedMaxTxTime: %04d\r\n", suggestedMaxTxTime);
	}
	else
	{
		printf("Error code: 0x%02x", status);
	}
#endif	//PRINT_DEBUG_INFO	
//==============================	 
  while (1) {
    /* BlueNRG-LP stack tick */
		PA5_ON();
    ModulesTick();
		PA5_OFF();
		BluetoothConnectionHandler(&bleConnection);
		L2capClientConnectionHandler(&l2capConnection);
		//set indication
    if(bleConnection.state == BT_CONNECTED)
		{
			LED_RED_OFF();
			LED_GREEN_ON();
		}
		else
		{
			LED_RED_ON();
			LED_GREEN_OFF();
		}
    /* Application Tick */
    APP_Tick();
//==============================	     
    /* Power Save Request */
    //HAL_PWR_MNGR_Request(POWER_SAVE_LEVEL_CPU_HALT, WakeupSourceConfig, &stopLevel);
  }
}
/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/
PowerSaveLevels App_PowerSaveLevel_Check(PowerSaveLevels level)
{
  if(BSP_COM_TxFifoNotEmpty() || BSP_COM_UARTBusy())
    return POWER_SAVE_LEVEL_RUNNING;
  return POWER_SAVE_LEVEL_STOP_NOTIMER;
}
/* Hardware Error event. 
   This event is used to notify the Host that a hardware failure has occurred in the Controller. 
   Hardware_Code Values:
   - 0x01: Radio state error
   - 0x02: Timer overrun error
   - 0x03: Internal queue overflow error
   - 0x04: Late Radio ISR
    After this event with error code 0x01, 0x02 or 0x03, it is recommended to force a device reset.
  */
//=============================================================================
/**
 * @brief  Hardware Error event
 * @param  uint8_t Hardware_Code.
 * @retval void.
*/
void hci_hardware_error_event(uint8_t Hardware_Code)
{
  if (Hardware_Code <= 0x03)
  {
    NVIC_SystemReset();
  }
}
//=============================================================================
/**
  * This event is generated to report firmware error informations.
  * FW_Error_Type possible values: 
  * Values:
  - 0x01: L2CAP recombination failure
  - 0x02: GATT unexpected response
  - 0x03: GATT unexpected request
    After this event with error type (0x01, 0x02, 0x3) it is recommended to disconnect. 
*/
void aci_hal_fw_error_event(uint8_t FW_Error_Type,
                            uint8_t Data_Length,
                            uint8_t Data[])
{
  if (FW_Error_Type <= 0x03)
  {
    uint16_t connHandle;
    
    /* Data field is the connection handle where error has occurred */
    connHandle = LE_TO_HOST_16(Data);
    
    aci_gap_terminate(connHandle, BLE_ERROR_TERMINATED_REMOTE_USER); 
  }
}
//=============================================================================
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/