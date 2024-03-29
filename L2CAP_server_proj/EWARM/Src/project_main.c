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

#define TX_BUFFER_SIZE 32000
// PDU SETTINGS
#define	PDU_SUGGESTED_MAX_TX_OCTETS	(uint16_t) 251 // max value is 251
#define	PDU_SUGGESTED_MAX_TX_TIME (uint16_t)((PDU_SUGGESTED_MAX_TX_OCTETS + 14) * 8)

uint32_t time;
l2capConnectionServer_t l2capConnectionServer;
uint8_t txBuffer[TX_BUFFER_SIZE + 2];
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
#endif //PRINTF_DEBUG_INFO	
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
	PowerSaveLevels stopLevel;
  /* System initialization function */
  if (SystemInit(SYSCLK_64M, BLE_SYSCLK_32M) != SUCCESS)
	{
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  Clock_Init();
	bluenrg_lp_initialization();
	BufferInit(txBuffer, 32000, 0xFF);
	L2capConnectionServerInit(&l2capConnectionServer);
	l2capConnectionServer.localCID = 0x41;
	l2capConnectionServer.sendDataEn = 1;
	l2capConnectionServer.state = IDLE;
	WakeupSourceConfig.RTC_enable = 0;
  WakeupSourceConfig.LPU_enable = 0;
   /* Init BLE stack, HAL virtual timer and NVM modules */
	BSP_COM_Init(&ComPortRead);	
  ModulesInit(); 
  /*NOTE: In order to generate an user-friendly C code and helping user on his coding post-build activities, 
      the generated BLE stack APIs are placed within some predefined C functions (device_initialization(), set_database(), device_scanning(), 
      set_device_discoverable(), set_connection(), set_connection_update(), discovery(), read_write_characteristics() and update_characterists()).

      Once C code has been generated, user can move and modify APIs sequence order, within and outside these default user functions, 
      according to his specific needs.
  */
  /* Init the Bluetooth LE stack layers */
	device_initialization();
	time = Clock_Time();
	uint8_t result = SysTick_Config(64000);
	result = result + 0;
	//read controller buffer size
	uint16_t hcLeAclDataPacketLength = 0;
	uint8_t	hcTotalNumLeAclDataPackets = 0;
	uint8_t status = hci_le_read_buffer_size(&hcLeAclDataPacketLength, &hcTotalNumLeAclDataPackets);
#ifdef PRINT_DEBUG_INFO
	printf("====================================\r\n");	 
	printf("Server: hci_le_read_buffer_size()\r\n");
	if(status == BLE_STATUS_SUCCESS)
	{
		printf("HC LE ACL data packet length: %06d\r\n", hcLeAclDataPacketLength);
		printf("HC total num of data packets: %03d\r\n", hcTotalNumLeAclDataPackets);
	}
	else
	{
		printf("Error code: 0x%02x\r\n", status);
	}
#endif //PRINT_DEBUG_INFO	 
	//read PDU size
	uint16_t suggestedMaxTxOctets;
	uint16_t suggestedMaxTxTime;
	//read PDU size	
	#ifdef PRINT_DEBUG_INFO		
	status = hci_le_read_suggested_default_data_length(&suggestedMaxTxOctets,
																									&suggestedMaxTxTime);
	printf("====================================\r\n");	
	printf("Server: hci_le_read_suggested_default_data_length()\r\n");
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
	status = hci_le_write_suggested_default_data_length(PDU_SUGGESTED_MAX_TX_OCTETS,
																									PDU_SUGGESTED_MAX_TX_TIME);
#ifdef PRINT_DEBUG_INFO	
	printf("====================================\r\n");	
	printf("Server: hci_le_write_suggested_default_data_length()\r\n");
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
	printf("Server: hci_le_read_suggested_default_data_length()\r\n");
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
	//set PHY
	status = hci_le_set_default_phy(0x00, LE_2M_PHY, LE_2M_PHY);
#ifdef PRINT_DEBUG_INFO
	printf("====================================\r\n");	
	printf("Server: hci_le_set_default_phy()\r\n");
	if(status == BLE_STATUS_SUCCESS)
	{
		printf("OK");
	}
	else
	{
		printf("Error code: 0x%02x\r\n", status);
	}	
#endif //PRINT_DEBUG_INFO	
  while (1) 
	{
    /* BlueNRG-LP stack tick */
		PA1_ON();
    ModulesTick();
		PA1_OFF();
		
    L2capDataHandler(&l2capConnectionServer);
    /* Application Tick */
    APP_Tick();
    if(l2capConnectedFlag == 1)
		{
			if((Clock_Time() - time) > 1000)
			{
				if(l2capConnectionServer.state == READY_TO_SEND)
				{
					l2capConnectionServer.sendDataCmd = 1;
				}
				time = Clock_Time();
			}	
		}
    /* Power Save Request */
   //HAL_PWR_MNGR_Request(POWER_SAVE_LEVEL_CPU_HALT, WakeupSourceConfig, &stopLevel);

  }
}
//=============================================================================

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
/**
 * @brief  Hardware Error event
 * @param  uint8_t Hardware_Code.
 * @retval void.
*/
//=============================================================================
void hci_hardware_error_event(uint8_t Hardware_Code)
{
  if(Hardware_Code <= 0x03)
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
