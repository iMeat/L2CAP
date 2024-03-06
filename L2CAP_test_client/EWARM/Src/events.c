#include <stdio.h>
#include "bluenrg_lp_stack.h"
#include "user.h"
#include "ble_const.h"

//#define PRINT_DEBUG_INFO // enable the printf for additional information


#define L2CAP_CONN_SUCCESSFUL 												0x0000
#define L2CAP_CONN_FAIL_SPSM_NOT_SUPPORTED						0x0002
#define L2CAP_CONN_FAIL_INSUFFICIENT_RESOURCES				0x0004
#define L2CAP_CONN_FAIL_INSUFFICIENT_AUTHENTICATION 	0x0005
#define L2CAP_CONN_FAIL_INSUFFICIENT_AUTHORIZATION		0x0006
#define L2CAP_CONN_FAIL_KEY_SIZE_TOO_SHORT						0x0007
#define L2CAP_CONN_FAIL_INSUFFICIENT_ENCRYPTION 			0x0008
#define L2CAP_CONN_FAIL_INVALID_SOURCE_CID						0x0009
#define L2CAP_CONN_FAIL_SOURCE_CID_ALREADY_ALLOCATED	0x000A
#define L2CAP_CONN_FAIL_UNACCEPTABLE_PARAMETERS				0x000B
#define L2CAP_CONN_FAIL_INVALID_PARAMETERS						0x000C
#define L2CAP_CONN_FAIL_NO_FURTHER_INFORMATON					0x000D
#define L2CAP_CONN_FAIL_AUTHENTICATION_PENDING				0x000E
#define L2CAP_CONN_FAIL_AUTHORIZATION_PENDING					0x000F



extern bluetoothConnection_t bleConnection;
extern l2capConnection_t l2capConnection;
extern uint8_t rxDataExtracted[2000];
extern uint16_t inputDataSize;


llc_conn_per_statistic_st packetsStatistic;
static uint16_t recievedDataCounter = 0;
//=============================================================================
void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy)

{ 
#ifdef PRINT_DEBUG_INFO
	printf("====================================\r\n");
	printf("Client: hci_le_connection_complete_event()\r\n");
	if(Status == BLE_STATUS_SUCCESS)
	{
		printf("OK\r\n");
		printf("Connection Handle: 0x%04x\r\n", Connection_Handle);
		printf("Role: 0x%02x\r\n", Role);
		printf("Peer address:");
		for(uint8_t i = 0; i < 6; i++)
		{
			printf("%02x:", Peer_Address[i]);
		}
		printf("\r\n");		 
		printf("Connection interval: %05d\r\n", Conn_Interval);
		printf("Connection latency: %05d\r\n", Conn_Latency);
		printf("Supervision timeout: %05d\r\n", Supervision_Timeout);	
		printf("Master clock accuracy: %05d\r\n", Master_Clock_Accuracy);					
	}
	else
	{
		printf("Error code: 0x%02x \r\n", Status);
	}
#endif //PRINT_DEBUG_INFO	
	uint8_t txPHY = LE_2M_PHY;
	uint8_t rxPHY = LE_2M_PHY;
	
	Status = hci_le_set_phy(Connection_Handle, 0x00, txPHY, rxPHY, 0x0000);
#ifdef PRINT_DEBUG_INFO
	printf("====================================\r\n");	
	printf("Client: hci_le_set_phy()\r\n");
	if(Status == BLE_STATUS_SUCCESS)
	{
		printf("OK\r\n");
	}
	else
	{
		printf("Error code: 0x%02x\r\n", Status);
	}
	Status = hci_le_read_phy(Connection_Handle, &txPHY, &rxPHY);
	printf("====================================\r\n");
	printf("Client: hci_le_read_phy()\r\n");
	if(Status == BLE_STATUS_SUCCESS)
	{
		printf("PHY TX: %1d\r\n", txPHY);
		printf("PHY RX: %1d\r\n", rxPHY);
	}
	else
	{
		printf("Error code: 0x%02x\r\n", Status);
	}
	
#endif // PRINT_DEBUG_INFO			
	bleConnection.connectedFlag = 1;
	bleConnection.connectionParameters.connectionHandle = Connection_Handle;
}/* end hci_le_connection_complete_event() */
//=============================================================================
void hci_le_enhanced_connection_complete_event(uint8_t Status,
                                               uint16_t Connection_Handle,
                                               uint8_t Role,
                                               uint8_t Peer_Address_Type,
                                               uint8_t Peer_Address[6],
                                               uint8_t Local_Resolvable_Private_Address[6],
                                               uint8_t Peer_Resolvable_Private_Address[6],
                                               uint16_t Conn_Interval,
                                               uint16_t Conn_Latency,
                                               uint16_t Supervision_Timeout,
                                               uint8_t Master_Clock_Accuracy)
{	
  //USER ACTION IS NEEDED
  hci_le_connection_complete_event(Status,
                                   Connection_Handle,
                                   Role,
                                   Peer_Address_Type,
                                   Peer_Address,
                                   Conn_Interval,
                                   Conn_Latency,
                                   Supervision_Timeout,
                                   Master_Clock_Accuracy);
	
}
//=============================================================================
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
  //USER ACTION IS NEEDED
//  set_discoverable = TRUE;
//  PRINTF ("DISCONNECTION, Status:  0x%0x\r\n", Status);
#ifdef PRINT_DEBUG_INFO
	printf("====================================\r\n");
	printf("Client: hci_disconnection_complete_event() ");
	if(Status == BLE_STATUS_SUCCESS)
	{
		printf("OK\r\n");
	}
	else
	{
		printf("Error code: 0x%02x \r\n", Status);
	}
#endif // PRINT_DEBUG_INFO
	if(Status == BLE_STATUS_SUCCESS)
	{
		bleConnection.disconnectedFlag = 1;
	}
}/* end hci_disconnection_complete_event() */
//=============================================================================
void aci_l2cap_cos_connection_event(uint16_t Connection_Handle,
                                    uint8_t Event_Type,
                                    uint8_t Identifier,
                                    uint16_t SPSM,
                                    uint16_t Peer_MTU,
                                    uint16_t Peer_MPS,
                                    uint16_t Initial_Credits,
                                    uint16_t Result,
                                    uint8_t CID_Count,
                                    conn_cid_t conn_cid[])
{
#ifdef PRINT_DEBUG_INFO
	printf("====================================\r\n");
	printf("Client: aci_l2cap_cos_connection_event()\r\n");
#ifdef PRINT_DEBUG_INFO
	printf("====================================\r\n");
	printf("Client: aci_l2cap_cos_connection_event()\r\n");
	printf("Event type: 0x%02x\r\n", Event_Type);
	printf("Identifier: 0x%02x\r\n", Identifier);
	printf("SPSM: 0x%04x\r\n", SPSM);
	printf("Peer MTU: %06d\r\n", Peer_MTU);
	printf("Peer MPS: %04d\r\n", Peer_MPS);
	printf("Initial credit: %06d\r\n", Initial_Credits);
	printf("Result: 0x%02x\r\n", Result);
	printf("CID count: %02d\r\n", CID_Count);
	printf("CID LIST SIZE:\r\n");
	for(uint8_t i = 0; i < CID_Count; i++)
	{
		printf("CID #%02d: Local CID = 0x%04x Remote CID = 0x%04x\r\n", i, conn_cid[i].Local_CID, conn_cid[i].Remote_CID);
	}					
#endif //PRINT_DEBUG_INFO		
#endif//PRINT_DEBUG_INFO
	if(Result == L2CAP_CONN_SUCCESSFUL)
	{
		l2capConnection.ConnectedFlag = 1;
	}
	else
	{
#ifdef PRINT_DEBUG_INFO		
		printf("L2CAP connection error. Code: 0x%04x\r\n", Result);
#endif//PRINT_DEBUG_INFO	
	}
}
//=============================================================================
void aci_l2cap_sdu_data_rx_event(uint16_t Connection_Handle,
                                 uint16_t CID,
                                 uint16_t RX_Credit_Balance,
                                 uint16_t SDU_Length)
{
	PB0_ON();
	uint8_t status;
#ifdef PRINT_DEBUG_INFO
	//stop statistic counter
	llc_conn_per_statistic(Connection_Handle,
												 NULL);
	printf("====================================\r\n");		
	printf("Client: aci_l2cap_sdu_data_rx_event()\r\n");
	printf("Data #%05d has been recieved!\r\n", recievedDataCounter);
	recievedDataCounter++;
	printf("CID: 0x%04x\r\n", CID);
	printf("Rx credit balance: %06d\r\n", RX_Credit_Balance);
	printf("SDU length: %06d\r\n", SDU_Length);
#endif//PRINT_DEBUG_INFO
	PB0_OFF();
	PA13_ON();
	status = aci_l2cap_extract_sdu_data(Connection_Handle,
																			CID,
																			12000,//buffer size where data will be copped
																			rxDataExtracted,
																			&SDU_Length);
	PA13_OFF();	
#ifdef PRINT_DEBUG_INFO
	printf("====================================\r\n");		
	printf("Client: aci_l2cap_extract_sdu_data()\r\n");
	printf("====================================\r\n");
	printf("Client: ll_conn_per_statistic()\r\n");
	printf("RECIEVED DATA STATISTICS:\r\n");
	printf("Packets recieved:\t%05d\r\n", packetsStatistic.num_pkts);
	printf("CRC errors:\t\t%05d\r\n", packetsStatistic.num_crc_err);	
	printf("Events:\t\t%05d\r\n", packetsStatistic.num_evts);
	printf("Missed events:\t%05d\r\n", packetsStatistic.num_miss_evts);	
	//reset and start statistic counter for next data transmission
	llc_conn_per_statistic(Connection_Handle,
												 &packetsStatistic);
#endif//PRINT_DEBUG_INFO	
}
//=============================================================================
void aci_l2cap_sdu_data_tx_event(uint16_t Connection_Handle,
                                 uint16_t CID,
                                 uint16_t SDU_Length,
                                 void * SDU_Data_Buffer,
                                 uint16_t TX_Credit_Balance)
{
#ifdef PRINT_DEBUG_INFO	
	printf("====================================\r\n");
	printf("Client: aci_l2cap_sdu_data_tx_event()\r\n");
	printf("Data transmitted!\r\n");
	printf("CID: 0x%04x\r\n", CID);
	printf("SDU length: %06d\r\n", SDU_Length);
	printf("Tx credit balance: %06d\r\n", TX_Credit_Balance);
#endif//PRINT_DEBUG_INFO	
}
//=============================================================================
void aci_l2cap_disconnection_complete_event(uint16_t Connection_Handle,
                                            uint16_t CID)
{
#ifdef PRINT_DEBUG_INFO	
	printf("====================================\r\n");
	printf("Client: aci_l2cap_disconnection_complete_event()\r\n");
	printf("Connection: 0x%04x , channel ID: 0x%04x is closed.",
				 Connection_Handle,
				 CID);
	l2capConnection.disconnectedFlag = 1;
#endif//PRINT_DEBUG_INFO	
}
//=============================================================================
void hci_le_read_remote_used_features_complete_event(uint8_t Status,
                                                     uint16_t Connection_Handle,
                                                     uint8_t LE_Features[8])
{
#ifdef PRINT_DEBUG_INFO	
	printf("====================================\r\n");
	printf("Client: hci_le_read_remote_used_features_complete_event()\r\n");
	printf("Status code: 0x%02x\r\n",Status);
	printf("Connection Handle: 0x%04x\r\n", Connection_Handle);
	printf("LEFeature:\r\n");
	for(uint8_t i = 0; i < 8; i++)
	{
		printf("Byte %01d --- 0x%02x\r\n", i ,LE_Features[i]);
	}
	bleConnection.remoteFeaturesReadDoneFlag = 1;
#endif //PRINT_DEBUG_INFO		
}
//=============================================================================
void hci_le_data_length_change_event(uint16_t Connection_Handle,
                                     uint16_t MaxTxOctets,
                                     uint16_t MaxTxTime,
                                     uint16_t MaxRxOctets,
                                     uint16_t MaxRxTime)
{
#ifdef PRINT_DEBUG_INFO	
	printf("====================================\r\n");
	printf("Client: hci_le_data_length_change_event()\r\n");
	printf("Connection handle: 0x%04x\r\n", Connection_Handle);
	printf("MaxTxOctets: %06d\r\n",MaxTxOctets);
	printf("MaxTxTime: %06d\r\n", MaxTxTime);
	printf("MaxRxOctets: %06d\r\n", MaxRxOctets);
	printf("MaxRxTime: %06d\r\n", MaxRxTime);	
#endif //PRINT_DEBUG_INFO	
	bleConnection.payloadSizeSetDoneFlag = 1;
}

//=============================================================================	
/** \endcond 
*/