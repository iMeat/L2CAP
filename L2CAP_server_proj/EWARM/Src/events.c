#include <stdio.h>
#include "bluenrg_lp_stack.h"
#include "events.h"
#include "user.h"

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

/* ***************** BlueNRG-LP Stack Callbacks ********************************/
extern uint8_t set_discoverable;
extern uint8_t txBuffer[32000+2];
extern l2capConnectionServer_t l2capConnectionServer;
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
#endif // PRINT_DEBUG_INFO																			
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
  set_discoverable = TRUE;
#ifdef PRINT_DEBUG_INFO
	printf("====================================\r\n");
	printf("Server: hci_disconnection_complete_event() ");
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
		l2capConnectedFlag = 0;
	}	
}/* end hci_disconnection_complete_event() */
/** \endcond 
*/
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
	printf("Server: aci_l2cap_cos_connection_event()\r\n");
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
#endif	//PRINTF_DEBUG_INFO
	l2capConnectionServer.connectionHandle = Connection_Handle;
	l2capConnectionServer.remoteCID = conn_cid[0].Remote_CID;		
	if(Result == L2CAP_CONN_SUCCESSFUL)
	{
		uint8_t status = aci_l2cap_cfc_connection_resp(Connection_Handle,
										Identifier,
										l2capConnectionServer.localCID,//local CID
										Peer_MTU,// same as the peer value. Later there should be the value from server
										Peer_MPS,//same as the peer value. Later therer should be the value from server
										L2CAP_CONN_SUCCESSFUL,
										0x01,//CFCpolicy
										32000+2,//Buffer size
										txBuffer);//buffer pointer
#ifdef PRINT_DEBUG_INFO
	printf("====================================\r\n");
	printf("Server: aci_l2cap_cfc_connection_resp()");
	if(status == BLE_STATUS_SUCCESS)
	{
		printf("OK\r\n");
		l2capConnectedFlag = 1;
	}
	else
	{
		printf("Error code: 0x%02x\r\n", status);
	}
#endif //PRINT_DEBUG_INFO		
}
}
//=============================================================================
void aci_l2cap_sdu_data_tx_event(uint16_t Connection_Handle,
                                 uint16_t CID,
                                 uint16_t SDU_Length,
                                 void * SDU_Data_Buffer,
                                 uint16_t TX_Credit_Balance)
{
	PA13_ON();
#ifdef PRINT_DEBUG_INFO	
	printf("====================================\r\n");
	printf("Server: aci_l2cap_sdu_data_tx_event()\r\n");
	printf("Data transmitted!\r\n");
	printf("CID: 0x%04x\r\n", CID);
	printf("SDU length: %06d\r\n", SDU_Length);
	printf("Tx credit balance: %06d\r\n", TX_Credit_Balance);
#endif//PRINT_DEBUG_INFO	
	l2capConnectionServer.sentDataFlag = 1;
	PA13_OFF();
}
//=============================================================================
void aci_l2cap_flow_control_credit_event(uint16_t Connection_Handle,
                                         uint16_t CID,
                                         uint16_t TX_Credits,
                                         uint16_t TX_Credit_Balance)
{
#ifdef PRINT_DEBUG_INFO	
	printf("====================================\r\n");
	printf("Server: aci_l2cap_flow_control_credit_event()\r\n");
	printf("CID: 0x%04x\r\n", CID);
	printf("Tx credits: 0x%04x\r\n", TX_Credits);	
	printf("Tx credit balance: %06d\r\n", TX_Credit_Balance);
#endif//PRINT_DEBUG_INFO		
}
//=============================================================================
void aci_l2cap_disconnection_complete_event(uint16_t Connection_Handle,
                                            uint16_t CID)
{
#ifdef PRINT_DEBUG_INFO	
	printf("====================================\r\n");
	printf("Server: aci_l2cap_disconnection_complete_event()\r\n");
	printf("Connection: 0x%04x , channel ID: 0x%04x is closed.",
				 Connection_Handle,
				 CID);
//	l2capConnection.disconnectedFlag = 1;
	l2capConnectedFlag = 0;
#endif//PRINT_DEBUG_INFO	
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
	printf("Server: hci_le_data_length_change_event()\r\n");
	printf("Connection Handle: 0x%04x\r\n", Connection_Handle);
	printf("Max Tx Octets: %04d\r\n", MaxTxOctets);
	printf("Max Tx Time: %04d\r\n", MaxTxTime	);
	printf("Max Rx Octets: %04d\r\n", MaxRxOctets);
	printf("Max Rx Time: %04d\r\n", MaxRxTime);	
#endif//PRINT_DEBUG_INFO		
}
//=============================================================================