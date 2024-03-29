
/**
  ******************************************************************************
  * @file    stack_user_cfg.h
  * @author  AMS - RF Application team
  * @version V1.3.0
  * @date    15 Nov 2023
  * @brief   BLE stack modular configuration options header file
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
  * <h2><center>&copy; COPYRIGHT STMicroelectronics</center></h2>
  ******************************************************************************
* \section BLE_Config BLE stack configuration options  

    - The BLE stack v3.0 or later supports the following outstanding features:
      -- Controller Privacy (v3.0)
      -- LE Secure Connections (v3.0)
      -- Controller Master (v3.0)
      -- Controller Data Length Extension (v3.0)
      -- LE 2M/Coded PHY (v3.0)
      -- Extended Advertising (v3.0)
      -- Periodic Advertising and Synchronizer (v3.1)
      -- L2CAP Connection Oriented Channels (v3.0)
      -- Constant Tone Extension (v3.1)
      -- Power Control & Path Loss Monitoring (v3.1)
      -- Connection Support (v3.1)
      -- LE Channel Classification (v3.2)
      -- Broadcast Isochronous Streams (v3.2)
      -- Connected Isochronous Streams (v3.2)
    
    - In order to configure the BLE stack v3.0 or later the following options are available:
        
        - BLE stack full configuration: all the outstanding features. 
          - To enable this configuration, the user is requested to add the following preprocessor option
            to project configuration: 
            
            BLE_STACK_FULL_CONF
        
        - BLE stack basic configuration: none of the outstanding features is included except for the Connection Support mode.
          - To enable this configuration, the user is requested to add the following preprocessor option
            to project configuration: 
            
            BLE_STACK_BASIC_CONF 
        
        - BLE stack configuration with Slave Only mode, Data Length Extension, and Connection Support:
          - To enable this configuration, the user is requested to add the following preprocessor option
            to project configuration:
            
            BLE_STACK_SLAVE_DLE_CONF 
        
        - BLE stack configuration with Slave Only mode, Data Length Extension, LE 2M PHY, LE Coded PHY, and Connection Support.
          - To enable this configuration, the user is requested to add the following preprocessor option
            to project configuration:
            
            BLE_STACK_SLAVE_DLE_LE_2M_CODED_CONF
        
        - BLE stack custom configuration: the user decides the outstanding features to include.
          - To enable this configuration, the user is requested to add the following preprocessor option
            to project configuration:
            
            BLE_STACK_CUSTOM_CONF
**/

#ifndef _STACK_USER_CFG_H_
#define _STACK_USER_CFG_H_

#include <bluenrg_lp_api.h>
#include <system_util.h>
#include <stdint.h>


/* --------------------- BLE stack configuration options --------------------------------------------------- */


/** @defgroup BLE_Stack_Config BLE Stack Modular Configuration
 *  @li BLE_STACK_BASIC_CONF: only basic features are enabled (e.g., Slave Only mode)
 *  @li BLE_STACK_FULL_CONF: all features are enabled
 *  @li BLE_STACK_SLAVE_DLE_CONF: Slave Only + Data Length Extension enabled
 *  @li BLE_STACK_SLAVE_DLE_LE_2M_CODED_CONF: Slave Only + Data Length Extension + PHY 2M/Coded PHY enabled
 *
 *  Uncomment one of the lines or define the configuration in your toolchain compiler preprocessor.
 *  @note It is mandatory to define one (and only one) of the options.
 *  @{
 */

/* #define BLE_STACK_BASIC_CONF */
/* #define BLE_STACK_FULL_CONF */
/* #define BLE_STACK_SLAVE_DLE_CONF */
/* #define BLE_STACK_SLAVE_DLE_LE_2M_CODED_CONF */
/* #define BLE_STACK_CUSTOM_CONF */

/**
 * @}
 */

/* --------------------- BLE stack configuration options and associated module inclusion -------------------- */

#if !(defined(BLE_STACK_FULL_CONF)                  ||\
      defined(BLE_STACK_BASIC_CONF)                 ||\
      defined(BLE_STACK_SLAVE_DLE_CONF)             ||\
      defined(BLE_STACK_SLAVE_DLE_LE_2M_CODED_CONF) ||\
      defined(BLE_STACK_CUSTOM_CONF))
#  error "No BLE_STACK_*_CONF has been defined."
#endif

#if !(defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS) )
#  error "No CONFIG_DEVICE_BLUENRG_* has been defined."
#endif


/**
 * @brief Full BLE stack configuration
 */
#if defined(BLE_STACK_FULL_CONF)
#  define CONTROLLER_MASTER_ENABLED                 (1U) /* Master role:                               ENABLED  */
#  define CONTROLLER_PRIVACY_ENABLED                (1U) /* Controller Privacy feature:                ENABLED  */
#  define SECURE_CONNECTIONS_ENABLED                (1U) /* LE Secure Connections feature:             ENABLED  */
#  define CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED  (1U) /* Data Length Extension feature:             ENABLED  */
#  define CONTROLLER_2M_CODED_PHY_ENABLED           (1U) /* LE 2M + Coded PHYs:                        ENABLED  */
#  define CONTROLLER_EXT_ADV_SCAN_ENABLED           (1U) /* Extended Advertising and Scanning feature: ENABLED  */
#  define L2CAP_COS_ENABLED                         (1U) /* L2CAP COS feature:                         ENABLED  */
#  define CONTROLLER_PERIODIC_ADV_ENABLED           (1U) /* Periodic Advertising and Synchronizer:     ENABLED  */
#if defined(CONFIG_DEVICE_BLUENRG_LP)
#    define CONTROLLER_CTE_ENABLED                  (0U) /* Constant Tone Extension:                   DISABLED */
# elif defined(CONFIG_DEVICE_BLUENRG_LPS)
#    define CONTROLLER_CTE_ENABLED                  (1U) /* Constant Tone Extension:                   ENABLED  */
#endif
#  define CONTROLLER_POWER_CONTROL_ENABLED          (1U) /* Power Control & Path Loss Monitoring:      ENABLED  */
#  define CONNECTION_ENABLED                        (1U) /* Connection Support:                        ENABLED  */
#  define CONTROLLER_CHAN_CLASS_ENABLED             (1U) /* LE Channel Classification:                 ENABLED  */
#  define CONTROLLER_BIS_ENABLED                    (1U) /* BIS Support:                               ENABLED */
#  define EATT_ENABLED                              (1U) /* EATT Support:                              ENABLED */
#  define CONTROLLER_CIS_ENABLED                    (1U) /* CIS Support:                               ENABLED */
#  define CONNECTION_SUBRATING_ENABLED              (1U) /* Connection Subrating feature:              ENABLED */

/**
 * @brief Basic BLE stack configuration
 */
#elif defined(BLE_STACK_BASIC_CONF)
#  define CONTROLLER_PRIVACY_ENABLED                  (0U) /* Controller Privacy feature:                DISABLED */
#  define SECURE_CONNECTIONS_ENABLED                  (0U) /* LE Secure Connections feature:             DISABLED */
#  define CONTROLLER_MASTER_ENABLED                   (0U) /* Master role:                               DISABLED */
#  define CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED    (0U) /* Data Length Extension feature:             DISABLED */
#  define CONTROLLER_2M_CODED_PHY_ENABLED             (0U) /* LE 2M + Coded PHYs:                        DISABLED */
#  define CONTROLLER_EXT_ADV_SCAN_ENABLED             (0U) /* Extended Advertising and Scanning feature: DISABLED */
#  define L2CAP_COS_ENABLED                           (0U) /* L2CAP COS feature:                         DISABLED */
#  define CONTROLLER_PERIODIC_ADV_ENABLED             (0U) /* Periodic Advertising and Synchronizer:     DISABLED */
#  define CONTROLLER_CTE_ENABLED                      (0U) /* Constant Tone Extension:                   DISABLED */
#  define CONTROLLER_POWER_CONTROL_ENABLED            (0U) /* Power Control & Path Loss Monitoring:      DISABLED */
#  define CONNECTION_ENABLED                          (1U) /* Connection Support:                        ENABLED  */
#  define CONTROLLER_CHAN_CLASS_ENABLED               (0U) /* LE Channel Classification:                 DISABLED */
#  define CONTROLLER_BIS_ENABLED                      (0U) /* BIS Support:                               DISABLED */
#  define EATT_ENABLED                                (0U) /* EATT Support:                              DISABLED */
#  define CONTROLLER_CIS_ENABLED                      (0U) /* CIS Support:                               DISABLED */
#  define CONNECTION_SUBRATING_ENABLED                (0U) /* Connection Subrating feature:              DISABLED */

/**
 * @brief BLE stack configuration: only Slave role with Data Length Extension feature enabled
 */
#elif defined(BLE_STACK_SLAVE_DLE_CONF)
#  define CONTROLLER_PRIVACY_ENABLED                  (0U) /* Controller Privacy feature:                DISABLED */
#  define SECURE_CONNECTIONS_ENABLED                  (0U) /* LE Secure Connections feature:             DISABLED */
#  define CONTROLLER_MASTER_ENABLED                   (0U) /* Master role:                               DISABLED */
#  define CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED    (1U) /* Data Length Extension feature:             ENABLED  */
#  define CONTROLLER_2M_CODED_PHY_ENABLED             (0U) /* LE 2M + Coded PHYs:                        DISABLED */
#  define CONTROLLER_EXT_ADV_SCAN_ENABLED             (0U) /* Extended Advertising and Scanning feature: DISABLED */
#  define L2CAP_COS_ENABLED                           (0U) /* L2CAP COS feature:                         DISABLED */
#  define CONTROLLER_PERIODIC_ADV_ENABLED             (0U) /* Periodic Advertising and Synchronizer:     DISABLED */
#  define CONTROLLER_CTE_ENABLED                      (0U) /* Constant Tone Extension:                   DISABLED */
#  define CONTROLLER_POWER_CONTROL_ENABLED            (0U) /* Power Control & Path Loss Monitoring:      DISABLED */
#  define CONNECTION_ENABLED                          (1U) /* Connection Support:                        ENABLED  */
#  define CONTROLLER_CHAN_CLASS_ENABLED               (0U) /* LE Channel Classification:                 DISABLED */
#  define CONTROLLER_BIS_ENABLED                      (0U) /* BIS Support:                               DISABLED */
#  define EATT_ENABLED                                (0U) /* EATT Support:                              DISABLED */
#  define CONTROLLER_CIS_ENABLED                      (0U) /* CIS Support:                               DISABLED */
#  define CONNECTION_SUBRATING_ENABLED                (0U) /* Connection Subrating feature:              DISABLED */

/**
 * @brief BLE stack configuration: only Slave role with Data Length Extension feature and LE 2M + Coded PHYs
 */
#elif defined(BLE_STACK_SLAVE_DLE_LE_2M_CODED_CONF)
#  define CONTROLLER_PRIVACY_ENABLED                  (0U) /* Controller Privacy feature:                DISABLED */
#  define SECURE_CONNECTIONS_ENABLED                  (0U) /* LE Secure Connections feature:             DISABLED */
#  define CONTROLLER_MASTER_ENABLED                   (0U) /* Master role:                               DISABLED */
#  define CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED    (1U) /* Data Length Extension feature:             ENABLED  */
# define CONTROLLER_2M_CODED_PHY_ENABLED              (1U) /* LE 2M + Coded PHYs:                        ENABLED  */
#  define CONTROLLER_EXT_ADV_SCAN_ENABLED             (0U) /* Extended Advertising and Scanning feature: DISABLED */
#  define L2CAP_COS_ENABLED                           (0U) /* L2CAP COS feature:                         DISABLED */
#  define CONTROLLER_PERIODIC_ADV_ENABLED             (0U) /* Periodic Advertising and Synchronizer:     DISABLED */
#  define CONTROLLER_CTE_ENABLED                      (0U) /* Constant Tone Extension:                   DISABLED */
#  define CONTROLLER_POWER_CONTROL_ENABLED            (0U) /* Power Control & Path Loss Monitoring:      DISABLED */
#  define CONNECTION_ENABLED                          (1U) /* Connection Support:                        ENABLED  */
#  define CONTROLLER_CHAN_CLASS_ENABLED               (0U) /* LE Channel Classification:                 DISABLED */
#  define CONTROLLER_BIS_ENABLED                      (0U) /* BIS Support:                               DISABLED */
#  define EATT_ENABLED                                (0U) /* EATT Support:                              DISABLED */
#  define CONTROLLER_CIS_ENABLED                      (0U) /* CIS Support:                               DISABLED */
#  define CONNECTION_SUBRATING_ENABLED                (0U) /* Connection Subrating feature:              DISABLED */

#elif defined(BLE_STACK_CUSTOM_CONF)
#  include "custom_ble_stack_conf.h"
#endif

/* --------------------- Derived defines --------------------- */
#if (CONTROLLER_BIS_ENABLED == 1U) || (CONTROLLER_CIS_ENABLED == 1U)
#  define CONTROLLER_ISO_ENABLED (1U) /* ISO Support: ENABLED */
#else
#  define CONTROLLER_ISO_ENABLED (0U) /* ISO Support: DISABLED */
#endif

/* --------------------- Defines used by function prototypes -------------------- */

#if !defined(LLC_MAX_NUM_DATA_CHAN)
#   define LLC_MAX_NUM_DATA_CHAN (37U)
#endif
#define LLC_MIN_NUM_DATA_CHAN_MAP_BYTES   (((LLC_MAX_NUM_DATA_CHAN - 1U) >> 3U) + 1U)
#define LLC_MIN_NUM_DATA_CHAN_CLASS_BYTES (((LLC_MAX_NUM_DATA_CHAN - 1U) >> 2U) + 1U)

tBleStatus GAP_set_advertising_configuration_ucfg(uint8_t Advertising_Handle,
                                                  uint8_t Discoverability_Mode,
                                                  uint16_t Advertising_Event_Properties,
                                                  uint32_t Primary_Advertising_Interval_Min,
                                                  uint32_t Primary_Advertising_Interval_Max,
                                                  uint8_t Primary_Advertising_Channel_Map,
                                                  uint8_t Peer_Address_Type,
                                                  uint8_t Peer_Address[6],
                                                  uint8_t Advertising_Filter_Policy,
                                                  int8_t Advertising_Tx_Power,
                                                  uint8_t Primary_Advertising_PHY,
                                                  uint8_t Secondary_Advertising_Max_Skip,
                                                  uint8_t Secondary_Advertising_PHY,
                                                  uint8_t Advertising_SID,
                                                  uint8_t Scan_Request_Notification_Enable);
tBleStatus GAP_set_advertising_configuration_ucfg_weak(uint8_t Advertising_Handle,
                                                       uint8_t Discoverability_Mode,
                                                       uint16_t Advertising_Event_Properties,
                                                       uint32_t Primary_Advertising_Interval_Min,
                                                       uint32_t Primary_Advertising_Interval_Max,
                                                       uint8_t Primary_Advertising_Channel_Map,
                                                       uint8_t Peer_Address_Type,
                                                       uint8_t Peer_Address[6],
                                                       uint8_t Advertising_Filter_Policy,
                                                       int8_t Advertising_Tx_Power,
                                                       uint8_t Primary_Advertising_PHY,
                                                       uint8_t Secondary_Advertising_Max_Skip,
                                                       uint8_t Secondary_Advertising_PHY,
                                                       uint8_t Advertising_SID,
                                                       uint8_t Scan_Request_Notification_Enable);
tBleStatus GAP_set_extended_advertising_configuration(uint8_t Advertising_Handle,
                                                      uint8_t Discoverability_Mode,
                                                      uint16_t Advertising_Event_Properties,
                                                      uint32_t Primary_Advertising_Interval_Min,
                                                      uint32_t Primary_Advertising_Interval_Max,
                                                      uint8_t Primary_Advertising_Channel_Map,
                                                      uint8_t Peer_Address_Type,
                                                      uint8_t Peer_Address[6],
                                                      uint8_t Advertising_Filter_Policy,
                                                      int8_t Advertising_Tx_Power,
                                                      uint8_t Primary_Advertising_PHY,
                                                      uint8_t Secondary_Advertising_Max_Skip,
                                                      uint8_t Secondary_Advertising_PHY,
                                                      uint8_t Advertising_SID,
                                                      uint8_t Scan_Request_Notification_Enable);

tBleStatus GAP_set_scan_response_data_ucfg(uint8_t Advertising_Handle,
                                           uint16_t Scan_Response_Data_Length,
                                           uint8_t* Scan_Response_Data);
tBleStatus GAP_set_scan_response_data_ucfg_weak(uint8_t Advertising_Handle,
                                                uint16_t Scan_Response_Data_Length,
                                                uint8_t* Scan_Response_Data);
tBleStatus GAP_set_extended_scan_response_data(uint8_t Advertising_Handle,
                                               uint16_t Scan_Response_Data_Length,
                                               uint8_t* Scan_Response_Data);

tBleStatus GAP_set_advertising_data_ucfg(uint8_t Advertising_Handle,
                                         uint8_t Operation,
                                         uint16_t Advertising_Data_Length,
                                         uint8_t* Advertising_Data);
tBleStatus GAP_set_advertising_data_ucfg_weak(uint8_t Advertising_Handle,
                                              uint8_t Operation,
                                              uint16_t Advertising_Data_Length,
                                              uint8_t* Advertising_Data);
tBleStatus GAP_set_extended_advertising_data(uint8_t Advertising_Handle,
                                             uint8_t Operation,
                                             uint16_t Advertising_Data_Length,
                                             uint8_t* Advertising_Data);

tBleStatus GAP_set_advertising_enable_ucfg(uint8_t Enable,
                                           uint8_t Num_Of_Sets,
                                           Advertising_Set_Parameters_t* Advertising_Set_Parameters);
tBleStatus GAP_set_advertising_enable_ucfg_weak(uint8_t Enable,
                                                uint8_t Num_Of_Sets,
                                                Advertising_Set_Parameters_t* Advertising_Set_Parameters);
tBleStatus GAP_set_extended_advertising_enable(uint8_t Enable,
                                               uint8_t Num_Of_Sets,
                                               Advertising_Set_Parameters_t* Advertising_Set_Parameters);

void aci_gap_limited_discoverable_event_cb_ucfg(uint8_t header_type,
                                                uint8_t* buff_p);
void aci_gap_limited_discoverable_event_cb_ucfg_weak(uint8_t header_type,
                                                     uint8_t* buff_p);
void aci_gap_limited_discoverable_event_cb(uint8_t header_type,
                                           uint8_t* buff_p);

void GAP_LimDiscTimeoutcb_ucfg(uint8_t timer_id);
void GAP_LimDiscTimeoutcb_ucfg_weak(uint8_t timer_id);
void GAP_LimDiscTimeoutcb(uint8_t timer_id);

void GAP_hci_le_advertising_set_terminated_evt_hndl_ucfg(uint8_t status,
                                                         uint8_t Advertising_Handle);
void GAP_hci_le_advertising_set_terminated_evt_hndl_ucfg_weak(uint8_t status,
                                                              uint8_t Advertising_Handle);
void GAP_hci_le_advertising_set_terminated_evt_hndl(uint8_t status,
                                                    uint8_t Advertising_Handle);

void GAP_slave_connection_complete_handler_ucfg(uint16_t connectionHandle);
void GAP_slave_connection_complete_handler_ucfg_weak(uint16_t connectionHandle);
void GAP_slave_connection_complete_handler(uint16_t connectionHandle);

tBleStatus GAP_enable_controller_privacy_ucfg(uint8_t* gapRole,
                                              uint8_t* numServiceRec);
tBleStatus GAP_enable_controller_privacy_ucfg_weak(uint8_t* gapRole,
                                                   uint8_t* numServiceRec);
tBleStatus GAP_enable_controller_privacy(uint8_t* gapRole,
                                         uint8_t* numServiceRec);

tBleStatus GAP_add_device_to_white_and_resolving_list_ucfg(uint8_t lists,
                                                           uint8_t addr_type,
                                                           uint8_t addr[6]);
tBleStatus GAP_add_device_to_white_and_resolving_list_ucfg_weak(uint8_t lists,
                                                                uint8_t addr_type,
                                                                uint8_t addr[6]);
tBleStatus GAP_add_device_to_white_and_resolving_list_full(uint8_t lists,
                                                           uint8_t addr_type,
                                                           uint8_t addr[6]);

tBleStatus GAP_clear_white_and_resolving_list_ucfg(uint8_t lists);
tBleStatus GAP_clear_white_and_resolving_list_ucfg_weak(uint8_t lists);
tBleStatus GAP_clear_white_and_resolving_list_full(uint8_t lists);

tBleStatus GAP_set_controller_random_address_ucfg(uint8_t random_address[6]);
tBleStatus GAP_set_controller_random_address_ucfg_weak(uint8_t random_address[6]);
tBleStatus GAP_set_controller_random_address_extended(uint8_t random_address[6]);

tBleStatus GAP_init_advertising_sets_ucfg(uint8_t own_address_type);
tBleStatus GAP_init_advertising_sets_ucfg_weak(uint8_t own_address_type);
tBleStatus GAP_init_advertising_sets(uint8_t own_address_type);

tBleStatus GAP_create_connection_ucfg(uint8_t* peer_address,
                                      uint8_t peer_address_type,
                                      uint8_t own_address_type,
                                      uint8_t initiator_filter_policy,
                                      uint8_t phys);
tBleStatus GAP_create_connection_ucfg_weak(uint8_t* peer_address,
                                           uint8_t peer_address_type,
                                           uint8_t own_address_type,
                                           uint8_t initiator_filter_policy,
                                           uint8_t phys);
tBleStatus GAP_create_connection_ext(uint8_t* peer_address,
                                     uint8_t peer_address_type,
                                     uint8_t own_address_type,
                                     uint8_t initiator_filter_policy,
                                     uint8_t phys);

tBleStatus GAP_set_scan_parameters_ucfg(uint8_t own_address_type,
                                        uint8_t phys);
tBleStatus GAP_set_scan_parameters_ucfg_weak(uint8_t own_address_type,
                                             uint8_t phys);
tBleStatus GAP_set_scan_parameters_ext(uint8_t own_address_type,
                                       uint8_t phys);

tBleStatus GAP_enable_disable_scan_ucfg(BOOL enable,
                                        uint8_t duplicate_filtering);
tBleStatus GAP_enable_disable_scan_ucfg_weak(BOOL enable,
                                             uint8_t duplicate_filtering);
tBleStatus GAP_enable_disable_scan_legacy(BOOL enable,
                                          uint8_t duplicate_filtering);
tBleStatus GAP_enable_disable_scan_ext(BOOL enable,
                                       uint8_t duplicate_filtering);

tBleStatus GAP_connection_procedure_ucfg(uint8_t procedure_code,
                                         uint8_t phys,
                                         uint8_t peer_address_type,
                                         uint8_t peer_address[6]);
tBleStatus GAP_connection_procedure_ucfg_weak(uint8_t procedure_code,
                                              uint8_t phys,
                                              uint8_t peer_address_type,
                                              uint8_t peer_address[6]);
tBleStatus GAP_connection_procedure(uint8_t procedure_code,
                                    uint8_t phys,
                                    uint8_t peer_address_type,
                                    uint8_t peer_address[6]);

tBleStatus GAP_terminate_gap_procedure_ucfg(uint8_t procedure_code);
tBleStatus GAP_terminate_gap_procedure_ucfg_weak(uint8_t procedure_code);
tBleStatus GAP_terminate_gap_procedure(uint8_t procedure_code);

tBleStatus GAP_discover_peer_name_ucfg(void);
tBleStatus GAP_discover_peer_name_ucfg_weak(void);
tBleStatus GAP_discover_peer_name(void);

void GAP_name_disc_proc_connected_check_ucfg(uint16_t task_idx);
void GAP_name_disc_proc_connected_check_ucfg_weak(uint16_t task_idx);
void GAP_name_disc_proc_connected_check(uint16_t task_idx);

void GAP_master_connection_complete_handler_ucfg(uint8_t status,
                                                 uint16_t connectionHandle);
void GAP_master_connection_complete_handler_ucfg_weak(uint8_t status,
                                                      uint16_t connectionHandle);
void GAP_master_connection_complete_handler(uint8_t status,
                                            uint16_t connectionHandle);

BOOL GAP_parse_connectable_advertising_report_ucfg(uint8_t* adv_buf,
                                                   BOOL extended);
BOOL GAP_parse_connectable_advertising_report_ucfg_weak(uint8_t* adv_buf,
                                                        BOOL extended);
BOOL GAP_parse_connectable_advertising_report(uint8_t* adv_buf,
                                              BOOL extended);

BOOL GAP_parse_advertising_report_ucfg(uint8_t* adv_buf,
                                       BOOL extended);
BOOL GAP_parse_advertising_report_ucfg_weak(uint8_t* adv_buf,
                                            BOOL extended);
BOOL GAP_parse_advertising_report(uint8_t* adv_buf,
                                  BOOL extended);

void GAP_DiscProcTimeoutcb_ucfg(uint8_t timer_id);
void GAP_DiscProcTimeoutcb_ucfg_weak(uint8_t timer_id);
void GAP_DiscProcTimeoutcb(uint8_t timer_id);

tBleStatus GAP_suspend_resume_active_advertising_sets_ucfg(BOOL resume);
tBleStatus GAP_suspend_resume_active_advertising_sets_ucfg_weak(BOOL resume);
tBleStatus GAP_suspend_resume_active_advertising_sets_extended(BOOL resume);

void GAP_ResumeAdvertising_ucfg(uint16_t task_idx);
void GAP_ResumeAdvertising_ucfg_weak(uint16_t task_idx);
void GAP_ResumeAdvertising(uint16_t task_idx);

tBleStatus hci_acl_data_tx_cmpl_event_int_cb_ucfg(void* header_p,
                                                  uint8_t* buff_p);
tBleStatus hci_acl_data_tx_cmpl_event_int_cb_ucfg_weak(void* header_p,
                                                       uint8_t* buff_p);
tBleStatus hci_acl_data_tx_cmpl_event_int_cb(void* header_p,
                                             uint8_t* buff_p);

tBleStatus hci_acl_data_ind_event_int_cb_ucfg(void* header_p,
                                              uint8_t* buff_p);
tBleStatus hci_acl_data_ind_event_int_cb_ucfg_weak(void* header_p,
                                                   uint8_t* buff_p);
tBleStatus hci_acl_data_ind_event_int_cb(void* header_p,
                                         uint8_t* buff_p);

uint32_t chc_csr_ucfg(void);
uint32_t chc_csr_ucfg_weak(void);
uint32_t chc_csr(void);

void Controller_Process_Q_ucfg(uint16_t task_idx);
void Controller_Process_Q_ucfg_weak(uint16_t task_idx);
void Controller_Process_Q(uint16_t task_idx);

void LLC_offline_control_procedures_processing_ucfg(uint16_t task_idx);
void LLC_offline_control_procedures_processing_ucfg_weak(uint16_t task_idx);
void LLC_offline_control_procedures_processing(uint16_t task_idx);

uint32_t cte_csr_ucfg(void);
uint32_t cte_csr_ucfg_weak(void);
uint32_t cte_csr(void);

uint8_t hci_le_check_own_address_type_max_value_ucfg(void);
uint8_t hci_le_check_own_address_type_max_value_ucfg_weak(void);
uint8_t hci_le_check_own_address_type_max_value(void);

uint32_t data_length_extension_csr_ucfg(void);
uint32_t data_length_extension_csr_ucfg_weak(void);
uint32_t data_length_extension_csr(void);

uint32_t pcl_csr_ucfg(void);
uint32_t pcl_csr_ucfg_weak(void);
uint32_t pcl_csr(void);

uint32_t phy_upd_csr_ucfg(void);
uint32_t phy_upd_csr_ucfg_weak(void);
uint32_t phy_upd_csr(void);

uint32_t master_csr_ucfg(void);
uint32_t master_csr_ucfg_weak(void);
uint32_t master_csr(void);

uint32_t conn_supp_csr_ucfg(void);
uint32_t conn_supp_csr_ucfg_weak(void);
uint32_t conn_supp_csr(void);

uint32_t subrate_csr_ucfg(void);
uint32_t subrate_csr_ucfg_weak(void);
uint32_t subrate_csr(void);

tBleStatus LLC_test_check_cte_params_ucfg(void* params);
tBleStatus LLC_test_check_cte_params_ucfg_weak(void* params);
tBleStatus LLC_test_check_cte_params(void* params);

tBleStatus hci_disconnection_complete_event_int_cb_ucfg(void* header_p,
                                                        uint8_t* buff_p);
tBleStatus hci_disconnection_complete_event_int_cb_ucfg_weak(void* header_p,
                                                             uint8_t* buff_p);
tBleStatus hci_disconnection_complete_event_int_cb(void* header_p,
                                                   uint8_t* buff_p);

tBleStatus hci_encryption_change_event_int_cb_ucfg(void* header_p,
                                                   uint8_t* buff_p);
tBleStatus hci_encryption_change_event_int_cb_ucfg_weak(void* header_p,
                                                        uint8_t* buff_p);
tBleStatus hci_encryption_change_event_int_cb(void* header_p,
                                              uint8_t* buff_p);

tBleStatus hci_encryption_key_refresh_complete_event_int_cb_ucfg(void* header_p,
                                                                 uint8_t* buff_p);
tBleStatus hci_encryption_key_refresh_complete_event_int_cb_ucfg_weak(void* header_p,
                                                                      uint8_t* buff_p);
tBleStatus hci_encryption_key_refresh_complete_event_int_cb(void* header_p,
                                                            uint8_t* buff_p);

tBleStatus hci_le_connection_complete_event_int_cb_ucfg(void* header_p,
                                                        uint8_t* buff_p);
tBleStatus hci_le_connection_complete_event_int_cb_ucfg_weak(void* header_p,
                                                             uint8_t* buff_p);
tBleStatus hci_le_connection_complete_event_int_cb(void* header_p,
                                                   uint8_t* buff_p);

tBleStatus hci_le_enhanced_connection_complete_event_int_cb_ucfg(void* header_p,
                                                                 uint8_t* buff_p);
tBleStatus hci_le_enhanced_connection_complete_event_int_cb_ucfg_weak(void* header_p,
                                                                      uint8_t* buff_p);
tBleStatus hci_le_enhanced_connection_complete_event_int_cb(void* header_p,
                                                            uint8_t* buff_p);

tBleStatus hci_le_long_term_key_request_event_int_cb_ucfg(void* header_p,
                                                          uint8_t* buff_p);
tBleStatus hci_le_long_term_key_request_event_int_cb_ucfg_weak(void* header_p,
                                                               uint8_t* buff_p);
tBleStatus hci_le_long_term_key_request_event_int_cb(void* header_p,
                                                     uint8_t* buff_p);

tBleStatus hci_le_read_local_p256_public_key_complete_event_int_cb_ucfg(void* header_p,
                                                                        uint8_t* buff_p);
tBleStatus hci_le_read_local_p256_public_key_complete_event_int_cb_ucfg_weak(void* header_p,
                                                                             uint8_t* buff_p);
tBleStatus hci_le_read_local_p256_public_key_complete_event_int_cb(void* header_p,
                                                                   uint8_t* buff_p);

tBleStatus hci_le_generate_dhkey_complete_event_int_cb_ucfg(void* header_p,
                                                            uint8_t* buff_p);
tBleStatus hci_le_generate_dhkey_complete_event_int_cb_ucfg_weak(void* header_p,
                                                                 uint8_t* buff_p);
tBleStatus hci_le_generate_dhkey_complete_event_int_cb(void* header_p,
                                                       uint8_t* buff_p);

tBleStatus hci_le_advertising_report_event_int_cb_ucfg(void* header_p,
                                                       uint8_t* buff_p);
tBleStatus hci_le_advertising_report_event_int_cb_ucfg_weak(void* header_p,
                                                            uint8_t* buff_p);
tBleStatus hci_le_advertising_report_event_int_cb(void* header_p,
                                                  uint8_t* buff_p);

tBleStatus hci_le_direct_advertising_report_event_int_cb_ucfg(void* header_p,
                                                              uint8_t* buff_p);
tBleStatus hci_le_direct_advertising_report_event_int_cb_ucfg_weak(void* header_p,
                                                                   uint8_t* buff_p);
tBleStatus hci_le_direct_advertising_report_event_int_cb(void* header_p,
                                                         uint8_t* buff_p);

tBleStatus hci_le_extended_advertising_report_event_int_cb_ucfg(void* header_p,
                                                                uint8_t* buff_p);
tBleStatus hci_le_extended_advertising_report_event_int_cb_ucfg_weak(void* header_p,
                                                                     uint8_t* buff_p);
tBleStatus hci_le_extended_advertising_report_event_int_cb(void* header_p,
                                                           uint8_t* buff_p);

tBleStatus hci_le_scan_timeout_event_int_cb_ucfg(void* header_p,
                                                 uint8_t* buff_p);
tBleStatus hci_le_scan_timeout_event_int_cb_ucfg_weak(void* header_p,
                                                      uint8_t* buff_p);
tBleStatus hci_le_scan_timeout_event_int_cb(void* header_p,
                                            uint8_t* buff_p);

tBleStatus hci_le_advertising_set_terminated_event_int_cb_ucfg(void* header_p,
                                                               uint8_t* buff_p);
tBleStatus hci_le_advertising_set_terminated_event_int_cb_ucfg_weak(void* header_p,
                                                                    uint8_t* buff_p);
tBleStatus hci_le_advertising_set_terminated_event_int_cb(void* header_p,
                                                          uint8_t* buff_p);

void aci_gatt_tx_pool_available_event_cb_ucfg(uint8_t header_type,
                                              uint8_t* buff_p);
void aci_gatt_tx_pool_available_event_cb_ucfg_weak(uint8_t header_type,
                                                   uint8_t* buff_p);
void aci_gatt_tx_pool_available_event_cb(uint8_t header_type,
                                         uint8_t* buff_p);

void aci_gatt_eatt_srv_attribute_modified_event_cb_ucfg(uint8_t header_type,
                                                        uint8_t* buff_p);
void aci_gatt_eatt_srv_attribute_modified_event_cb_ucfg_weak(uint8_t header_type,
                                                             uint8_t* buff_p);
void aci_gatt_eatt_srv_attribute_modified_event_cb(uint8_t header_type,
                                                   uint8_t* buff_p);
void aci_gatt_eatt_srv_attribute_modified_event_eatt_cb(uint8_t header_type,
                                                        uint8_t* buff_p);

void aci_gatt_eatt_proc_timeout_event_cb_ucfg(uint8_t header_type,
                                              uint8_t* buff_p);
void aci_gatt_eatt_proc_timeout_event_cb_ucfg_weak(uint8_t header_type,
                                                   uint8_t* buff_p);
void aci_gatt_eatt_proc_timeout_event_cb(uint8_t header_type,
                                         uint8_t* buff_p);
void aci_gatt_eatt_proc_timeout_event_eatt_cb(uint8_t header_type,
                                              uint8_t* buff_p);

void aci_gatt_eatt_clt_indication_event_cb_ucfg(uint8_t header_type,
                                                uint8_t* buff_p);
void aci_gatt_eatt_clt_indication_event_cb_ucfg_weak(uint8_t header_type,
                                                     uint8_t* buff_p);
void aci_gatt_eatt_clt_indication_event_cb(uint8_t header_type,
                                           uint8_t* buff_p);
void aci_gatt_eatt_clt_indication_event_eatt_cb(uint8_t header_type,
                                                uint8_t* buff_p);

void aci_gatt_eatt_clt_notification_event_cb_ucfg(uint8_t header_type,
                                                  uint8_t* buff_p);
void aci_gatt_eatt_clt_notification_event_cb_ucfg_weak(uint8_t header_type,
                                                       uint8_t* buff_p);
void aci_gatt_eatt_clt_notification_event_cb(uint8_t header_type,
                                             uint8_t* buff_p);
void aci_gatt_eatt_clt_notification_event_eatt_cb(uint8_t header_type,
                                                  uint8_t* buff_p);

void aci_gatt_eatt_clt_proc_complete_event_cb_ucfg(uint8_t header_type,
                                                   uint8_t* buff_p);
void aci_gatt_eatt_clt_proc_complete_event_cb_ucfg_weak(uint8_t header_type,
                                                        uint8_t* buff_p);
void aci_gatt_eatt_clt_proc_complete_event_cb(uint8_t header_type,
                                              uint8_t* buff_p);
void aci_gatt_eatt_clt_proc_complete_event_eatt_cb(uint8_t header_type,
                                                   uint8_t* buff_p);

void aci_gatt_eatt_clt_error_resp_event_cb_ucfg(uint8_t header_type,
                                                uint8_t* buff_p);
void aci_gatt_eatt_clt_error_resp_event_cb_ucfg_weak(uint8_t header_type,
                                                     uint8_t* buff_p);
void aci_gatt_eatt_clt_error_resp_event_cb(uint8_t header_type,
                                           uint8_t* buff_p);
void aci_gatt_eatt_clt_error_resp_event_eatt_cb(uint8_t header_type,
                                                uint8_t* buff_p);

void aci_gatt_eatt_clt_disc_read_char_by_uuid_resp_event_cb_ucfg(uint8_t header_type,
                                                                 uint8_t* buff_p);
void aci_gatt_eatt_clt_disc_read_char_by_uuid_resp_event_cb_ucfg_weak(uint8_t header_type,
                                                                      uint8_t* buff_p);
void aci_gatt_eatt_clt_disc_read_char_by_uuid_resp_event_cb(uint8_t header_type,
                                                            uint8_t* buff_p);
void aci_gatt_eatt_clt_disc_read_char_by_uuid_resp_event_eatt_cb(uint8_t header_type,
                                                                 uint8_t* buff_p);

void aci_gatt_eatt_srv_confirmation_event_cb_ucfg(uint8_t header_type,
                                                  uint8_t* buff_p);
void aci_gatt_eatt_srv_confirmation_event_cb_ucfg_weak(uint8_t header_type,
                                                       uint8_t* buff_p);
void aci_gatt_eatt_srv_confirmation_event_cb(uint8_t header_type,
                                             uint8_t* buff_p);
void aci_gatt_eatt_srv_confirmation_event_eatt_cb(uint8_t header_type,
                                                  uint8_t* buff_p);

void aci_gatt_eatt_srv_read_event_cb_ucfg(uint8_t header_type,
                                          uint8_t* buff_p);
void aci_gatt_eatt_srv_read_event_cb_ucfg_weak(uint8_t header_type,
                                               uint8_t* buff_p);
void aci_gatt_eatt_srv_read_event_cb(uint8_t header_type,
                                     uint8_t* buff_p);
void aci_gatt_eatt_srv_read_event_eatt_cb(uint8_t header_type,
                                          uint8_t* buff_p);

void aci_gatt_eatt_srv_write_event_cb_ucfg(uint8_t header_type,
                                           uint8_t* buff_p);
void aci_gatt_eatt_srv_write_event_cb_ucfg_weak(uint8_t header_type,
                                                uint8_t* buff_p);
void aci_gatt_eatt_srv_write_event_cb(uint8_t header_type,
                                      uint8_t* buff_p);
void aci_gatt_eatt_srv_write_event_eatt_cb(uint8_t header_type,
                                           uint8_t* buff_p);

void aci_gatt_clt_multi_notification_int_event_cb_ucfg(uint8_t header_type,
                                                       uint8_t* buff_p);
void aci_gatt_clt_multi_notification_int_event_cb_ucfg_weak(uint8_t header_type,
                                                            uint8_t* buff_p);
void aci_gatt_clt_multi_notification_int_event_eatt_cb(uint8_t header_type,
                                                       uint8_t* buff_p);

void aci_att_exchange_mtu_resp_event_cb_ucfg(uint8_t header_type,
                                             uint8_t* buff_p);
void aci_att_exchange_mtu_resp_event_cb_ucfg_weak(uint8_t header_type,
                                                  uint8_t* buff_p);
void aci_att_exchange_mtu_resp_event_cb(uint8_t header_type,
                                        uint8_t* buff_p);

void aci_eatt_clt_find_info_resp_event_cb_ucfg(uint8_t header_type,
                                               uint8_t* buff_p);
void aci_eatt_clt_find_info_resp_event_cb_ucfg_weak(uint8_t header_type,
                                                    uint8_t* buff_p);
void aci_eatt_clt_find_info_resp_event_cb(uint8_t header_type,
                                          uint8_t* buff_p);
void aci_eatt_clt_find_info_resp_event_eatt_cb(uint8_t header_type,
                                               uint8_t* buff_p);

void aci_eatt_clt_find_by_type_value_resp_event_cb_ucfg(uint8_t header_type,
                                                        uint8_t* buff_p);
void aci_eatt_clt_find_by_type_value_resp_event_cb_ucfg_weak(uint8_t header_type,
                                                             uint8_t* buff_p);
void aci_eatt_clt_find_by_type_value_resp_event_cb(uint8_t header_type,
                                                   uint8_t* buff_p);
void aci_eatt_clt_find_by_type_value_resp_event_eatt_cb(uint8_t header_type,
                                                        uint8_t* buff_p);

void aci_eatt_clt_read_by_type_resp_event_cb_ucfg(uint8_t header_type,
                                                  uint8_t* buff_p);
void aci_eatt_clt_read_by_type_resp_event_cb_ucfg_weak(uint8_t header_type,
                                                       uint8_t* buff_p);
void aci_eatt_clt_read_by_type_resp_event_cb(uint8_t header_type,
                                             uint8_t* buff_p);
void aci_eatt_clt_read_by_type_resp_event_eatt_cb(uint8_t header_type,
                                                  uint8_t* buff_p);

void aci_eatt_clt_read_resp_event_cb_ucfg(uint8_t header_type,
                                          uint8_t* buff_p);
void aci_eatt_clt_read_resp_event_cb_ucfg_weak(uint8_t header_type,
                                               uint8_t* buff_p);
void aci_eatt_clt_read_resp_event_cb(uint8_t header_type,
                                     uint8_t* buff_p);
void aci_eatt_clt_read_resp_event_eatt_cb(uint8_t header_type,
                                          uint8_t* buff_p);

void aci_eatt_clt_read_blob_resp_event_cb_ucfg(uint8_t header_type,
                                               uint8_t* buff_p);
void aci_eatt_clt_read_blob_resp_event_cb_ucfg_weak(uint8_t header_type,
                                                    uint8_t* buff_p);
void aci_eatt_clt_read_blob_resp_event_cb(uint8_t header_type,
                                          uint8_t* buff_p);
void aci_eatt_clt_read_blob_resp_event_eatt_cb(uint8_t header_type,
                                               uint8_t* buff_p);

void aci_eatt_clt_read_multiple_resp_event_cb_ucfg(uint8_t header_type,
                                                   uint8_t* buff_p);
void aci_eatt_clt_read_multiple_resp_event_cb_ucfg_weak(uint8_t header_type,
                                                        uint8_t* buff_p);
void aci_eatt_clt_read_multiple_resp_event_cb(uint8_t header_type,
                                              uint8_t* buff_p);
void aci_eatt_clt_read_multiple_resp_event_eatt_cb(uint8_t header_type,
                                                   uint8_t* buff_p);

void aci_eatt_clt_read_by_group_type_resp_event_cb_ucfg(uint8_t header_type,
                                                        uint8_t* buff_p);
void aci_eatt_clt_read_by_group_type_resp_event_cb_ucfg_weak(uint8_t header_type,
                                                             uint8_t* buff_p);
void aci_eatt_clt_read_by_group_type_resp_event_cb(uint8_t header_type,
                                                   uint8_t* buff_p);
void aci_eatt_clt_read_by_group_type_resp_event_eatt_cb(uint8_t header_type,
                                                        uint8_t* buff_p);

void aci_eatt_clt_prepare_write_resp_event_cb_ucfg(uint8_t header_type,
                                                   uint8_t* buff_p);
void aci_eatt_clt_prepare_write_resp_event_cb_ucfg_weak(uint8_t header_type,
                                                        uint8_t* buff_p);
void aci_eatt_clt_prepare_write_resp_event_cb(uint8_t header_type,
                                              uint8_t* buff_p);
void aci_eatt_clt_prepare_write_resp_event_eatt_cb(uint8_t header_type,
                                                   uint8_t* buff_p);

void aci_eatt_clt_exec_write_resp_event_cb_ucfg(uint8_t header_type,
                                                uint8_t* buff_p);
void aci_eatt_clt_exec_write_resp_event_cb_ucfg_weak(uint8_t header_type,
                                                     uint8_t* buff_p);
void aci_eatt_clt_exec_write_resp_event_cb(uint8_t header_type,
                                           uint8_t* buff_p);
void aci_eatt_clt_exec_write_resp_event_eatt_cb(uint8_t header_type,
                                                uint8_t* buff_p);

void aci_eatt_srv_exec_write_req_event_cb_ucfg(uint8_t header_type,
                                               uint8_t* buff_p);
void aci_eatt_srv_exec_write_req_event_cb_ucfg_weak(uint8_t header_type,
                                                    uint8_t* buff_p);
void aci_eatt_srv_exec_write_req_event_cb(uint8_t header_type,
                                          uint8_t* buff_p);
void aci_eatt_srv_exec_write_req_event_eatt_cb(uint8_t header_type,
                                               uint8_t* buff_p);

void aci_eatt_clt_read_multiple_var_len_resp_event_cb_ucfg(uint8_t header_type,
                                                           uint8_t* buff_p);
void aci_eatt_clt_read_multiple_var_len_resp_event_cb_ucfg_weak(uint8_t header_type,
                                                                uint8_t* buff_p);
void aci_eatt_clt_read_multiple_var_len_resp_event_cb(uint8_t header_type,
                                                      uint8_t* buff_p);
void aci_eatt_clt_read_multiple_var_len_resp_event_eatt_cb(uint8_t header_type,
                                                           uint8_t* buff_p);

void aci_eatt_srv_prepare_write_req_event_cb_ucfg(uint8_t header_type,
                                                  uint8_t* buff_p);
void aci_eatt_srv_prepare_write_req_event_cb_ucfg_weak(uint8_t header_type,
                                                       uint8_t* buff_p);
void aci_eatt_srv_prepare_write_req_event_cb(uint8_t header_type,
                                             uint8_t* buff_p);
void aci_eatt_srv_prepare_write_req_event_eatt_cb(uint8_t header_type,
                                                  uint8_t* buff_p);

void hci_le_cis_established_event_cb_ucfg(uint8_t header_type,
                                          uint8_t* buff_p);
void hci_le_cis_established_event_cb_ucfg_weak(uint8_t header_type,
                                               uint8_t* buff_p);
void hci_le_cis_established_event_cb(uint8_t header_type,
                                     uint8_t* buff_p);

void hci_le_cis_request_event_cb_ucfg(uint8_t header_type,
                                      uint8_t* buff_p);
void hci_le_cis_request_event_cb_ucfg_weak(uint8_t header_type,
                                           uint8_t* buff_p);
void hci_le_cis_request_event_cb(uint8_t header_type,
                                 uint8_t* buff_p);

void hci_disconnection_complete_event_cb_ucfg(uint8_t header_type,
                                              uint8_t* buff_p);
void hci_disconnection_complete_event_cb_ucfg_weak(uint8_t header_type,
                                                   uint8_t* buff_p);
void hci_disconnection_complete_event_cb(uint8_t header_type,
                                         uint8_t* buff_p);

void hci_encryption_change_event_cb_ucfg(uint8_t header_type,
                                         uint8_t* buff_p);
void hci_encryption_change_event_cb_ucfg_weak(uint8_t header_type,
                                              uint8_t* buff_p);
void hci_encryption_change_event_cb(uint8_t header_type,
                                    uint8_t* buff_p);

void hci_encryption_key_refresh_complete_event_cb_ucfg(uint8_t header_type,
                                                       uint8_t* buff_p);
void hci_encryption_key_refresh_complete_event_cb_ucfg_weak(uint8_t header_type,
                                                            uint8_t* buff_p);
void hci_encryption_key_refresh_complete_event_cb(uint8_t header_type,
                                                  uint8_t* buff_p);

void hci_le_connection_complete_event_cb_ucfg(uint8_t header_type,
                                              uint8_t* buff_p);
void hci_le_connection_complete_event_cb_ucfg_weak(uint8_t header_type,
                                                   uint8_t* buff_p);
void hci_le_connection_complete_event_cb(uint8_t header_type,
                                         uint8_t* buff_p);

void hci_le_advertising_report_event_cb_ucfg(uint8_t header_type,
                                             uint8_t* buff_p);
void hci_le_advertising_report_event_cb_ucfg_weak(uint8_t header_type,
                                                  uint8_t* buff_p);
void hci_le_advertising_report_event_cb(uint8_t header_type,
                                        uint8_t* buff_p);

void hci_le_long_term_key_request_event_cb_ucfg(uint8_t header_type,
                                                uint8_t* buff_p);
void hci_le_long_term_key_request_event_cb_ucfg_weak(uint8_t header_type,
                                                     uint8_t* buff_p);
void hci_le_long_term_key_request_event_cb(uint8_t header_type,
                                           uint8_t* buff_p);

void hci_le_read_local_p256_public_key_complete_event_cb_ucfg(uint8_t header_type,
                                                              uint8_t* buff_p);
void hci_le_read_local_p256_public_key_complete_event_cb_ucfg_weak(uint8_t header_type,
                                                                   uint8_t* buff_p);
void hci_le_read_local_p256_public_key_complete_event_cb(uint8_t header_type,
                                                         uint8_t* buff_p);

void hci_le_generate_dhkey_complete_event_cb_ucfg(uint8_t header_type,
                                                  uint8_t* buff_p);
void hci_le_generate_dhkey_complete_event_cb_ucfg_weak(uint8_t header_type,
                                                       uint8_t* buff_p);
void hci_le_generate_dhkey_complete_event_cb(uint8_t header_type,
                                             uint8_t* buff_p);

void hci_le_enhanced_connection_complete_event_cb_ucfg(uint8_t header_type,
                                                       uint8_t* buff_p);
void hci_le_enhanced_connection_complete_event_cb_ucfg_weak(uint8_t header_type,
                                                            uint8_t* buff_p);
void hci_le_enhanced_connection_complete_event_cb(uint8_t header_type,
                                                  uint8_t* buff_p);

void hci_le_direct_advertising_report_event_cb_ucfg(uint8_t header_type,
                                                    uint8_t* buff_p);
void hci_le_direct_advertising_report_event_cb_ucfg_weak(uint8_t header_type,
                                                         uint8_t* buff_p);
void hci_le_direct_advertising_report_event_cb(uint8_t header_type,
                                               uint8_t* buff_p);

void hci_le_extended_advertising_report_event_cb_ucfg(uint8_t header_type,
                                                      uint8_t* buff_p);
void hci_le_extended_advertising_report_event_cb_ucfg_weak(uint8_t header_type,
                                                           uint8_t* buff_p);
void hci_le_extended_advertising_report_event_cb(uint8_t header_type,
                                                 uint8_t* buff_p);

void hci_le_scan_timeout_event_cb_ucfg(uint8_t header_type,
                                       uint8_t* buff_p);
void hci_le_scan_timeout_event_cb_ucfg_weak(uint8_t header_type,
                                            uint8_t* buff_p);
void hci_le_scan_timeout_event_cb(uint8_t header_type,
                                  uint8_t* buff_p);

void hci_le_advertising_set_terminated_event_cb_ucfg(uint8_t header_type,
                                                     uint8_t* buff_p);
void hci_le_advertising_set_terminated_event_cb_ucfg_weak(uint8_t header_type,
                                                          uint8_t* buff_p);
void hci_le_advertising_set_terminated_event_cb(uint8_t header_type,
                                                uint8_t* buff_p);

void hci_read_remote_version_information_complete_event_cb_ucfg(uint8_t header_type,
                                                                uint8_t* buff_p);
void hci_read_remote_version_information_complete_event_cb_ucfg_weak(uint8_t header_type,
                                                                     uint8_t* buff_p);
void hci_read_remote_version_information_complete_event_cb(uint8_t header_type,
                                                           uint8_t* buff_p);

void hci_number_of_completed_packets_event_cb_ucfg(uint8_t header_type,
                                                   uint8_t* buff_p);
void hci_number_of_completed_packets_event_cb_ucfg_weak(uint8_t header_type,
                                                        uint8_t* buff_p);
void hci_number_of_completed_packets_event_cb(uint8_t header_type,
                                              uint8_t* buff_p);

void hci_data_buffer_overflow_event_cb_ucfg(uint8_t header_type,
                                            uint8_t* buff_p);
void hci_data_buffer_overflow_event_cb_ucfg_weak(uint8_t header_type,
                                                 uint8_t* buff_p);
void hci_data_buffer_overflow_event_cb(uint8_t header_type,
                                       uint8_t* buff_p);

void hci_authenticated_payload_timeout_expired_event_cb_ucfg(uint8_t header_type,
                                                             uint8_t* buff_p);
void hci_authenticated_payload_timeout_expired_event_cb_ucfg_weak(uint8_t header_type,
                                                                  uint8_t* buff_p);
void hci_authenticated_payload_timeout_expired_event_cb(uint8_t header_type,
                                                        uint8_t* buff_p);

void hci_le_connection_update_complete_event_cb_ucfg(uint8_t header_type,
                                                     uint8_t* buff_p);
void hci_le_connection_update_complete_event_cb_ucfg_weak(uint8_t header_type,
                                                          uint8_t* buff_p);
void hci_le_connection_update_complete_event_cb(uint8_t header_type,
                                                uint8_t* buff_p);

void hci_le_read_remote_used_features_complete_event_cb_ucfg(uint8_t header_type,
                                                             uint8_t* buff_p);
void hci_le_read_remote_used_features_complete_event_cb_ucfg_weak(uint8_t header_type,
                                                                  uint8_t* buff_p);
void hci_le_read_remote_used_features_complete_event_cb(uint8_t header_type,
                                                        uint8_t* buff_p);

void hci_le_data_length_change_event_cb_ucfg(uint8_t header_type,
                                             uint8_t* buff_p);
void hci_le_data_length_change_event_cb_ucfg_weak(uint8_t header_type,
                                                  uint8_t* buff_p);
void hci_le_data_length_change_event_cb(uint8_t header_type,
                                        uint8_t* buff_p);

void hci_le_phy_update_complete_event_cb_ucfg(uint8_t header_type,
                                              uint8_t* buff_p);
void hci_le_phy_update_complete_event_cb_ucfg_weak(uint8_t header_type,
                                                   uint8_t* buff_p);
void hci_le_phy_update_complete_event_cb(uint8_t header_type,
                                         uint8_t* buff_p);

void hci_le_periodic_advertising_sync_established_event_cb_ucfg(uint8_t header_type,
                                                                uint8_t* buff_p);
void hci_le_periodic_advertising_sync_established_event_cb_ucfg_weak(uint8_t header_type,
                                                                     uint8_t* buff_p);
void hci_le_periodic_advertising_sync_established_event_cb(uint8_t header_type,
                                                           uint8_t* buff_p);

void hci_le_periodic_advertising_report_event_cb_ucfg(uint8_t header_type,
                                                      uint8_t* buff_p);
void hci_le_periodic_advertising_report_event_cb_ucfg_weak(uint8_t header_type,
                                                           uint8_t* buff_p);
void hci_le_periodic_advertising_report_event_cb(uint8_t header_type,
                                                 uint8_t* buff_p);

void hci_le_periodic_advertising_sync_lost_event_cb_ucfg(uint8_t header_type,
                                                         uint8_t* buff_p);
void hci_le_periodic_advertising_sync_lost_event_cb_ucfg_weak(uint8_t header_type,
                                                              uint8_t* buff_p);
void hci_le_periodic_advertising_sync_lost_event_cb(uint8_t header_type,
                                                    uint8_t* buff_p);

void hci_le_scan_request_received_event_cb_ucfg(uint8_t header_type,
                                                uint8_t* buff_p);
void hci_le_scan_request_received_event_cb_ucfg_weak(uint8_t header_type,
                                                     uint8_t* buff_p);
void hci_le_scan_request_received_event_cb(uint8_t header_type,
                                           uint8_t* buff_p);

void hci_le_channel_selection_algorithm_event_cb_ucfg(uint8_t header_type,
                                                      uint8_t* buff_p);
void hci_le_channel_selection_algorithm_event_cb_ucfg_weak(uint8_t header_type,
                                                           uint8_t* buff_p);
void hci_le_channel_selection_algorithm_event_cb(uint8_t header_type,
                                                 uint8_t* buff_p);

void hci_le_connectionless_iq_report_event_cb_ucfg(uint8_t header_type,
                                                   uint8_t* buff_p);
void hci_le_connectionless_iq_report_event_cb_ucfg_weak(uint8_t header_type,
                                                        uint8_t* buff_p);
void hci_le_connectionless_iq_report_event_cb(uint8_t header_type,
                                              uint8_t* buff_p);

void hci_le_connection_iq_report_event_cb_ucfg(uint8_t header_type,
                                               uint8_t* buff_p);
void hci_le_connection_iq_report_event_cb_ucfg_weak(uint8_t header_type,
                                                    uint8_t* buff_p);
void hci_le_connection_iq_report_event_cb(uint8_t header_type,
                                          uint8_t* buff_p);

void hci_le_cte_request_failed_event_cb_ucfg(uint8_t header_type,
                                             uint8_t* buff_p);
void hci_le_cte_request_failed_event_cb_ucfg_weak(uint8_t header_type,
                                                  uint8_t* buff_p);
void hci_le_cte_request_failed_event_cb(uint8_t header_type,
                                        uint8_t* buff_p);

void hci_le_periodic_advertising_sync_transfer_received_event_cb_ucfg(uint8_t header_type,
                                                                      uint8_t* buff_p);
void hci_le_periodic_advertising_sync_transfer_received_event_cb_ucfg_weak(uint8_t header_type,
                                                                           uint8_t* buff_p);
void hci_le_periodic_advertising_sync_transfer_received_event_cb(uint8_t header_type,
                                                                 uint8_t* buff_p);

void hci_le_create_big_complete_event_cb_ucfg(uint8_t header_type,
                                              uint8_t* buff_p);
void hci_le_create_big_complete_event_cb_ucfg_weak(uint8_t header_type,
                                                   uint8_t* buff_p);
void hci_le_create_big_complete_event_cb(uint8_t header_type,
                                         uint8_t* buff_p);

void hci_le_terminate_big_complete_event_cb_ucfg(uint8_t header_type,
                                                 uint8_t* buff_p);
void hci_le_terminate_big_complete_event_cb_ucfg_weak(uint8_t header_type,
                                                      uint8_t* buff_p);
void hci_le_terminate_big_complete_event_cb(uint8_t header_type,
                                            uint8_t* buff_p);

void hci_le_big_sync_established_event_cb_ucfg(uint8_t header_type,
                                               uint8_t* buff_p);
void hci_le_big_sync_established_event_cb_ucfg_weak(uint8_t header_type,
                                                    uint8_t* buff_p);
void hci_le_big_sync_established_event_cb(uint8_t header_type,
                                          uint8_t* buff_p);

void hci_le_big_sync_lost_event_cb_ucfg(uint8_t header_type,
                                        uint8_t* buff_p);
void hci_le_big_sync_lost_event_cb_ucfg_weak(uint8_t header_type,
                                             uint8_t* buff_p);
void hci_le_big_sync_lost_event_cb(uint8_t header_type,
                                   uint8_t* buff_p);

void hci_le_path_loss_threshold_event_cb_ucfg(uint8_t header_type,
                                              uint8_t* buff_p);
void hci_le_path_loss_threshold_event_cb_ucfg_weak(uint8_t header_type,
                                                   uint8_t* buff_p);
void hci_le_path_loss_threshold_event_cb(uint8_t header_type,
                                         uint8_t* buff_p);

void hci_le_transmit_power_reporting_event_cb_ucfg(uint8_t header_type,
                                                   uint8_t* buff_p);
void hci_le_transmit_power_reporting_event_cb_ucfg_weak(uint8_t header_type,
                                                        uint8_t* buff_p);
void hci_le_transmit_power_reporting_event_cb(uint8_t header_type,
                                              uint8_t* buff_p);

void hci_le_biginfo_advertising_report_event_cb_ucfg(uint8_t header_type,
                                                     uint8_t* buff_p);
void hci_le_biginfo_advertising_report_event_cb_ucfg_weak(uint8_t header_type,
                                                          uint8_t* buff_p);
void hci_le_biginfo_advertising_report_event_cb(uint8_t header_type,
                                                uint8_t* buff_p);

void hci_le_subrate_change_event_cb_ucfg(uint8_t header_type,
                                         uint8_t* buff_p);
void hci_le_subrate_change_event_cb_ucfg_weak(uint8_t header_type,
                                              uint8_t* buff_p);
void hci_le_subrate_change_event_cb(uint8_t header_type,
                                    uint8_t* buff_p);

void hci_le_request_peer_sca_complete_event_cb_ucfg(uint8_t header_type,
                                                    uint8_t* buff_p);
void hci_le_request_peer_sca_complete_event_cb_ucfg_weak(uint8_t header_type,
                                                         uint8_t* buff_p);
void hci_le_request_peer_sca_complete_event_cb(uint8_t header_type,
                                               uint8_t* buff_p);

void hci_rx_acl_data_event_cb_ucfg(uint8_t header_type,
                                   uint8_t* buff_p);
void hci_rx_acl_data_event_cb_ucfg_weak(uint8_t header_type,
                                        uint8_t* buff_p);
void hci_rx_acl_data_event_cb(uint8_t header_type,
                              uint8_t* buff_p);

void hci_le_rx_iso_data_event_cb_ucfg(uint8_t header_type,
                                      uint8_t* buff_p);
void hci_le_rx_iso_data_event_cb_ucfg_weak(uint8_t header_type,
                                           uint8_t* buff_p);
void hci_le_rx_iso_data_event_cb(uint8_t header_type,
                                 uint8_t* buff_p);

void aci_l2cap_connection_update_resp_event_cb_ucfg(uint8_t header_type,
                                                    uint8_t* buff_p);
void aci_l2cap_connection_update_resp_event_cb_ucfg_weak(uint8_t header_type,
                                                         uint8_t* buff_p);
void aci_l2cap_connection_update_resp_event_cb(uint8_t header_type,
                                               uint8_t* buff_p);

void aci_l2cap_proc_timeout_event_cb_ucfg(uint8_t header_type,
                                          uint8_t* buff_p);
void aci_l2cap_proc_timeout_event_cb_ucfg_weak(uint8_t header_type,
                                               uint8_t* buff_p);
void aci_l2cap_proc_timeout_event_cb(uint8_t header_type,
                                     uint8_t* buff_p);

void aci_l2cap_connection_update_req_event_cb_ucfg(uint8_t header_type,
                                                   uint8_t* buff_p);
void aci_l2cap_connection_update_req_event_cb_ucfg_weak(uint8_t header_type,
                                                        uint8_t* buff_p);
void aci_l2cap_connection_update_req_event_cb(uint8_t header_type,
                                              uint8_t* buff_p);

void aci_l2cap_disconnection_complete_event_cb_ucfg(uint8_t header_type,
                                                    uint8_t* buff_p);
void aci_l2cap_disconnection_complete_event_cb_ucfg_weak(uint8_t header_type,
                                                         uint8_t* buff_p);
void aci_l2cap_disconnection_complete_event_cb(uint8_t header_type,
                                               uint8_t* buff_p);

void aci_l2cap_flow_control_credit_event_cb_ucfg(uint8_t header_type,
                                                 uint8_t* buff_p);
void aci_l2cap_flow_control_credit_event_cb_ucfg_weak(uint8_t header_type,
                                                      uint8_t* buff_p);
void aci_l2cap_flow_control_credit_event_cb(uint8_t header_type,
                                            uint8_t* buff_p);

void aci_l2cap_cos_connection_event_cb_ucfg(uint8_t header_type,
                                            uint8_t* buff_p);
void aci_l2cap_cos_connection_event_cb_ucfg_weak(uint8_t header_type,
                                                 uint8_t* buff_p);
void aci_l2cap_cos_connection_event_cb(uint8_t header_type,
                                       uint8_t* buff_p);

void aci_l2cap_ecfc_reconfiguration_event_cb_ucfg(uint8_t header_type,
                                                  uint8_t* buff_p);
void aci_l2cap_ecfc_reconfiguration_event_cb_ucfg_weak(uint8_t header_type,
                                                       uint8_t* buff_p);
void aci_l2cap_ecfc_reconfiguration_event_cb(uint8_t header_type,
                                             uint8_t* buff_p);

void aci_l2cap_command_reject_event_cb_ucfg(uint8_t header_type,
                                            uint8_t* buff_p);
void aci_l2cap_command_reject_event_cb_ucfg_weak(uint8_t header_type,
                                                 uint8_t* buff_p);
void aci_l2cap_command_reject_event_cb(uint8_t header_type,
                                       uint8_t* buff_p);

void aci_l2cap_sdu_data_tx_event_cb_ucfg(uint8_t header_type,
                                         uint8_t* buff_p);
void aci_l2cap_sdu_data_tx_event_cb_ucfg_weak(uint8_t header_type,
                                              uint8_t* buff_p);
void aci_l2cap_sdu_data_tx_event_cb(uint8_t header_type,
                                    uint8_t* buff_p);

void aci_l2cap_sdu_data_rx_event_cb_ucfg(uint8_t header_type,
                                         uint8_t* buff_p);
void aci_l2cap_sdu_data_rx_event_cb_ucfg_weak(uint8_t header_type,
                                              uint8_t* buff_p);
void aci_l2cap_sdu_data_rx_event_cb(uint8_t header_type,
                                    uint8_t* buff_p);

void aci_gap_pairing_complete_event_cb_ucfg(uint8_t header_type,
                                            uint8_t* buff_p);
void aci_gap_pairing_complete_event_cb_ucfg_weak(uint8_t header_type,
                                                 uint8_t* buff_p);
void aci_gap_pairing_complete_event_cb(uint8_t header_type,
                                       uint8_t* buff_p);

void aci_gap_pass_key_req_event_cb_ucfg(uint8_t header_type,
                                        uint8_t* buff_p);
void aci_gap_pass_key_req_event_cb_ucfg_weak(uint8_t header_type,
                                             uint8_t* buff_p);
void aci_gap_pass_key_req_event_cb(uint8_t header_type,
                                   uint8_t* buff_p);

void aci_gap_slave_security_initiated_event_cb_ucfg(uint8_t header_type,
                                                    uint8_t* buff_p);
void aci_gap_slave_security_initiated_event_cb_ucfg_weak(uint8_t header_type,
                                                         uint8_t* buff_p);
void aci_gap_slave_security_initiated_event_cb(uint8_t header_type,
                                               uint8_t* buff_p);

void aci_gap_bond_lost_event_cb_ucfg(uint8_t header_type,
                                     uint8_t* buff_p);
void aci_gap_bond_lost_event_cb_ucfg_weak(uint8_t header_type,
                                          uint8_t* buff_p);
void aci_gap_bond_lost_event_cb(uint8_t header_type,
                                uint8_t* buff_p);

void aci_gap_numeric_comparison_value_event_cb_ucfg(uint8_t header_type,
                                                    uint8_t* buff_p);
void aci_gap_numeric_comparison_value_event_cb_ucfg_weak(uint8_t header_type,
                                                         uint8_t* buff_p);
void aci_gap_numeric_comparison_value_event_cb(uint8_t header_type,
                                               uint8_t* buff_p);

void aci_gap_keypress_notification_event_cb_ucfg(uint8_t header_type,
                                                 uint8_t* buff_p);
void aci_gap_keypress_notification_event_cb_ucfg_weak(uint8_t header_type,
                                                      uint8_t* buff_p);
void aci_gap_keypress_notification_event_cb(uint8_t header_type,
                                            uint8_t* buff_p);

tBleStatus PM_init_ucfg(void);
tBleStatus PM_init_ucfg_weak(void);
tBleStatus PM_init(void);

uint32_t secure_connections_csr_ucfg(void);
uint32_t secure_connections_csr_ucfg_weak(void);
uint32_t secure_connections_csr(void);

tBleStatus smp_sap_scp_force_debug_key_usage_ucfg(uint8_t config_val);
tBleStatus smp_sap_scp_force_debug_key_usage_ucfg_weak(uint8_t config_val);
tBleStatus smp_sap_scp_force_debug_key_usage(uint8_t config_val);

void smp_sap_hci_le_read_local_p256_public_key_complete_evt_hndl_ucfg(uint8_t status,
                                                                      uint8_t local_p256_public_key[64]);
void smp_sap_hci_le_read_local_p256_public_key_complete_evt_hndl_ucfg_weak(uint8_t status,
                                                                           uint8_t local_p256_public_key[64]);
void smp_sap_hci_le_read_local_p256_public_key_complete_evt_hndl(uint8_t status,
                                                                 uint8_t local_p256_public_key[64]);

void smp_sap_hci_le_generate_dhkey_complete_evt_hndl_ucfg(uint8_t status,
                                                          uint8_t dhkey[32]);
void smp_sap_hci_le_generate_dhkey_complete_evt_hndl_ucfg_weak(uint8_t status,
                                                               uint8_t dhkey[32]);
void smp_sap_hci_le_generate_dhkey_complete_evt_hndl(uint8_t status,
                                                     uint8_t dhkey[32]);

tBleStatus L2C_init_ucfg(uint8_t cos_enabled);
tBleStatus L2C_init_ucfg_weak(uint8_t cos_enabled);
tBleStatus L2C_init(uint8_t cos_enabled);

tBleStatus L2C_cos_process_cfc_mode_command_ucfg(void* params);
tBleStatus L2C_cos_process_cfc_mode_command_ucfg_weak(void* params);
tBleStatus L2C_cos_process_cfc_mode_command(void* params);

BOOL L2C_cos_is_pdu_cframe_cfc_command_opcode_ucfg(uint8_t opCode);
BOOL L2C_cos_is_pdu_cframe_cfc_command_opcode_ucfg_weak(uint8_t opCode);
BOOL L2C_cos_is_pdu_cframe_cfc_command_opcode(uint8_t opCode);

uint32_t l2c_cos_csr_ucfg(void);
uint32_t l2c_cos_csr_ucfg_weak(void);
uint32_t l2c_cos_csr(void);

tBleStatus L2C_cos_cfc_init_ucfg(void);
tBleStatus L2C_cos_cfc_init_ucfg_weak(void);
tBleStatus L2C_cos_cfc_init(void);

void L2C_cos_process_pending_actions_tsk_ucfg(uint16_t task_idx);
void L2C_cos_process_pending_actions_tsk_ucfg_weak(uint16_t task_idx);
void L2C_cos_process_pending_actions_tsk(uint16_t task_idx);

void L2C_cos_physical_link_disconnection_hndl_ucfg(uint16_t connection_handle);
void L2C_cos_physical_link_disconnection_hndl_ucfg_weak(uint16_t connection_handle);
void L2C_cos_physical_link_disconnection_hndl(uint16_t connection_handle);

tBleStatus l2c_cos_enqueue_sdu_over_ecfc_ucfg(void* params);
tBleStatus l2c_cos_enqueue_sdu_over_ecfc_ucfg_weak(void* params);
tBleStatus l2c_cos_enqueue_sdu_over_ecfc(void* params);

void L2C_cos_ecfc_perform_sdu_segmentation_tsk_ucfg(uint16_t task_idx);
void L2C_cos_ecfc_perform_sdu_segmentation_tsk_ucfg_weak(uint16_t task_idx);
void L2C_cos_ecfc_perform_sdu_segmentation_tsk(uint16_t task_idx);

void L2C_cos_perform_sdu_segmentation_tsk_ucfg(uint16_t task_idx);
void L2C_cos_perform_sdu_segmentation_tsk_ucfg_weak(uint16_t task_idx);
void L2C_cos_perform_sdu_segmentation_tsk(uint16_t task_idx);

void L2C_cos_transmit_pdu_packets_tsk_ucfg(uint16_t task_idx);
void L2C_cos_transmit_pdu_packets_tsk_ucfg_weak(uint16_t task_idx);
void L2C_cos_transmit_pdu_packets_tsk(uint16_t task_idx);

void L2C_cos_process_pending_pdu_for_reassembly_tsk_ucfg(uint16_t task_idx);
void L2C_cos_process_pending_pdu_for_reassembly_tsk_ucfg_weak(uint16_t task_idx);
void L2C_cos_process_pending_pdu_for_reassembly_tsk(uint16_t task_idx);

tBleStatus L2C_cos_le_frame_data_hndl_ucfg(void* params);
tBleStatus L2C_cos_le_frame_data_hndl_ucfg_weak(void* params);
tBleStatus L2C_cos_le_frame_data_hndl(void* params);

void llc_conn_check_subrate_and_set_params_ucfg(void* cntxt_p,
                                                void* params_p);
void llc_conn_check_subrate_and_set_params_ucfg_weak(void* cntxt_p,
                                                     void* params_p);
void llc_conn_check_subrate_and_set_params(void* cntxt_p,
                                           void* params_p);

tBleStatus llc_conn_multi_link_connection_ucfg(uint8_t enable);
tBleStatus llc_conn_multi_link_connection_ucfg_weak(uint8_t enable);
tBleStatus llc_conn_multi_link_connection(uint8_t enable);

uint32_t llc_conn_calc_skip_ucfg(void* cntxt_p,
                                 uint16_t event_counter,
                                 uint16_t latency);
uint32_t llc_conn_calc_skip_ucfg_weak(void* cntxt_p,
                                      uint16_t event_counter,
                                      uint16_t latency);
uint32_t llc_conn_calc_skip(void* cntxt_p,
                            uint16_t event_counter,
                            uint16_t latency);

void llc_conn_slave_latency_cancellation_tsk_ucfg(uint16_t task_idx);
void llc_conn_slave_latency_cancellation_tsk_ucfg_weak(uint16_t task_idx);
void llc_conn_slave_latency_cancellation_tsk(uint16_t task_idx);

tBleStatus LL_LE_Setup_ISO_Data_Path_For_CIS_ucfg(void* IsoDataPathParams_p,
                                                  void* llc_cigcis_context_per_p);
tBleStatus LL_LE_Setup_ISO_Data_Path_For_CIS_ucfg_weak(void* IsoDataPathParams_p,
                                                       void* llc_cigcis_context_per_p);
tBleStatus LL_LE_Setup_ISO_Data_Path_For_CIS(void* IsoDataPathParams_p,
                                             void* llc_cigcis_context_per_p);

tBleStatus LL_LE_Setup_ISO_Data_Path_For_BIS_Brc_ucfg(void* IsoDataPathParams_p);
tBleStatus LL_LE_Setup_ISO_Data_Path_For_BIS_Brc_ucfg_weak(void* IsoDataPathParams_p);
tBleStatus LL_LE_Setup_ISO_Data_Path_For_BIS_Brc(void* IsoDataPathParams_p);

tBleStatus LL_LE_Setup_ISO_Data_Path_For_BIS_Sync_ucfg(void* IsoDataPathParams_p);
tBleStatus LL_LE_Setup_ISO_Data_Path_For_BIS_Sync_ucfg_weak(void* IsoDataPathParams_p);
tBleStatus LL_LE_Setup_ISO_Data_Path_For_BIS_Sync(void* IsoDataPathParams_p);

void llc_bigbis_brc_update_all_bisbuffer_mloop_ucfg(uint16_t task_idx);
void llc_bigbis_brc_update_all_bisbuffer_mloop_ucfg_weak(uint16_t task_idx);
void llc_bigbis_brc_update_all_bisbuffer_mloop(uint16_t task_idx);

BOOL llc_bigbis_brc_ISO_retrieve_CIGBIG_configuration_callback_ucfg(void* pointer,
                                                                    uint16_t connection_handle);
BOOL llc_bigbis_brc_ISO_retrieve_CIGBIG_configuration_callback_ucfg_weak(void* pointer,
                                                                         uint16_t connection_handle);
BOOL llc_bigbis_brc_ISO_retrieve_CIGBIG_configuration_callback(void* pointer,
                                                               uint16_t connection_handle);

void* llc_bigbisbrc_return_BIGcontext_from_connection_handle_ucfg(uint16_t connection_handle,
                                                                  uint8_t* index);
void* llc_bigbisbrc_return_BIGcontext_from_connection_handle_ucfg_weak(uint16_t connection_handle,
                                                                       uint8_t* index);
void* llc_bigbisbrc_return_BIGcontext_from_connection_handle(uint16_t connection_handle,
                                                             uint8_t* index);

BOOL llc_bigbisbrc_bigbis_is_big_enabled_ucfg(void* periodic_per_pointer_in);
BOOL llc_bigbisbrc_bigbis_is_big_enabled_ucfg_weak(void* periodic_per_pointer_in);
BOOL llc_bigbisbrc_bigbis_is_big_enabled(void* periodic_per_pointer_in);

void llc_bigbisbrc_bigbis_add_biginfo_to_periodic_sync_packet_ucfg(void* pointer_in,
                                                                   uint8_t data_position,
                                                                   uint8_t* packet);
void llc_bigbisbrc_bigbis_add_biginfo_to_periodic_sync_packet_ucfg_weak(void* pointer_in,
                                                                        uint8_t data_position,
                                                                        uint8_t* packet);
void llc_bigbisbrc_bigbis_add_biginfo_to_periodic_sync_packet(void* pointer_in,
                                                              uint8_t data_position,
                                                              uint8_t* packet);

uint32_t llc_bigbis_return_periodic_anchor_ucfg(void* periodic_per_pointer_in);
uint32_t llc_bigbis_return_periodic_anchor_ucfg_weak(void* periodic_per_pointer_in);
uint32_t llc_bigbis_return_periodic_anchor(void* periodic_per_pointer_in);

void llc_cig_common_mem_alloc_ucfg(void);
void llc_cig_common_mem_alloc_ucfg_weak(void);
void llc_cig_common_mem_alloc(void);

BOOL llc_cig_active_cis_on_acl_ucfg(void* LL_return_cig_cis_context_p,
                                    uint8_t conn_idx);
BOOL llc_cig_active_cis_on_acl_ucfg_weak(void* LL_return_cig_cis_context_p,
                                         uint8_t conn_idx);
BOOL llc_cig_active_cis_on_acl(void* LL_return_cig_cis_context_p,
                               uint8_t conn_idx);

void terminate_active_cis_on_acl_ucfg(uint8_t conn_idx);
void terminate_active_cis_on_acl_ucfg_weak(uint8_t conn_idx);
void terminate_active_cis_on_acl(uint8_t conn_idx);

void llc_conn_cis_create_ucfg(uint8_t conn_idx,
                              uint16_t instant);
void llc_conn_cis_create_ucfg_weak(uint8_t conn_idx,
                                   uint16_t instant);
void llc_conn_cis_create(uint8_t conn_idx,
                         uint16_t instant);

void* LL_cig_return_cis_context_from_CIS_handle_ucfg(uint16_t cis_handle);
void* LL_cig_return_cis_context_from_CIS_handle_ucfg_weak(uint16_t cis_handle);
void* LL_cig_return_cis_context_from_CIS_handle(uint16_t cis_handle);

void llc_iso_common_mem_alloc_ucfg(void);
void llc_iso_common_mem_alloc_ucfg_weak(void);
void llc_iso_common_mem_alloc(void);

void llc_bigbis_common_mem_alloc_ucfg(BOOL master_role_enabled);
void llc_bigbis_common_mem_alloc_ucfg_weak(BOOL master_role_enabled);
void llc_bigbis_common_mem_alloc(BOOL master_role_enabled);

uint32_t bis_csr_ucfg(void);
uint32_t bis_csr_ucfg_weak(void);
uint32_t bis_csr(void);

uint32_t cis_csr_ucfg(void);
uint32_t cis_csr_ucfg_weak(void);
uint32_t cis_csr(void);

uint32_t iso_csr_ucfg(void);
uint32_t iso_csr_ucfg_weak(void);
uint32_t iso_csr(void);

void llc_bigbis_sync_update_bisbuffer_mloop_ucfg(uint16_t task_idx);
void llc_bigbis_sync_update_bisbuffer_mloop_ucfg_weak(uint16_t task_idx);
void llc_bigbis_sync_update_bisbuffer_mloop(uint16_t task_idx);

BOOL llc_bigbis_sync_ISO_retrieve_CIGBIG_configuration_callback_ucfg(void* pointer,
                                                                     uint16_t connection_handle);
BOOL llc_bigbis_sync_ISO_retrieve_CIGBIG_configuration_callback_ucfg_weak(void* pointer,
                                                                          uint16_t connection_handle);
BOOL llc_bigbis_sync_ISO_retrieve_CIGBIG_configuration_callback(void* pointer,
                                                                uint16_t connection_handle);

void llc_bigbis_sync_deploysync_ucfg(uint8_t* biginfo,
                                     uint16_t sync_handle,
                                     uint32_t periodicreferenceanchor,
                                     uint8_t per_ADV_SCA);
void llc_bigbis_sync_deploysync_ucfg_weak(uint8_t* biginfo,
                                          uint16_t sync_handle,
                                          uint32_t periodicreferenceanchor,
                                          uint8_t per_ADV_SCA);
void llc_bigbis_sync_deploysync(uint8_t* biginfo,
                                uint16_t sync_handle,
                                uint32_t periodicreferenceanchor,
                                uint8_t per_ADV_SCA);

void* llc_bigbis_sync_return_BIGcontext_from_connection_handle_ucfg(uint16_t connection_handle,
                                                                    uint8_t* index);
void* llc_bigbis_sync_return_BIGcontext_from_connection_handle_ucfg_weak(uint16_t connection_handle,
                                                                         uint8_t* index);
void* llc_bigbis_sync_return_BIGcontext_from_connection_handle(uint16_t connection_handle,
                                                               uint8_t* index);

void llc_priv_generate_peer_rpa_from_peer_id_ucfg(void* peer_p,
                                                  BOOL in_isr);
void llc_priv_generate_peer_rpa_from_peer_id_ucfg_weak(void* peer_p,
                                                       BOOL in_isr);
void llc_priv_generate_peer_rpa_from_peer_id(void* peer_p,
                                             BOOL in_isr);

void llc_priv_init_ucfg(BOOL first_call);
void llc_priv_init_ucfg_weak(BOOL first_call);
void llc_priv_init(BOOL first_call);

void llc_priv_generate_local_rpa_from_peer_id_ucfg(void* peer_id_p,
                                                   void* local_rpa_p);
void llc_priv_generate_local_rpa_from_peer_id_ucfg_weak(void* peer_id_p,
                                                        void* local_rpa_p);
void llc_priv_generate_local_rpa_from_peer_id(void* peer_id_p,
                                              void* local_rpa_p);

void llc_priv_offline_rpa_resolution_ucfg(uint16_t task_idx);
void llc_priv_offline_rpa_resolution_ucfg_weak(uint16_t task_idx);
void llc_priv_offline_rpa_resolution(uint16_t task_idx);

uint32_t llc_priv_controller_privacy_csr_ucfg(void);
uint32_t llc_priv_controller_privacy_csr_ucfg_weak(void);
uint32_t llc_priv_controller_privacy_csr(void);

void llc_priv_set_white_list_flag_in_resolving_list_ucfg(void* addr8_p,
                                                         BOOL set);
void llc_priv_set_white_list_flag_in_resolving_list_ucfg_weak(void* addr8_p,
                                                              BOOL set);
void llc_priv_set_white_list_flag_in_resolving_list(void* addr8_p,
                                                    BOOL set);

void llc_priv_clr_all_white_list_flags_in_resolving_list_ucfg(void);
void llc_priv_clr_all_white_list_flags_in_resolving_list_ucfg_weak(void);
void llc_priv_clr_all_white_list_flags_in_resolving_list(void);

void llc_priv_start_privacy_timer_ucfg(void);
void llc_priv_start_privacy_timer_ucfg_weak(void);
void llc_priv_start_privacy_timer(void);

void llc_priv_stop_privacy_timer_ucfg(void);
void llc_priv_stop_privacy_timer_ucfg_weak(void);
void llc_priv_stop_privacy_timer(void);

void llc_priv_init_random_part_of_one_local_rpa_ucfg(void* peer_id_p);
void llc_priv_init_random_part_of_one_local_rpa_ucfg_weak(void* peer_id_p);
void llc_priv_init_random_part_of_one_local_rpa(void* peer_id_p);

uint8_t llc_check_adv_or_srsp_or_crsp_tx_addr_ucfg(uint8_t* tx_addr7_p,
                                                   uint8_t adv_pdu,
                                                   uint8_t filter_policy,
                                                   uint8_t* peer_id_addr7_p,
                                                   uint8_t* rl_index_p,
                                                   uint8_t* res_peer_id_addr7_p);
uint8_t llc_check_adv_or_srsp_or_crsp_tx_addr_ucfg_weak(uint8_t* tx_addr7_p,
                                                        uint8_t adv_pdu,
                                                        uint8_t filter_policy,
                                                        uint8_t* peer_id_addr7_p,
                                                        uint8_t* rl_index_p,
                                                        uint8_t* res_peer_id_addr7_p);
uint8_t llc_check_adv_or_srsp_or_crsp_tx_addr(uint8_t* tx_addr7_p,
                                              uint8_t adv_pdu,
                                              uint8_t filter_policy,
                                              uint8_t* peer_id_addr7_p,
                                              uint8_t* rl_index_p,
                                              uint8_t* res_peer_id_addr7_p);

void llc_scan_conn_ind_sent_ucfg(void* ptr,
                                 uint8_t idx);
void llc_scan_conn_ind_sent_ucfg_weak(void* ptr,
                                      uint8_t idx);
void llc_scan_conn_ind_sent(void* ptr,
                            uint8_t idx);

BOOL llc_scan_process_ext_adv_ucfg(void* scan_p,
                                   void* params_p,
                                   uint32_t direct_addr[2],
                                   uint8_t idx,
                                   BOOL advertiser_addr_flag,
                                   BOOL* send_report_p);
BOOL llc_scan_process_ext_adv_ucfg_weak(void* scan_p,
                                        void* params_p,
                                        uint32_t direct_addr[2],
                                        uint8_t idx,
                                        BOOL advertiser_addr_flag,
                                        BOOL* send_report_p);
BOOL llc_scan_process_ext_adv(void* scan_p,
                              void* params_p,
                              uint32_t direct_addr[2],
                              uint8_t idx,
                              BOOL advertiser_addr_flag,
                              BOOL* send_report_p);

void llc_scan_aux_conn_rsp_rcvd_ucfg(void* scan_p,
                                     void* aux_p,
                                     uint8_t idx);
void llc_scan_aux_conn_rsp_rcvd_ucfg_weak(void* scan_p,
                                          void* aux_p,
                                          uint8_t idx);
void llc_scan_aux_conn_rsp_rcvd(void* scan_p,
                                void* aux_p,
                                uint8_t idx);

void llc_scan_init_ucfg(uint8_t master_en,
                        uint8_t ext_en);
void llc_scan_init_ucfg_weak(uint8_t master_en,
                             uint8_t ext_en);
void llc_scan_init(uint8_t master_en,
                   uint8_t ext_en);

void llc_scan_mem_allocate_ucfg(uint8_t master_en,
                                uint8_t ext_en);
void llc_scan_mem_allocate_ucfg_weak(uint8_t master_en,
                                     uint8_t ext_en);
void llc_scan_mem_allocate(uint8_t master_en,
                           uint8_t ext_en);

void llc_scan_prepare_conn_ind_req_ucfg(void* ptr,
                                        uint8_t idx,
                                        BOOL aux_conn_req);
void llc_scan_prepare_conn_ind_req_ucfg_weak(void* ptr,
                                             uint8_t idx,
                                             BOOL aux_conn_req);
void llc_scan_prepare_conn_ind_req(void* ptr,
                                   uint8_t idx,
                                   BOOL aux_conn_req);

void llc_scan_enable_extended_ucfg(void* scan_p);
void llc_scan_enable_extended_ucfg_weak(void* scan_p);
void llc_scan_enable_extended(void* scan_p);

void llc_scan_push_ext_adv_report_ucfg(void* ptr,
                                       uint8_t data_len,
                                       uint8_t data_offset,
                                       uint8_t event_type);
void llc_scan_push_ext_adv_report_ucfg_weak(void* ptr,
                                            uint8_t data_len,
                                            uint8_t data_offset,
                                            uint8_t event_type);
void llc_scan_push_ext_adv_report(void* ptr,
                                  uint8_t data_len,
                                  uint8_t data_offset,
                                  uint8_t event_type);

tBleStatus llc_scan_reserve_taskslots_ucfg(void* scan_en_p,
                                           void* scan_p);
tBleStatus llc_scan_reserve_taskslots_ucfg_weak(void* scan_en_p,
                                                void* scan_p);
tBleStatus llc_scan_reserve_taskslots(void* scan_en_p,
                                      void* scan_p);

void llc_scan_set_conn_params_ucfg(Extended_Create_Connection_Parameters_t ext_create_conn_params,
                                   uint8_t initiating_phy);
void llc_scan_set_conn_params_ucfg_weak(Extended_Create_Connection_Parameters_t ext_create_conn_params,
                                        uint8_t initiating_phy);
void llc_scan_set_conn_params(Extended_Create_Connection_Parameters_t ext_create_conn_params,
                              uint8_t initiating_phy);

void llc_scan_disable_ucfg(void* scan_p);
void llc_scan_disable_ucfg_weak(void* scan_p);
void llc_scan_disable(void* scan_p);

BOOL llc_subrate_get_active_sr_req_proc_ucfg(uint8_t conn_idx);
BOOL llc_subrate_get_active_sr_req_proc_ucfg_weak(uint8_t conn_idx);
BOOL llc_subrate_get_active_sr_req_proc(uint8_t conn_idx);

void llc_subrate_new_sr_base_event_ucfg(uint16_t sr_factor,
                                        uint16_t* sr_base_event_p);
void llc_subrate_new_sr_base_event_ucfg_weak(uint16_t sr_factor,
                                             uint16_t* sr_base_event_p);
void llc_subrate_new_sr_base_event(uint16_t sr_factor,
                                   uint16_t* sr_base_event_p);

void llc_subrate_update_cont_counter_ucfg(void* cntxt_p);
void llc_subrate_update_cont_counter_ucfg_weak(void* cntxt_p);
void llc_subrate_update_cont_counter(void* cntxt_p);

BOOL llc_subrate_offline_processing_ucfg(void* cntxt_p);
BOOL llc_subrate_offline_processing_ucfg_weak(void* cntxt_p);
BOOL llc_subrate_offline_processing(void* cntxt_p);

uint8_t llc_check_sreq_or_creq_tx_addr_ucfg(void* tx_addr7_p,
                                            uint8_t pdu_type,
                                            uint8_t adv_event_prop,
                                            uint8_t adv_filter_policy,
                                            void* peer_id_addr7_p,
                                            void* res_peer_id_addr7_p,
                                            uint8_t* rl_index_p);
uint8_t llc_check_sreq_or_creq_tx_addr_ucfg_weak(void* tx_addr7_p,
                                                 uint8_t pdu_type,
                                                 uint8_t adv_event_prop,
                                                 uint8_t adv_filter_policy,
                                                 void* peer_id_addr7_p,
                                                 void* res_peer_id_addr7_p,
                                                 uint8_t* rl_index_p);
uint8_t llc_check_sreq_or_creq_tx_addr(void* tx_addr7_p,
                                       uint8_t pdu_type,
                                       uint8_t adv_event_prop,
                                       uint8_t adv_filter_policy,
                                       void* peer_id_addr7_p,
                                       void* res_peer_id_addr7_p,
                                       uint8_t* rl_index_p);

uint8_t llc_check_adv_or_crsp_rx_addr_ucfg(uint8_t* rx_addr7_p,
                                           uint8_t adv_pdu,
                                           uint8_t filter_policy,
                                           uint8_t* local_addr7_p,
                                           uint8_t local_addr_type,
                                           uint8_t rl_index);
uint8_t llc_check_adv_or_crsp_rx_addr_ucfg_weak(uint8_t* rx_addr7_p,
                                                uint8_t adv_pdu,
                                                uint8_t filter_policy,
                                                uint8_t* local_addr7_p,
                                                uint8_t local_addr_type,
                                                uint8_t rl_index);
uint8_t llc_check_adv_or_crsp_rx_addr(uint8_t* rx_addr7_p,
                                      uint8_t adv_pdu,
                                      uint8_t filter_policy,
                                      uint8_t* local_addr7_p,
                                      uint8_t local_addr_type,
                                      uint8_t rl_index);

void LL_cpe_init_length_update_ucfg(void);
void LL_cpe_init_length_update_ucfg_weak(void);
void LL_cpe_init_length_update(void);

void LL_cpe_init_phy_update_ucfg(void);
void LL_cpe_init_phy_update_ucfg_weak(void);
void LL_cpe_init_phy_update(void);

void LL_cpe_init_cte_ucfg(void);
void LL_cpe_init_cte_ucfg_weak(void);
void LL_cpe_init_cte(void);

void LL_cpe_init_past_ucfg(void);
void LL_cpe_init_past_ucfg_weak(void);
void LL_cpe_init_past(void);

void LL_cpe_init_pcl_ucfg(void);
void LL_cpe_init_pcl_ucfg_weak(void);
void LL_cpe_init_pcl(void);

void LL_cpe_init_chmap_update_ucfg(void);
void LL_cpe_init_chmap_update_ucfg_weak(void);
void LL_cpe_init_chmap_update(void);

void LL_cpe_init_chc_enable_ucfg(void);
void LL_cpe_init_chc_enable_ucfg_weak(void);
void LL_cpe_init_chc_enable(void);

void LL_cpe_init_chc_reporting_ucfg(void);
void LL_cpe_init_chc_reporting_ucfg_weak(void);
void LL_cpe_init_chc_reporting(void);

void LL_cpe_init_subrate_ucfg(void);
void LL_cpe_init_subrate_ucfg_weak(void);
void LL_cpe_init_subrate(void);

void LL_cpe_init_sca_upd_ucfg(void);
void LL_cpe_init_sca_upd_ucfg_weak(void);
void LL_cpe_init_sca_upd(void);

void LL_cpe_init_cis_ucfg(void);
void LL_cpe_init_cis_ucfg_weak(void);
void LL_cpe_init_cis(void);

void LL_cpe_init_ucfg(void);
void LL_cpe_init_ucfg_weak(void);
void LL_cpe_init(void);

void LLC_channel_map_copy_to_cpf_context_ucfg(void* cntxt_p,
                                              uint8_t conn_idx);
void LLC_channel_map_copy_to_cpf_context_ucfg_weak(void* cntxt_p,
                                                   uint8_t conn_idx);
void LLC_channel_map_copy_to_cpf_context(void* cntxt_p,
                                         uint8_t conn_idx);

BOOL LLC_channel_map_update_offline_processing_ucfg(uint8_t conn_idx);
BOOL LLC_channel_map_update_offline_processing_ucfg_weak(uint8_t conn_idx);
BOOL LLC_channel_map_update_offline_processing(uint8_t conn_idx);

void LLC_conn_init_chc_ctxt_ucfg(uint8_t conn_idx);
void LLC_conn_init_chc_ctxt_ucfg_weak(uint8_t conn_idx);
void LLC_conn_init_chc_ctxt(uint8_t conn_idx);

void LLC_chc_ll_cntxt_init_ucfg(void);
void LLC_chc_ll_cntxt_init_ucfg_weak(void);
void LLC_chc_ll_cntxt_init(void);

BOOL LLC_chc_reporting_offline_processing_ucfg(uint8_t conn_idx);
BOOL LLC_chc_reporting_offline_processing_ucfg_weak(uint8_t conn_idx);
BOOL LLC_chc_reporting_offline_processing(uint8_t conn_idx);

void LL_conn_upd_max_tx_time_coded_ucfg(void* params);
void LL_conn_upd_max_tx_time_coded_ucfg_weak(void* params);
void LL_conn_upd_max_tx_time_coded(void* params);

void LL_conn_upd_data_length_change_event_ucfg(void* params);
void LL_conn_upd_data_length_change_event_ucfg_weak(void* params);
void LL_conn_upd_data_length_change_event(void* params);

void LL_cpf_CREATE_CIS_offline_process_ucfg(uint16_t task_idx);
void LL_cpf_CREATE_CIS_offline_process_ucfg_weak(uint16_t task_idx);
void LL_cpf_CREATE_CIS_offline_process(uint16_t task_idx);

tBleStatus llc_cig_disconnect_cis_ucfg(uint16_t cis_handle,
                                       uint8_t reason);
tBleStatus llc_cig_disconnect_cis_ucfg_weak(uint16_t cis_handle,
                                            uint8_t reason);
tBleStatus llc_cig_disconnect_cis(uint16_t cis_handle,
                                  uint8_t reason);

void llc_conn_init_cte_ctxt_ucfg(uint8_t conn_idx);
void llc_conn_init_cte_ctxt_ucfg_weak(uint8_t conn_idx);
void llc_conn_init_cte_ctxt(uint8_t conn_idx);

BOOL LLC_cte_request_procedure_ucfg(void* params);
BOOL LLC_cte_request_procedure_ucfg_weak(void* params);
BOOL LLC_cte_request_procedure(void* params);

void LLC_connection_cte_response_pause_resume_ucfg(uint8_t conn_idx,
                                                   uint8_t tx_phy);
void LLC_connection_cte_response_pause_resume_ucfg_weak(uint8_t conn_idx,
                                                        uint8_t tx_phy);
void LLC_connection_cte_response_pause_resume(uint8_t conn_idx,
                                              uint8_t tx_phy);

void LLC_connection_cte_response_disable_ucfg(uint8_t conn_idx,
                                              uint8_t taskslot_no);
void LLC_connection_cte_response_disable_ucfg_weak(uint8_t conn_idx,
                                                   uint8_t taskslot_no);
void LLC_connection_cte_response_disable(uint8_t conn_idx,
                                         uint8_t taskslot_no);

void llc_cte_process_rx_cte_ucfg(void* params,
                                 uint8_t cte_type);
void llc_cte_process_rx_cte_ucfg_weak(void* params,
                                      uint8_t cte_type);
void llc_cte_process_rx_cte(void* params,
                            uint8_t cte_type);

void LLC_authenticated_payload_timeout_processing_ucfg(uint16_t task_idx);
void LLC_authenticated_payload_timeout_processing_ucfg_weak(uint16_t task_idx);
void LLC_authenticated_payload_timeout_processing(uint16_t task_idx);

void LL_past_reload_txctrl_packet_ucfg(uint8_t conn_idx,
                                       uint8_t* pdu_ptr);
void LL_past_reload_txctrl_packet_ucfg_weak(uint8_t conn_idx,
                                            uint8_t* pdu_ptr);
void LL_past_reload_txctrl_packet(uint8_t conn_idx,
                                  uint8_t* pdu_ptr);

void LL_past_initialize_connect_context_ucfg(uint8_t conn_idx);
void LL_past_initialize_connect_context_ucfg_weak(uint8_t conn_idx);
void LL_past_initialize_connect_context(uint8_t conn_idx);

void LL_past_default_params_ucfg(uint8_t conn_idx);
void LL_past_default_params_ucfg_weak(uint8_t conn_idx);
void LL_past_default_params(uint8_t conn_idx);

void llc_conn_init_pcl_ctxt_ucfg(uint8_t conn_idx);
void llc_conn_init_pcl_ctxt_ucfg_weak(uint8_t conn_idx);
void llc_conn_init_pcl_ctxt(uint8_t conn_idx);

void LLC_pcl_hal_cntxt_init_ucfg(uintptr_t pcl_cntxt_p);
void LLC_pcl_hal_cntxt_init_ucfg_weak(uintptr_t pcl_cntxt_p);
void LLC_pcl_hal_cntxt_init(uintptr_t pcl_cntxt_p);

void LLC_pcl_incr_rx_packets_ucfg(uint8_t conn_idx);
void LLC_pcl_incr_rx_packets_ucfg_weak(uint8_t conn_idx);
void LLC_pcl_incr_rx_packets(uint8_t conn_idx);

void LLC_pcl_update_avg_rssi_ucfg(void* params);
void LLC_pcl_update_avg_rssi_ucfg_weak(void* params);
void LLC_pcl_update_avg_rssi(void* params);

void LLC_pcl_set_radio_tx_power_conn_start_ucfg(uint8_t conn_idx);
void LLC_pcl_set_radio_tx_power_conn_start_ucfg_weak(uint8_t conn_idx);
void LLC_pcl_set_radio_tx_power_conn_start(uint8_t conn_idx);

uint8_t LLC_pcl_get_radio_tx_power_ucfg(uint8_t conn_idx,
                                        uint8_t phy_idx);
uint8_t LLC_pcl_get_radio_tx_power_ucfg_weak(uint8_t conn_idx,
                                             uint8_t phy_idx);
uint8_t LLC_pcl_get_radio_tx_power(uint8_t conn_idx,
                                   uint8_t phy_idx);

void LLC_pcl_set_radio_tx_power_and_eval_indication_ucfg(uint8_t conn_idx,
                                                         BOOL send_ind,
                                                         int8_t ind_delta,
                                                         BOOL change_txpower);
void LLC_pcl_set_radio_tx_power_and_eval_indication_ucfg_weak(uint8_t conn_idx,
                                                              BOOL send_ind,
                                                              int8_t ind_delta,
                                                              BOOL change_txpower);
void LLC_pcl_set_radio_tx_power_and_eval_indication(uint8_t conn_idx,
                                                    BOOL send_ind,
                                                    int8_t ind_delta,
                                                    BOOL change_txpower);

BOOL LLC_pcl_offline_processing_ucfg(uint8_t conn_idx);
BOOL LLC_pcl_offline_processing_ucfg_weak(uint8_t conn_idx);
BOOL LLC_pcl_offline_processing(uint8_t conn_idx);

tBleStatus LL_Read_RSSI_ucfg(int8_t* rssiVal,
                             uint16_t connHandle);
tBleStatus LL_Read_RSSI_ucfg_weak(int8_t* rssiVal,
                                  uint16_t connHandle);
tBleStatus LL_Read_RSSI(int8_t* rssiVal,
                        uint16_t connHandle);

uint8_t LLC_pcl_get_number_of_phys_ucfg(void);
uint8_t LLC_pcl_get_number_of_phys_ucfg_weak(void);
uint8_t LLC_pcl_get_number_of_phys(void);

void LL_phy_upd_compute_data_PDU_length_params_ucfg(void* params);
void LL_phy_upd_compute_data_PDU_length_params_ucfg_weak(void* params);
void LL_phy_upd_compute_data_PDU_length_params(void* params);

BOOL LL_phy_upd_pending_ucfg(uint8_t conn_idx);
BOOL LL_phy_upd_pending_ucfg_weak(uint8_t conn_idx);
BOOL LL_phy_upd_pending(uint8_t conn_idx);

tBleStatus LL_phy_update_init_ucfg(void);
tBleStatus LL_phy_update_init_ucfg_weak(void);
tBleStatus LL_phy_update_init(void);

tBleStatus LL_phy_update_init_per_st_data_ucfg(uint8_t conn_idx);
tBleStatus LL_phy_update_init_per_st_data_ucfg_weak(uint8_t conn_idx);
tBleStatus LL_phy_update_init_per_st_data(uint8_t conn_idx);

void LL_phy_upd_evt_pending_check_isr_ucfg(uint8_t conn_idx);
void LL_phy_upd_evt_pending_check_isr_ucfg_weak(uint8_t conn_idx);
void LL_phy_upd_evt_pending_check_isr(uint8_t conn_idx);

void LLC_blueapi_cte_ucfg(void* params);
void LLC_blueapi_cte_ucfg_weak(void* params);
void LLC_blueapi_cte(void* params);

void LLC_connless_process_rx_cte_ucfg(BOOL iq_samples_ready,
                                      void* params,
                                      uint8_t iq_samples_number,
                                      uint8_t rx_cte_type,
                                      uint8_t channel);
void LLC_connless_process_rx_cte_ucfg_weak(BOOL iq_samples_ready,
                                           void* params,
                                           uint8_t iq_samples_number,
                                           uint8_t rx_cte_type,
                                           uint8_t channel);
void LLC_connless_process_rx_cte(BOOL iq_samples_ready,
                                 void* params,
                                 uint8_t iq_samples_number,
                                 uint8_t rx_cte_type,
                                 uint8_t channel);

BOOL LLC_check_iq_samples_ready_ucfg(uint8_t* iq_samples_number,
                                     uint8_t* channel,
                                     uint8_t taskslot_no);
BOOL LLC_check_iq_samples_ready_ucfg_weak(uint8_t* iq_samples_number,
                                          uint8_t* channel,
                                          uint8_t taskslot_no);
BOOL LLC_check_iq_samples_ready(uint8_t* iq_samples_number,
                                uint8_t* channel,
                                uint8_t taskslot_no);

void llc_cte_init_ucfg(void);
void llc_cte_init_ucfg_weak(void);
void llc_cte_init(void);

void llc_cte_mem_allocate_ucfg(uint8_t cte_en,
                               uint8_t master_en,
                               uint8_t ext_en,
                               uint8_t periodic_en);
void llc_cte_mem_allocate_ucfg_weak(uint8_t cte_en,
                                    uint8_t master_en,
                                    uint8_t ext_en,
                                    uint8_t periodic_en);
void llc_cte_mem_allocate(uint8_t cte_en,
                          uint8_t master_en,
                          uint8_t ext_en,
                          uint8_t periodic_en);

void llc_cte_timer_error_ucfg(void);
void llc_cte_timer_error_ucfg_weak(void);
void llc_cte_timer_error(void);

void llc_cte_timer_start_ucfg(void* params);
void llc_cte_timer_start_ucfg_weak(void* params);
void llc_cte_timer_start(void* params);

BOOL llc_cte_timer_stop_ucfg(void);
BOOL llc_cte_timer_stop_ucfg_weak(void);
BOOL llc_cte_timer_stop(void);

void LLC_test_process_rx_cte_ucfg(void* params);
void LLC_test_process_rx_cte_ucfg_weak(void* params);
void LLC_test_process_rx_cte(void* params);

void LLC_test_set_cte_ucfg(void* params);
void LLC_test_set_cte_ucfg_weak(void* params);
void LLC_test_set_cte(void* params);

void ADV_ISR_connect_request_received_ucfg(void* pointer,
                                           uint8_t* packet,
                                           void* PeerIDAddress_p);
void ADV_ISR_connect_request_received_ucfg_weak(void* pointer,
                                                uint8_t* packet,
                                                void* PeerIDAddress_p);
void ADV_ISR_connect_request_received(void* pointer,
                                      uint8_t* packet,
                                      void* PeerIDAddress_p);

void LL_eadv_EauxIsr_connect_response_sent_ucfg(void* pointer);
void LL_eadv_EauxIsr_connect_response_sent_ucfg_weak(void* pointer);
void LL_eadv_EauxIsr_connect_response_sent(void* pointer);

void LL_eadv_EauxIsr_connect_request_received_ucfg(void* pointer,
                                                   BOOL* calibration_required);
void LL_eadv_EauxIsr_connect_request_received_ucfg_weak(void* pointer,
                                                        BOOL* calibration_required);
void LL_eadv_EauxIsr_connect_request_received(void* pointer,
                                              BOOL* calibration_required);

uint8_t EADV_start_request_radio_tasks_ucfg(void* pointer);
uint8_t EADV_start_request_radio_tasks_ucfg_weak(void* pointer);
uint8_t EADV_start_request_radio_tasks(void* pointer);

tBleStatus LL_eadv_max_supported_data_check_ucfg(uint16_t Data_Length,
                                                 void* linkpointer);
tBleStatus LL_eadv_max_supported_data_check_ucfg_weak(uint16_t Data_Length,
                                                      void* linkpointer);
tBleStatus LL_eadv_max_supported_data_check(uint16_t Data_Length,
                                            void* linkpointer);

void LL_eadv_start_extended_ucfg(void* pointer);
void LL_eadv_start_extended_ucfg_weak(void* pointer);
void LL_eadv_start_extended(void* pointer);

uint8_t ext_adv_scan_enabled_ucfg(void);
uint8_t ext_adv_scan_enabled_ucfg_weak(void);
uint8_t ext_adv_scan_enabled(void);

tBleStatus LL_Clear_Advertising_Sets_ucfg(void);
tBleStatus LL_Clear_Advertising_Sets_ucfg_weak(void);
tBleStatus LL_Clear_Advertising_Sets(void);

tBleStatus LL_Remove_Advertising_Set_ucfg(uint16_t Advertising_Handle);
tBleStatus LL_Remove_Advertising_Set_ucfg_weak(uint16_t Advertising_Handle);
tBleStatus LL_Remove_Advertising_Set(uint16_t Advertising_Handle);

BOOL LL_extended_adv_add_enable_sync_info_auxind_pdu_ucfg(void* pointer);
BOOL LL_extended_adv_add_enable_sync_info_auxind_pdu_ucfg_weak(void* pointer);
BOOL LL_extended_adv_add_enable_sync_info_auxind_pdu(void* pointer);

void LL_periodic_event_terminate_decision_routine_ucfg(void* pointer);
void LL_periodic_event_terminate_decision_routine_ucfg_weak(void* pointer);
void LL_periodic_event_terminate_decision_routine(void* pointer);

void LL_periodicadv_init_ucfg(uint8_t number_of_periodicadv_contexts,
                              void* LL_periodicadv_context_periodic_per_array);
void LL_periodicadv_init_ucfg_weak(uint8_t number_of_periodicadv_contexts,
                                   void* LL_periodicadv_context_periodic_per_array);
void LL_periodicadv_init(uint8_t number_of_periodicadv_contexts,
                         void* LL_periodicadv_context_periodic_per_array);

void LL_periodicadv_periodicenable_ucfg(void* periodic_per_pointer);
void LL_periodicadv_periodicenable_ucfg_weak(void* periodic_per_pointer);
void LL_periodicadv_periodicenable(void* periodic_per_pointer);

uint8_t periodic_csr_ucfg(void);
uint8_t periodic_csr_ucfg_weak(void);
uint8_t periodic_csr(void);

tBleStatus LL_LE_Set_periodic_adv_data_ptr_function_ucfg(uint16_t advertising_handle,
                                                         uint16_t data_length,
                                                         uint8_t* data);
tBleStatus LL_LE_Set_periodic_adv_data_ptr_function_ucfg_weak(uint16_t advertising_handle,
                                                              uint16_t data_length,
                                                              uint8_t* data);
tBleStatus LL_LE_Set_periodic_adv_data_ptr_function(uint16_t advertising_handle,
                                                    uint16_t data_length,
                                                    uint8_t* data);

BOOL LL_periodicadv_update_sync_info_field_ucfg(void* periodic_context_ptr,
                                                uint8_t* packet,
                                                uint32_t myanchor);
BOOL LL_periodicadv_update_sync_info_field_ucfg_weak(void* periodic_context_ptr,
                                                     uint8_t* packet,
                                                     uint32_t myanchor);
BOOL LL_periodicadv_update_sync_info_field(void* periodic_context_ptr,
                                           uint8_t* packet,
                                           uint32_t myanchor);

uint8_t LL_periodicadv_prepare_periodic_advertising_payload_ucfg(void* pointer,
                                                                 uint8_t packet_type,
                                                                 uint8_t extended_flags_and_data_indication,
                                                                 uint8_t* packet);
uint8_t LL_periodicadv_prepare_periodic_advertising_payload_ucfg_weak(void* pointer,
                                                                      uint8_t packet_type,
                                                                      uint8_t extended_flags_and_data_indication,
                                                                      uint8_t* packet);
uint8_t LL_periodicadv_prepare_periodic_advertising_payload(void* pointer,
                                                            uint8_t packet_type,
                                                            uint8_t extended_flags_and_data_indication,
                                                            uint8_t* packet);

void LL_periodicadv_init_periodic_advertising_context_per_ucfg(void* periodic_per_pointer_dummy);
void LL_periodicadv_init_periodic_advertising_context_per_ucfg_weak(void* periodic_per_pointer_dummy);
void LL_periodicadv_init_periodic_advertising_context_per(void* periodic_per_pointer_dummy);

void LL_padv_adjust_cte_count_with_interval_ucfg(void* pointer,
                                                 BOOL no_data_length_set,
                                                 uint16_t data_length);
void LL_padv_adjust_cte_count_with_interval_ucfg_weak(void* pointer,
                                                      BOOL no_data_length_set,
                                                      uint16_t data_length);
void LL_padv_adjust_cte_count_with_interval(void* pointer,
                                            BOOL no_data_length_set,
                                            uint16_t data_length);

BOOL LL_periodicscan_isr_ucfg(void* pointer);
BOOL LL_periodicscan_isr_ucfg_weak(void* pointer);
BOOL LL_periodicscan_isr(void* pointer);

void LL_pscan_clr_reject_flags_in_per_adv_list_ucfg(void* pointer);
void LL_pscan_clr_reject_flags_in_per_adv_list_ucfg_weak(void* pointer);
void LL_pscan_clr_reject_flags_in_per_adv_list(void* pointer);

void LL_periodicscan_deploy_or_connect_periodic_ucfg(void* sync_info_field,
                                                     uint32_t* address_with_SID,
                                                     uint8_t phy,
                                                     uint8_t tx_power,
                                                     BOOL past_mode,
                                                     uint32_t connect_anchor,
                                                     uint8_t conn_idx,
                                                     void* periodicperpointer_dummy,
                                                     uint32_t* privateaddress);
void LL_periodicscan_deploy_or_connect_periodic_ucfg_weak(void* sync_info_field,
                                                          uint32_t* address_with_SID,
                                                          uint8_t phy,
                                                          uint8_t tx_power,
                                                          BOOL past_mode,
                                                          uint32_t connect_anchor,
                                                          uint8_t conn_idx,
                                                          void* periodicperpointer_dummy,
                                                          uint32_t* privateaddress);
void LL_periodicscan_deploy_or_connect_periodic(void* sync_info_field,
                                                uint32_t* address_with_SID,
                                                uint8_t phy,
                                                uint8_t tx_power,
                                                BOOL past_mode,
                                                uint32_t connect_anchor,
                                                uint8_t conn_idx,
                                                void* periodicperpointer_dummy,
                                                uint32_t* privateaddress);

BOOL LL_periodicscan_check_received_txaddress_true_on_accept_ucfg(uint32_t* received_transmit_address);
BOOL LL_periodicscan_check_received_txaddress_true_on_accept_ucfg_weak(uint32_t* received_transmit_address);
BOOL LL_periodicscan_check_received_txaddress_true_on_accept(uint32_t* received_transmit_address);

void LL_periodicscan_init_ucfg(uint8_t number_of_periodic_contexts,
                               void* periodic_context_complete_p,
                               void* pointer,
                               uint8_t PeriodicListSizeLog2,
                               void* pa_list_p,
                               void* cpf_past_procedure_overall_context,
                               uint8_t num_connect_link,
                               void* cpf_past_procedure_connect_context_per);
void LL_periodicscan_init_ucfg_weak(uint8_t number_of_periodic_contexts,
                                    void* periodic_context_complete_p,
                                    void* pointer,
                                    uint8_t PeriodicListSizeLog2,
                                    void* pa_list_p,
                                    void* cpf_past_procedure_overall_context,
                                    uint8_t num_connect_link,
                                    void* cpf_past_procedure_connect_context_per);
void LL_periodicscan_init(uint8_t number_of_periodic_contexts,
                          void* periodic_context_complete_p,
                          void* pointer,
                          uint8_t PeriodicListSizeLog2,
                          void* pa_list_p,
                          void* cpf_past_procedure_overall_context,
                          uint8_t num_connect_link,
                          void* cpf_past_procedure_connect_context_per);

BOOL LL_pscan_validate_CTE_type_ucfg(void* params,
                                     uint8_t CTE_type);
BOOL LL_pscan_validate_CTE_type_ucfg_weak(void* params,
                                          uint8_t CTE_type);
BOOL LL_pscan_validate_CTE_type(void* params,
                                uint8_t CTE_type);

BOOL LL_periodicscan_cancel_slot_cte_ucfg(void* pointer_complete,
                                          void* pointer_per);
BOOL LL_periodicscan_cancel_slot_cte_ucfg_weak(void* pointer_complete,
                                               void* pointer_per);
BOOL LL_periodicscan_cancel_slot_cte(void* pointer_complete,
                                     void* pointer_per);

BOOL Data_Len_Update_Offline_Processing_ucfg(void* params,
                                             uint32_t ctrl_flds);
BOOL Data_Len_Update_Offline_Processing_ucfg_weak(void* params,
                                                  uint32_t ctrl_flds);
BOOL Data_Len_Update_Offline_Processing(void* params,
                                        uint32_t ctrl_flds);

tBleStatus ll_write_supported_data_ucfg(uint16_t Supported_Max_Tx_Octets,
                                        uint16_t Supported_Max_Tx_Time,
                                        uint16_t Supported_Max_Rx_Octets,
                                        uint16_t Supported_Max_Rx_Time);
tBleStatus ll_write_supported_data_ucfg_weak(uint16_t Supported_Max_Tx_Octets,
                                             uint16_t Supported_Max_Tx_Time,
                                             uint16_t Supported_Max_Rx_Octets,
                                             uint16_t Supported_Max_Rx_Time);
tBleStatus ll_write_supported_data(uint16_t Supported_Max_Tx_Octets,
                                   uint16_t Supported_Max_Tx_Time,
                                   uint16_t Supported_Max_Rx_Octets,
                                   uint16_t Supported_Max_Rx_Time);

void LL_init_ucfg(uint8_t dataLenExt,
                  uint8_t PhyUpd,
                  uint8_t ExtAdvScan,
                  uint8_t CtrlPriv,
                  uint8_t MasterRole,
                  uint8_t PerAdvScan,
                  uint8_t Cte,
                  uint8_t Pcl,
                  uint8_t Cns,
                  uint8_t Chc,
                  uint8_t Iso,
                  uint8_t Bis,
                  uint8_t Cis);
void LL_init_ucfg_weak(uint8_t dataLenExt,
                       uint8_t PhyUpd,
                       uint8_t ExtAdvScan,
                       uint8_t CtrlPriv,
                       uint8_t MasterRole,
                       uint8_t PerAdvScan,
                       uint8_t Cte,
                       uint8_t Pcl,
                       uint8_t Cns,
                       uint8_t Chc,
                       uint8_t Iso,
                       uint8_t Bis,
                       uint8_t Cis);
void LL_init(uint8_t dataLenExt,
             uint8_t PhyUpd,
             uint8_t ExtAdvScan,
             uint8_t CtrlPriv,
             uint8_t MasterRole,
             uint8_t PerAdvScan,
             uint8_t Cte,
             uint8_t Pcl,
             uint8_t Cns,
             uint8_t Chc,
             uint8_t Iso,
             uint8_t Bis,
             uint8_t Cis);

tBleStatus smp_debug_trudy__set_config_ucfg(uint32_t config);
tBleStatus smp_debug_trudy__set_config_ucfg_weak(uint32_t config);
tBleStatus smp_debug_trudy__set_config(uint32_t config);

tBleStatus smp_start_encryption_ucfg(void* params);
tBleStatus smp_start_encryption_ucfg_weak(void* params);
tBleStatus smp_start_encryption(void* params);

tBleStatus smp_pp1_cei_rxp_pairing_response_excerpt_scp_ucfg(void* params);
tBleStatus smp_pp1_cei_rxp_pairing_response_excerpt_scp_ucfg_weak(void* params);
tBleStatus smp_pp1_cei_rxp_pairing_response_excerpt_scp(void* params);

void smp_pp2_cei_auth_stage1_start_excerpt_scp_ucfg(void* params);
void smp_pp2_cei_auth_stage1_start_excerpt_scp_ucfg_weak(void* params);
void smp_pp2_cei_auth_stage1_start_excerpt_scp(void* params);

void smp_pp2_per_auth_stage1_start_excerpt_scp_ucfg(void* params);
void smp_pp2_per_auth_stage1_start_excerpt_scp_ucfg_weak(void* params);
void smp_pp2_per_auth_stage1_start_excerpt_scp(void* params);

void smp_pp2_cei_send_pairing_confirm_excerpt_scp_ucfg(void* params);
void smp_pp2_cei_send_pairing_confirm_excerpt_scp_ucfg_weak(void* params);
void smp_pp2_cei_send_pairing_confirm_excerpt_scp(void* params);

tBleStatus smp_pp2_cei_rx_process_pairing_random_excerpt1_scp_ucfg(void* params);
tBleStatus smp_pp2_cei_rx_process_pairing_random_excerpt1_scp_ucfg_weak(void* params);
tBleStatus smp_pp2_cei_rx_process_pairing_random_excerpt1_scp(void* params);

void smp_pp2_cei_rx_process_pairing_random_excerpt2_scp_ucfg(void* params);
void smp_pp2_cei_rx_process_pairing_random_excerpt2_scp_ucfg_weak(void* params);
void smp_pp2_cei_rx_process_pairing_random_excerpt2_scp(void* params);

void smp_pp2_per_rx_process_pairing_confirm_excerpt_scp_ucfg(void* params);
void smp_pp2_per_rx_process_pairing_confirm_excerpt_scp_ucfg_weak(void* params);
void smp_pp2_per_rx_process_pairing_confirm_excerpt_scp(void* params);

void smp_pp2_per_rx_process_pairing_random_excerpt_scp_ucfg(void* params);
void smp_pp2_per_rx_process_pairing_random_excerpt_scp_ucfg_weak(void* params);
void smp_pp2_per_rx_process_pairing_random_excerpt_scp(void* params);

void smp_scp_init_ucfg(BOOL use_debug_key);
void smp_scp_init_ucfg_weak(BOOL use_debug_key);
void smp_scp_init(BOOL use_debug_key);

tBleStatus smp_scp_continue_on_exclusive_sc_pairing_ucfg(void* params);
tBleStatus smp_scp_continue_on_exclusive_sc_pairing_ucfg_weak(void* params);
tBleStatus smp_scp_continue_on_exclusive_sc_pairing(void* params);

tBleStatus smp_scp_public_key_prepare_and_exchange_ucfg(void* params);
tBleStatus smp_scp_public_key_prepare_and_exchange_ucfg_weak(void* params);
tBleStatus smp_scp_public_key_prepare_and_exchange(void* params);

tBleStatus smp_scp_oob_generate_new_local_data_ucfg(void);
tBleStatus smp_scp_oob_generate_new_local_data_ucfg_weak(void);
tBleStatus smp_scp_oob_generate_new_local_data(void);

void smp_scp_stats_update_on_pairing_complete_ucfg(void* params);
void smp_scp_stats_update_on_pairing_complete_ucfg_weak(void* params);
void smp_scp_stats_update_on_pairing_complete(void* params);

void smp_fsm_process_actions_wrt_state_tsk_ucfg(uint16_t task_idx);
void smp_fsm_process_actions_wrt_state_tsk_ucfg_weak(uint16_t task_idx);
void smp_fsm_process_actions_wrt_state_tsk(uint16_t task_idx);

void smp_fsm_execute_actions_scp_phase2as2_ucfg(void* params);
void smp_fsm_execute_actions_scp_phase2as2_ucfg_weak(void* params);
void smp_fsm_execute_actions_scp_phase2as2(void* params);

void smp_fsm_execute_actions_excerpt_cei_ucfg(void* params);
void smp_fsm_execute_actions_excerpt_cei_ucfg_weak(void* params);
void smp_fsm_execute_actions_excerpt_cei(void* params);

void smp_pka_process_ecc_computation_tsk_ucfg(uint16_t task_idx);
void smp_pka_process_ecc_computation_tsk_ucfg_weak(uint16_t task_idx);
void smp_pka_process_ecc_computation_tsk(uint16_t task_idx);

void smp_rxp_process_rxed_packets_tsk_ucfg(uint16_t task_idx);
void smp_rxp_process_rxed_packets_tsk_ucfg_weak(uint16_t task_idx);
void smp_rxp_process_rxed_packets_tsk(uint16_t task_idx);

tBleStatus smp_rxp_process_exception_cases_excerpt_scp_ucfg(void* params);
tBleStatus smp_rxp_process_exception_cases_excerpt_scp_ucfg_weak(void* params);
tBleStatus smp_rxp_process_exception_cases_excerpt_scp(void* params);

tBleStatus smp_rxp_process_wrt_current_fsm_excerpt_scp_ucfg(void* params);
tBleStatus smp_rxp_process_wrt_current_fsm_excerpt_scp_ucfg_weak(void* params);
tBleStatus smp_rxp_process_wrt_current_fsm_excerpt_scp(void* params);

tBleStatus smp_rxp_process_wrt_current_fsm_excerpt_cei_ucfg(void* params);
tBleStatus smp_rxp_process_wrt_current_fsm_excerpt_cei_ucfg_weak(void* params);
tBleStatus smp_rxp_process_wrt_current_fsm_excerpt_cei(void* params);

void smp_init_ucfg(void);
void smp_init_ucfg_weak(void);
void smp_init(void);

void GAT_att_cmn_init_ucfg(void);
void GAT_att_cmn_init_ucfg_weak(void);
void GAT_att_cmn_init(void);

void GAT_att_cmn_timer_expire_tsk_ucfg(uint16_t task_idx);
void GAT_att_cmn_timer_expire_tsk_ucfg_weak(uint16_t task_idx);
void GAT_att_cmn_timer_expire_tsk(uint16_t task_idx);

void GAT_att_cmn_tx_pool_evt_tsk_ucfg(uint16_t task_idx);
void GAT_att_cmn_tx_pool_evt_tsk_ucfg_weak(uint16_t task_idx);
void GAT_att_cmn_tx_pool_evt_tsk(uint16_t task_idx);

void GAT_att_srv_process_rx_pckt_tsk_ucfg(uint16_t task_idx);
void GAT_att_srv_process_rx_pckt_tsk_ucfg_weak(uint16_t task_idx);
void GAT_att_srv_process_rx_pckt_tsk(uint16_t task_idx);

void GAT_srv_db_hash_tsk_ucfg(uint16_t task_idx);
void GAT_srv_db_hash_tsk_ucfg_weak(uint16_t task_idx);
void GAT_srv_db_hash_tsk(uint16_t task_idx);

void GAT_srv_send_srv_change_tsk_ucfg(uint16_t task_idx);
void GAT_srv_send_srv_change_tsk_ucfg_weak(uint16_t task_idx);
void GAT_srv_send_srv_change_tsk(uint16_t task_idx);

tBleStatus aci_gap_slave_security_req_api(uint16_t Connection_Handle);

tBleStatus aci_gap_set_io_capability_api(uint8_t IO_Capability);

tBleStatus aci_gap_set_authentication_requirement_api(uint8_t Bonding_Mode,
                                                      uint8_t MITM_Mode,
                                                      uint8_t SC_Support,
                                                      uint8_t KeyPress_Notification_Support,
                                                      uint8_t Min_Encryption_Key_Size,
                                                      uint8_t Max_Encryption_Key_Size,
                                                      uint8_t Use_Fixed_Pin,
                                                      uint32_t Fixed_Pin);

tBleStatus aci_gap_pass_key_resp_api(uint16_t Connection_Handle,
                                     uint32_t Pass_Key);

tBleStatus aci_gap_get_security_level_api(uint16_t Connection_Handle,
                                          uint8_t* Security_Mode,
                                          uint8_t* Security_Level);

tBleStatus aci_gap_terminate_api(uint16_t Connection_Handle,
                                 uint8_t Reason);

tBleStatus aci_gap_clear_security_db_api(void);

tBleStatus aci_gap_remove_bonded_device_api(uint8_t peerIdentityAddressType,
                                            uint8_t peerIdentityDeviceAddress[6]);

tBleStatus aci_gap_allow_rebond_api(uint16_t Connection_Handle);

tBleStatus aci_gap_start_connection_update_api(uint16_t Connection_Handle,
                                               uint16_t Conn_Interval_Min,
                                               uint16_t Conn_Interval_Max,
                                               uint16_t Conn_Latency,
                                               uint16_t Supervision_Timeout,
                                               uint16_t Minimum_CE_Length,
                                               uint16_t Maximum_CE_Length);

tBleStatus aci_gap_send_pairing_req_api(uint16_t Connection_Handle,
                                        uint8_t Force_Rebond);

tBleStatus aci_gap_get_bonded_devices_api(uint8_t Offset,
                                          uint8_t Max_Num_Of_Addresses,
                                          uint8_t* Num_of_Addresses,
                                          Bonded_Device_Entry_t* Bonded_Device_Entry);

tBleStatus aci_gap_is_device_bonded_api(uint8_t Peer_Address_Type,
                                        uint8_t Peer_Address[6]);

tBleStatus aci_gap_numeric_comparison_value_confirm_yesno_api(uint16_t Connection_Handle,
                                                              uint8_t Confirm_Yes_No);

tBleStatus aci_gap_passkey_input_api(uint16_t Connection_Handle,
                                     uint8_t Input_Type);

tBleStatus aci_gap_get_oob_data_api(uint8_t OOB_Data_Type,
                                    uint8_t* Address_Type,
                                    uint8_t Address[6],
                                    uint8_t* OOB_Data_Len,
                                    uint8_t OOB_Data[16]);

tBleStatus aci_gap_set_oob_data_api(uint8_t Device_Type,
                                    uint8_t Address_Type,
                                    uint8_t Address[6],
                                    uint8_t OOB_Data_Type,
                                    uint8_t OOB_Data_Len,
                                    uint8_t OOB_Data[16]);

tBleStatus aci_gap_set_scan_configuration_api(uint8_t duplicate_filtering,
                                              uint8_t scanning_filter_policy,
                                              uint8_t phy,
                                              uint8_t scan_type,
                                              uint16_t scan_interval,
                                              uint16_t scan_window);

tBleStatus aci_gap_set_connection_configuration_api(uint8_t phy,
                                                    uint16_t conn_interval_min,
                                                    uint16_t conn_interval_max,
                                                    uint16_t conn_latency,
                                                    uint16_t supervision_timeout,
                                                    uint16_t minimum_ce_length,
                                                    uint16_t maximum_ce_length);

tBleStatus aci_gap_create_connection_api(uint8_t Initiating_PHY,
                                         uint8_t Peer_Address_Type,
                                         uint8_t Peer_Address[6]);

tBleStatus aci_gap_start_procedure_api(uint8_t procedure_code,
                                       uint8_t phys,
                                       uint16_t duration,
                                       uint16_t period);

tBleStatus aci_gap_discover_name_api(uint8_t PHYs,
                                     uint8_t Peer_Address_Type,
                                     uint8_t Peer_Address[6]);

tBleStatus aci_gap_terminate_proc_api(uint8_t Procedure_Code);

tBleStatus aci_gap_periodic_advertising_create_sync_api(uint8_t Options,
                                                        uint8_t Advertising_SID,
                                                        uint8_t Advertising_Address_Type,
                                                        uint8_t Advertiser_Address[6],
                                                        uint16_t Skip,
                                                        uint16_t Sync_Timeout,
                                                        uint8_t Sync_CTE_Type);

tBleStatus aci_gap_periodic_advertising_create_sync_cancel_api(void);

tBleStatus aci_gap_periodic_advertising_terminate_sync_api(uint16_t Sync_Handle);

tBleStatus aci_gap_set_periodic_advertising_receive_enable_api(uint16_t Sync_Handle,
                                                               uint8_t Enable);

tBleStatus aci_gap_add_device_to_periodic_advertiser_list_api(uint8_t Advertiser_Address_Type,
                                                              uint8_t Advertiser_Address[6],
                                                              uint8_t Advertising_SID);

tBleStatus aci_gap_remove_device_from_periodic_advertising_list_api(uint8_t Advertiser_Address_Type,
                                                                    uint8_t Advertiser_Address[6],
                                                                    uint8_t Advertising_SID);

tBleStatus aci_gap_clear_periodic_advertiser_list_api(void);

tBleStatus aci_gap_read_periodic_advertiser_list_size_api(uint8_t* Periodic_Advertiser_List_Size);

tBleStatus aci_gap_periodic_advertising_set_info_transfer_api(uint16_t Connection_Handle,
                                                              uint16_t Service_Data,
                                                              uint8_t Advertising_Handle);

tBleStatus aci_gap_periodic_advertising_sync_transfer_api(uint16_t Connection_Handle,
                                                          uint16_t Service_Data,
                                                          uint16_t Sync_Handle);

tBleStatus aci_gap_set_periodic_advertising_sync_transfer_parameters_api(uint16_t Connection_Handle,
                                                                         uint8_t Mode,
                                                                         uint16_t Skip,
                                                                         uint16_t Sync_Timeout,
                                                                         uint8_t CTE_Type);

tBleStatus aci_gap_set_default_periodic_advertising_sync_transfer_parameters_api(uint8_t Mode,
                                                                                 uint16_t Skip,
                                                                                 uint16_t Sync_Timeout,
                                                                                 uint8_t CTE_Type);

tBleStatus aci_gap_set_periodic_advertising_enable_api(uint8_t Enable,
                                                       uint8_t Advertising_Handle);

tBleStatus aci_gap_set_periodic_advertising_data_api(uint8_t Advertising_Handle,
                                                     uint16_t Advertising_Data_Length,
                                                     uint8_t* Advertising_Data);

tBleStatus aci_gap_set_periodic_advertising_configuration_api(uint8_t Advertising_Handle,
                                                              uint16_t Periodic_Advertising_Interval_Min,
                                                              uint16_t Periodic_Advertising_Interval_Max,
                                                              uint16_t Periodic_Advertising_Properties);

tBleStatus aci_gap_remove_advertising_set_api(uint8_t Advertising_Handle);

tBleStatus aci_gap_clear_advertising_sets_api(void);

tBleStatus aci_gatt_srv_init_api(void);

tBleStatus aci_gatt_srv_add_service_api(ble_gatt_srv_def_t* Serv_p);

tBleStatus aci_gatt_srv_rm_service_api(uint16_t Serv_Attr_H);

uint16_t aci_gatt_srv_get_service_handle_api(ble_gatt_srv_def_t* Serv_p);

tBleStatus aci_gatt_srv_include_service_api(uint16_t Serv_Attr_H,
                                            uint16_t Incl_Serv_Attr_H);

tBleStatus aci_gatt_srv_rm_include_service_api(uint16_t Incl_Serv_Attr_H);

uint16_t aci_gatt_srv_get_include_service_handle_api(uint16_t Serv_Attr_H,
                                                     ble_gatt_srv_def_t* Included_Srv_p);

tBleStatus aci_gatt_srv_add_char_api(ble_gatt_chr_def_t* Char_p,
                                     uint16_t Serv_Attr_H);

tBleStatus aci_gatt_srv_rm_char_api(uint16_t Char_Decl_Attr_H);

uint16_t aci_gatt_srv_get_char_decl_handle_api(ble_gatt_chr_def_t* Char_p);

tBleStatus aci_gatt_srv_add_char_desc_api(ble_gatt_descr_def_t* Descr_p,
                                          uint16_t Char_Attr_H);

uint16_t aci_gatt_srv_get_descriptor_handle_api(ble_gatt_descr_def_t* Descr_p);

tBleStatus aci_gatt_srv_notify_api(uint16_t Connection_Handle,
                                   uint16_t Attr_Handle,
                                   uint8_t Flags,
                                   uint16_t Val_Length,
                                   uint8_t* Val_p);

tBleStatus aci_gatt_srv_multi_notify_api(uint16_t Connection_Handle,
                                         uint8_t Flags,
                                         uint8_t Num_Of_Attr,
                                         Gatt_Srv_Notify_Attr_t* Gatt_Srv_Notify_Attr);

tBleStatus aci_gatt_srv_resp_api(uint16_t Connection_Handle,
                                 uint16_t Attr_Handle,
                                 uint8_t Error_Code,
                                 uint16_t Data_Len,
                                 uint8_t* Data_p);

tBleStatus aci_gatt_srv_read_handle_value_api(uint16_t Attr_Handle,
                                              uint16_t* Val_Length_p,
                                              uint8_t** Val_pp);

tBleStatus aci_gatt_srv_read_multiple_instance_handle_value_api(uint16_t Connection_Handle,
                                                                uint16_t Attr_Handle,
                                                                uint16_t* Val_Length_p,
                                                                uint8_t** Val_pp);

tBleStatus aci_gatt_srv_write_multiple_instance_handle_value_api(uint16_t Connection_Handle,
                                                                 uint16_t Attr_Handle,
                                                                 uint16_t Char_Value_Length,
                                                                 uint8_t* Char_Value);

tBleStatus aci_gatt_set_event_mask_api(uint32_t GATT_Evt_Mask);

tBleStatus aci_gatt_clt_exchange_config_api(uint16_t Connection_Handle);

tBleStatus aci_gatt_clt_disc_all_primary_services_api(uint16_t Connection_Handle);

tBleStatus aci_gatt_clt_disc_primary_service_by_uuid_api(uint16_t Connection_Handle,
                                                         uint8_t UUID_Type,
                                                         UUID_t* UUID);

tBleStatus aci_gatt_clt_disc_all_char_of_service_api(uint16_t Connection_Handle,
                                                     uint16_t Start_Handle,
                                                     uint16_t End_Handle);

tBleStatus aci_gatt_clt_disc_char_by_uuid_api(uint16_t Connection_Handle,
                                              uint16_t Start_Handle,
                                              uint16_t End_Handle,
                                              uint8_t UUID_Type,
                                              UUID_t* UUID);

tBleStatus aci_gatt_clt_disc_all_char_desc_api(uint16_t Connection_Handle,
                                               uint16_t Char_Handle,
                                               uint16_t End_Handle);

tBleStatus aci_gatt_clt_find_included_services_api(uint16_t Connection_Handle,
                                                   uint16_t Start_Handle,
                                                   uint16_t End_Handle);

tBleStatus aci_gatt_clt_read_api(uint16_t Connection_Handle,
                                 uint16_t Attr_Handle);

tBleStatus aci_gatt_clt_read_long_api(uint16_t Connection_Handle,
                                      uint16_t Attr_Handle,
                                      uint16_t Val_Offset);

tBleStatus aci_gatt_clt_read_using_char_uuid_api(uint16_t Connection_Handle,
                                                 uint16_t Start_Handle,
                                                 uint16_t End_Handle,
                                                 uint8_t UUID_Type,
                                                 UUID_t* UUID);

tBleStatus aci_gatt_clt_read_multiple_char_value_api(uint16_t Connection_Handle,
                                                     uint8_t Number_of_Handles,
                                                     Handle_Entry_t* Handle_Entry);

tBleStatus aci_gatt_clt_read_multiple_var_len_char_value_api(uint16_t Connection_Handle,
                                                             uint8_t Number_of_Handles,
                                                             Handle_Entry_t* Handle_Entry);

tBleStatus aci_gatt_clt_write_without_resp_api(uint16_t Connection_Handle,
                                               uint16_t Attr_Handle,
                                               uint16_t Attribute_Val_Length,
                                               uint8_t* Attribute_Val);

tBleStatus aci_gatt_clt_signed_write_without_resp_api(uint16_t Connection_Handle,
                                                      uint16_t Attr_Handle,
                                                      uint16_t Attribute_Val_Length,
                                                      uint8_t* Attribute_Val);

tBleStatus aci_gatt_clt_write_api(uint16_t Connection_Handle,
                                  uint16_t Attr_Handle,
                                  uint16_t Attribute_Val_Length,
                                  uint8_t* Attribute_Val);

tBleStatus aci_gatt_clt_write_long_api(uint16_t Connection_Handle,
                                       ble_gatt_clt_write_ops_t* Write_Ops_p);

tBleStatus aci_gatt_clt_write_char_reliable_api(uint16_t Connection_Handle,
                                                uint8_t Num_Attrs,
                                                ble_gatt_clt_write_ops_t* Write_Ops_p);

tBleStatus aci_gatt_clt_prepare_write_req_api(uint16_t Connection_Handle,
                                              uint16_t Attr_Handle,
                                              uint16_t Val_Offset,
                                              uint16_t Attribute_Val_Length,
                                              uint8_t* Attribute_Val);

tBleStatus aci_gatt_clt_execute_write_req_api(uint16_t Connection_Handle,
                                              uint8_t Execute);

tBleStatus aci_gatt_clt_confirm_indication_api(uint16_t Connection_Handle);

tBleStatus aci_gatt_eatt_srv_init_api(void);

tBleStatus aci_gatt_eatt_srv_notify_api(uint16_t Connection_Handle,
                                        uint16_t CID,
                                        uint16_t Attr_Handle,
                                        uint8_t Flags,
                                        uint16_t Val_Length,
                                        uint8_t* Val);

tBleStatus aci_gatt_eatt_srv_multi_notify_api(uint16_t Connection_Handle,
                                              uint16_t CID,
                                              uint8_t Flags,
                                              uint8_t Num_Of_Attr,
                                              Gatt_Srv_Notify_Attr_t* Gatt_Srv_Notify_Attr);

tBleStatus aci_gatt_eatt_srv_resp_api(uint16_t Connection_Handle,
                                      uint16_t CID,
                                      uint16_t Attr_Handle,
                                      uint8_t Error_Code,
                                      uint16_t Data_Len,
                                      uint8_t* Data_p);

tBleStatus aci_gatt_eatt_clt_disc_all_primary_services_api(uint16_t Connection_Handle,
                                                           uint16_t CID);

tBleStatus aci_gatt_eatt_clt_disc_primary_service_by_uuid_api(uint16_t Connection_Handle,
                                                              uint16_t CID,
                                                              uint8_t UUID_Type,
                                                              UUID_t* UUID);

tBleStatus aci_gatt_eatt_clt_disc_all_char_of_service_api(uint16_t Connection_Handle,
                                                          uint16_t CID,
                                                          uint16_t Start_Handle,
                                                          uint16_t End_Handle);

tBleStatus aci_gatt_eatt_clt_disc_char_by_uuid_api(uint16_t Connection_Handle,
                                                   uint16_t CID,
                                                   uint16_t Start_Handle,
                                                   uint16_t End_Handle,
                                                   uint8_t UUID_Type,
                                                   UUID_t* UUID);

tBleStatus aci_gatt_eatt_clt_disc_all_char_desc_api(uint16_t Connection_Handle,
                                                    uint16_t CID,
                                                    uint16_t Char_Handle,
                                                    uint16_t End_Handle);

tBleStatus aci_gatt_eatt_clt_find_included_services_api(uint16_t Connection_Handle,
                                                        uint16_t CID,
                                                        uint16_t Start_Handle,
                                                        uint16_t End_Handle);

tBleStatus aci_gatt_eatt_clt_read_api(uint16_t Connection_Handle,
                                      uint16_t CID,
                                      uint16_t Attr_Handle);

tBleStatus aci_gatt_eatt_clt_read_long_api(uint16_t Connection_Handle,
                                           uint16_t CID,
                                           uint16_t Attr_Handle,
                                           uint16_t Val_Offset);

tBleStatus aci_gatt_eatt_clt_read_using_char_uuid_api(uint16_t Connection_Handle,
                                                      uint16_t CID,
                                                      uint16_t Start_Handle,
                                                      uint16_t End_Handle,
                                                      uint8_t UUID_Type,
                                                      UUID_t* UUID);

tBleStatus aci_gatt_eatt_clt_read_multiple_char_value_api(uint16_t Connection_Handle,
                                                          uint16_t CID,
                                                          uint8_t Number_of_Handles,
                                                          Handle_Entry_t* Handle_Entry);

tBleStatus aci_gatt_eatt_clt_read_multiple_var_len_char_value_api(uint16_t Connection_Handle,
                                                                  uint16_t CID,
                                                                  uint8_t Number_of_Handles,
                                                                  Handle_Entry_t* Handle_Entry);

tBleStatus aci_gatt_eatt_clt_write_without_resp_api(uint16_t Connection_Handle,
                                                    uint16_t CID,
                                                    uint16_t Attr_Handle,
                                                    uint16_t Attribute_Val_Length,
                                                    uint8_t* Attribute_Val);

tBleStatus aci_gatt_eatt_clt_write_api(uint16_t Connection_Handle,
                                       uint16_t CID,
                                       uint16_t Attr_Handle,
                                       uint16_t Attribute_Val_Length,
                                       uint8_t* Attribute_Val);

tBleStatus aci_gatt_eatt_clt_write_long_api(uint16_t Connection_Handle,
                                            uint16_t CID,
                                            ble_gatt_clt_write_ops_t* Write_Ops_p);

tBleStatus aci_gatt_eatt_clt_write_char_reliable_api(uint16_t Connection_Handle,
                                                     uint16_t CID,
                                                     uint8_t Num_Attrs,
                                                     ble_gatt_clt_write_ops_t* Write_Ops_p);

tBleStatus aci_gatt_eatt_clt_prepare_write_req_api(uint16_t Connection_Handle,
                                                   uint16_t CID,
                                                   uint16_t Attr_Handle,
                                                   uint16_t Val_Offset,
                                                   uint16_t Attribute_Val_Length,
                                                   uint8_t* Attribute_Val);

tBleStatus aci_gatt_eatt_clt_execute_write_req_api(uint16_t Connection_Handle,
                                                   uint16_t CID,
                                                   uint8_t Execute);

tBleStatus aci_gatt_eatt_clt_confirm_indication_api(uint16_t Connection_Handle,
                                                    uint16_t CID);

tBleStatus aci_hal_peripheral_latency_enable_api(uint16_t Connection_Handle,
                                                 uint8_t Enable);

tBleStatus aci_hal_set_le_power_control_api(uint8_t Enable,
                                            uint8_t PHY,
                                            int8_t RSSI_Target,
                                            uint8_t RSSI_Hysteresis,
                                            int8_t Initial_TX_Power,
                                            uint8_t RSSI_Filtering_Coefficient);

tBleStatus aci_hal_get_anchor_point_api(uint16_t connection_handle,
                                        uint16_t* event_counter,
                                        uint32_t* anchor_point);

tBleStatus aci_l2cap_connection_parameter_update_req_api(uint16_t Connection_Handle,
                                                         uint16_t Conn_Interval_Min,
                                                         uint16_t Conn_Interval_Max,
                                                         uint16_t Slave_latency,
                                                         uint16_t Timeout_Multiplier);

tBleStatus aci_l2cap_connection_parameter_update_resp_api(uint16_t Connection_Handle,
                                                          uint16_t Conn_Interval_Min,
                                                          uint16_t Conn_Interval_Max,
                                                          uint16_t Slave_latency,
                                                          uint16_t Timeout_Multiplier,
                                                          uint16_t Minimum_CE_Length,
                                                          uint16_t Maximum_CE_Length,
                                                          uint8_t Identifier,
                                                          uint8_t Accept);

tBleStatus aci_l2cap_cfc_connection_req_api(uint16_t Connection_Handle,
                                            uint16_t SPSM,
                                            uint16_t CID,
                                            uint16_t MTU,
                                            uint16_t MPS,
                                            uint8_t CFC_Policy,
                                            uint16_t RX_SDU_Buffer_Size,
                                            void* RX_SDU_Buffer);

tBleStatus aci_l2cap_cfc_connection_resp_api(uint16_t Connection_Handle,
                                             uint8_t Identifier,
                                             uint16_t CID,
                                             uint16_t MTU,
                                             uint16_t MPS,
                                             uint16_t Result,
                                             uint8_t CFC_Policy,
                                             uint16_t RX_SDU_Buffer_Size,
                                             void* RX_SDU_Buffer);

tBleStatus aci_l2cap_send_flow_control_credits_api(uint16_t Connection_Handle,
                                                   uint16_t CID,
                                                   uint16_t RX_Credits,
                                                   uint8_t CFC_Policy,
                                                   uint16_t* RX_Credit_Balance);

tBleStatus aci_l2cap_disconnect_api(uint16_t Connection_Handle,
                                    uint16_t CID);

tBleStatus aci_l2cap_ecfc_connection_req_api(uint16_t connection_handle,
                                             uint16_t spsm,
                                             uint16_t mtu,
                                             uint16_t mps,
                                             uint8_t cid_count,
                                             cid_t* local_cid_array);

tBleStatus aci_l2cap_ecfc_connection_resp_api(uint16_t connection_handle,
                                              uint8_t identifier,
                                              uint16_t mtu,
                                              uint16_t mps,
                                              uint16_t result,
                                              uint8_t cid_count,
                                              cid_t* local_cid_array);

tBleStatus aci_l2cap_ecfc_reconfigure_req_api(uint16_t connection_handle,
                                              uint16_t mtu,
                                              uint16_t mps,
                                              uint8_t cid_count,
                                              cid_t* local_cid_array);

tBleStatus aci_l2cap_ecfc_reconfigure_resp_api(uint16_t connection_handle,
                                               uint8_t identifier,
                                               uint16_t result);

tBleStatus aci_l2cap_transmit_sdu_data_api(uint16_t Connection_Handle,
                                           uint16_t CID,
                                           uint16_t SDU_Length,
                                           uint8_t* SDU_Data);

tBleStatus aci_l2cap_extract_sdu_data_api(uint16_t Connection_Handle,
                                          uint16_t CID,
                                          uint16_t SDU_Data_Buffer_Size,
                                          void* SDU_Data_Buffer,
                                          uint16_t* SDU_Length);

tBleStatus hci_le_read_buffer_size_v2_api(uint16_t* HC_LE_ACL_Data_Packet_Length,
                                          uint8_t* HC_Total_Num_LE_ACL_Data_Packets,
                                          uint16_t* HC_LE_ISO_Data_Packet_Length,
                                          uint8_t* HC_Total_Num_LE_ISO_Data_Packets);

tBleStatus hci_le_create_big_test_api(uint8_t big_handle,
                                      uint8_t advertising_handle,
                                      uint8_t num_bis,
                                      uint8_t* sdu_interval_us,
                                      uint16_t iso_interval_1_25ms,
                                      uint8_t nse,
                                      uint16_t max_sdu,
                                      uint16_t max_pdu,
                                      uint8_t phy,
                                      uint8_t packing_interleaved,
                                      uint8_t framing,
                                      uint8_t bn,
                                      uint8_t irc,
                                      uint8_t pto,
                                      uint8_t encryption,
                                      uint8_t* broadcast_code_ext);

tBleStatus hci_le_big_create_sync_api(uint8_t BIG_Handle,
                                      uint16_t Sync_Handle,
                                      uint8_t Encryption,
                                      uint8_t* Broadcast_Code,
                                      uint8_t MSE,
                                      uint16_t BIG_Sync_Timeout,
                                      uint8_t Num_BIS,
                                      BIS_t* BIS);

tBleStatus hci_le_terminate_big_api(uint8_t Terminate_Big,
                                    uint8_t Reason);

tBleStatus hci_le_big_terminate_sync_api(uint8_t Big_handle);

tBleStatus hci_le_create_big_api(uint8_t big_handle,
                                 uint8_t advertising_handle,
                                 uint8_t num_bis,
                                 uint8_t* sdu_interval_us,
                                 uint16_t max_sdu,
                                 uint16_t Max_Transport_Latency,
                                 uint8_t RTN,
                                 uint8_t phy,
                                 uint8_t packing_interleaved,
                                 uint8_t framing,
                                 uint8_t encryption,
                                 uint8_t* broadcast_code_ext);

tBleStatus hci_read_afh_channel_assessment_mode_api(uint8_t* AFH_Channel_Assessment_Mode);

tBleStatus hci_write_afh_channel_assessment_mode_api(uint8_t AFH_Channel_Assessment_Mode);

tBleStatus hci_le_set_host_channel_classification_api(uint8_t Channel_Map[LLC_MIN_NUM_DATA_CHAN_MAP_BYTES]);

tBleStatus hci_le_set_cig_parameters_api(uint8_t CIG_ID,
                                         uint8_t SDU_Interval_C_To_P[3],
                                         uint8_t SDU_Interval_P_To_C[3],
                                         uint8_t Worst_Case_SCA,
                                         uint8_t Packing,
                                         uint8_t Framing,
                                         uint16_t Max_Transport_Latency_C_To_P,
                                         uint16_t Max_Transport_Latency_P_To_C,
                                         uint8_t CIS_Count,
                                         CIS_Param_t* cis_params,
                                         CIS_Conn_Handles_t* CIS_Conn_Handles);

tBleStatus hci_le_set_cig_parameters_test_api(uint8_t CIG_ID,
                                              uint8_t SDU_Interval_C_To_P[3],
                                              uint8_t SDU_Interval_P_To_C[3],
                                              uint8_t FT_C_To_P,
                                              uint8_t FT_P_To_C,
                                              uint16_t ISO_Interval,
                                              uint8_t Worst_Case_SCA,
                                              uint8_t Packing,
                                              uint8_t Framing,
                                              uint8_t CIS_Count,
                                              CIS_Param_Test_t* CIS_Param_test,
                                              CIS_Conn_Handles_t* CIS_Conn_Handles);

tBleStatus hci_le_create_cis_api(uint8_t CIS_Count,
                                 CIS_Handles_t* create_cis_connection_params);

tBleStatus hci_le_remove_cig_api(uint8_t CIG_ID);

tBleStatus hci_le_accept_cis_request_api(uint16_t Connection_Handle);

tBleStatus hci_le_reject_cis_request_api(uint16_t Connection_Handle,
                                         uint8_t Reason);

tBleStatus hci_read_connection_accept_timeout_api(uint16_t* Connection_Accept_Timeout);

tBleStatus hci_write_connection_accept_timeout_api(uint16_t Connection_Accept_Timeout);

tBleStatus hci_le_set_host_feature_api(uint8_t Bit_Number,
                                       uint8_t Bit_Value);

tBleStatus hci_read_remote_version_information_api(uint16_t Connection_Handle);

tBleStatus hci_le_read_remote_used_features_api(uint16_t Connection_Handle);

tBleStatus hci_read_transmit_power_level_api(uint16_t Connection_Handle,
                                             uint8_t Type,
                                             int8_t* Transmit_Power_Level);

tBleStatus hci_read_rssi_api(uint16_t Connection_Handle,
                             int8_t* RSSI);

tBleStatus hci_le_read_channel_map_api(uint16_t Connection_Handle,
                                       uint8_t LE_Channel_Map[LLC_MIN_NUM_DATA_CHAN_MAP_BYTES]);

tBleStatus hci_disconnect_api(uint16_t Connection_Handle,
                              uint8_t Reason);

tBleStatus hci_le_create_connection_api(uint16_t LE_Scan_Interval,
                                        uint16_t LE_Scan_Window,
                                        uint8_t Initiator_Filter_Policy,
                                        uint8_t Peer_Address_Type,
                                        uint8_t Peer_Address[6],
                                        uint8_t Own_Address_Type,
                                        uint16_t Conn_Interval_Min,
                                        uint16_t Conn_Interval_Max,
                                        uint16_t Conn_Latency,
                                        uint16_t Supervision_Timeout,
                                        uint16_t Minimum_CE_Length,
                                        uint16_t Maximum_CE_Length);

tBleStatus hci_le_create_connection_cancel_api(void);

tBleStatus hci_le_extended_create_connection_api(uint8_t Initiating_Filter_Policy,
                                                 uint8_t Own_Address_Type,
                                                 uint8_t Peer_Address_Type,
                                                 uint8_t Peer_Address[6],
                                                 uint8_t Initiating_PHYs,
                                                 Extended_Create_Connection_Parameters_t* Extended_Create_Connection_Parameters);

tBleStatus hci_le_set_connectionless_cte_transmit_parameters_api(uint8_t Advertising_Handle,
                                                                 uint8_t CTE_Length,
                                                                 uint8_t CTE_Type,
                                                                 uint8_t CTE_Count,
                                                                 uint8_t Switching_Pattern_Length,
                                                                 uint8_t* Antenna_IDs);

tBleStatus hci_le_set_connectionless_cte_transmit_enable_api(uint8_t Advertising_Handle,
                                                             uint8_t CTE_Enable);

tBleStatus hci_le_set_connectionless_iq_sampling_enable_api(uint16_t Sync_Handle,
                                                            uint8_t Sampling_Enable,
                                                            uint8_t Slot_Durations,
                                                            uint8_t Max_Sampled_CTEs,
                                                            uint8_t Switching_Pattern_Length,
                                                            uint8_t* Antenna_IDs);

tBleStatus hci_le_set_connection_cte_receive_parameters_api(uint16_t Connection_Handle,
                                                            uint8_t Sampling_Enable,
                                                            uint8_t Slot_Durations,
                                                            uint8_t Switching_Pattern_Length,
                                                            uint8_t* Antenna_IDs);

tBleStatus hci_le_set_connection_cte_transmit_parameters_api(uint16_t Connection_Handle,
                                                             uint8_t CTE_Type,
                                                             uint8_t Switching_Pattern_Length,
                                                             uint8_t* Antenna_IDs);

tBleStatus hci_le_connection_cte_request_enable_api(uint16_t Connection_Handle,
                                                    uint8_t Enable,
                                                    uint16_t CTE_Request_Interval,
                                                    uint8_t Requested_CTE_Length,
                                                    uint8_t Requested_CTE_Type);

tBleStatus hci_le_connection_cte_response_enable_api(uint16_t Connection_Handle,
                                                     uint8_t Enable);

tBleStatus hci_le_read_antenna_information_api(uint8_t* Supported_Switching_Sampling_Rates,
                                               uint8_t* Num_Antennae,
                                               uint8_t* Max_Switching_Pattern_Length,
                                               uint8_t* Max_CTE_Length);

tBleStatus hci_le_start_encryption_api(uint16_t Connection_Handle,
                                       uint8_t Random_Number[8],
                                       uint16_t Encrypted_Diversifier,
                                       uint8_t Long_Term_Key[16]);

tBleStatus hci_le_long_term_key_request_reply_api(uint16_t Connection_Handle,
                                                  uint8_t Long_Term_Key[16]);

tBleStatus hci_le_long_term_key_requested_negative_reply_api(uint16_t Connection_Handle);

tBleStatus hci_read_authenticated_payload_timeout_api(uint16_t Connection_Handle,
                                                      uint16_t* Authenticated_Payload_Timeout);

tBleStatus hci_write_authenticated_payload_timeout_api(uint16_t Connection_Handle,
                                                       uint16_t Authenticated_Payload_Timeout);

tBleStatus hci_le_set_extended_advertising_parameters_api(uint8_t Advertising_Handle,
                                                          uint16_t Advertising_Event_Properties,
                                                          uint8_t Primary_Advertising_Interval_Min[3],
                                                          uint8_t Primary_Advertising_Interval_Max[3],
                                                          uint8_t Primary_Advertising_Channel_Map,
                                                          uint8_t Own_Address_Type,
                                                          uint8_t Peer_Address_Type,
                                                          uint8_t Peer_Address[6],
                                                          uint8_t Advertising_Filter_Policy,
                                                          int8_t Advertising_Tx_Power,
                                                          uint8_t Primary_Advertising_PHY,
                                                          uint8_t Secondary_Advertising_Max_Skip,
                                                          uint8_t Secondary_Advertising_PHY,
                                                          uint8_t Advertising_SID,
                                                          uint8_t Scan_Request_Notification_Enable,
                                                          int8_t* Selected_Tx_Power);

tBleStatus hci_le_set_advertising_set_random_address_api(uint8_t Advertising_Handle,
                                                         uint8_t Advertising_Random_Address[6]);

tBleStatus hci_le_set_extended_advertising_enable_api(uint8_t Enable,
                                                      uint8_t Number_of_Sets,
                                                      Advertising_Set_Parameters_t* Advertising_Set_Parameters);

tBleStatus hci_le_read_number_of_supported_advertising_sets_api(uint8_t* Num_Supported_Advertising_Sets);

tBleStatus hci_le_remove_advertising_set_api(uint8_t Advertising_Handle);

tBleStatus hci_le_clear_advertising_sets_api(void);

tBleStatus hci_le_set_periodic_advertising_parameters_api(uint8_t Advertising_Handle,
                                                          uint16_t Periodic_Advertising_Interval_Min,
                                                          uint16_t Periodic_Advertising_Interval_Max,
                                                          uint16_t Periodic_Advertising_Properties);

tBleStatus hci_le_set_periodic_advertising_enable_api(uint8_t Enable,
                                                      uint8_t Advertising_Handle);

tBleStatus hci_le_periodic_advertising_create_sync_api(uint8_t Options,
                                                       uint8_t Advertising_SID,
                                                       uint8_t Advertising_Address_Type,
                                                       uint8_t Advertiser_Address[6],
                                                       uint16_t Skip,
                                                       uint16_t Sync_Timeout,
                                                       uint8_t Sync_CTE_Type);

tBleStatus hci_le_periodic_advertising_create_sync_cancel_api(void);

tBleStatus hci_le_periodic_advertising_terminate_sync_api(uint16_t Sync_Handle);

tBleStatus hci_le_add_device_to_periodic_advertiser_list_api(uint8_t Advertiser_Address_Type,
                                                             uint8_t Advertiser_Address[6],
                                                             uint8_t Advertising_SID);

tBleStatus hci_le_remove_device_from_periodic_advertising_list_api(uint8_t Advertiser_Address_Type,
                                                                   uint8_t Advertiser_Address[6],
                                                                   uint8_t Advertising_SID);

tBleStatus hci_le_clear_periodic_advertiser_list_api(void);

tBleStatus hci_le_read_periodic_advertiser_list_size_api(uint8_t* Periodic_Advertiser_List_Size);

tBleStatus hci_le_set_periodic_advertising_receive_enable_api(uint16_t Sync_Handle,
                                                              uint8_t Enable);

tBleStatus hci_le_set_default_periodic_advertising_sync_transfer_parameters_api(uint8_t Mode,
                                                                                uint16_t Skip,
                                                                                uint16_t Sync_Timeout,
                                                                                uint8_t CTE_Type);

tBleStatus hci_le_set_periodic_advertising_sync_transfer_parameters_api(uint16_t Connection_Handle,
                                                                        uint8_t Mode,
                                                                        uint16_t Skip,
                                                                        uint16_t Sync_Timeout,
                                                                        uint8_t CTE_Type);

tBleStatus hci_le_periodic_advertising_set_info_transfer_api(uint16_t Connection_Handle,
                                                             uint16_t Service_Data,
                                                             uint8_t Advertising_Handle);

tBleStatus hci_le_periodic_advertising_sync_transfer_api(uint16_t Connection_Handle,
                                                         uint16_t Service_Data,
                                                         uint16_t Sync_Handle);

tBleStatus hci_le_iso_transmit_test_api(uint16_t Connection_Handle,
                                        uint8_t Payload_Type);

tBleStatus hci_le_iso_receive_test_api(uint16_t Connection_Handle,
                                       uint8_t Payload_Type);

tBleStatus hci_le_iso_read_test_counters_api(uint16_t Connection_Handle,
                                             uint32_t* Received_Packet_Count,
                                             uint32_t* Missed_Packet_Count,
                                             uint32_t* Failed_Packet_Count);

tBleStatus hci_le_iso_test_end_api(uint16_t Connection_Handle,
                                   uint32_t* Received_Packet_Count,
                                   uint32_t* Missed_Packet_Count,
                                   uint32_t* Failed_Packet_Count);

tBleStatus hci_le_setup_iso_data_path_api(uint16_t Connection_Handle,
                                          uint8_t Data_Path_Direction,
                                          uint8_t Data_Path_ID,
                                          uint8_t* CODEC_ID,
                                          uint8_t* Controller_Delay,
                                          uint8_t Codec_Configuration_Length,
                                          uint8_t* Codec_Configuration);

tBleStatus hci_le_remove_iso_data_path_api(uint16_t Connection_Handle,
                                           uint8_t Data_Path_Direction);

tBleStatus hci_le_read_iso_link_quality_api(uint16_t connection_handle,
                                            uint32_t* tx_unacked_packets,
                                            uint32_t* tx_flushed_packets,
                                            uint32_t* tx_last_subevent_packets,
                                            uint32_t* retransmitted_packets,
                                            uint32_t* CRC_error_packets,
                                            uint32_t* rx_unreceived_packets,
                                            uint32_t* duplicate_packets);

tBleStatus hci_le_read_iso_tx_sync_api(uint16_t connection_handle,
                                       uint16_t* packet_sequence_number,
                                       uint32_t* timestamp,
                                       uint8_t* timeoffset);

tBleStatus hci_tx_iso_data_api(uint16_t connection_handle,
                               uint8_t pb_flag,
                               uint8_t ts_flag,
                               uint16_t iso_data_load_len,
                               uint8_t* iso_data_load_p);

tBleStatus hci_le_set_data_length_api(uint16_t Connection_Handle,
                                      uint16_t Tx_Octets,
                                      uint16_t Tx_Time);

tBleStatus hci_le_read_suggested_default_data_length_api(uint16_t* Suggested_Max_Tx_Octets,
                                                         uint16_t* Suggested_Max_Tx_Time);

tBleStatus hci_le_write_suggested_default_data_length_api(uint16_t Suggested_Max_Tx_Octets,
                                                          uint16_t Suggested_Max_Tx_Time);

tBleStatus hci_le_read_maximum_data_length_api(uint16_t* Supported_Max_Tx_Octets,
                                               uint16_t* Supported_Max_Tx_Time,
                                               uint16_t* Supported_Max_Rx_Octets,
                                               uint16_t* Supported_Max_Rx_Time);

tBleStatus hci_le_enhanced_read_transmit_power_level_api(uint16_t Connection_Handle,
                                                         uint8_t PHY,
                                                         int8_t* Current_TX_Power_Level,
                                                         int8_t* Max_TX_Power_Level);

tBleStatus hci_le_read_remote_transmit_power_level_api(uint16_t Connection_Handle,
                                                       uint8_t PHY);

tBleStatus hci_le_set_path_loss_reporting_parameters_api(uint16_t Connection_Handle,
                                                         uint8_t High_Threshold,
                                                         uint8_t High_Hysteresis,
                                                         uint8_t Low_Threshold,
                                                         uint8_t Low_Hysteresis,
                                                         uint16_t Min_Time_Spent);

tBleStatus hci_le_set_path_loss_reporting_enable_api(uint16_t Connection_Handle,
                                                     uint8_t Enable);

tBleStatus hci_le_set_transmit_power_reporting_enable_api(uint16_t Connection_Handle,
                                                          uint8_t Local_Enable,
                                                          uint8_t Remote_Enable);

tBleStatus hci_le_read_phy_api(uint16_t Connection_Handle,
                               uint8_t* TX_PHY,
                               uint8_t* RX_PHY);

tBleStatus hci_le_set_default_phy_api(uint8_t ALL_PHYS,
                                      uint8_t TX_PHYS,
                                      uint8_t RX_PHYS);

tBleStatus hci_le_set_phy_api(uint16_t Connection_Handle,
                              uint8_t ALL_PHYS,
                              uint8_t TX_PHYS,
                              uint8_t RX_PHYS,
                              uint16_t PHY_options);

tBleStatus hci_le_add_device_to_resolving_list_api(uint8_t Peer_Identity_Address_Type,
                                                   uint8_t Peer_Identity_Address[6],
                                                   uint8_t Peer_IRK[16],
                                                   uint8_t Local_IRK[16]);

tBleStatus hci_le_remove_device_from_resolving_list_api(uint8_t Peer_Identity_Address_Type,
                                                        uint8_t Peer_Identity_Address[6]);

tBleStatus hci_le_clear_resolving_list_api(void);

tBleStatus hci_le_read_resolving_list_size_api(uint8_t* Resolving_List_Size);

tBleStatus hci_le_read_peer_resolvable_address_api(uint8_t Peer_Identity_Address_Type,
                                                   uint8_t Peer_Identity_Address[6],
                                                   uint8_t Peer_Resolvable_Address[6]);

tBleStatus hci_le_read_local_resolvable_address_api(uint8_t Peer_Identity_Address_Type,
                                                    uint8_t Peer_Identity_Address[6],
                                                    uint8_t Local_Resolvable_Address[6]);

tBleStatus hci_le_set_address_resolution_enable_api(uint8_t Address_Resolution_Enable);

tBleStatus hci_le_set_resolvable_private_address_timeout_api(uint16_t RPA_Timeout);

tBleStatus hci_le_set_privacy_mode_api(uint8_t Peer_Identity_Address_Type,
                                       uint8_t Peer_Identity_Address[6],
                                       uint8_t Privacy_Mode);

tBleStatus hci_le_set_data_related_address_changes_api(uint8_t Advertising_Handle,
                                                       uint8_t Change_Reasons);

tBleStatus hci_le_request_peer_sca_api(uint16_t Connection_Handle);

tBleStatus hci_le_set_scan_parameters_api(uint8_t LE_Scan_Type,
                                          uint16_t LE_Scan_Interval,
                                          uint16_t LE_Scan_Window,
                                          uint8_t Own_Address_Type,
                                          uint8_t Scanning_Filter_Policy);

tBleStatus hci_le_set_scan_enable_api(uint8_t LE_Scan_Enable,
                                      uint8_t Filter_Duplicates);

tBleStatus hci_le_set_extended_scan_parameters_api(uint8_t Own_Address_Type,
                                                   uint8_t Scanning_Filter_Policy,
                                                   uint8_t Scanning_PHYs,
                                                   Extended_Scan_Parameters_t* Extended_Scan_Parameters);

tBleStatus hci_le_set_extended_scan_enable_api(uint8_t Enable,
                                               uint8_t Filter_Duplicates,
                                               uint16_t Duration,
                                               uint16_t Period);

tBleStatus hci_le_connection_update_api(uint16_t Connection_Handle,
                                        uint16_t Conn_Interval_Min,
                                        uint16_t Conn_Interval_Max,
                                        uint16_t Conn_Latency,
                                        uint16_t Supervision_Timeout,
                                        uint16_t Minimum_CE_Length,
                                        uint16_t Maximum_CE_Length);

tBleStatus hci_le_set_default_subrate_api(uint16_t Subrate_Min,
                                          uint16_t Subrate_Max,
                                          uint16_t Max_Latency,
                                          uint16_t Continuation_Number,
                                          uint16_t Supervision_Timeout);

tBleStatus hci_le_subrate_request_api(uint16_t Connection_Handle,
                                      uint16_t Subrate_Min,
                                      uint16_t Subrate_Max,
                                      uint16_t Max_Latency,
                                      uint16_t Continuation_Number,
                                      uint16_t Supervision_Timeout);

tBleStatus hci_le_enhanced_receiver_test_api(uint8_t RX_Frequency,
                                             uint8_t Phy,
                                             uint8_t Modulation_index);

tBleStatus hci_le_receiver_test_v3_api(uint8_t RX_Channel,
                                       uint8_t PHY,
                                       uint8_t Modulation_Index,
                                       uint8_t Expected_CTE_Length,
                                       uint8_t Expected_CTE_Type,
                                       uint8_t Slot_Durations,
                                       uint8_t Switching_Pattern_Length,
                                       uint8_t* Antenna_IDs);

tBleStatus hci_le_enhanced_transmitter_test_api(uint8_t TX_Frequency,
                                                uint8_t Length_Of_Test_Data,
                                                uint8_t Packet_Payload,
                                                uint8_t Phy);

tBleStatus hci_le_transmitter_test_v3_api(uint8_t TX_Channel,
                                          uint8_t Test_Data_Length,
                                          uint8_t Packet_Payload,
                                          uint8_t PHY,
                                          uint8_t CTE_Length,
                                          uint8_t CTE_Type,
                                          uint8_t Switching_Pattern_Length,
                                          uint8_t* Antenna_IDs);

tBleStatus hci_le_transmitter_test_v4_api(uint8_t TX_Channel,
                                          uint8_t Test_Data_Length,
                                          uint8_t Packet_Payload,
                                          uint8_t PHY,
                                          uint8_t CTE_Length,
                                          uint8_t CTE_Type,
                                          uint8_t Switching_Pattern_Length,
                                          uint8_t* Antenna_IDs,
                                          int8_t Transmit_Power_Level);


void Get_Random_Addr(uint8_t *randBdAddr);

#endif /* _STACK_USER_CFG_H_ */

