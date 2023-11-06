#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_gatt_util.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_app_common.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_nvram.h"
#include "wiced_result.h"
#include "wiced_hal_platform.h"
#include "wiced_transport.h"
#include "wiced_bt_hac.h"
#include "wiced_hal_puart.h"
#include "string.h"
#include "wiced_bt_stack.h"

#ifdef WICED_BT_TRACE_ENABLE
#include "wiced_bt_trace.h"
#endif

/*******************************************
 *      Constants
 ******************************************/
extern const wiced_bt_cfg_settings_t wiced_app_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_app_cfg_buf_pools[];

/*************************************************************************************************************/
/******************************************************************************************************************/
#ifdef CWY20706A2
#define APP_BUTTON               WICED_GPIO_BUTTON
#define APP_BUTTON_SETTINGS      (WICED_GPIO_BUTTON_SETTINGS(GPIO_EN_INT_BOTH_EDGE))
#define APP_BUTTON_DEFAULT_STATE WICED_GPIO_BUTTON_DEFAULT_STATE
#define APP_LED                   WICED_PLATFORM_LED_1
#endif

#if (defined (CYW20719B1) || defined(CYW20721B1) || defined(CYW20735B1) )
#define APP_LED     WICED_GPIO_PIN_LED_2
#endif

#ifdef CYW20735B0
#define APP_BUTTON                  WICED_GPIO_PIN_BUTTON
#define APP_LED                     WICED_GPIO_PIN_LED1
#define APP_BUTTON_SETTINGS         (GPIO_INPUT_ENABLE | GPIO_PULL_DOWN | GPIO_EN_INT_BOTH_EDGE)
#define APP_BUTTON_DEFAULT_STATE    GPIO_PIN_OUTPUT_LOW
#endif

#define HANDLE_HOME_AUTOMATION_CHAR_CFG_DESC 0X63
/******************************************************************************
 *                                  FUNCTION_PROTOTYPES
 *******************************************************************************/
static wiced_result_t           hac_management_callback(wiced_bt_management_evt_t event,wiced_bt_management_evt_data_t *p_event_data);
static void                     hac_scan_result_cback(wiced_bt_ble_scan_results_t *p_scan_results,uint8_t *p_adv_data);
static void                     hac_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status);
static void                     hac_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status);
static void                     hac_process_pairing_complete(uint8_t result);
static wiced_bt_gatt_status_t   hac_gatts_callback(wiced_bt_gatt_evt_t event,wiced_bt_gatt_event_data_t *p_data);
static wiced_bt_gatt_status_t   hac_gatt_discovery_result(wiced_bt_gatt_discovery_result_t *p_data);
static wiced_bt_gatt_status_t   hac_gatt_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data);
static wiced_bt_gatt_status_t   hac_gatt_operation_complete(wiced_bt_gatt_operation_complete_t *p_data);
static wiced_bool_t             hac_save_link_keys(wiced_bt_device_link_keys_t *p_keys);
static wiced_bool_t             hac_read_link_keys(wiced_bt_device_link_keys_t *p_keys);
void                            hac_timeout(uint32_t count);
static void                     hac_fan_control_callback(uint16_t conn_id,wiced_bt_gatt_status_t result);
static void                     hac_fan_off_control_callback(uint16_t conn_id,wiced_bt_gatt_status_t result);
static void                     hac_light_on_control_callback(uint16_t  conn_id , wiced_bt_gatt_status_t result);
static void                     hac_light_off_control_callback(uint16_t  conn_id , wiced_bt_gatt_status_t result);
static void                     hac_fan_speed_control_callback(uint16_t  conn_id , wiced_bt_gatt_status_t result);
static void                     hac_light_brightness_control_callback(uint16_t  conn_id , wiced_bt_gatt_status_t result);
void                            hac_load_keys_to_addr_resolution_db(void);
void                            hac_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length,uint8_t* p_data);
void                            hac_discovery_complete_callback(uint16_t conn_id, wiced_bool_t result);
void                            hac_stop_complete_callback(uint16_t conn_id, wiced_bt_gatt_status_t result);
void                            hac_start_complete_callback(uint16_t conn_id,wiced_bt_gatt_status_t result);
void                            puar_rx_interrupt_callback(void *unused);
/***********************************************************************************************************
 *                                  VARIABLES DEFINITION
 * **********************************************************************************************************/
typedef struct {

    uint16_t conn_id;
#define HAC_DISCOVERY_STATE_SERVICE    0
#define HAC_DISCOVERY_STATE_HAS        1
    uint8_t discovery_state;
    uint8_t started;
    uint32_t timeout;
    uint16_t home_automation_service_s_handle;
    uint16_t home_automation_service_e_handle;
    BD_ADDR remote_addr;
    wiced_bt_ble_address_type_t addr_type;
} hac_app_cb_t;

hac_app_cb_t hac_app_cb;
const wiced_transport_cfg_t transport_cfg =
{
        WICED_TRANSPORT_UART,
        { WICED_TRANSPORT_UART_HCI_MODE, HCI_UART_DEFAULT_BAUD },
        { 0, 0 },
        NULL,
        NULL,
        NULL
 };

void test_puart_driver(void) {
    uint8_t read_5_bytes[5];
    wiced_hal_puart_init();
    wiced_hal_puart_flow_off();
    wiced_hal_puart_register_interrupt(puar_rx_interrupt_callback);
    wiced_hal_puart_set_watermark_level(1);
    wiced_hal_puart_enable_tx();
    wiced_hal_puart_print("enter the values to control");
    wiced_hal_puart_set_baudrate(115200);
}
wiced_transport_buffer_pool_t *host_trans_pool;
wiced_bt_hac_reg_t hac_reg =
{
        .p_discovery_complete_callback          = hac_discovery_complete_callback,
        .p_start_complete_callback              = hac_start_complete_callback,
        .p_stop_complete_callback               = hac_stop_complete_callback,
        .p_fan_control_callback                 = hac_fan_control_callback,
        .p_fan_off_control_callback             = hac_fan_off_control_callback,
        .p_light_on_control_callback            = hac_light_on_control_callback,
        .p_light_off_control_callback           = hac_light_off_control_callback,
        .p_fan_speed_control_callback           = hac_fan_speed_control_callback,
        .p_light_brightness_control_callback    = hac_light_brightness_control_callback,
};
wiced_timer_t app_timer;

#ifndef CYW207355B0
APPLICATION_START()
#else
void application_start( void )
#endif
{
    wiced_result_t result;
    wiced_transport_init(&transport_cfg);
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);
    WICED_BT_TRACE("HAC APP START\n");
    wiced_bt_hac_initialize(&hac_reg);
    memset(&hac_app_cb, 0, sizeof(hac_app_cb_t));
    wiced_bt_stack_init(hac_management_callback, &wiced_app_cfg_settings,wiced_app_cfg_buf_pools);
}
void hac_application_init(void) {

    wiced_bt_gatt_status_t gatt_status;
    wiced_result_t result;
#ifdef CYW20735B1
    wiced_bt_app_init();
#endif
    wiced_hal_gpio_configure_pin(WICED_GPIO_PIN_LED_2, GPIO_OUTPUT_ENABLE,GPIO_PIN_OUTPUT_HIGH);
    gatt_status = wiced_bt_gatt_register(hac_gatts_callback);
    WICED_BT_TRACE("wiced_bt_gatt_register: %d\n", gatt_status);
    /* Register callback for receiving hci traces */
    wiced_bt_dev_register_hci_trace(hac_trace_callback);
    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);
    /* Load the address resolution DB with the keys stored in the NVRAM */
    hac_load_keys_to_addr_resolution_db();
    /* Start scan to find ANS Client */
    result = wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_HIGH_DUTY,  WICED_TRUE, hac_scan_result_cback);
    WICED_BT_TRACE("wiced_bt_ble_scan: %d \n", result);
}
/********************************************************************************************************************
 *                                  Functiion definitions
 * ******************************************************************************************************************/
wiced_result_t hac_management_callback(wiced_bt_management_evt_t event,wiced_bt_management_evt_data_t *p_event_data) {

    wiced_result_t                      result = WICED_BT_SUCCESS;
    wiced_bt_dev_encryption_status_t    *p_status;
    wiced_bt_dev_ble_pairing_info_t     *p_info;
    wiced_bt_ble_advert_mode_t          *p_mode;
    wiced_bt_device_link_keys_t         paired_device_link_key_request;
    uint8_t                             bytes_written, bytes_read;
    WICED_BT_TRACE("HAC Home_automation_cb :%d\n", event);
    switch (event) {
    case BTM_ENABLED_EVT: /**< Bluetooth controller and host stack enabled. Event data: wiced_bt_dev_enabled_t */
        hac_application_init();
        test_puart_driver();
        break;
    case BTM_DISABLED_EVT: /**< Bluetooth controller and host stack disabled. Event data: NULL */
        break;
    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;
        p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_SC_BOND;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size = 0x10;
        p_event_data->pairing_io_capabilities_ble_request.init_keys = BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK| BTM_LE_KEY_LENC;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys =  BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK | BTM_LE_KEY_LENC;
        break;
    case BTM_PAIRING_COMPLETE_EVT:
        WICED_BT_TRACE("Pairing Complete of the device: %d",p_event_data->pairing_complete.pairing_complete_info.ble.reason);
        hac_process_pairing_complete(p_event_data->pairing_complete.pairing_complete_info.ble.reason);
        break;
    case BTM_ENCRYPTION_STATUS_EVT:
        WICED_BT_TRACE("Encryption Status Event: bd (%B) res %d",p_event_data->encryption_status.bd_addr,p_event_data->encryption_status.result);
        break;
    case BTM_SECURITY_REQUEST_EVT:
        wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr,WICED_BT_SUCCESS);
        break;
    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        hac_save_link_keys(&p_event_data->paired_device_link_keys_update);
        break;
    case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        if (hac_read_link_keys(&p_event_data->paired_device_link_keys_request))
        {
            result = WICED_BT_SUCCESS;
            WICED_BT_TRACE("Key retrieval success\n");
        }
        else
        {
            result = WICED_BT_ERROR;
            WICED_BT_TRACE("Key retrieval failure\n");
        }
        break;
    case BTM_BLE_SCAN_STATE_CHANGED_EVT:
        WICED_BT_TRACE("Scan State Change: %d\n",p_event_data->ble_scan_state_changed);
        test_puart_driver();
        break;
    default:
        break;
    }
    return result;
}
void hac_scan_result_cback(wiced_bt_ble_scan_results_t *p_scan_result,uint8_t *p_adv_data) {
    wiced_result_t      status;
    wiced_bool_t        ret_status;
    uint8_t             length;
    uint8_t             *p_data;
    uint16_t            home_automation_service_uuid = UUID_SERVICE_HOME_AUTOMATION;
    if (p_scan_result) {
        p_data = wiced_bt_ble_check_advertising_data(p_adv_data, BTM_BLE_ADVERT_TYPE_16SRV_COMPLETE, &length);
        if ((p_data == NULL) || (length != LEN_UUID_16) || (memcmp(p_data, &home_automation_service_uuid, LEN_UUID_16) != 0)) {
            return;
        }
        WICED_BT_TRACE("Found Device : %B \n", p_scan_result->remote_bd_addr);
        status = wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_NONE, WICED_TRUE, hac_scan_result_cback);
        WICED_BT_TRACE("scan off %d\n", status);
        ret_status = wiced_bt_gatt_le_connect(p_scan_result->remote_bd_addr,p_scan_result->ble_addr_type, BLE_CONN_MODE_HIGH_DUTY, TRUE);
        WICED_BT_TRACE("wiced_bt_gatt_connect status %d\n", ret_status);
    }
    else
    {
        WICED_BT_TRACE("Scan completed :\n");
    }
}

wiced_bt_gatt_status_t hac_gatts_callback(wiced_bt_gatt_evt_t event,wiced_bt_gatt_event_data_t *p_data) {

    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;
    switch (event) {
    case GATT_CONNECTION_STATUS_EVT:
        if (p_data->connection_status.connected)
        {
            hac_connection_up(&p_data->connection_status);
        } else
        {
            hac_connection_down(&p_data->connection_status);
        }
        break;
    case GATT_DISCOVERY_RESULT_EVT:
        result = hac_gatt_discovery_result(&p_data->discovery_result);
        break;
    case GATT_DISCOVERY_CPLT_EVT:
        result = hac_gatt_discovery_complete(&p_data->discovery_complete);
        break;
    case GATT_OPERATION_CPLT_EVT:
        result = hac_gatt_operation_complete(&p_data->operation_complete);
        break;
    default:
        break;
    }

    return result;

}

void hac_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_gatt_status_t status;
    WICED_BT_TRACE("%s\n", __FUNCTION__);
    hac_app_cb.conn_id = p_conn_status->conn_id;
    //save address of connected device
    memcpy(hac_app_cb.remote_addr, p_conn_status->bd_addr,sizeof(hac_app_cb.remote_addr));
    hac_app_cb.addr_type = p_conn_status->addr_type;
    wiced_bt_hac_connection_up(p_conn_status->conn_id); //connection is up
    hac_app_cb.discovery_state = HAC_DISCOVERY_STATE_SERVICE;
    hac_app_cb.home_automation_service_s_handle = 0;
    hac_app_cb.home_automation_service_e_handle = 0;
    status = wiced_bt_util_send_gatt_discover(hac_app_cb.conn_id,GATT_DISCOVER_SERVICES_ALL, UUID_ATTRIBUTE_PRIMARY_SERVICE, 1,0xffff);
    WICED_BT_TRACE("start discover status:%d\n", status);
}

void hac_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    WICED_BT_TRACE("%s\n", __FUNCTION__);
    hac_app_cb.conn_id = 0;
    hac_app_cb.discovery_state = HAC_DISCOVERY_STATE_SERVICE;
    hac_app_cb.home_automation_service_s_handle = 0;
    hac_app_cb.home_automation_service_e_handle = 0;
    wiced_hal_gpio_set_pin_output(WICED_GPIO_PIN_LED_2, GPIO_PIN_OUTPUT_HIGH);
    wiced_bt_hac_connection_down(p_conn_status->conn_id);
}

void hac_process_pairing_complete(uint8_t result) {
    wiced_bt_gatt_status_t status;
    if (result == WICED_SUCCESS) {
        if ((hac_app_cb.home_automation_service_s_handle != 0)&& (hac_app_cb.home_automation_service_e_handle != 0)) {
            status = wiced_bt_hac_start(hac_app_cb.conn_id);
            WICED_BT_TRACE("HAC START %d \n ", status);
        }
    }
}


wiced_bt_gatt_status_t hac_gatt_discovery_result(wiced_bt_gatt_discovery_result_t *p_data)\

{
    uint16_t HOME_AUTOMATION_SERVICE_UUID = UUID_SERVICE_HOME_AUTOMATION;
    WICED_BT_TRACE("[%s] conn %d type %d state 0x%02x\n", __FUNCTION__,p_data->conn_id, p_data->discovery_type,hac_app_cb.discovery_state);
    switch (hac_app_cb.discovery_state)
    {
    case HAC_DISCOVERY_STATE_HAS:
        wiced_bt_hac_discovery_result(p_data);
        break;
    default:
        if (p_data->discovery_type == GATT_DISCOVER_SERVICES_ALL)
        {
            if (p_data->discovery_data.group_value.service_type.len == 2)
            {
                WICED_BT_TRACE("s:%04x e:%04x uuid:%x\n",p_data->discovery_data.group_value.s_handle,p_data->discovery_data.group_value.e_handle,p_data->discovery_data.group_value.service_type.uu.uuid16);
                if (memcmp(&p_data->discovery_data.group_value.service_type.uu,&HOME_AUTOMATION_SERVICE_UUID, 2) == 0)
                {
                    WICED_BT_TRACE("home automation service found s:%04x e:%04x\n",
                            p_data->discovery_data.group_value.s_handle,
                            p_data->discovery_data.group_value.e_handle);
                    hac_app_cb.home_automation_service_s_handle = p_data->discovery_data.group_value.s_handle;
                    hac_app_cb.home_automation_service_e_handle =p_data->discovery_data.group_value.e_handle;
                }
            }
        }
        else
        {
            WICED_BT_TRACE("!!!! invalid op:%d\n", p_data->discovery_type);
        }
    }
    return WICED_BT_GATT_SUCCESS;
}

wiced_bt_gatt_status_t hac_gatt_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data)
{
    wiced_result_t result;
    WICED_BT_TRACE("[%s] conn %d type %d state %d\n", __FUNCTION__,p_data->conn_id, p_data->disc_type, hac_app_cb.discovery_state);
    switch (hac_app_cb.discovery_state)
    {
    case HAC_DISCOVERY_STATE_HAS:
        wiced_bt_hac_discovery_complete(p_data);
        break;
    default:
        if (p_data->disc_type == GATT_DISCOVER_SERVICES_ALL)
        {
            WICED_BT_TRACE("HAC:%04x-%04x\n",hac_app_cb.home_automation_service_s_handle,hac_app_cb.home_automation_service_e_handle);
            /* If Heart Rate Service found tell WICED BT HRC library to start its discovery */
            if ((hac_app_cb.home_automation_service_s_handle != 0)&& (hac_app_cb.home_automation_service_e_handle != 0))
            {
                hac_app_cb.discovery_state = HAC_DISCOVERY_STATE_HAS;
                if (wiced_bt_hac_discover(hac_app_cb.conn_id,hac_app_cb.home_automation_service_s_handle,hac_app_cb.home_automation_service_e_handle))
                    break;
            }
        }
        else
        {
            WICED_BT_TRACE("!!!! invalid op:%d\n", p_data->disc_type);
        }
    }
    return WICED_BT_GATT_SUCCESS;
}
wiced_bt_gatt_status_t hac_gatt_operation_complete(wiced_bt_gatt_operation_complete_t *p_data)
{
    switch (p_data->op){
        case GATTC_OPTYPE_WRITE:
            wiced_bt_hac_gatt_op_complete(p_data);
            break;
        case GATTC_OPTYPE_NOTIFICATION:
        case GATTC_OPTYPE_CONFIG:
        case GATTC_OPTYPE_READ:
        case GATTC_OPTYPE_INDICATION:
            WICED_BT_TRACE("This app does not support op:%d\n", p_data->op);
        break;
    }
    return WICED_BT_GATT_SUCCESS;
}
void hac_discovery_complete_callback(uint16_t conn_id, wiced_bool_t result) {
    WICED_BT_TRACE("[%s] result:%d\n", __FUNCTION__, result);
    // This app automatically starts the client
    if (result == WICED_TRUE)
    {
        wiced_bt_hac_start(hac_app_cb.conn_id);
    }
    else
    {
        //Disconnect. In the snippet, no point maintain connection without heart rate service
        result = wiced_bt_gatt_disconnect(conn_id);
        WICED_BT_TRACE("wiced_bt_gatt_discnnect %d \n", result);
    }
}
void hac_start_complete_callback(uint16_t conn_id , wiced_bt_gatt_status_t result) {

    wiced_result_t rc;
    WICED_BT_TRACE("[%s] result:%d\n", __FUNCTION__, result);
    // Special case when we try to register for notification and we are not paired yet
    if (result == WICED_BT_GATT_INSUF_AUTHENTICATION)
    {
        wiced_bt_device_link_keys_t link_keys;
        if (hac_read_link_keys(&link_keys))
        {
            wiced_bt_ble_sec_action_type_t sec_act = BTM_BLE_SEC_ENCRYPT;
            rc = wiced_bt_dev_set_encryption(hac_app_cb.remote_addr,BT_TRANSPORT_LE, &sec_act);
            WICED_BT_TRACE("start encrypt result:%d\n", rc);
        } else {
            rc = wiced_bt_dev_sec_bond(hac_app_cb.remote_addr,hac_app_cb.addr_type, BT_TRANSPORT_LE, 0, NULL);
            WICED_BT_TRACE("start bond result:%d\n", rc);
        }
        return;
    } else if (result == WICED_BT_GATT_SUCCESS) {
        hac_app_cb.started = WICED_TRUE;
    }
}

void hac_stop_complete_callback(uint16_t conn_id, wiced_bt_gatt_status_t result)
{
    WICED_BT_TRACE("[%s] result:%d\n", __FUNCTION__, result);
    hac_app_cb.started = WICED_FALSE;
}
void  puar_rx_interrupt_callback(void *unused) {

   uint8_t readbytes;
   wiced_result_t          result;
   wiced_bt_gatt_status_t  status;
       wiced_hal_puart_read(&readbytes);
       wiced_hal_puart_print( "\nYou typed \t" );
       wiced_hal_puart_write(readbytes);
       switch (readbytes) {
           case '1':
               status = wiced_bt_hac_fan_control(hac_app_cb.conn_id);
               WICED_BT_TRACE(" %d \n", status);
               break;
           case '2':
               status = wiced_bt_hac_fan_off_control(hac_app_cb.conn_id);
               WICED_BT_TRACE(" %d \n", status);
               break;
           case '3':
               status = wiced_bt_hac_light_on_control(hac_app_cb.conn_id);
               WICED_BT_TRACE("%d \n" , status);
               break;
           case '4':
               status = wiced_bt_hac_light_off_control(hac_app_cb.conn_id);
               WICED_BT_TRACE("%d \n" , status);
               break;
           case '5':
               status = wiced_bt_hac_fan_speed_control(hac_app_cb.conn_id);
               WICED_BT_TRACE("%d \n" , status);
               break;
           case '6':
               status = wiced_bt_hac_light_brightness_control(hac_app_cb.conn_id);
               WICED_BT_TRACE("%d \n" , status);
               break;
         default :
           WICED_BT_TRACE("THE INPUT IS NOT present");
           break;
           }
       wiced_hal_puart_reset_puart_interrupt( );
       test_puart_driver();
}
void hac_timeout(uint32_t count)
{
    hac_app_cb.timeout++;
    WICED_BT_TRACE(" %d\n", hac_app_cb.timeout);
}
void hac_load_keys_to_addr_resolution_db(void)
{
    uint8_t bytes_read;
    wiced_result_t result;
    wiced_bt_device_link_keys_t keys;
    bytes_read = wiced_hal_read_nvram(WICED_NVRAM_VSID_START, sizeof(keys),(uint8_t *) &keys, &result);
    if (result == WICED_SUCCESS) {
#ifdef CYW20706A2
        result = wiced_bt_dev_add_device_to_address_resolution_db(&keys, keys.key_data.ble_addr_type);
#else
        result = wiced_bt_dev_add_device_to_address_resolution_db(&keys);
#endif
    }
}
wiced_bool_t hac_save_link_keys(wiced_bt_device_link_keys_t *p_keys)
{
    uint8_t         bytes_written;
    wiced_result_t  result;
    bytes_written = wiced_hal_write_nvram(WICED_NVRAM_VSID_START,sizeof(wiced_bt_device_link_keys_t), (uint8_t *) p_keys, &result);
    WICED_BT_TRACE("Saved %d bytes at id:%d %d\n", bytes_written,WICED_NVRAM_VSID_START);
    return (bytes_written == sizeof(wiced_bt_device_link_keys_t));
}
wiced_bool_t hac_read_link_keys(wiced_bt_device_link_keys_t *p_keys)
{
    uint8_t         bytes_read;
    wiced_result_t  result;
    bytes_read = wiced_hal_read_nvram(WICED_NVRAM_VSID_START,sizeof(wiced_bt_device_link_keys_t), (uint8_t *) p_keys, &result);
    WICED_BT_TRACE("read %d bytes at id:%d %d\n", bytes_read,WICED_NVRAM_VSID_START);
    return (bytes_read == sizeof(wiced_bt_device_link_keys_t));
}

void hac_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length,uint8_t* p_data)
{
    wiced_transport_send_hci_trace(NULL, type, length, p_data);
}
void hac_fan_control_callback(uint16_t conn_id,wiced_bt_gatt_status_t result) {
    WICED_BT_TRACE("[%s]  result : %d\n", __FUNCTION__, result);
    if (result == WICED_BT_GATT_SUCCESS) {
    }
}
void hac_fan_off_control_callback(uint16_t conn_id,wiced_bt_gatt_status_t result)
{
    WICED_BT_TRACE("[%s]  result : %d\n", __FUNCTION__, result);
    if (result == WICED_BT_GATT_SUCCESS) {
    }
}
void hac_light_on_control_callback(uint16_t  conn_id , wiced_bt_gatt_status_t result)
{
    WICED_BT_TRACE("[%s] result : %d\n", __FUNCTION__ , result );
    if( result == WICED_BT_GATT_SUCCESS) {
    }
}
void hac_light_off_control_callback(uint16_t  conn_id , wiced_bt_gatt_status_t result)
{
    WICED_BT_TRACE("[%s] result : %d\n", __FUNCTION__ , result );

    if( result == WICED_BT_GATT_SUCCESS) {
    }
}

void hac_fan_speed_control_callback(uint16_t  conn_id , wiced_bt_gatt_status_t result)
{
    WICED_BT_TRACE("[%s] result : %d\n", __FUNCTION__ , result );
    if( result == WICED_BT_GATT_SUCCESS) {
        WICED_BT_TRACE("Changed the speed successfully");
    }
}

void hac_light_brightness_control_callback(uint16_t  conn_id , wiced_bt_gatt_status_t result)
{
    WICED_BT_TRACE("[%s] result : %d\n", __FUNCTION__ , result );
    if( result == WICED_BT_GATT_SUCCESS) {
        WICED_BT_TRACE("Changed the brightness successfully");
    }
}
