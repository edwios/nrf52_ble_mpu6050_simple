 /* 
  * This code is not extensively tested and only 
  * meant as a simple explanation and for inspiration. 
  * NO WARRANTY of ANY KIND is provided. 
  */

#include <stdint.h>
#include <string.h>
#include "nrf_gpio.h"
#include "ble_ltr329.h"
#include "ble_srv_common.h"
#include "app_error.h"
#include "app_ltr329.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


void ble_ltr329_on_ble_evt(ble_ltr329_t * p_ltr329, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            p_ltr329->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            p_ltr329->conn_handle = BLE_CONN_HANDLE_INVALID;
        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for adding our new characterstic to "Our service" that we initiated in the previous tutorial. 
 *
 * @param[in]   p_ltr329        ltr329 structure.
 *
 */
static uint32_t ble_char_visible_add(ble_ltr329_t * p_ltr329)
{
    uint32_t   err_code = 0; // Variable to hold return codes from library and softdevice functions
    
    ble_uuid_t          char_uuid;   
    BLE_UUID_BLE_ASSIGN(char_uuid, BLE_UUID_VISIBLE_CHARACTERISTC_UUID);
    
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
    
    ble_gatts_attr_md_t cccd_md;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
        
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc = BLE_GATTS_VLOC_STACK;    
    
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    attr_char_value.max_len     = sizeof(int16_t);
    attr_char_value.init_len    = sizeof(int16_t);
    uint8_t value[2]            = {0};
    attr_char_value.p_value     = value;

//    NRF_LOG_DEBUG("ble_char_accel_add: add gatt char"); NRF_LOG_FLUSH();
    err_code = sd_ble_gatts_characteristic_add(p_ltr329->service_handle,
                                       &char_md,
                                       &attr_char_value,
                                       &p_ltr329->visible_char_handles);
    APP_ERROR_CHECK(err_code);   

    return NRF_SUCCESS;
}

static uint32_t ble_char_ir_add(ble_ltr329_t * p_ltr329)
{
    uint32_t   err_code = 0; // Variable to hold return codes from library and softdevice functions
    
    ble_uuid_t          char_uuid;   
    BLE_UUID_BLE_ASSIGN(char_uuid, BLE_UUID_IR_CHARACTERISTC_UUID);
    
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
    
    ble_gatts_attr_md_t cccd_md;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
        
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc = BLE_GATTS_VLOC_STACK;    
    
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    attr_char_value.max_len     = sizeof(int16_t);
    attr_char_value.init_len    = sizeof(int16_t);
    uint8_t value[2]            = {0};
    attr_char_value.p_value     = value;

//    NRF_LOG_DEBUG("ble_char_accel_add: add gatt char"); NRF_LOG_FLUSH();
    err_code = sd_ble_gatts_characteristic_add(p_ltr329->service_handle,
                                       &char_md,
                                       &attr_char_value,
                                       &p_ltr329->ir_char_handles);
    APP_ERROR_CHECK(err_code);   

    return NRF_SUCCESS;
}

static uint32_t ble_char_lux_add(ble_ltr329_t * p_ltr329)
{
    uint32_t   err_code = 0; // Variable to hold return codes from library and softdevice functions
    
    ble_uuid_t          char_uuid;   
    BLE_UUID_BLE_ASSIGN(char_uuid, BLE_UUID_IRRADIANCE_CHARACTERISTC_UUID);
    
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
    
    ble_gatts_attr_md_t cccd_md;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
        
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc = BLE_GATTS_VLOC_STACK;    
    
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    attr_char_value.max_len     = sizeof(int16_t);
    attr_char_value.init_len    = sizeof(int16_t);
    uint8_t value[2]            = {0};
    attr_char_value.p_value     = value;

//    NRF_LOG_DEBUG("ble_char_accel_add: add gatt char"); NRF_LOG_FLUSH();
    err_code = sd_ble_gatts_characteristic_add(p_ltr329->service_handle,
                                       &char_md,
                                       &attr_char_value,
                                       &p_ltr329->lux_char_handles);
    APP_ERROR_CHECK(err_code);   

    return NRF_SUCCESS;
}

static uint32_t ble_char_temp_add(ble_ltr329_t * p_ltr329)
{
    uint32_t   err_code = 0; // Variable to hold return codes from library and softdevice functions
    
    ble_uuid_t          char_uuid;   
    BLE_UUID_BLE_ASSIGN(char_uuid, BLE_UUID_TEMPERATURE_CHARACTERISTC_UUID);
    
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
    
    ble_gatts_attr_md_t cccd_md;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
        
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc = BLE_GATTS_VLOC_STACK;    
    
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    attr_char_value.max_len     = sizeof(int16_t);
    attr_char_value.init_len    = sizeof(int16_t);
    uint8_t value[2]            = {0};
    attr_char_value.p_value     = value;

//    NRF_LOG_DEBUG("ble_char_accel_add: add gatt char"); NRF_LOG_FLUSH();
    err_code = sd_ble_gatts_characteristic_add(p_ltr329->service_handle,
                                       &char_md,
                                       &attr_char_value,
                                       &p_ltr329->temp_char_handles);
    APP_ERROR_CHECK(err_code);   

    return NRF_SUCCESS;
}

/**@brief Function for initiating our new service.
 *
 * @param[in]   p_ltr329        Our Service structure.
 *
 */
void ble_ltr329_service_init(ble_ltr329_t * p_ltr329)
{
    uint32_t   err_code; // Variable to hold return codes from library and softdevice functions

    ble_uuid_t        service_uuid;
    ble_uuid128_t     base_uuid = {BLE_UUID_BASE_UUID};
    service_uuid.uuid = BLE_UUID_ENVIRONMENTAL_SENSING_SERVICE;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
    APP_ERROR_CHECK(err_code);    

    p_ltr329->conn_handle = BLE_CONN_HANDLE_INVALID;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_ltr329->service_handle);
    
    APP_ERROR_CHECK(err_code);
    
    ble_char_visible_add(p_ltr329);
    ble_char_ir_add(p_ltr329);
    ble_char_lux_add(p_ltr329);
    ble_char_temp_add(p_ltr329);
}

// ALREADY_DONE_FOR_YOU: Function to be called when updating characteristic value
void ble_ltr329_update(ble_ltr329_t *p_ltr329, ltr329_ambient_values_t * ltr329_ambient_values)
{
    // Send value if connected and notifying
    if (p_ltr329->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint16_t               len = sizeof(uint16_t);
        ble_gatts_hvx_params_t hvx_params;
        memset(&hvx_params, 0, sizeof(hvx_params));

        uint16_t visible_value = ltr329_ambient_values->ambient_visible_value;
        uint16_t ir_value = ltr329_ambient_values->ambient_ir_value;
        uint16_t lux_value = ltr329_ambient_values->ambient_lux_value;

        hvx_params.handle = p_ltr329->visible_char_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &len;
        hvx_params.p_data = (uint8_t*)&visible_value;  

        sd_ble_gatts_hvx(p_ltr329->conn_handle, &hvx_params);
 
        memset(&hvx_params, 0, sizeof(hvx_params));
        hvx_params.handle = p_ltr329->ir_char_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &len;
        hvx_params.p_data = (uint8_t*)&ir_value;  

        sd_ble_gatts_hvx(p_ltr329->conn_handle, &hvx_params);

        memset(&hvx_params, 0, sizeof(hvx_params));
        hvx_params.handle = p_ltr329->lux_char_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &len;
        hvx_params.p_data = (uint8_t*)&lux_value;  

        sd_ble_gatts_hvx(p_ltr329->conn_handle, &hvx_params);
    } 
}

void ble_ltr329_temperature_update(ble_ltr329_t *p_ltr329, temp_value_t * temperature_value)
{
    // Send value if connected and notifying
    if (p_ltr329->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint16_t               len = sizeof(uint16_t);
        ble_gatts_hvx_params_t hvx_params;
        memset(&hvx_params, 0, sizeof(hvx_params));

        memset(&hvx_params, 0, sizeof(hvx_params));
        hvx_params.handle = p_ltr329->temp_char_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &len;
        hvx_params.p_data = (uint8_t*)temperature_value;  

        sd_ble_gatts_hvx(p_ltr329->conn_handle, &hvx_params);
    }
}


