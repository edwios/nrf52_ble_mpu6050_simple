 /* 
  * This code is not extensively tested and only 
  * meant as a simple explanation and for inspiration. 
  * NO WARRANTY of ANY KIND is provided. 
  */

#ifndef BLE_LTR329_SERVICE_H__
#define BLE_LTR329_SERVICE_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "app_ltr329.h"
#include "ble_common_items.h"

#define BLE_UUID_AMBIENTLIGHT_SERVICE_UUID         0xF00E // Just a random, but recognizable value
#define BLE_UUID_VISIBLE_CHARACTERISTC_UUID        0x815B // Just a random, but recognizable value
#define BLE_UUID_IR_CHARACTERISTC_UUID             0x815C // Just a random, but recognizable value

typedef struct
{
    uint16_t                    conn_handle;    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
    uint16_t                    service_handle; /**< Handle of ble Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    visible_char_handles;   /**< Handles related to the our new characteristic. */
    ble_gatts_char_handles_t    ir_char_handles;   /**< Handles related to the our new characteristic. */
}ble_ltr329_t;

/**@brief Function for handling BLE Stack events related to LTR-329ALS service and characteristic.
 *
 * @details Handles all events from the BLE stack of interest to ltr329 Service.
 *
 * @param[in]   p_ltr329       ltr329 structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_ltr329_on_ble_evt(ble_ltr329_t * p_ltr329, ble_evt_t * p_ble_evt);

/**@brief Function for initializing our new service.
 *
 * @param[in]   p_ltr329       Pointer to ble ltr329 structure.
 */
void ble_ltr329_service_init(ble_ltr329_t * p_ltr329);

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_ltr329                 ltr329 structure.
 * @param[in]   characteristic_value     New characteristic value.
 */
void ble_ltr329_update(ble_ltr329_t *p_ltr329, ltr329_ambient_values_t * ltr329_ambient_values);

#endif  /* _ BLE_LTR329_SERVICE_H__ */
