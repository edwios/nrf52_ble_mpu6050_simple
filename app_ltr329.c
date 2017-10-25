 /*
  * The library is not extensively tested and only
  * meant as a simple explanation and for inspiration.
  * NO WARRANTY of ANY KIND is provided.
  */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "app_ltr329.h"
#include "nrf_gpio.h"
#include "nrf_drv_ltr329_twi.h"
#include "nrf_error.h"
#include "sdk_config.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


uint32_t ltr329_config(void)
{
    uint32_t err_code;
    uint8_t control_config = 0;
    uint8_t measurement_config = 0;

    NRF_LOG_DEBUG("Configurating LTR-329ALS"); NRF_LOG_FLUSH();
    control_config |= ALS_MODE_STANDBY;      // Standby mode
    control_config |= LTR329_GAIN_1X << 2;
    err_code = nrf_drv_ltr329_write_single_register(ALS_CONTR, control_config);
    if(err_code != NRF_SUCCESS) return err_code;

    measurement_config |= LTR329_RATE_500ms;
    measurement_config |= LTR329_TIME_100ms << 3;
    err_code = nrf_drv_ltr329_write_single_register(ALS_MEAS_RATE, measurement_config);
    if(err_code != NRF_SUCCESS) return err_code;

    return NRF_SUCCESS;
}



uint32_t ltr329_init(void)
{
    uint32_t err_code;
	
	// Initate TWI or SPI driver dependent on what is defined from the project
    NRF_LOG_DEBUG("Initialising LTR-329ALS"); NRF_LOG_FLUSH();
	err_code = nrf_drv_ltr329_init();
    if(err_code != NRF_SUCCESS) return err_code;

    uint8_t reset_value = 0; // Resets sensor signal paths.
    err_code = nrf_drv_ltr329_write_single_register(ALS_CONTR, reset_value);
    if(err_code != NRF_SUCCESS) return err_code;

    return NRF_SUCCESS;
}



bool ltr329_has_new_data(void)
{
    uint32_t err_code;
    uint8_t raw_value;
    err_code = nrf_drv_ltr329_read_registers(ALS_STATUS, &raw_value, 1);
    if(err_code != NRF_SUCCESS) {
        NRF_LOG_DEBUG("read error ..."); NRF_LOG_FLUSH();
        return false;
    }
    NRF_LOG_DEBUG("LTR-329ALS Status %02x", raw_value); NRF_LOG_FLUSH();
    if (!(raw_value & 0x80) && (raw_value & 0x04)) {
        return true;
    }
    return false;
}



uint32_t ltr329_read_ambient(ltr329_ambient_values_t * ltr329_ambient_values)
{
    uint32_t err_code;
    uint8_t visible_values[2];
    uint8_t ir_values[2];
    err_code = nrf_drv_ltr329_read_registers(ALS_DATA_CH1, visible_values, 2);
    if(err_code != NRF_SUCCESS) return err_code;

    uint16_t visible = (visible_values[1] << 8) + visible_values[0];

    err_code = nrf_drv_ltr329_read_registers(ALS_DATA_CH0, ir_values, 2);
    if(err_code != NRF_SUCCESS) return err_code;

    uint16_t ir = (ir_values[1] << 8) + ir_values[0];

    ltr329_ambient_values->ambient_visible_value = visible;
    ltr329_ambient_values->ambient_ir_value = ir;

    return NRF_SUCCESS;
}



uint32_t ltr329_read_partid(ltr329_part_id_t * partid)
{
    uint32_t err_code;
    uint8_t raw_values;
    err_code = nrf_drv_ltr329_read_registers(ALS_PART_ID, &raw_values, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    return NRF_SUCCESS;
}



uint32_t ltr329_config_activate(void)
{
    uint32_t err_code;
    uint8_t raw_value;

    err_code = nrf_drv_ltr329_read_registers(ALS_CONTR, &raw_value, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    raw_value |= ALS_MODE_ACTIVE;
    err_code = nrf_drv_ltr329_write_single_register(ALS_CONTR, raw_value);
    if(err_code != NRF_SUCCESS) return err_code;

    return NRF_SUCCESS;
}

uint32_t ltr329_config_deactivate(void)
{
    uint32_t err_code;
    uint8_t raw_value;

    err_code = nrf_drv_ltr329_read_registers(ALS_CONTR, &raw_value, 1);
    if(err_code != NRF_SUCCESS) return err_code;
    NRF_LOG_DEBUG("LTR-329ALS ALS_CONTR %02x", raw_value); NRF_LOG_FLUSH();

    raw_value &= ALS_MODE_INACTIVE;
    err_code = nrf_drv_ltr329_write_single_register(ALS_CONTR, raw_value);
    if(err_code != NRF_SUCCESS) return err_code;

    return NRF_SUCCESS;
}


/**
  @}
*/
