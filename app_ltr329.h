 /* 
  * The library is not extensively tested and only 
  * meant as a simple explanation and for inspiration. 
  * NO WARRANTY of ANY KIND is provided. 
  */

#ifndef APP_LTR329_H__
#define APP_LTR329_H__


#include <stdbool.h>
#include <stdint.h>
#include "sdk_config.h"
#include "ltr329als_register_map.h"

#define ALS_MODE_STANDBY 0
#define ALS_MODE_ACTIVE 1

/**
 * Controls the range and resolution of illuminance values
 */
typedef enum {
    LTR329_GAIN_1X= 0,   ///< Illuminance range between [1, 64k] lux (default)
    LTR329_GAIN_2X,      ///< Illuminance range between [0.5, 32k] lux
    LTR329_GAIN_4X,      ///< Illuminance range between [0.25, 16k] lux
    LTR329_GAIN_8X,      ///< Illuminance range between [0.125, 8k] lux
    LTR329_GAIN_48X,     ///< Illuminance range between [0.02, 1.3k] lux
    LTR329_GAIN_96X      ///< Illuminance range between [0.01, 600] lux
} ltr329Gain;

/**
 * Measurement time for each cycle
 */
typedef enum {
    LTR329_TIME_100ms= 0,    ///< Default setting
    LTR329_TIME_50ms,
    LTR329_TIME_200ms,
    LTR329_TIME_400ms,
    LTR329_TIME_150ms,
    LTR329_TIME_250ms,
    LTR329_TIME_300ms,
    LTR329_TIME_350ms
} ltr329IntegrationTime;

/**
 * How frequently to update the illuminance data.
 */
typedef enum {
    LTR329_RATE_50ms= 0,
    LTR329_RATE_100ms,
    LTR329_RATE_200ms,
    LTR329_RATE_500ms,       ///< Default setting
    LTR329_RATE_1000ms,
    LTR329_RATE_2000ms
} ltr329MeasurementRate;

/**@brief Simple typedef to hold ambient values */
typedef struct {
    int16_t ambient_visible_value;
    int16_t ambient_ir_value;
} ltr329_ambient_values_t;

/**@brief Control register structure */
typedef struct {
    uint8_t als_mode    :1;
    uint8_t sw_reset    :1;
    uint8_t als_gain    :3;
    uint8_t reserved    :3;
} ltr329_als_contr_t;

/**@brief Measurement rate register structure */
typedef struct {
    uint8_t als_measurement_repeat_rate :3;
    uint8_t als_integration_time        :3;
    uint8_t reserved                    :2;
} ltr329_als_meas_rate_t;

/**@brief Part ID register structure */
typedef struct {
    uint8_t revision_id     :4;
    uint8_t part_number_id  :4;
} ltr329_part_id_t;

/**@brief Status register structure */
typedef struct {
    uint8_t reserved        :2;
    uint8_t als_data_status :1;
    uint8_t reserved2       :1;
    uint8_t als_gain        :3;
    uint8_t als_data_valid  :1;
} ltr329_als_status_t;


/**@brief Function for initiating LTR-329ALS library
 * 
 * Resets sensor signal paths.
 * Function resets the analog and digital signal paths of the light sensor,
 * and temperature sensors.
 * The reset will revert the signal path analog to digital converters and filters to their power up
 * configurations.
 *
 * @retval      uint32_t        Error code
 */
uint32_t ltr329_init(void);

    

/**@brief Function for basic configuring of the LTR-329ALS
 * 
 * Register 80 � Control Register. This register specifies 
 * the ALS operation mode and software reset.
 * 
 * Register 85 � Configuration CONFIG. This register configures 
 * the LTR-329ALS for different measurement rates 
 *
 * @param[in]   config          Pointer to configuration structure
 * @retval      uint32_t        Error code
 */
uint32_t ltr329_config(void);

/**@brief Function for reading LTR-329ALS part id data.
 *
 * @param[in]   partid_values   Pointer to variable to hold part id data
 * @retval      uint32_t        Error code
 */
uint32_t ltr329_read_partid(ltr329_part_id_t * partid);

/**@brief Function for reading LTR-329ALS ambient data.
 *
 * @param[in]   ambient_values  Pointer to variable to hold ambient data
 * @retval      uint32_t        Error code
 */
uint32_t ltr329_read_ambient(ltr329_ambient_values_t * ltr329_ambient_values);


/**@brief Function for activating LTR-329ALS .
 *
 * @param[in]   
 * @retval      uint32_t        Error code
 */
uint32_t ltr329_config_activate(void);

/**@brief Function for detecting LTR-329ALS new data.
 *
 * @param[in]   
 * @retval      bool        True if new data is avaiable
 */
bool ltr329_has_new_data(void);


#endif /* APP_LTR329_H__ */

/**
  @}
*/

