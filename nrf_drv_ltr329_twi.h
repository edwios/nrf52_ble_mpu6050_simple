 /* 
  * The library is not extensively tested and only 
  * meant as a simple explanation and for inspiration. 
  * NO WARRANTY of ANY KIND is provided. 
  */

#ifndef NRF_DRV_LTR329__
#define NRF_DRV_LTR329__


#include <stdbool.h>
#include <stdint.h>



/**@brief Function to initiate TWI drivers
 *
 * @retval      uint32_t        Error code
 */
uint32_t nrf_drv_ltr329_init(void);
	


/**@brief Function for reading an arbitrary register
 *
 * @param[in]   reg             Register to write
 * @param[in]   data            Value
 * @retval      uint32_t        Error code
 */
uint32_t nrf_drv_ltr329_write_single_register(uint8_t reg, uint8_t data);



/**@brief Function for reading arbitrary register(s)
 *
 * @param[in]   reg             Register to read
 * @param[in]   p_data          Pointer to place to store value(s)
 * @param[in]   length          Number of registers to read
 * @retval      uint32_t        Error code
 */
uint32_t nrf_drv_ltr329_read_registers(uint8_t reg, uint8_t * p_data, uint32_t length);
    

#endif /* NRF_DRV_LTR329__ */

/**
  @}
*/

