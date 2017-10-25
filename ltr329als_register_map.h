 /* 
  * The library is not extensively tested and only 
  * meant as a simple explanation and for inspiration. 
  * NO WARRANTY of ANY KIND is provided. 
  */

#ifndef LTR329ALS_REG_MAP_H__
#define LTR329ALS_REG_MAP_H__

#define ALS_CONTR 		0x80
#define ALS_MEAS_RATE	0x85
#define ALS_PART_ID			0x86	// R/O
#define ALS_MANUFAC_ID		0x87	// R/O
#define ALS_DATA_CH1	0x88	// R/O 2 bytes
#define ALS_DATA_CH0	0x8A	// R/O 2 bytes
#define ALS_STATUS		0x8C	// R/O

#endif /* LTR329ALS_REG_MAP_H__ */
