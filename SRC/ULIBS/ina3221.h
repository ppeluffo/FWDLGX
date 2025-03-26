/*
 * l_ina3223.h
 *
 *  Created on: 8 dic. 2018
 *      Author: pablo
 */

#ifndef SRC_SPX_LIBS_L_INA3221_H_
#define SRC_SPX_LIBS_L_INA3221_H_

#include "frtos-io.h"
#include "stdint.h"
#include "i2c.h"
#include "xprintf.h"

//------------------------------------------------------------------------------------

#define INA0	0
#define INA1	1

// WORDS de configuracion de los INAs
#define CONF_INA_SLEEP			0x7920
#define CONF_INA_AVG128         0x7927
#define CONF_INA_PWRDOWN		0x0000


// Direcciones de los registros de los INA
#define INA3231_CONF			0x00
#define INA3221_CH1_SHV			0x01
#define INA3221_CH1_BUSV		0x02
#define INA3221_CH2_SHV			0x03
#define INA3221_CH2_BUSV		0x04
#define INA3221_CH3_SHV			0x05
#define INA3221_CH3_BUSV		0x06
#define INA3221_MFID			0xFE
#define INA3221_DIEID			0xFF

#define INA3221_VCC_SETTLE_TIME	500


//------------------------------------------------------------------------------------
// API publica
bool INA_config( uint8_t ina_id, uint16_t conf_reg_value );
bool INA_sleep(void);
bool INA_awake(void);
//
int16_t INA_read( uint8_t ina_id, uint16_t rdAddress, char *data, uint8_t length, bool debug );
int16_t INA_write( uint8_t ina_id, uint16_t wrAddress, char *data, uint8_t length, bool debug );
//
int16_t INA_test_write ( char *s_ina_id, char *rconf_val_str );
int16_t INA_test_read ( char *s_ina_id, char *regs );
//
// API END
//------------------------------------------------------------------------------


#endif /* SRC_SPX_LIBS_L_INA3221_H_ */
