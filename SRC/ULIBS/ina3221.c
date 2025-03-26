/*
 * ina3232.c
 *
 *  Created on: 8 dic. 2018
 *      Author: pablo
 */

#include "ina3221.h"

uint8_t INA_id2devaddr( uint8_t ina_id );

//------------------------------------------------------------------------------
uint8_t INA_id2devaddr( uint8_t ina_id )
{
	switch(ina_id) {
	case INA0:
		// ina_U1 en SPX_5CH. Canales 0,1,2
		return(DEVADDRESS_INA0);
		break;
	case INA1:
		// ina_U2 en SPX_5CH. Canales 3,4,5
		return(DEVADDRESS_INA1);
		break;
	default:
		return(99);
		break;
	}

	return(99);

}
//------------------------------------------------------------------------------
bool INA_sleep(void)
{
  
bool ret = false;
    
    //xprintf_P(PSTR("DEBUG: INA_SLEEP\r\n"));

#ifdef HW_AVRDA
    ret = INA_config(INA1, CONF_INA_SLEEP);
#endif
    
#ifdef HW_XMEGA
    ret = INA_config(INA0, CONF_INA_SLEEP);
    if (ret) 
        ret = INA_config(INA1, CONF_INA_SLEEP);
#endif
    
    return(ret);
    
}
//------------------------------------------------------------------------------
bool INA_awake(void)
{
bool ret = false;
 
     //xprintf_P(PSTR("DEBUG: INA_AWAKE\r\n"));
     
#ifdef HW_AVRDA
    ret = INA_config(INA1, CONF_INA_AVG128);
#endif
    
#ifdef HW_XMEGA
    ret = INA_config(INA0, CONF_INA_AVG128);
    if (ret) 
        ret = INA_config(INA1, CONF_INA_AVG128);
#endif
    
    return(ret);
    
}
//------------------------------------------------------------------------------
bool INA_config( uint8_t ina_id, uint16_t conf_reg_value )
{
char res[3] = { 0 };
int16_t xBytes = 0;

	res[0] = ( conf_reg_value & 0xFF00 ) >> 8;
	res[1] = ( conf_reg_value & 0x00FF );

    //xprintf_P(PSTR("DEBUG: INA_config res0=0x%02x, res1=0x%02x, ina_id=%d\r\n"), res[0], res[1], ina_id);
	xBytes = INA_write( ina_id, INA3231_CONF, res, 2, false );
	if ( xBytes == -1 ) {
		xprintf("ERROR: I2C:INA_config\r\n");
        return (false);
    }
    return (true);

}
//------------------------------------------------------------------------------
int16_t INA_test_write ( char *s_ina_id, char *rconf_val_str )
{

	// Escribe en el registro de configuracion de un INA ( 0, 1, 2)

uint16_t val = 0;
char data[3] = { 0 };
int16_t xBytes = 0;
uint8_t ina_id = atoi(s_ina_id);

#ifdef HW_AVRDA
    ina_id = 1;
#endif

    //INA_awake();
    
    if ((strcmp_P( rconf_val_str, PSTR("AWAKE")) == 0) ) {
        INA_awake();
        return(1);
    }

    if ((strcmp_P( rconf_val_str, PSTR("SLEEP")) == 0) ) {
        INA_sleep();
        return(1);
    }
    
	val = atoi( rconf_val_str);
	data[0] = ( val & 0xFF00 ) >> 8;
	data[1] = ( val & 0x00FF );
    
    //xprintf_P(PSTR("DEBUG: data=[0x%02x][0x%02x], val=%d, sval=%s\r\n"), data[0], data[1], val, rconf_val_str);
    
	xBytes = INA_write( ina_id, INA3231_CONF, data, 2, false );
	if ( xBytes == -1 )
		xprintf("ERROR: I2C:INA_test_write\r\n");

    //INA_sleep();
    
	return (xBytes);
}
//------------------------------------------------------------------------------
int16_t INA_test_read ( char *s_ina_id, char *regs )
{

uint16_t val = 0;
char data[3] = { ' ' };
int16_t xBytes = 0;
char l_data[10] = { ' ' };
uint8_t ina_id = atoi(s_ina_id);

	memcpy(l_data, regs, sizeof(l_data));
	strupr(l_data);

    // Awake
    //if ( ! INA_awake() ) {
    //    return (-1);
    //}

#ifdef HW_AVRDA
    ina_id = 1;
#endif
            
    //INA_config_avg128(INA_A);
    
	// read ina id {conf|chxshv|chxbusv|mfid|dieid}
	if (strcmp( l_data, "CONF") == 0 ) {
		xBytes = INA_read( ina_id, INA3231_CONF, data, 2, false );
	} else if (strcmp( l_data, "CH1SHV") == 0) {
		xBytes = INA_read( ina_id, INA3221_CH1_SHV, data, 2, false );
	} else if (strcmp( l_data,"CH1BUSV\0") == 0) {
		xBytes = INA_read( ina_id, INA3221_CH1_BUSV, data, 2, false );
	} else if (strcmp( l_data, "CH2SHV\0") == 0) {
		xBytes = INA_read( ina_id, INA3221_CH2_SHV, data, 2, false );
	} else if (strcmp( l_data, "CH2BUSV\0") == 0) {
		xBytes = INA_read( ina_id, INA3221_CH2_BUSV, data, 2, false );
	} else if (strcmp( l_data, "CH3SHV\0") == 0) {
		xBytes = INA_read( ina_id, INA3221_CH3_SHV, data, 2, false );
	} else if (strcmp( l_data, "CH3BUSV\0") == 0) {
		xBytes = INA_read( ina_id, INA3221_CH3_BUSV, data, 2, false );
	} else if (strcmp( l_data, "MFID\0") == 0) {
		xBytes = INA_read( ina_id,  INA3221_MFID, data, 2, false );
	} else if (strcmp( l_data, "DIEID\0") == 0) {
		xBytes = INA_read( ina_id, INA3221_DIEID, data, 2, false );
	} else {
		xBytes = -1;
	}

	if ( xBytes == -1 ) {
		xprintf("ERROR: I2C:INA_test_read\r\n\0");

	} else {

		val = ( data[0]<< 8 ) + data[1];
		xprintf("INA_VAL=0x%04x\r\n\0", val);
	}

    // Sleep
    //if ( ! INA_sleep() ) {
    //    return(-1);
    //}
    //INA_config_sleep(INA_A);
    
	return(xBytes);

}
//------------------------------------------------------------------------------
int16_t INA_read( uint8_t ina_id, uint16_t rdAddress, char *data, uint8_t length, bool debug )
{

int16_t rcode = 0;

    if (debug) {
        xprintf_P(PSTR("INA_read.\r\n"));
    }

#ifdef HW_AVRDA
    ina_id = 1;
#endif
    
	rcode =  I2C_read( fdI2C, INA_id2devaddr(ina_id), rdAddress, 0x01, data, length, debug );
	if ( rcode == -1 ) {
		// Hubo error: trato de reparar el bus y reintentar la operacion
		// Espero 1s que se termine la fuente de ruido.
		vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
		// Reconfiguro los dispositivos I2C del bus que pueden haberse afectado
		xprintf("ERROR: INA_read recovering i2c bus\r\n" );
		goto quit;
	}

quit:

	return( rcode );

}
//------------------------------------------------------------------------------
int16_t INA_write( uint8_t ina_id, uint16_t wrAddress, char *data, uint8_t length, bool debug )
{

int16_t rcode = 0;

    //debug = true;
    
    if (debug) {
        xprintf_P(PSTR("INA_write.\r\n"));
    }

#ifdef HW_AVRDA
    ina_id = 1;
#endif
        
 	rcode =  I2C_write ( fdI2C, INA_id2devaddr(ina_id), wrAddress, 0x01, data, length, debug );
	if ( rcode == -1 ) {
		// Hubo error: trato de reparar el bus y reintentar la operacion
		// Espero 1s que se termine la fuente de ruido.
		vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
		// Reconfiguro los dispositivos I2C del bus que pueden haberse afectado
		xprintf("ERROR: INA_write recovering i2c bus\r\n" );
	}

	return( rcode );

}
//------------------------------------------------------------------------------
