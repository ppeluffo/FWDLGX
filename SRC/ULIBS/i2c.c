/*
 * l_i2c.c
 *
 *  Created on: 26 de mar. de 2018
 *      Author: pablo
 */

#include "i2c.h"

char buffer[10] = { 0 };

//------------------------------------------------------------------------------
void I2C_init(void)
{
}
//------------------------------------------------------------------------------
int16_t I2C_write ( int fdTWI, uint8_t devAddress, 
        uint16_t dataAddress, 
        uint8_t dataAddress_length, 
        char *data, 
        uint8_t data_length, bool i2c_debug )
{
	

int16_t xReturn = 0U;
uint8_t i2c_error_code = 0;
   

    if ( i2c_debug ) {
        xprintf_P(PSTR("I2C_write: devAddress=0x%02X, dataAddress=0x%02X, dataAddressLength=%d, dataLength=%d\r\n"), devAddress,dataAddress,dataAddress_length,data_length);
        xprintf_P(PSTR("I2C_write: d[0]=%0x%02x, d[1]=0x%02x\r\n") , data[0], data[1]);
        xprintf_P(PSTR("I2C_write: debug on\r\n"));
    }

    frtos_ioctl( fdTWI, ioctl_OBTAIN_BUS_SEMPH, NULL);
    
    // Prendo el debug
    if ( i2c_debug ) {
        xprintf_P(PSTR("I2C_write: ioctl set debug on\r\n"));
        frtos_ioctl( fdTWI, ioctl_I2C_SET_DEBUG, NULL);
    }
    
	// 1) Indicamos el periferico i2c en el cual queremos escribir ( variable de 8 bits !!! )
	frtos_ioctl( fdTWI, ioctl_I2C_SET_DEVADDRESS, &devAddress );

	// 2) Indicamos al direccion interna del chip donde comenzar a escribir
	frtos_ioctl( fdTWI, ioctl_I2C_SET_DATAADDRESS, &dataAddress );
	frtos_ioctl( fdTWI, ioctl_I2C_SET_DATAADDRESSLENGTH, &dataAddress_length );

	// 3) Por ultimo escribimos. No controlo fronteras.
	xReturn = frtos_write( fdTWI, data, data_length);

	// 4) Controlo errores
	i2c_error_code = frtos_ioctl( fdTWI, ioctl_I2C_GET_LAST_ERROR, NULL );
	if (i2c_error_code != I2C_OK ) {
        if ( i2c_debug ) 
            xprintf_P(PSTR("I2C_write ERROR 0x%02X. Exit\r\n"),i2c_error_code);
		xReturn = -1;
	}
    
    // Apago el debug
    if ( i2c_debug ) {
        xprintf_P(PSTR("I2C_write: ioctl set debug off\r\n"));
        frtos_ioctl( fdTWI, ioctl_I2C_CLEAR_DEBUG, NULL);
    }
    
    frtos_ioctl( fdTWI, ioctl_RELEASE_BUS_SEMPH, NULL);
    
	return(xReturn);
}
//------------------------------------------------------------------------------
int16_t I2C_read  ( int fdTWI, uint8_t devAddress, 
        uint16_t dataAddress, 
        uint8_t dataAddress_length, 
        char *data, 
        uint8_t data_length, bool i2c_debug )
{

	// Implementa solo la parte de lectura del ciclo.
	// La primera parte que es solo escribir la direccion de donde leer la hacemos
	// con I2C_write_R1. ( Dummy Write )
    
int16_t xReturn = 0U;
uint8_t i2c_error_code = 0;

    if ( i2c_debug ) {
        xprintf_P(PSTR("I2C_read: devAddress=0x%02X, dataAddress=0x%02X, dataAddressLength=%d, dataLength=%d\r\n"), devAddress,dataAddress,dataAddress_length,data_length);
    }

    frtos_ioctl( fdTWI, ioctl_OBTAIN_BUS_SEMPH, NULL);

    // Prendo el debug
    if ( i2c_debug ) {
        frtos_ioctl( fdTWI, ioctl_I2C_SET_DEBUG, NULL);
    }
    
	// 1) Indicamos el periferico i2c en el cual queremos escribir ( variable de 8 bits !!! )
	frtos_ioctl( fdTWI, ioctl_I2C_SET_DEVADDRESS, &devAddress );

	// 2) Indicamos al direccion interna del chip donde comenzar a escribir
	frtos_ioctl( fdTWI, ioctl_I2C_SET_DATAADDRESS, &dataAddress );
	frtos_ioctl( fdTWI, ioctl_I2C_SET_DATAADDRESSLENGTH, &dataAddress_length );

 	// 3) Leemos. No controlo fronteras.
	xReturn = frtos_read( fdTWI, data, data_length);

 	// 4) Controlo errores.
	i2c_error_code = frtos_ioctl( fdTWI, ioctl_I2C_GET_LAST_ERROR, NULL );
	if (i2c_error_code != I2C_OK ) {
        if ( i2c_debug ) 
            xprintf_P(PSTR("I2C_read ERROR 0x%02X. Exit\r\n"),i2c_error_code);
		xReturn = -1;
	}

	if (xReturn != data_length ) {
		xReturn = -1;
	}

    // Apago el debug
    if ( i2c_debug ) 
        frtos_ioctl( fdTWI, ioctl_I2C_CLEAR_DEBUG, NULL);
    
    frtos_ioctl( fdTWI, ioctl_RELEASE_BUS_SEMPH, NULL);
    
	return(xReturn);

}
//------------------------------------------------------------------------------
