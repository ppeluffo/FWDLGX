/* 
 * File:   modbus.h
 * Author: pablo
 *
 * Created on 6 de marzo de 2023, 09:35 AM
 */

#ifndef MODBUS_H
#define	MODBUS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
    
#include "frtos-io.h"
#include "stdint.h"
#include "xprintf.h"
#include "linearBuffer.h"
#include "utils.h"
#include "rs485.h"

#define MODBUS_RX_BUFFER_SIZE 64
char modbus_rx_buffer[MODBUS_RX_BUFFER_SIZE];
lBuffer_s modbus_rx_lbuffer;

#define MODBUS_PARAMNAME_LENGTH     12
#define NRO_MODBUS_CHANNELS         5
    
typedef enum { u16=0,i16,u32,i32,FLOAT } t_modbus_types;
typedef enum { CODEC0123, CODEC1032, CODEC3210, CODEC2301 } t_modbus_codec;

/*
 * En modbus leemos de a 1 canal, no bloques !!
 * Los canales pueden ser holding_registers ( 0x03 ) o input_registers (0x04).
 * Cada registro son 2 bytes por lo que siempre leemos 2 x N.
 * N: nro_recds.
 * En tipo ( 'F','I' ) indicamos como los interpretamos.
 * En divisor_10 indicamos el factor por el que multiplicamos para tener la magnitud.
 *
 */

typedef struct {
    bool enabled;
    char name[MODBUS_PARAMNAME_LENGTH];
    uint8_t slave_address;		// Direccion del dispositivo en el bus.
    uint16_t reg_address;		// Direccion de la posicion de memoria donde leer.
    uint8_t nro_regs; 			// Cada registro son 2 bytes por lo que siempre leemos 2 x N.
    uint8_t fcode;
	t_modbus_types type;		// Indica si es entero con o sin signo, float, etc
	t_modbus_codec codec;		// Indica el orden de los bytes en numeros de mas de 1 byte
	uint8_t divisor_p10;		// factor de potencia de 10 por el que dividimos para tener la magnitud
	
} modbus_channel_conf_t;

typedef struct {
    bool enabled;
    uint8_t localaddr;
    modbus_channel_conf_t mbch[NRO_MODBUS_CHANNELS];
    uint8_t checksum;
} modbus_conf_t;

modbus_conf_t modbus_conf;

/*
 * Los registros de modbus pueden ser enteros, long o float.
 * Recibo un string en modo BIGENDIAN que lo guardo en raw_value.
 * De acuerdo a lo que lea es como queda interpretado
 */
typedef union {
	uint8_t raw_value[4];
	// Interpretaciones
	uint16_t u16_value;
	int16_t i16_value;
	uint32_t u32_value;
	int32_t i32_value;
	float float_value;
	char str_value[4];		// Almaceno NaN cuando hay un error.
} modbus_hold_t; // (4)

#define MBUS_TXMSG_LENGTH	16
#define MBUS_RXMSG_LENGTH	32

typedef struct {
	modbus_channel_conf_t channel;
	uint8_t tx_buffer[MBUS_TXMSG_LENGTH];
	uint8_t rx_buffer[MBUS_RXMSG_LENGTH];
	uint8_t tx_size;		// Cantidad de bytes en el txbuffer para transmitir
	uint8_t rx_size;		// Cantidad de bytes leidos en el rxbufer.
	uint8_t payload_ptr;	// Indice al primer byte del payload.
	modbus_hold_t udata;	// estructura para interpretar los distintos formatos de los datos.
	bool io_status;			// Flag que indica si la operacion de lectura fue correcta o no
} mbus_CONTROL_BLOCK_t;


void modbus_config_debug(bool debug );
bool modbus_read_debug(void);
void modbus_config_defaults(void);
void modbus_print_configuration(void);
int16_t modbus_save_config_in_NVM(uint16_t nvm_ptr);
int16_t modbus_load_config_from_NVM( uint16_t nvm_address);

bool modbus_config_channel(uint8_t ch, 
        char *s_enable, 
        char *s_name, 
        char *s_sla, 
        char *s_regaddr, 
        char *s_nroregs,
        char *s_fcode,
        char *s_mtype,
        char *s_codec,
        char *s_pow10 );

bool modbus_config_enable(char *s_enable);
bool modbus_config_localaddr( char *s_localaddr);

void modbus_txmit_ADU( mbus_CONTROL_BLOCK_t *mbus_cb );
void modbus_rcvd_ADU( mbus_CONTROL_BLOCK_t *mbus_cb );
void modbus_make_ADU( mbus_CONTROL_BLOCK_t *mbus_cb );
void modbus_decode_ADU ( mbus_CONTROL_BLOCK_t *mbus_cb );
void modbus_io( mbus_CONTROL_BLOCK_t *mbus_cb );

void modbus_read( float modbus_values[] );
float modbus_read_channel ( uint8_t ch );

uint8_t modbus_hash(void);
uint16_t modbus_CRC16( uint8_t *msg, uint8_t msg_size );;
bool modbus_test(char **argv);


#ifdef	__cplusplus
}
#endif

#endif	/* MODBUS_H */

