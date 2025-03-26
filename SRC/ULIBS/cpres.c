

#include "cpres.h"

/*
 status byte de consignas:
 * 
 * b7: 0-idle, 1-working
 * b6
 * b5
 * b4
 * b3,b2: v1 status: 0-open,1-close, 2-3 unknown
 * b1,b0: v0 status
 * 
 */

//------------------------------------------------------------------------------
void cpres_prender_sensor(void)
{
    /*
     * Prendo la fuente del sensor y espero 2 secs que se estabilize.
     */
    
    xprintf_P(PSTR("CPRES: prender_sensor\r\n"));

    SET_EN_PWR_CPRES();
    vTaskDelay( ( TickType_t)( 2000 / portTICK_PERIOD_MS ) );

}
//------------------------------------------------------------------------------
void cpres_apagar_sensor(void)
{
    /*
     * Apago la alimentacion del sensor
     */
    
    xprintf_P(PSTR("CPRES: apagar_sensor\r\n"));
 
    CLEAR_EN_PWR_CPRES();
    vTaskDelay( ( TickType_t)( 2000 / portTICK_PERIOD_MS ) );

}
//------------------------------------------------------------------------------
bool cpres_consigna_set( uint8_t consigna)
{
    /*
     * Implementa el algoritmo de enviar comandos modbus al sensor de presion
     * para setear la consigna indicada, esperar, y leer el status.
     */
    
bool res = false;

    xprintf_P(PSTR("CPRES: Set (%d)\r\n"), consigna);

    // Energizo el dispositivo de las consignas
    cpres_prender_sensor(); 
    // Me quedo con el bus de RS485 (modbus)
    
    rs485_ENTER_CRITICAL();
    // Despierto la tarea de RS485_RX
    rs485_AWAKE();
    // Espero 5s que todo se estabilize
    vTaskDelay( ( TickType_t)( 5000 / portTICK_PERIOD_MS ) ); 
    
    
    // Espero a estar en estado IDLE
    if ( ! cpres_sensor_is_idle() ) {
        xprintf_P(PSTR("CPRES: ERROR sensor not idle\r\n"));
        goto exit;
    }
    
    // Indico que aplique la consigna
    if ( ! cpres_sensor_write(consigna) ) {
        xprintf_P(PSTR("CPRES: ERROR write consigna\r\n"));
        goto exit;
    }   
    
    // Espero que se aplique la consigna
    vTaskDelay( ( TickType_t)( 5000 / portTICK_PERIOD_MS ) );
    
    // Espero que vuelva a estado IDLE
    if ( ! cpres_sensor_is_idle() ) {
        xprintf_P(PSTR("CPRES: ERROR sensor not idle\r\n"));
        goto exit;
    }
    
    res = true;
    
exit:
    
    rs485_SLEEP();
    rs485_EXIT_CRITICAL();
    
    cpres_apagar_sensor();
    return(res);
            
}
//------------------------------------------------------------------------------
bool cpres_sensor_is_idle(void)
{
    /*
     * Lee por modbus el sensor de control de presion hasta 3 intentos.
     * Si el status bit es 0 (IDLE) retorna True.
     * Si el status bit es 1 luego de los 3 intentos o no lo puede leer retorna False
     * 
     */
    
uint8_t size;
uint16_t crc;
uint8_t i;
uint16_t status;

    /*
     * Envio el comando modbus al sensor para leer el registro del status
     * y lo decodifico.
     * 
     * [RTU]>Tx > 14:32:05:052 - 64  03  00  01  00  01  DC  3F  
     * [RTU]>Rx > 14:32:05:075 - 64  03  02  00  01  35  8C  
     * 
     * Al recibir, el bit 7 indica en 0: standby, en 1 activo.
     * Para poder seguir mandando comandos, debe estar en 0 (standby)
     * 
     */
    
    xprintf_P(PSTR("CPRES: read_status\r\n"));

    // Preparo el CB de modbus
    memset( &consigna_mbus_cb, '\0', sizeof(mbus_CONTROL_BLOCK_t));
    
    consigna_mbus_cb.tx_buffer[0] = SENSOR_PRESION_ADDR;    // [0xFE]
    consigna_mbus_cb.tx_buffer[1] = 0x3;                    // FCODE=0x03
    
    // Starting address
    consigna_mbus_cb.tx_buffer[2] = 0x00;// DST_ADDR_H
    consigna_mbus_cb.tx_buffer[3] = 0x01;// DST_ADDR_L
    
    // Quantity of recs.
    consigna_mbus_cb.tx_buffer[4] = 0x00;
    consigna_mbus_cb.tx_buffer[5] = 0x01;
    
    size = 6;
    crc = modbus_CRC16( consigna_mbus_cb.tx_buffer, size );
    consigna_mbus_cb.tx_buffer[size++] = (uint8_t)( crc & 0x00FF );			// CRC Low
    consigna_mbus_cb.tx_buffer[size++] = (uint8_t)( (crc & 0xFF00) >> 8 );	// CRC High
    consigna_mbus_cb.tx_size = size;
    consigna_mbus_cb.io_status = false;
    
    // Esto lo requiero para la decodificación
    consigna_mbus_cb.channel.fcode = 0x03;
    consigna_mbus_cb.channel.type = u16;
    consigna_mbus_cb.channel.codec = CODEC3210;
    consigna_mbus_cb.channel.divisor_p10 = 0;
    
    for (i=0; i<3; i++) {
        // Realizo la transaccion MODBUS
        modbus_txmit_ADU( &consigna_mbus_cb );
        modbus_rcvd_ADU( &consigna_mbus_cb );
        modbus_decode_ADU ( &consigna_mbus_cb );
      
        // Análisis de resultados
        if (consigna_mbus_cb.io_status == false) {
            xprintf_P(PSTR("CPRES: read_status ERROR\r\n"));
            vTaskDelay( ( TickType_t)( 5000 / portTICK_PERIOD_MS ) );
            continue;    
        }
                
        status = consigna_mbus_cb.udata.u16_value; 
        if ( word_isBitSet(status, STATUS_BIT )) {
            xprintf_P(PSTR("CPRES: read_status (busy) OK(%u)[0x%02x]"), status, status);  
            xprintf_P(PSTR(BYTE_TO_BINARY_PATTERN), BYTE_TO_BINARY(status) );
            xprintf_P(PSTR("\r\n"));
        } else {
            xprintf_P(PSTR("CPRES: read_status (idle) OK(%u)[0x%02x]"), status, status);    
            xprintf_P(PSTR(BYTE_TO_BINARY_PATTERN), BYTE_TO_BINARY(status) );
            xprintf_P(PSTR("\r\n"));
            return(true);
        }
        vTaskDelay( ( TickType_t)( 5000 / portTICK_PERIOD_MS ) );
        
    }
    
    return(false);
}
//------------------------------------------------------------------------------
bool cpres_sensor_write( uint8_t consigna)
{
    /*
     * Envio el comando modbus 0x6 al sensor para escribir la orden de poner
     * la consigna
     * 
     * En resp. enviamos el status.(con el bit en working)
     * [RTU]>Tx > 14:33:18:797 - 64  06  00  01  00  05  11  FC  
     * [RTU]>Rx > 14:33:18:822 - 64  06  00  01  00  01  10  3F  
     * 
     * [RTU]>Tx > 14:33:29:625 - 64  06  00  01  00  06  51  FD  
     * [RTU]>Rx > 14:33:29:663 - 64  06  00  01  00  04  D0  3C 
     */
    
uint8_t size;
uint16_t crc;
uint8_t i;

    xprintf_P(PSTR("CPRES: write %d\r\n"), consigna);

    // Preparo el memblock modbus
    memset( &consigna_mbus_cb, '\0', sizeof(mbus_CONTROL_BLOCK_t));
    
    consigna_mbus_cb.tx_buffer[0] = SENSOR_PRESION_ADDR;    // 
    consigna_mbus_cb.tx_buffer[1] = 0x6;                    // FCODE=0x06
    
    // Starting address
    consigna_mbus_cb.tx_buffer[2] = 0x00;// DST_ADDR_H
    consigna_mbus_cb.tx_buffer[3] = 0x01;		 // DST_ADDR_L
    
    // Register Value
    switch (consigna ) {
        case CONSIGNA_DIURNA:
            consigna_mbus_cb.tx_buffer[4] = 0x00;
            consigna_mbus_cb.tx_buffer[5] = 0x05;
            break;
        case CONSIGNA_NOCTURNA:
            consigna_mbus_cb.tx_buffer[4] = 0x00;
            consigna_mbus_cb.tx_buffer[5] = 0x06;
            break;
        default:
            xprintf_P(PSTR("CPRES: write ERROR\r\n"));
            break;
    }
    
    size = 6;
    crc = modbus_CRC16( consigna_mbus_cb.tx_buffer, size );
    consigna_mbus_cb.tx_buffer[size++] = (uint8_t)( crc & 0x00FF );			// CRC Low
    consigna_mbus_cb.tx_buffer[size++] = (uint8_t)( (crc & 0xFF00) >> 8 );	// CRC High
    consigna_mbus_cb.tx_size = size;
    consigna_mbus_cb.io_status = false;

    consigna_mbus_cb.channel.fcode = 0x06;
    consigna_mbus_cb.channel.type = u16;
    consigna_mbus_cb.channel.codec = CODEC3210;
    consigna_mbus_cb.channel.divisor_p10 = 0;
    
    // Realizo la transaccion MODBUS
    for (i=0; i<3; i++) {
        modbus_txmit_ADU( &consigna_mbus_cb );
        modbus_rcvd_ADU( &consigna_mbus_cb );
        modbus_decode_ADU ( &consigna_mbus_cb );
        //
        if (consigna_mbus_cb.io_status == false) {
            xprintf_P(PSTR("CPRES: write ERROR\r\n"));
            vTaskDelay( ( TickType_t)( 5000 / portTICK_PERIOD_MS ) );
            continue;    
        } else {
            xprintf_P(PSTR("CPRES: write OK\r\n"));
            return(true);
        }
    }
    return(false);
            
}
//------------------------------------------------------------------------------
