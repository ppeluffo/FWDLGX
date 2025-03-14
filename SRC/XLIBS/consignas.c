
#include "consignas.h"

/*
 * consigna_status es un byte que almacena el valor a setear de c/consigna
 * El bit0 representa la valvula 0
 * El bit1 reprsenta la valvula 1
 * 
 * Los valores son: 0=cerrada, 1=abierta.
 * 
 * El modulo externo de control de presión lo vamos a manejar como un 
 * dispositivo MODBUS.
 * Van a tener fija la dirección 100 (0x64)
 * Las consignas van a tener un registro de 2 bytes en la posición 1 y 2.
 * El byte HIGH va a estar siempre en 0.
 * El byte LOW va a tener un valor de 0,1,2,3 dependiendo del estado que queremos
 * poner las válvulas, y el bit 7 en 1 indica que hay una operacion en curso y en 
 * 0 que ya termino y esta en reposo.
 * 
 * Para mandar una consigna debemos armar un MODBUS_BLOCK con los datos y realizar
 * una operacion de modbus.
 * La escritura implica que pongamos el valor de las valvulas y el bit RUN en 1.
 * Luego, leyendo el registro vemos cuando termino porque el bit RUN esta en 0.
 * 
 */


bool f_debug_consigna = true;

//------------------------------------------------------------------------------
void consigna_config_defaults(void)
{
    consigna_conf.enabled = false;
    consigna_conf.consigna_diurna = 700;
    consigna_conf.consigna_nocturna = 2300;
}
//------------------------------------------------------------------------------
bool consigna_config( char *s_enable, char *s_cdiurna, char *s_cnocturna )
{
    
    //xprintf_P(PSTR("CONSIGNA DEBUG: %s,%s,%s\r\n"),s_enable,s_cdiurna,s_cnocturna);
    
    if (!strcmp_P( strupr(s_enable), PSTR("TRUE"))  ) {
        consigna_conf.enabled = true;
    }
    
    if (!strcmp_P( strupr(s_enable), PSTR("FALSE"))  ) {
        consigna_conf.enabled = false;
    }
    
    consigna_conf.consigna_diurna = atoi(s_cdiurna);
    consigna_conf.consigna_nocturna = atoi(s_cnocturna);
    
    return (true);
    
}
//------------------------------------------------------------------------------
void consigna_print_configuration(void)
{
    xprintf_P( PSTR("Consigna:\r\n"));
    xprintf_P(PSTR(" debug: "));
    f_debug_consigna ? xprintf_P(PSTR("on\r\n")) : xprintf_P(PSTR("off\r\n"));
    
    if (  ! consigna_conf.enabled ) {
        xprintf_P( PSTR(" status=disabled\r\n"));
        return;
    }
    
    xprintf_P( PSTR(" status=enabled"));
    if ( consigna_aplicada == CONSIGNA_DIURNA ) {
        xprintf_P(PSTR("(diurna)\r\n"));
    } else {
        xprintf_P(PSTR("(nocturna)\r\n"));
    }
    
	xprintf_P( PSTR(" cDiurna=%04d, cNocturna=%04d\r\n"), consigna_conf.consigna_diurna, consigna_conf.consigna_nocturna  );

}
//------------------------------------------------------------------------------
void consigna_config_debug(bool debug )
{
    if ( debug ) {
        f_debug_consigna = true;
    } else {
        f_debug_consigna = false;
    }
    
}
//------------------------------------------------------------------------------
uint8_t consigna_hash( void )
{
    
uint8_t hash = 0;
uint8_t l_hash_buffer[64];
char *p;

   // Calculo el hash de la configuracion modbus

    memset( l_hash_buffer, '\0', sizeof(l_hash_buffer) );
    if ( consigna_conf.enabled ) {
        sprintf_P( (char *)&l_hash_buffer, PSTR("[TRUE,%04d,%04d]"),consigna_conf.consigna_diurna, consigna_conf.consigna_nocturna);
    } else {
        sprintf_P( (char *)&l_hash_buffer, PSTR("[FALSE,%04d,%04d]"),consigna_conf.consigna_diurna, consigna_conf.consigna_nocturna);
    }
    p = (char *)l_hash_buffer;
    while (*p != '\0') {
        hash = u_hash(hash, *p++);
    }
    //xprintf_P(PSTR("HASH_PILOTO:%s, hash=%d\r\n"), hash_buffer, hash ); 
    return(hash);   
}
//------------------------------------------------------------------------------
bool consigna_debug_flag(void)
{
    return (f_debug_consigna);
}
//------------------------------------------------------------------------------
bool CONSIGNA_set_diurna(void)
{
    // Setea la consigna DIURNA
        
    if (f_debug_consigna) {
        xprintf_P(PSTR("CONSIGNA: Set_diurna\r\n"));
    }

    if ( cpres_consigna_set(CONSIGNA_DIURNA ) ) {
        consigna_aplicada = CONSIGNA_DIURNA;
        return(true);
    }

    return(false);
}
//------------------------------------------------------------------------------
bool CONSIGNA_set_nocturna(void)
{
    // Setea la consigna NOCTURNA

 
     if ( f_debug_consigna ) {
        xprintf_P(PSTR("CONSIGNA: Set_nocturna\r\n"));
    }

    if ( cpres_consigna_set(CONSIGNA_NOCTURNA ) ) {
        consigna_aplicada = CONSIGNA_NOCTURNA; 
        return(true);
    }

    return(false);
}
//------------------------------------------------------------------------------
int16_t consigna_save_config_in_NVM(uint16_t nvm_ptr)
{
   
int8_t retVal;
uint8_t cks;

    cks = u_checksum ( (uint8_t *)&consigna_conf, ( sizeof(consigna_conf) - 1));
    consigna_conf.checksum = cks;
    retVal = NVMEE_write( nvm_ptr, (char *)&consigna_conf, sizeof(consigna_conf) );
    xprintf_P(PSTR("SAVE NVM Consigna: memblock size = %d\r\n"), sizeof(consigna_conf));       
    if (retVal == -1 )
        return(-1);
    
    nvm_ptr += sizeof(consigna_conf);
    
    return(nvm_ptr);
   
}
//------------------------------------------------------------------------------
int16_t consigna_load_config_from_NVM( uint16_t nvm_address)
{
    /*
     * Leo la memoria, calculo el chechsum.
     * Aunque sea erroneo, copio los datos de modo que en el caso del modem
     * tenga algo para adivinar !!.
     * 
     */

uint8_t rd_cks, calc_cks;
    
    xprintf_P(PSTR("LOAD NVM Consigna: memblock size=%d\r\n"), sizeof(consigna_conf));
    memset( &consigna_conf, '\0', sizeof(consigna_conf));
    NVMEE_read( nvm_address, (char *)&consigna_conf, sizeof(consigna_conf) );
    
    rd_cks = consigna_conf.checksum;
    calc_cks = u_checksum ( (uint8_t *)&consigna_conf, ( sizeof(consigna_conf) - 1));
    
    if ( calc_cks != rd_cks ) {
		xprintf_P( PSTR("ERROR: Consigna checksum failed: calc[0x%0x], read[0x%0x]\r\n"), calc_cks, rd_cks );
        return(-1);
	} 
        
    return( nvm_address + sizeof(consigna_conf) );
}
//------------------------------------------------------------------------------