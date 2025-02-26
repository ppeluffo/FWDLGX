
#include "base.h"

//------------------------------------------------------------------------------
void base_config_defaults( void )
{
    
    memcpy(base_conf.dlgid, "DEFAULT\0", sizeof(base_conf.dlgid));
    
    base_conf.timerdial = 0;
    base_conf.timerpoll = 60;
    
    base_conf.pwr_modo = PWR_CONTINUO;
    base_conf.pwr_hhmm_on = 2330;
    base_conf.pwr_hhmm_off = 630;
  
}
//------------------------------------------------------------------------------
void base_print_configuration( void )
{
 
    /*
     * Muestra en pantalla el modo de energia configurado
     */
    
uint16_t hh, mm;

    xprintf_P(PSTR("Base:\r\n"));
    xprintf_P(PSTR(" dlgid: %s\r\n"), base_conf.dlgid );       
    xprintf_P(PSTR(" timerdial=%d\r\n"), base_conf.timerdial);
    xprintf_P(PSTR(" timerpoll=%d\r\n"), base_conf.timerpoll);

    switch( base_conf.pwr_modo ) {
        case PWR_CONTINUO:
            xprintf_P(PSTR(" pwr_modo: continuo\r\n"));
            break;
        case PWR_DISCRETO:
            xprintf_P(PSTR(" pwr_modo: discreto (%d s)\r\n"), base_conf.timerdial);
            break;
        case PWR_MIXTO:
            xprintf_P(PSTR(" pwr_modo: mixto\r\n"));
            hh = (uint8_t)(base_conf.pwr_hhmm_on / 100);
            mm = (uint8_t)(base_conf.pwr_hhmm_on % 100);
            xprintf_P(PSTR("    inicio continuo -> %02d:%02d\r\n"), hh,mm);
            
            hh = (uint8_t)(base_conf.pwr_hhmm_off / 100);
            mm = (uint8_t)(base_conf.pwr_hhmm_off % 100);
            xprintf_P(PSTR("    inicio discreto -> %02d:%02d\r\n"), hh,mm);
            break;
    }
    
    xprintf_P(PSTR(" pwr_on:%d, pwr_off:%d\r\n"),base_conf.pwr_hhmm_on, base_conf.pwr_hhmm_off );
    
}
//------------------------------------------------------------------------------
bool base_config_timerdial ( char *s_timerdial )
{
	// El timer dial puede ser 0 si vamos a trabajar en modo continuo o mayor a
	// 15 minutos.
	// Es una variable de 32 bits para almacenar los segundos de 24hs.

uint16_t l_timerdial;
    
    l_timerdial = atoi(s_timerdial);
    if ( (l_timerdial > 0) && (l_timerdial < TDIAL_MIN_DISCRETO ) ) {
        xprintf_P( PSTR("TDIAL warn: continuo TDIAL=0, discreto TDIAL >= 900)\r\n"));
        l_timerdial = TDIAL_MIN_DISCRETO;
    }
    
	base_conf.timerdial = atoi(s_timerdial);
	return(true);    
}
//------------------------------------------------------------------------------
bool base_config_timerpoll ( char *s_timerpoll )
{
	// Configura el tiempo de poleo.
	// Se utiliza desde el modo comando como desde el modo online
	// El tiempo de poleo debe estar entre 15s y 3600s


	base_conf.timerpoll = atoi(s_timerpoll);

	if ( base_conf.timerpoll < 15 )
		base_conf.timerpoll = 15;

	if ( base_conf.timerpoll > 3600 )
		base_conf.timerpoll = 3600;

	return(true);
}
//------------------------------------------------------------------------------    
bool base_config_dlgid ( char *s_dlgid )
{
	if ( s_dlgid == NULL ) {
		return(false);
	} else;
   
    memset(base_conf.dlgid,'\0', sizeof(base_conf.dlgid) );
	memcpy(base_conf.dlgid, s_dlgid, sizeof(base_conf.dlgid));
	base_conf.dlgid[DLGID_LENGTH - 1] = '\0';
	return(true);    
}
//------------------------------------------------------------------------------  
bool base_config_pwrmodo ( char *s_pwrmodo )
{
    
    if ((strcmp_P( strupr(s_pwrmodo), PSTR("CONTINUO")) == 0) ) {
        base_conf.pwr_modo = PWR_CONTINUO;
        return(true);
    }
    
    if ((strcmp_P( strupr(s_pwrmodo), PSTR("DISCRETO")) == 0) ) {
        base_conf.pwr_modo = PWR_DISCRETO;
        return(true);
    }
    
    if ((strcmp_P( strupr(s_pwrmodo), PSTR("MIXTO")) == 0) ) {
        base_conf.pwr_modo = PWR_MIXTO;
        return(true);
    }
    
    return(false);    
}
//------------------------------------------------------------------------------
bool base_config_pwron ( char *s_pwron )
{
    base_conf.pwr_hhmm_on = atoi(s_pwron);
    return(true);    
}
//------------------------------------------------------------------------------
bool base_config_pwroff ( char *s_pwroff )
{
    base_conf.pwr_hhmm_off = atoi(s_pwroff);
    return(true);    
}
//------------------------------------------------------------------------------
int16_t base_save_config_in_NVM(uint16_t nvm_ptr)
{
   
int8_t retVal;
uint8_t cks;


    // AINPUTS
    cks = u_checksum ( (uint8_t *)&base_conf, ( sizeof(base_conf) - 1));
    base_conf.checksum = cks;
    //xprintf_P(PSTR("DEBUG: SAVE NVM Base. cks=%d, nvm_ptr=%d\r\n"), cks, nvm_ptr);   
    retVal = NVMEE_write( nvm_ptr, (char *)&base_conf, sizeof(base_conf) );
    xprintf_P(PSTR("SAVE NVM Base: memblock size = %d\r\n"), sizeof(base_conf));       
    if (retVal == -1 ) {
        xprintf_P(PSTR("ERROR: SAVE NVM Base ret=-1\r\n"));     
        return(-1);
    }
    
    nvm_ptr += sizeof(base_conf);
    
    return(nvm_ptr);
   
}
//------------------------------------------------------------------------------
int16_t base_load_config_from_NVM( uint16_t nvm_address)
{
    /*
     * Leo la memoria, calculo el chechsum.
     * Aunque sea erroneo, copio los datos de modo que en el caso del modem
     * tenga algo para adivinar !!.
     * 
     */

uint8_t rd_cks, calc_cks;
    
    xprintf_P(PSTR("LOAD NVM Base: memblock size=%d\r\n"), sizeof(base_conf));
    memset( &base_conf, '\0', sizeof(base_conf));
    NVMEE_read( nvm_address, (char *)&base_conf, sizeof(base_conf) );
    
    rd_cks = base_conf.checksum;
    calc_cks = u_checksum ( (uint8_t *)&base_conf, ( sizeof(base_conf) - 1));
    
    if ( calc_cks != rd_cks ) {
		xprintf_P( PSTR("ERROR: Base checksum failed: calc[0x%0x], read[0x%0x]\r\n"), calc_cks, rd_cks );
        return(-1);
	} 
        
    return( nvm_address + sizeof(base_conf) );
}
//------------------------------------------------------------------------------