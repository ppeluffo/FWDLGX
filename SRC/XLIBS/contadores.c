
#include "contadores.h"
#include <avr/interrupt.h>


#ifdef HW_AVRDA

#ifdef R2

    #define PE6_INTERRUPT               ( PORTE.INTFLAGS & PIN6_bm )
    #define PE6_CLEAR_INTERRUPT_FLAG    ( PORTE.INTFLAGS &= PIN6_bm )

    #define CNT0_PORT	PORTE
    #define CNT0_PIN    6  
    #define CNT0_PIN_bm	PIN6_bm
    #define CNT0_PIN_bp	PIN6_bp
    #define CNT0_PIN_CONFIG()           PORTE.PIN6CTRL |= PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;

#endif

#ifdef R1

    #define PF4_INTERRUPT               ( PORTF.INTFLAGS & PIN4_bm )
    #define PF4_CLEAR_INTERRUPT_FLAG    ( PORTF.INTFLAGS &= PIN4_bm )

    #define CNT0_PORT	PORTF
    #define CNT0_PIN    4  
    #define CNT0_PIN_bm	PIN4_bm
    #define CNT0_PIN_bp	PIN4_bp
    #define CNT0_PIN_CONFIG()           PORTF.PIN4CTRL |= PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;

#endif

#endif


#ifdef HW_XMEGA
// Los contadores en M1 son PB2 y PA2. Solo usamos uno, el PORT_A

    #define CNT0_PORT	PORTA
    #define CNT0_PIN    2   
    #define CNT0_PIN_bm	PIN2_bm
    #define CNT0_PIN_bp	PIN2_bp

    #define CNT0_PIN_CONFIG()           PORTA.PIN2CTRL |= PORT_ISC_FALLING_gc;

#endif

static bool f_debug_counters = false;

void pv_count_pulse(void);

TaskHandle_t *l_xHandle_tkCtl;

//------------------------------------------------------------------------------
void counter_init_outofrtos( TaskHandle_t *xHandle )
{
    /*
     * El pin del contador es INPUT, PULLOPEN e interrumpe en flanco de bajada.
     * 
     * Utilizo el handler de tkCtl para mostrar el log del debug.
     * 
     */
    
    l_xHandle_tkCtl = xHandle;
    // Los CNTx son inputs
    CNT0_PORT.DIR &= ~CNT0_PIN_bm;
    // Habilito a interrumpir, pullup, flanco de bajada.
    cli();
    CNT0_PIN_CONFIG();
    //PORTF.PIN4CTRL |= PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
    sei();
}
// ----------------------------------------------------------------------------- 
void counter_init( void )
{
    contador.fsm_ticks_count = 0;
}
// ----------------------------------------------------------------------------- 
void counter_config_defaults( void )
{
    /*
     * Realiza la configuracion por defecto de los canales digitales.
     */

    //strncpy( counter_conf.name, "X", CNT_PARAMNAME_LENGTH );
    strlcpy( counter_conf.name, "X", CNT_PARAMNAME_LENGTH );
    counter_conf.enabled = false;
    counter_conf.magpp = 1;
    counter_conf.modo_medida = CAUDAL;

}
//------------------------------------------------------------------------------
void counter_print_configuration( void )
{
    /*
     * Muestra la configuracion de todos los canales de contadores en la terminal
     * La usa el comando tkCmd::status.
     */
    

    xprintf_P(PSTR("Counter:\r\n"));
    xprintf_P(PSTR(" debug: "));
    f_debug_counters ? xprintf_P(PSTR("on\r\n")) : xprintf_P(PSTR("off\r\n"));
    
        
    if ( counter_conf.enabled ) {
        xprintf_P( PSTR(" c0: +"));
    } else {
        xprintf_P( PSTR(" c0: -"));
    }
                
    xprintf_P( PSTR("[%s,magpp=%.03f,"), counter_conf.name, counter_conf.magpp );
    if ( counter_conf.modo_medida == CAUDAL ) {
        xprintf_P(PSTR("CAUDAL]\r\n"));
    } else {
        xprintf_P(PSTR("PULSO]\r\n"));
    }
      
}
//------------------------------------------------------------------------------
bool counter_config_channel( char *s_enable, char *s_name, char *s_magpp, char *s_modo )
{
	// Configuro un canal contador.
	// channel: id del canal
	// s_param0: string del nombre del canal
	// s_param1: string con el valor del factor magpp.
	//
	// {0..1} dname magPP

bool retS = false;

    //xprintf_P(PSTR("DEBUG COUNTERS: en=%s,name=%s,magpp=%s,modo=%s,rbsize=%s\r\n"), s_enable,s_name,s_magpp,s_modo,s_rb_size  );

	if ( s_name == NULL ) {
		return(retS);
	}

    // Enable ?
    if (!strcmp_P( strupr(s_enable), PSTR("TRUE"))  ) {
        counter_conf.enabled = true;
    } else if (!strcmp_P( strupr(s_enable), PSTR("FALSE"))  ) {
        counter_conf.enabled = false;
    }
        
    // NOMBRE
	//snprintf_P( cnt->channel[ch].name, CNT_PARAMNAME_LENGTH, PSTR("%s"), s_name );
    //strncpy( counter_conf.name, s_name, CNT_PARAMNAME_LENGTH );
    strlcpy( counter_conf.name, s_name, CNT_PARAMNAME_LENGTH );

	// MAGPP
	if ( s_magpp != NULL ) { counter_conf.magpp = atof(s_magpp); }

    // MODO ( PULSO/CAUDAL )
    if ( s_modo != NULL ) {
		if ( strcmp_P( strupr(s_modo), PSTR("PULSO")) == 0 ) {
			counter_conf.modo_medida = PULSOS;

		} else if ( strcmp_P( strupr(s_modo) , PSTR("CAUDAL")) == 0 ) {
			counter_conf.modo_medida = CAUDAL;

		} else {
			//xprintf_P(PSTR("ERROR: counters modo: PULSO/CAUDAL only!!\r\n"));
            return (false);
		}
    }
        
    retS = true;
	return(retS);

}
//------------------------------------------------------------------------------
void counter_config_debug(bool debug )
{
    if ( debug ) {
        f_debug_counters = true;
    } else {
        f_debug_counters = false;
    }
}
//------------------------------------------------------------------------------
bool counter_read_debug(void)
{
    return (f_debug_counters);
}
//------------------------------------------------------------------------------
uint8_t counter_read_pin(void)
{
    return ( ( CNT0_PORT.IN & CNT0_PIN_bm ) >> CNT0_PIN) ;
}
// -----------------------------------------------------------------------------
counter_value_t counter_read(void)
{
    return(contador);
}
// -----------------------------------------------------------------------------
void counter_clear(void)
{
   /*
    * Una vez por periodo ( timerpoll ) borro los contadores.
    * Si en el periodo NO llegaron pulsos, aqui debo entonces en los
    * caudales agregar un 0.0 al ring buffer para que luego de un tiempo
    * converja a 0.
    * 
    */
    contador.caudal = 0.0;
    contador.pulsos = 0;
}
//------------------------------------------------------------------------------
uint8_t counter_hash(void)
{
    
uint8_t j;
uint8_t hash = 0;
uint8_t l_hash_buffer[64];
char *p;

    // Calculo el hash de la configuracion de los contadores


    memset( l_hash_buffer, '\0', sizeof(l_hash_buffer));
    j = sprintf_P( (char *)&l_hash_buffer, PSTR("[C0:") );
    
    if ( counter_conf.enabled ) {
        j += sprintf_P( (char *)&l_hash_buffer[j], PSTR("TRUE,") );
    } else {
       j += sprintf_P( (char *)&l_hash_buffer[j], PSTR("FALSE,") );
    }
        
    j += sprintf_P( (char *)&l_hash_buffer[j], PSTR("%s,"), counter_conf.name );
    j += sprintf_P( (char *)&l_hash_buffer[j], PSTR("%.03f,"), counter_conf.magpp );
        
    if ( counter_conf.modo_medida == 0 ) {
        j += sprintf_P( (char *)&l_hash_buffer[j], PSTR("CAUDAL]"));
    } else {
        j += sprintf_P( (char *)&l_hash_buffer[j], PSTR("PULSOS]"));
    }
    
    p = (char *)l_hash_buffer;
    while (*p != '\0') {
        hash = u_hash(hash, *p++);
    }
        
    //xprintf_P(PSTR("HASH_CNT:<%s>, hash=%d(0x%02x)\r\n"), l_hash_buffer, hash, hash );
 
    return(hash);
    
}
//------------------------------------------------------------------------------
void pv_count_pulse(void)
{
  
BaseType_t xHigherPriorityTaskWoken = pdTRUE;

    contador.now_ticks = xTaskGetTickCountFromISR();
    contador.T_ticks = contador.now_ticks - contador.start_pulse_ticks;
    // Guardo el inicio del pulso para medir el caudal del proximo pulso
    contador.start_pulse_ticks = contador.now_ticks;
    
    // No cuento pulsos menores de 100 ticks de ancho
    if (contador.T_ticks < 10) {
        return;
    }
    
    contador.pulsos++;
    contador.T_secs =  (float)(1.0 * contador.T_ticks);    // Duracion en ticks
    contador.T_secs /= configTICK_RATE_HZ;                 // Duracion en secs.

    if ( contador.T_secs > 0 ) {
        contador.caudal = counter_conf.magpp / contador.T_secs;      // En mt3/s 
        contador.caudal *= 3600;                                     // En mt3/h
    } else {
        contador.caudal = 0.0;
    } 
            
    if (f_debug_counters) {
        xTaskNotifyFromISR( *l_xHandle_tkCtl,
                       0x01,
                       eSetBits,
                       &xHigherPriorityTaskWoken );
        
    }
            
}
//------------------------------------------------------------------------------
int16_t counter_save_config_in_NVM(uint16_t nvm_ptr)
{
   
int8_t retVal;
uint8_t cks;


    // AINPUTS
    cks = u_checksum ( (uint8_t *)&counter_conf, ( sizeof(counter_conf) - 1));
    counter_conf.checksum = cks;
    retVal = NVMEE_write( nvm_ptr, (char *)&counter_conf, sizeof(counter_conf) );
    xprintf_P(PSTR("SAVE NVM Counter: memblock size = %d\r\n"), sizeof(counter_conf));       
    if (retVal == -1 )
        return(-1);
    
    nvm_ptr += sizeof(counter_conf);
    
    return(nvm_ptr);
   
}
//------------------------------------------------------------------------------
int16_t counter_load_config_from_NVM( uint16_t nvm_address)
{
    /*
     * Leo la memoria, calculo el chechsum.
     * Aunque sea erroneo, copio los datos de modo que en el caso del modem
     * tenga algo para adivinar !!.
     * 
     */

uint8_t rd_cks, calc_cks;
    
    xprintf_P(PSTR("LOAD NVM Counter: memblock size=%d\r\n"), sizeof(counter_conf));
    memset( &counter_conf, '\0', sizeof(counter_conf));
    NVMEE_read( nvm_address, (char *)&counter_conf, sizeof(counter_conf) );
    
    rd_cks = counter_conf.checksum;
    calc_cks = u_checksum ( (uint8_t *)&counter_conf, ( sizeof(counter_conf) - 1));
    
    if ( calc_cks != rd_cks ) {
		xprintf_P( PSTR("ERROR: Counter checksum failed: calc[0x%0x], read[0x%0x]\r\n"), calc_cks, rd_cks );
        return(-1);
	} 
        
    return( nvm_address + sizeof(counter_conf) );
}
//------------------------------------------------------------------------------
/*
 * Note: If the flag is not cleared, the interrupt will keep triggering, so 
 * it is essential that the flag is always cleared before exiting the ISR. 
 * Any algorithm that relies on not clearing the interrupt flag is highly 
 * discouraged because this effectively overloads the ISR responsibility. 
 * The resulting responsibilities of the ISR will be to handle the interrupt 
 * and to keep firing until the flag is cleared. 
 * This violates the Single Responsibility principle in software design 
 * and extreme care must be taken to avoid bugs.
 */
#ifdef HW_AVRDA

#ifdef R2
ISR(PORTE_PORT_vect)
{
    // Borro las flags.
    if (PE6_INTERRUPT) {
        //led_toggle();
        pv_count_pulse();
                    
        // Se borra la flag de interrupcion para habilitarla de nuevo
        // Si no la borro antes de salir, se vuelve a generar la int.
        PE6_CLEAR_INTERRUPT_FLAG;
    }

}
#endif

#ifdef R1
ISR(PORTF_PORT_vect)
{
    // Borro las flags.
    if (PF4_INTERRUPT) {
        //led_toggle();
        pv_count_pulse();
                    
        // Se borra la flag de interrupcion para habilitarla de nuevo
        // Si no la borro antes de salir, se vuelve a generar la int.
        PF4_CLEAR_INTERRUPT_FLAG;
    }

}
#endif

#endif
//------------------------------------------------------------------------------
#ifdef HW_XMEGA

ISR ( PORTA_INT0_vect )
{
    // Q0 es PA2
	// Esta ISR se activa cuando el contador Q0 (PA2) genera un flaco se subida.
    // Arranca el timer (on-shot) para generar el debounce.
    // Se desactiva.
    // Al expirar el timer cuenta el pulso y hace las cuentas si es caudal o no.

	// Deshabilita la interrupcion por ahora ( enmascara )
	PORTA.INT0MASK = 0x00;
	PORTA.INTCTRL = 0x00;
    
    pv_count_pulse();

}
#endif

     