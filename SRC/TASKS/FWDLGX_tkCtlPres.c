/*
 * File:   tkPiloto.c
 * Author: pablo
 *
 */


#include "FWDLGX.h"
#include "consignas.h"

static void pv_consigna_initService(void);
static void pv_consigna_service(void);

//------------------------------------------------------------------------------
void tkCtlPres(void * pvParameters)
{

	/*
     * DOBLE CONSIGNA:
     * Esta tarea se encarga de cotrolar la hora en que debe aplicarse
     * e invocar a las funciones que la fijan.
     * 
     */
    
( void ) pvParameters;
uint32_t ulNotificationValue;


	while (! starting_flag )
		vTaskDelay( ( TickType_t)( 200 / portTICK_PERIOD_MS ) );

    SYSTEM_ENTER_CRITICAL();
    tk_running[TK_CPRES] = true;
    SYSTEM_EXIT_CRITICAL();
    
	vTaskDelay( ( TickType_t)( 500 / portTICK_PERIOD_MS ) );
    xprintf_P(PSTR("Starting tkCtlPresion..\r\n"));
           
    // Espero que todo este arrancado (30s)
    vTaskDelay( ( TickType_t)( 30000 / portTICK_PERIOD_MS ) );
    
    u_kick_wdt(TK_CPRES, T300S);
   
    VALVE_close();
        
    if ( consigna_conf.enabled ) {
        pv_consigna_initService();          
    }

	for( ;; )
	{
        /*
         * Corre cada 1 minuto porque el timeslot se mide como hhmm y no queremos
         * que se realmacene la orden de un mismo tslot
         * 
         */
        u_kick_wdt(TK_CPRES, T300S);
		//vTaskDelay( ( TickType_t)( 55000 / portTICK_PERIOD_MS ) );
      
        // Espero hasta 45 secs un mensaje
        ulNotificationValue = ulTaskNotifyTake(pdTRUE, ( TickType_t)( (1000 * 45 ) / portTICK_PERIOD_MS ));
        
        if( ( ulNotificationValue & SIGNAL_OPEN_VALVE ) != 0 ) {
            xprintf_P(PSTR("tkCtlPres OPEN_VALVE msg rcvd.\r\n")); 
            VALVE_open();
            
        } else if( ( ulNotificationValue & SIGNAL_CLOSE_VALVE ) != 0 ) {
            xprintf_P(PSTR("tkCtlPres CLOSE_VALVE msg rcvd.\r\n"));  
            VALVE_close();
       
        } else {
            vTaskDelay( ( TickType_t)( 10000 / portTICK_PERIOD_MS ) );
        }
               
	}
}
//------------------------------------------------------------------------------
static void pv_consigna_initService(void)
{
    /*
     * Determino en base a la fecha y hora actual, que consigna pongo
     * en el arranque
     *
     */
   
RtcTimeType_t rtc;
uint16_t now;
uint16_t c_dia;
uint16_t c_noche;

    xprintf_P(PSTR("CONSIGNA init service.\r\n")); 
    
    RTC_read_dtime(&rtc);
    now = rtc.hour * 100 + rtc.min;
    c_dia = consigna_conf.consigna_diurna;
    c_noche = consigna_conf.consigna_nocturna;
    
    // Ajusto todo a minutos.
    now = u_hhmm_to_mins(now);
    c_dia = u_hhmm_to_mins( c_dia );
    c_noche = u_hhmm_to_mins( c_noche );
    
    if (c_dia > c_noche ) {
        xprintf_P(PSTR("ERROR: No puedo determinar consigna inicial (c_dia > c_noche)\r\n"));
        // Aplico la nocturna que es la que deberia tener menos presion
        xprintf_P(PSTR("CONSIGNA Init:nocturna %02d:%02d [%04d]\r\n"),rtc.hour,rtc.min, now);
		if ( ! CONSIGNA_set_nocturna() ) {
            xprintf_P(PSTR("CONSIGNA nocturna ERR\r\n"));    
        } else {
            xprintf_P(PSTR("CONSIGNA nocturna OK\r\n"));   
        }
        goto exit;
    }
    
    if ( ( now <= c_dia ) || ( c_noche <= now ) ) {
        // Aplico consigna nocturna
         xprintf_P(PSTR("CONSIGNA Init:nocturna %02d:%02d [%04d]\r\n"),rtc.hour,rtc.min, now);
		if ( ! CONSIGNA_set_nocturna() ) {
            xprintf_P(PSTR("CONSIGNA nocturna ERR\r\n"));    
        } else {
            xprintf_P(PSTR("CONSIGNA nocturna OK\r\n"));   
        }
        goto exit;
       
    } else {
        // Aplico consigna diurna
        xprintf_P(PSTR("CONSIGNA Init:diurna %02d:%02d [%04d]\r\n"),rtc.hour,rtc.min, now);
		if ( ! CONSIGNA_set_diurna() ) {
            xprintf_P(PSTR("CONSIGNA diurna ERR\r\n"));    
        } else {
            xprintf_P(PSTR("CONSIGNA diurna OK\r\n"));   
        }
        goto exit;
    }
    
exit:

   return; 
    
}
//------------------------------------------------------------------------------
static void pv_consigna_service(void)
{
    /*
     * Chequea que sea la hora de aplicar alguna consigna y la aplica.
     */
    
RtcTimeType_t rtc;
uint16_t now;

//    if (consigna_debug_flag() ) {
//        xprintf_P(PSTR("DEBUG CONSIGNA: Service\r\n"));
//    }

	// Chequeo y aplico.
	// Las consignas se chequean y/o setean en cualquier modo de trabajo, continuo o discreto
	memset( &rtc, '\0', sizeof(RtcTimeType_t));
	if ( ! RTC_read_dtime(&rtc) ) {
		xprintf_P(PSTR("CONSIGNA ERROR: I2C:RTC chequear_consignas\r\n\0"));
		return;
	}

    now = rtc.hour * 100 + rtc.min;
    
	// Consigna diurna ?
	if ( now == consigna_conf.consigna_diurna  ) {
		xprintf_P(PSTR("Set CONSIGNA diurna %02d:%02d [%04d]\r\n"),rtc.hour,rtc.min, now);
        if ( ! CONSIGNA_set_diurna() ) {
            xprintf_P(PSTR("CONSIGNA diurna ERR\r\n"));    
        } else {
            xprintf_P(PSTR("CONSIGNA diurna OK\r\n"));   
        }
        goto exit;
	}

	// Consigna nocturna ?
	if ( now == consigna_conf.consigna_nocturna  ) {
		xprintf_P(PSTR("Set CONSIGNA nocturna %02d:%02d [%04d]\r\n"),rtc.hour,rtc.min, now);
        if ( ! CONSIGNA_set_nocturna() ) {
            xprintf_P(PSTR("CONSIGNA nocturna ERR\r\n"));    
        } else {
            xprintf_P(PSTR("CONSIGNA nocturna OK\r\n"));   
        }
        goto exit;
	}
    
    // No es hora de aplicar ninguna consigna
   
exit:

    return;

}
//------------------------------------------------------------------------------
