/*
 * File:   tkCtl.c
 * Author: pablo
 *
 * Created on 25 de octubre de 2021, 12:50 PM
 */


#include "FWDLGX.h"

#define TKCTL_DELAY_S	5

void sys_watchdog_check(void);
void sys_daily_reset(void);

//------------------------------------------------------------------------------
void tkCtl(void * pvParameters)
{

	// Esta es la primer tarea que arranca.
    // Se encarga del log de debug de los contadores porque no puedo hacerlo
    // en la ISR.

( void ) pvParameters;
uint32_t ulNotificationValue;
fat_s l_fat;
uint8_t i;

	vTaskDelay( ( TickType_t)( 500 / portTICK_PERIOD_MS ) );
    xprintf_P(PSTR("Starting tkCtl..\r\n"));
    
    // Inicializaciones con el RTOS corriendo
    ainputs_init();
    counter_init();
    
    // Leo la configuracion de EE
    xprintf_P(PSTR("Loading config...\r\n"));
    if ( ! u_load_config_from_NVM() )  {
       xprintf_P(PSTR("Loading config default..\r\n"));
       u_config_default(NULL);
    }
    
    // Inicializo la memoria EE ( fileSysyem)
	if ( FS_open() ) {
		xprintf_P( PSTR("FSInit OK\r\n"));
	} else {
        FAT_flush();
        xprintf_P( PSTR("FSInit FAIL !!.Reformatted...\r\n"));
	}

    wdt_reset();   
    FAT_read(&l_fat); 
	xprintf_P( PSTR("MEMsize=%d, wrPtr=%d, "),FF_MAX_RCDS, l_fat.head );
    xprintf_P( PSTR("rdPtr=%d, count=%d\r\n"),l_fat.tail, l_fat.count );

    // Imprimo el tamanio de registro de memoria
	xprintf_P( PSTR("RCD size %d bytes.\r\n"),sizeof(dataRcd_s));

	// Arranco el RTC. Si hay un problema lo inicializo.
    RTC_init();
    
    // Por ultimo habilito a todas las otras tareas a arrancar
    starting_flag = true;
       
	for( ;; )
	{
        led_flash();
        sys_watchdog_check();
        sys_daily_reset();       
        // Duerme 5 secs y corre.
        // vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
		//vTaskDelay( ( TickType_t)( 1000 * TKCTL_DELAY_S / portTICK_PERIOD_MS ) );
        // Estoy en tickless
        ulNotificationValue = ulTaskNotifyTake(pdTRUE, ( TickType_t)( (1000 * TKCTL_DELAY_S) / portTICK_PERIOD_MS ));
        
        if( ( ulNotificationValue & 0x01 ) != 0 )
        {
            xprintf_P(PSTR("COUNTER: PULSOS=%d, CAUDAL=%0.3f, PW(secs)=%0.3f\r\n"), contador.pulsos, contador.caudal, contador.T_secs );  
        }
        
	}
}
//------------------------------------------------------------------------------
void sys_watchdog_check(void)
{
    /*
     * El vector de watchdogs esta inicialmente en false.
     * Cada tarea con la funcion u_kick_wdt() pone su wdf en true.
     * Periodicamente lo ponemos en false.
     * Si al chequearlo, aparece alguno en false, es que no esta corriendo la
     * tarea que debería haberlo puesto en true por lo que nos reseteamos.
     * 
     */
    
uint8_t i;

    /*
     * Cada 5s entro y reseteo el watchdog.
     * 
     */
    wdt_reset();
    //return;
          
    // Analizo los watchdows individuales
    for (i = 0; i < NRO_TASKS; i++) {
        //xprintf_P(PSTR("tkCtl: check wdg[%d]->[%d][%d]\r\n"), i,tk_running[i], tk_watchdog[i]  );
        // Si la tarea esta corriendo...
        if ( tk_running[i] ) {
            tk_watchdog[i] -= TKCTL_DELAY_S;
            
            // Si el wdg llego a 0 es que no pudo borrarlo  !!!
            if ( tk_watchdog[i] <= 0 ) {        
                xprintf_P( PSTR("ALARM !!!. TKCTL: RESET BY WDG %d\r\n"), i );
                //reset();
                
                while(1)
                    ;
                
            }
        }
    }
    
}
//------------------------------------------------------------------------------
void sys_daily_reset(void)
{
	// Todos los dias debo resetearme para restaturar automaticamente posibles
	// problemas.
	// Se invoca 1 vez por minuto ( 60s ).

static uint32_t ticks_to_reset = 86400 / TKCTL_DELAY_S ; // ticks en 1 dia.

	//xprintf_P( PSTR("DEBUG dailyReset..\r\n\0"));
	while ( --ticks_to_reset > 0 ) {
		return;
	}

	xprintf_P( PSTR("Daily Reset !!\r\n\0") );
    vTaskDelay( ( TickType_t)( 2000 / portTICK_PERIOD_MS ) );
    reset();
    
}
//------------------------------------------------------------------------------
