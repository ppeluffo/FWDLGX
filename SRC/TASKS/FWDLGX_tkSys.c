/*
 * File:   tkSystem.c
 * Author: pablo
 *
 * Espera timerPoll y luego muestra los valores de las entradas analogicas.
 * 
 */


#include "FWDLGX.h"

dataRcd_s dataRcd;

//------------------------------------------------------------------------------
void tkSys(void * pvParameters)
{
    /*
     * Espero timerpoll y poleo todos los canales, dejando los datos en 
     * un dataRcd.
     * Le paso el dataRcd a la tarea de WAN para que lo envie o lo almacene
     * Finalmente muestro los datos en la terminal
     * 
     * TickType_t puede ser uint16 o uint32. Si usamos configUSE_16_BIT_TICKS 0
     * entonces es de 32 bits.
     * 
     */
TickType_t xLastWakeTime;
uint32_t waiting_secs;

	while (!starting_flag )
		vTaskDelay( ( TickType_t)( 100 / portTICK_PERIOD_MS ) );
    
    SYSTEM_ENTER_CRITICAL();
    tk_running[TK_SYS] = true;
    SYSTEM_EXIT_CRITICAL();
    
    xprintf_P(PSTR("Starting tkSys..\r\n"));
       
    u_kick_wdt(TK_SYS, T120S);
    
    // Espero solo 10s para el primer poleo ( no lo almaceno !!)
    vTaskDelay( ( TickType_t)( 10000 / portTICK_PERIOD_MS ) );
    
	for( ;; )
	{
       
        xLastWakeTime = xTaskGetTickCount();
        
        // Poleo
        u_poll_data(&dataRcd);
        
        // Imprimo
        u_xprint_dr(&dataRcd);
        
        // Envio a tkWAN
        WAN_process_data_rcd(&dataRcd);
                
        // Espero
        waiting_secs = base_conf.timerpoll;
        
        //vTaskDelay( ( TickType_t)( 60000 / portTICK_PERIOD_MS ) );
        
        while ( waiting_secs > 60 ) {
            
            u_kick_wdt(TK_SYS, T120S);
            
            vTaskDelayUntil( &xLastWakeTime, ( 60000 / portTICK_PERIOD_MS ) );
            xLastWakeTime = xTaskGetTickCount();
            waiting_secs -= 60;
        }
        // Espero el saldo
        u_kick_wdt(TK_SYS, T120S);
        vTaskDelayUntil( &xLastWakeTime, ( waiting_secs * 1000 / portTICK_PERIOD_MS ) );
        
    }

}
//------------------------------------------------------------------------------
dataRcd_s *get_dataRcd_ptr(void)
{
    return(&dataRcd);
}
//------------------------------------------------------------------------------
