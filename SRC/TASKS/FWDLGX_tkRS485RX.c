#include "FWDLGX.h"
#include "frtos_cmd.h"
#include "modbus.h"


void RS485_flush_RXbuffer(void);
uint16_t RS485_getRXCount(void);
char *RS485_get_RXBuffer(void);

//------------------------------------------------------------------------------
void tkRS485RX(void * pvParameters)
{

	// Esta tarea maneja la cola de datos de la UART RS485A que es donde
    // se coloca el bus MODBUS.
    // Cada vez que hay un dato lo pone el un buffer circular commsA_lbuffer
    // para su procesamiento externo.

( void ) pvParameters;
uint8_t c = 0;
uint32_t ulNotificationValue;

    while ( ! starting_flag )
        vTaskDelay( ( TickType_t)( 100 / portTICK_PERIOD_MS ) );

    SYSTEM_ENTER_CRITICAL();
    tk_running[TK_RS485RX] = true;
    SYSTEM_EXIT_CRITICAL();
    
	vTaskDelay( ( TickType_t)( 500 / portTICK_PERIOD_MS ) );

    lBchar_CreateStatic ( &modbus_rx_lbuffer, modbus_rx_buffer, MODBUS_RX_BUFFER_SIZE );
        
    // La variable tickless se fija externamente para controlar cuando entro y salgo del 
    // modo.
    rs485_SLEEP();
    
    xprintf_P(PSTR("Starting tkRS485RX..\r\n" ));
    
	// loop
	for( ;; )
	{
        
        u_kick_wdt(TK_RS485RX, T120S);
        
        //while ( true ) {
        while ( rs485_awake ) {
            c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
            u_kick_wdt(TK_RS485RX, T120S);
            
            // el read se bloquea 10ms. lo que genera la espera.
            while ( xfgetc( fdRS485A, (char *)&c ) == 1 ) {
                lBchar_Put( &modbus_rx_lbuffer, c);
            }
        }
        
        // Estoy en tickless
        u_kick_wdt(TK_RS485RX, T120S);
        ulNotificationValue = ulTaskNotifyTake(pdTRUE, ( TickType_t)(60000 / portTICK_PERIOD_MS ));
	}    
}
//------------------------------------------------------------------------------
