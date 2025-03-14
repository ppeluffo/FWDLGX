
#include "toyi_valves.h"

// -----------------------------------------------------------------------------
void VALVE_EN_init(void)
{
    // Configura el pin como output
	VALVE_EN_PORT.DIR |= VALVE_EN_PIN_bm;	

}
// -----------------------------------------------------------------------------
void VALVE_CTRL_init(void)
{
    // Configura el pin como output
	VALVE_CTRL_PORT.DIR |= VALVE_CTRL_PIN_bm;	
   

}
// -----------------------------------------------------------------------------
void VALVE_init(void)
{
    VALVE_EN_init();
    //VALVE_CTRL_init();
    
    // La dejo en modo low-power
    DISABLE_VALVE();
    //ENABLE_VALVE();
    //RESET_CTL_VALVE();
    //SET_CTL_VALVE();
}
// -----------------------------------------------------------------------------
t_valve_status get_valve_status(void)
{
    return valve_status;
}
// -----------------------------------------------------------------------------
bool VALVE_open(void)
{
    
    SET_CTL_VALVE();  // Open
    ENABLE_VALVE();
     
    vTaskDelay( ( TickType_t)( 10000 / portTICK_PERIOD_MS ) );
    DISABLE_VALVE();
    RESET_CTL_VALVE();
    valve_status = VALVE_OPEN;
    
    xprintf_P( PSTR("Valve: OPEN\r\n"));
    
    return(true);
    
}
// -----------------------------------------------------------------------------
bool VALVE_close(void)
{
    RESET_CTL_VALVE(); // Close
    ENABLE_VALVE();
    
    vTaskDelay( ( TickType_t)( 10000 / portTICK_PERIOD_MS ) );
    DISABLE_VALVE();
    RESET_CTL_VALVE();
    valve_status = VALVE_CLOSE;
    
    xprintf_P( PSTR("Valve: CLOSE\r\n"));
    return(true);
}
//-----------------------------------------------------------------------------
void valve_print_configuration( void )
{        
    //xprintf_P(PSTR("VALVES\r\n"));
    //return;
    
    if ( valve_status == VALVE_OPEN) {
        xprintf_P( PSTR("Valve: OPEN\r\n"));
    } else {
        xprintf_P( PSTR("Valve: CLOSE\r\n"));
    }
  
}
//------------------------------------------------------------------------------
bool test_valve( char *param1, char *param2)
{
    /* 
     * valve {open|close}
     {enable|ctl} {on|off}
     * 
     */

    if (!strcmp_P( strupr(param1), PSTR("OPEN"))  ) {
        VALVE_open();
        return (true);
    }

    if (!strcmp_P( strupr(param1), PSTR("CLOSE"))  ) {
        VALVE_close();
        return (true);
    }
        
    // enable, disable
    if (!strcmp_P( strupr(param1), PSTR("ENABLE"))  ) {
        if (!strcmp_P( strupr(param2), PSTR("ON"))  ) {
            ENABLE_VALVE();
            return (true);
        }
        if (!strcmp_P( strupr(param2), PSTR("OFF"))  ) {
            DISABLE_VALVE();
            return (true);
        }
        return(false);
    }

    if (!strcmp_P( strupr(param1), PSTR("CTL"))  ) {
        if (!strcmp_P( strupr(param2), PSTR("ON"))  ) {
            SET_CTL_VALVE();
            return (true);
        }
        if (!strcmp_P( strupr(param2), PSTR("OFF"))  ) {
            RESET_CTL_VALVE();
            return (true);
        }
        return(false);
    }

    return(false);
}
