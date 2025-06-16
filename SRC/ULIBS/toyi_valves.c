
#include "toyi_valves.h"

// -----------------------------------------------------------------------------
void VALVE_init(void)
{
#ifdef HW_AVRDA
    
    VALVE_EN_PORT.DIR |= VALVE_EN_PIN_bm;	
    VALVE_CTRL_PORT.DIR |= VALVE_CTRL_PIN_bm;	
    
    // La dejo en modo low-power
    DISABLE_VALVE();
    //ENABLE_VALVE();
    CLEAR_CTL_VALVE();
    //SET_CTL_VALVE();

#endif
   
}
// -----------------------------------------------------------------------------
t_valve_status get_valve_status(void)
{
    return valve_status;
}
// -----------------------------------------------------------------------------
bool VALVE_open(void)
{
    
#ifdef HW_AVRDA
    
    SET_CTL_VALVE();  // Open
    ENABLE_VALVE();
     
    vTaskDelay( ( TickType_t)( 10000 / portTICK_PERIOD_MS ) );
    DISABLE_VALVE();
    CLEAR_CTL_VALVE();
    
    valve_status = VALVE_OPEN;
    xprintf_P( PSTR("Valve: OPEN\r\n"));
 
#endif
    
    return(true);
    
}
// -----------------------------------------------------------------------------
bool VALVE_close(void)
{
    
#ifdef HW_AVRDA
    
    CLEAR_CTL_VALVE(); // Close
    ENABLE_VALVE();
    
    vTaskDelay( ( TickType_t)( 10000 / portTICK_PERIOD_MS ) );
    DISABLE_VALVE();
    CLEAR_CTL_VALVE();
    
    valve_status = VALVE_CLOSE;
    xprintf_P( PSTR("Valve: CLOSE\r\n"));
   
#endif
    
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
    
#ifdef HW_AVRDA
    
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
            CLEAR_CTL_VALVE();
            return (true);
        }
        return(false);
    }

    return(false);
    
#endif
    
}
