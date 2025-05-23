/* 
 * File:   frtos20_utils.c
 * Author: pablo
 *
 * Created on 22 de diciembre de 2021, 07:34 AM
 */

#include "FWDLGX.h"
#include "pines.h"

//------------------------------------------------------------------------------
int8_t WDT_init(void);
int8_t CLKCTRL_init(void);

//-----------------------------------------------------------------------------
void system_init(void)
{

    // Init OUT OF RTOS !!!
    
	CLKCTRL_init();
    WDT_init();
    LED_init();
    XPRINTF_init();
    MODEM_init();
    
    CONFIG_RTS_485();
    // RTS OFF: Habilita la recepcion del chip
	CLEAR_RTS_RS485();
    CONFIG_EN_PWR_QMBUS();
    
    CONFIG_EN_PWR_CPRES();
    CLEAR_EN_PWR_CPRES();
    
    ADC_init();
    
    CONFIG_EN_SENS3V3();
    CONFIG_EN_SENS12V();
    CONFIG_PWR_SENSORS();
    
    CLEAR_EN_SENS3V3();
    CLEAR_EN_SENS12V();
    CLEAR_PWR_SENSORS();
}
//-----------------------------------------------------------------------------
int8_t WDT_init(void)
{
	/* 8K cycles (8.2s) */
	/* Off */
#ifdef HW_AVRDA
	ccp_write_io((void *)&(WDT.CTRLA), WDT_PERIOD_8KCLK_gc | WDT_WINDOW_OFF_gc );  
    
#endif
    
	return 0;
}
//-----------------------------------------------------------------------------
int8_t CLKCTRL_init(void)
{
	// Configuro el clock para 24Mhz
	
#ifdef HW_AVRDA
    
	ccp_write_io((void *)&(CLKCTRL.OSCHFCTRLA), CLKCTRL_FRQSEL_24M_gc         /* 24 */
                                                
	| 0 << CLKCTRL_AUTOTUNE_bp /* Auto-Tune enable: disabled */
	| 0 << CLKCTRL_RUNSTDBY_bp /* Run standby: disabled */);

	// ccp_write_io((void*)&(CLKCTRL.MCLKCTRLA),CLKCTRL_CLKSEL_OSCHF_gc /* Internal high-frequency oscillator */
	//		 | 0 << CLKCTRL_CLKOUT_bp /* System clock out: disabled */);

	// ccp_write_io((void*)&(CLKCTRL.MCLKLOCK),0 << CLKCTRL_LOCKEN_bp /* lock enable: disabled */);

#endif
    
#ifdef HW_XMEGA
    
#if SYSMAINCLK == 32
	// Habilito el oscilador de 32Mhz
	OSC.CTRL |= OSC_RC32MEN_bm;

	// Espero que este estable
	do {} while ( (OSC.STATUS & OSC_RC32MRDY_bm) == 0 );

	// Seteo el clock para que use el oscilador de 32Mhz.
	// Uso la funcion CCPWrite porque hay que se cuidadoso al tocar estos
	// registros que son protegidos.
	CCPWrite(&CLK.CTRL, CLK_SCLKSEL_RC32M_gc);
	//
	// El prescaler A ( CLK.PSCCTRL ), B y C ( PSBCDIV ) los dejo en 0 de modo que no
	// hago division y con esto tengo un clk_per = clk_sys. ( 32 Mhz ).
	//
#endif
    
#endif
    
	return 0;
}
//-----------------------------------------------------------------------------
void reset(void)
{
    xprintf_P(PSTR("ALERT !!!. Going to reset...\r\n"));
    vTaskDelay( ( TickType_t)( 100 / portTICK_PERIOD_MS ) );
	/* Issue a Software Reset to initilize the CPU */
    
#ifdef HW_AVRDA
	ccp_write_io( (void *)&(RSTCTRL.SWRR), RSTCTRL_SWRST_bm ); 
#endif
    
#ifdef HW_XMEGA 
	CCPWrite( &RST.CTRL, RST_SWRST_bm );
#endif
    
}
//------------------------------------------------------------------------------
void u_kick_wdt( t_wdg_ids wdg_id, uint16_t wdg_timer)
{
    // Pone el wdg correspondiente en true
    tk_watchdog[wdg_id] = wdg_timer;
    
}
//------------------------------------------------------------------------------
bool u_config_debug( char *tipo, char *valor)
{
    /*
     * Configura las flags de debug para ayudar a visualizar los problemas
     * none,ainput,counter,modbus,piloto,wan,consigna
     */
    
    if (!strcmp_P( strupr(tipo), PSTR("NONE")) ) {
        ainputs_config_debug(false);
        counter_config_debug(false);
        return(true); 
    }
   
    if (!strcmp_P( strupr(tipo), PSTR("AINPUT")) ) {
        if (!strcmp_P( strupr(valor), PSTR("TRUE")) ) {
            ainputs_config_debug(true);
            return(true);
        }
        if (!strcmp_P( strupr(valor), PSTR("FALSE")) ) {
            ainputs_config_debug(false);
            return(true);
        }
    }
    
    if (!strcmp_P( strupr(tipo), PSTR("COUNTER")) ) {
        if (!strcmp_P( strupr(valor), PSTR("TRUE")) ) {
            counter_config_debug(true);
            return(true);
        }
        if (!strcmp_P( strupr(valor), PSTR("FALSE")) ) {
            counter_config_debug(false);
            return(true);
        }
    }
    
    if (!strcmp_P( strupr(tipo), PSTR("WAN")) ) {
        if (!strcmp_P( strupr(valor), PSTR("TRUE")) ) {
            WAN_config_debug(true);
            return(true);
        }
        if (!strcmp_P( strupr(valor), PSTR("FALSE")) ) {
            WAN_config_debug(false);
            return(true);
        }
    }

    if (!strcmp_P( strupr(tipo), PSTR("MODBUS")) ) {
        if (!strcmp_P( strupr(valor), PSTR("TRUE")) ) {
            modbus_config_debug(true);
            return(true);
        }
        if (!strcmp_P( strupr(valor), PSTR("FALSE")) ) {
            modbus_config_debug(false);
            return(true);
        }
    }
    
    return(false);
    
}
//------------------------------------------------------------------------------
void u_config_default( char *modo )
{

    // Configuro a default todas las configuraciones locales
    // y luego actualizo el systemConf
        
    base_config_defaults();
    ainputs_config_defaults();
    counter_config_defaults();
    modem_config_defaults(modo);
    modbus_config_defaults();
    consigna_config_defaults();
    
}
//------------------------------------------------------------------------------
bool u_save_config_in_NVM(void)
{

int16_t nvm_ptr;

    nvm_ptr = 0x00;
    
    // BASE
    nvm_ptr = base_save_config_in_NVM(nvm_ptr);
    if (nvm_ptr == -1) {
        return(false);
    }
    
    // AINPUTS
    nvm_ptr = ainputs_save_config_in_NVM(nvm_ptr);
    if (nvm_ptr == -1) {
        return(false);
    }

    // COUNTER
    nvm_ptr = counter_save_config_in_NVM(nvm_ptr);
    if (nvm_ptr == -1) {
        return(false);
    }

    // MODEM
    nvm_ptr = modem_save_config_in_NVM(nvm_ptr);
    if (nvm_ptr == -1) {
        return(false);
    }

    // MODBUS
    nvm_ptr = modbus_save_config_in_NVM(nvm_ptr);
    if (nvm_ptr == -1) {
        return(false);
    }
    
    // CONSIGNA
    nvm_ptr = consigna_save_config_in_NVM(nvm_ptr);
    if (nvm_ptr == -1) {
        return(false);
    }
    
    return(true);
   
}
//------------------------------------------------------------------------------
bool u_load_config_from_NVM(void)
{
    /*
     * Leo la memoria, calculo el chechsum.
     * Aunque sea erroneo, copio los datos de modo que en el caso del modem
     * tenga algo para adivinar !!.
     * 
     */

int16_t nvm_ptr;

    
    nvm_ptr = 0x00;
    
    // BASE
    nvm_ptr = base_load_config_from_NVM(nvm_ptr);
    if ( nvm_ptr == -1 ) {
        return(false);
    }

    // AINPUTS
    nvm_ptr = ainputs_load_config_from_NVM(nvm_ptr);
    if ( nvm_ptr == -1 ) {
        return(false);
    }

    // COUNTER
    nvm_ptr = counter_load_config_from_NVM(nvm_ptr);
    if ( nvm_ptr == -1 ) {
        return(false);
    }

    // MODEM
    nvm_ptr = modem_load_config_from_NVM(nvm_ptr);
    if ( nvm_ptr == -1 ) {
        return(false);
    }

    // MODBUS
    nvm_ptr = modbus_load_config_from_NVM(nvm_ptr);
    if ( nvm_ptr == -1 ) {
        return(false);
    }

    // CONSIGNA
    nvm_ptr = consigna_load_config_from_NVM(nvm_ptr);
    if ( nvm_ptr == -1 ) {
        return(false);
    }
    
    return(true);
}
//------------------------------------------------------------------------------
bool u_poll_data(dataRcd_s *dataRcd)
{
    /*
     * Se encarga de leer los datos.
     * Lo hacemos aqui asi es una funcion que se puede invocar desde Cmd.
     */
bool f_status;
uint8_t channel;
float mag;
uint16_t raw;
bool retS = false;
counter_value_t cnt;
    
    // Siempre prendo: si esta prendido no importa, no hace nada
    SET_EN_PWR_QMBUS();
    
    // AINPUTS
    ainputs_prender_sensores();  
    for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
        mag = 0.0;
        raw = 0;
        // Solo leo los canales configurados.
        if ( ainputs_conf.channel[channel].enabled ) {
            ainputs_read_channel ( channel, &mag, &raw );
        }
        dataRcd->ainputs[channel] = mag;
    }  
    ainputs_sleep();
    ainputs_apagar_sensores();
    
    // COUNTER
    cnt = counter_read();
    if ( counter_conf.enabled ) {
        if ( counter_conf.modo_medida == PULSOS ) {
            dataRcd->contador = (float) cnt.pulsos;
        } else {
            dataRcd->contador = cnt.caudal;
        }
    }
    counter_clear();
    
#ifdef HW_AVRDA
    // MODBUS
    if ( modbus_conf.enabled ) { 
        
        rs485_ENTER_CRITICAL();
        rs485_AWAKE();
        
        modbus_read ( dataRcd->modbus );  
        // Solo apago si estoy en modo discreto
        if ( u_get_sleep_time(false) > 0 ){
            // Espero 10s que se apliquen las consignas y apago el modulo
            vTaskDelay( ( TickType_t)( 2000 / portTICK_PERIOD_MS ) );
            CLEAR_EN_PWR_QMBUS(); 
        }
        
        rs485_SLEEP();
        rs485_EXIT_CRITICAL();
    }
#endif
    
    // Bateria
    dataRcd->bt3v3 = u_read_bat3v3(false);
    dataRcd->bt12v = u_read_bat12v(false);
    
    // Agrego el timestamp.
    f_status = RTC_read_dtime( &dataRcd->rtc );
    if ( ! f_status ) {
        xprintf_P(PSTR("u_poll_data: ERROR I2C RTC:data_read_inputs\r\n"));
        retS = false;
        goto quit;
    }
    
    // Control de errores
    // 1- Clock:
    if ( dataRcd->rtc.year == 0) {
        xprintf_P(PSTR("u_poll_data: DATA ERROR: byClock\r\n"));
        retS = false;
        goto quit;
    }
        
    retS = true;
    
quit:
            
    return(retS);
     
}
//------------------------------------------------------------------------------
void u_xprint_dr(dataRcd_s *dr)
{
    /*
     * Imprime en pantalla el dataRcd pasado
     */
    
uint8_t i;

    xprintf_P( PSTR("ID=%s;TYPE=%s;VER=%s;"), base_conf.dlgid, FW_TYPE, FW_REV);
 
    // Clock
    xprintf_P( PSTR("DATE=%02d%02d%02d;"), dr->rtc.year, dr->rtc.month, dr->rtc.day );
    xprintf_P( PSTR("TIME=%02d%02d%02d;"), dr->rtc.hour, dr->rtc.min, dr->rtc.sec);
    
    // Canales Analogicos:
    for ( i=0; i < NRO_ANALOG_CHANNELS; i++) {
        if ( ainputs_conf.channel[i].enabled ) {
            xprintf_P( PSTR("%s=%0.2f;"), ainputs_conf.channel[i].name, dr->ainputs[i]);
        }
    }
        
    // Contador
    if ( counter_conf.enabled) {
        if ( counter_conf.modo_medida == PULSOS ) {
            xprintf_P( PSTR("%s=%d;"), counter_conf.name, (uint16_t)(dr->contador) );
        } else {
            xprintf_P( PSTR("%s=%0.3f;"), counter_conf.name, dr->contador);
        }            
    }
    
#ifdef HW_AVRDA
    // Modbus:
    if ( modbus_conf.enabled ) { 
        for ( i=0; i < NRO_MODBUS_CHANNELS; i++) {
            if ( modbus_conf.mbch[i].enabled ) {
                xprintf_P( PSTR("%s=%0.2f;"), modbus_conf.mbch[i].name, dr->modbus[i]);
            }
        }
    }
#endif
    
    // Bateria
    xprintf_P( PSTR("bt3v3=%0.2f;bt12v=%0.2f"), dr->bt3v3, dr->bt12v);
    
    xprintf_P( PSTR("\r\n"));
}
//------------------------------------------------------------------------------
void u_print_tasks_running(void)
{
    
    xprintf_P(PSTR("Tasks running:"));
    
    if ( tk_running[TK_CMD] ) {
        xprintf_P(PSTR(" cmd[%d]"), tk_watchdog[TK_CMD]);
    }
    
    if ( tk_running[TK_SYS] ) {
        xprintf_P(PSTR(" sys[%d]"), tk_watchdog[TK_SYS]);
    }

    if ( tk_running[TK_WANRX] ) {
        xprintf_P(PSTR(" wanrx[%d]"), tk_watchdog[TK_WANRX]);
    }

    if ( tk_running[TK_WAN] ) {
        xprintf_P(PSTR(" wan[%d]"), tk_watchdog[TK_WAN]);
    }

 #ifdef HW_AVRDA

    if ( tk_running[TK_RS485RX] ) {
        xprintf_P(PSTR(" rs485rx[%d]"), tk_watchdog[TK_RS485RX]);
    }

    if ( tk_running[TK_CPRES] ) {
        xprintf_P(PSTR(" cpres[%d]"), tk_watchdog[TK_CPRES]);
    }
  
#endif
    
    xprintf_P(PSTR("\r\n"));
    
}
//------------------------------------------------------------------------------
