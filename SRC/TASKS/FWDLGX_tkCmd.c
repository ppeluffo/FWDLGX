
#include "FWDLGX.h"
#include "frtos_cmd.h"

static void cmdClsFunction(void);
static void cmdHelpFunction(void);
static void cmdResetFunction(void);
static void cmdReadFunction(void);
static void cmdWriteFunction(void);
static void cmdStatusFunction(void);
static void cmdConfigFunction(void);
static void cmdTestFunction(void);

static void pv_snprintfP_OK(void );
static void pv_snprintfP_ERR(void );

StaticTimer_t cmd_xTimerBuffer;
TimerHandle_t cmd_xTimer;
static void cmd_TimerCallback( TimerHandle_t xTimer );

#define T2MIN_INMS  ( 2 * 60000 )
#define T60MIN_INMS ( 60 * 60000 )

typedef enum { CMD_AWAKE=0, CMD_SLEEP } t_cmd_pwrmode;

t_cmd_pwrmode cmd_state;

//------------------------------------------------------------------------------
void tkCmd(void * pvParameters)
{

	// Esta es la primer tarea que arranca.

( void ) pvParameters;
uint8_t c = 0;
    
    while ( ! starting_flag )
        vTaskDelay( ( TickType_t)( 100 / portTICK_PERIOD_MS ) );

    // Marco la tarea activa
    SYSTEM_ENTER_CRITICAL();
    tk_running[TK_CMD] = true;
    SYSTEM_EXIT_CRITICAL();
             
    FRTOS_CMD_init();

    FRTOS_CMD_register( "cls", cmdClsFunction );
	FRTOS_CMD_register( "help", cmdHelpFunction );
    FRTOS_CMD_register( "reset", cmdResetFunction );
    FRTOS_CMD_register( "write", cmdWriteFunction );
    FRTOS_CMD_register( "read", cmdReadFunction );
    FRTOS_CMD_register( "status", cmdStatusFunction );
    FRTOS_CMD_register( "config", cmdConfigFunction );
    FRTOS_CMD_register( "test", cmdTestFunction );
    
    xprintf_P(PSTR("Starting tkCmd..\r\n" ));
    xprintf_P(PSTR("Spymovil %s %s %s %s \r\n") , HW_MODELO, FRTOS_VERSION, FW_REV, FW_DATE);
      
    cmd_state = CMD_AWAKE;

    // Arranco el timer que controla el tiempo que estamos awake
    cmd_xTimer = xTimerCreateStatic (
        "CMDT",
		pdSECS_TO_TICKS(120),           // periodo en ticks
		pdFALSE,                        // One-shot.
		( void * ) 0,
		cmd_TimerCallback,
		&cmd_xTimerBuffer
	);
       
    xTimerStart(cmd_xTimer, 10);  
            
    for(;;)
    {
        u_kick_wdt(TK_CMD);
        
        switch (cmd_state) {
            
        case CMD_AWAKE:
 
            c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
            while ( xgetc( (char *)&c ) == 1 ) {
                if ( FRTOS_CMD_process(c) ) {
                    // Cada tecla que leo reseteo el timer
                    xTimerStart(cmd_xTimer, 10);
                }
            }  
            
            // xgetc ya espera 10ms !!
            //vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
            break;
            
        case CMD_SLEEP:
            
            // Duermo 30s para que la tarea entre en modo tickless.
            vTaskDelay( ( TickType_t)(30000 / portTICK_PERIOD_MS ) );
            c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
            while ( xgetc( (char *)&c ) == 1 ) {
                FRTOS_CMD_process(c);
                
                // Si presione una tecla me despierto. Reseteo el timer
                // Lo reconfiguro siempre para 2MINS.
                xTimerChangePeriod( cmd_xTimer, pdSECS_TO_TICKS(120), 10 );
                xTimerStart(cmd_xTimer, 10); 
                cmd_state = CMD_AWAKE;
            } 
            
            // Indico que desperte y cambio de estado
            if (cmd_state == CMD_AWAKE ) {
                xprintf_P(PSTR("tkCmd awake..\r\n" ));
            }
            
            break;
        }                     
    }      
}

//------------------------------------------------------------------------------
static void cmd_TimerCallback( TimerHandle_t xTimer )
{
    // Indica que la tkCMD debe entrar en modo sleep.
    cmd_state = CMD_SLEEP;
    xprintf_P(PSTR("tkCmd sleep..\r\n" ));
}
//------------------------------------------------------------------------------
static void cmdHelpFunction(void)
{

    FRTOS_CMD_makeArgv();
        
    if ( !strcmp_P( strupr(argv[1]), PSTR("WRITE"))) {
		xprintf_P( PSTR("-write:\r\n"));
        xprintf_P( PSTR("  (ee,nvmee,rtcram) {pos string} {debug}\r\n"));
        xprintf_P( PSTR("  rtc YYMMDDhhmm\r\n"));
        xprintf_P( PSTR("  ina {0|1} {confValue|AWAKE|SLEEP}\r\n"));

        //xprintf_P( PSTR("  ain_sensors_pwr {on|off}\r\n"));
        
    }  else if ( !strcmp_P( strupr(argv[1]), PSTR("READ"))) {
		xprintf_P( PSTR("-read:\r\n"));
        xprintf_P( PSTR("  (ee,nvmee,rtcram) {pos} {lenght} {debug}\r\n"));
        xprintf_P( PSTR("  avrid,rtc {long,short}\r\n"));
        xprintf_P( PSTR("  ina {0|1} {conf|chXshv|chXbusv|mfid|dieid}\r\n"));
        xprintf_P( PSTR("  ainput {n}\r\n"));
        xprintf_P( PSTR("  cnt, cnt_pin\r\n"));
        xprintf_P( PSTR("  bat3v3,bat12v\r\n"));
        
	} else if (!strcmp_P( strupr(argv[1]), PSTR("RESET"))) {
		xprintf_P( PSTR("-reset\r\n"));
        xprintf_P( PSTR("  memory {soft|hard}\r\n"));
		return;
        
    }  else if ( !strcmp_P( strupr(argv[1]), PSTR("CONFIG"))) {
		xprintf_P( PSTR("-config:\r\n"));
        xprintf_P( PSTR("  default {SPY,OSE}, save, load\r\n"));
        xprintf_P( PSTR("  modocomando\r\n"));
        xprintf_P( PSTR("  debug {none,ainput} {true/false}\r\n"));
        xprintf_P( PSTR("  ainput {0..%d} enable{true/false} aname imin imax mmin mmax offset\r\n"),( NRO_ANALOG_CHANNELS - 1 ) );
        xprintf_P( PSTR("  dlgid\r\n"));
        xprintf_P( PSTR("  timerpoll,timerdial\r\n"));
        xprintf_P( PSTR("  pwrmodo {continuo,discreto,mixto}, pwron {hhmm}, pwroff {hhmm}\r\n"));
        xprintf_P( PSTR("  counter enable{true/false} cname magPP modo(PULSO/CAUDAL)\r\n") );
        
    } else if (!strcmp_P( strupr(argv[1]), PSTR("TEST"))) {
		xprintf_P( PSTR("-test\r\n"));
        //xprintf_P( PSTR("  kill, wkill {wan,sys,modemrx}\r\n"));
        xprintf_P( PSTR("  valve {open|close}\r\n"));
        xprintf_P( PSTR("        {enable|ctl} {on|off}\r\n"));
        xprintf_P( PSTR("  pwr_sens3v3, pwr_sens12V,pwr_sensors {on|off}\r\n"));

        //xprintf_P( PSTR("  sens3v3, sens12V, pwr_sensors {enable|disable}\r\n"));
        //xprintf_P( PSTR("  pwr_cpres,pwr_sensext,pwr_qmbus {enable|disable}\r\n"));
        //xprintf_P( PSTR("  rts {on|off}\r\n"));
        //xprintf_P( PSTR("  modbus genpoll {slaaddr,regaddr,nro_regs,fcode,type,codec}\r\n"));
        //xprintf_P( PSTR("         chpoll {ch}\r\n"));
        //xprintf_P( PSTR("  lte (dcin,vcap,pwr,reset,reload} {on|off}\r\n"));
        //xprintf_P( PSTR("      {on|off}\r\n"));
        //xprintf_P( PSTR("      link\r\n"));

        //xprintf_P( PSTR("  modem {prender|apagar|atmode|exitat|queryall|ids|save}\r\n"));
        //xprintf_P( PSTR("  modem set [apn {apn}, apiurl {apiurl}, server {ip port}], ftime {time_ms}]\r\n"));
        //xprintf_P( PSTR("  piloto {pres}\r\n"));
        //xprintf_P( PSTR("  rs485A write, read\r\n"));
        return;
        
    }  else {
        // HELP GENERAL
        xprintf("Available commands are:\r\n");
        xprintf("-cls\r\n");
        xprintf("-help\r\n");
        xprintf("-reset\r\n");
        xprintf("-write...\r\n");
        xprintf("-read...\r\n");
        xprintf("-status\r\n");
        xprintf("-config...\r\n");
        xprintf("-test...\r\n");

    }
   
	xprintf("Exit help \r\n");

}
//------------------------------------------------------------------------------
static void cmdClsFunction(void)
{
	// ESC [ 2 J
	xprintf("\x1B[2J\0");
}
//------------------------------------------------------------------------------
static void cmdResetFunction(void)
{
    
    FRTOS_CMD_makeArgv();
  
    // Reset memory ??
    if (!strcmp_P( strupr(argv[1]), PSTR("MEMORY"))) {
        
        /*
         * No puedo estar usando la memoria !!!
         */                       
        if ( !strcmp_P( strupr(argv[2]), PSTR("SOFT"))) {
			FS_format(false );
		} else if ( !strcmp_P( strupr(argv[2]), PSTR("HARD"))) {
			FS_format(true);
		} else {
			xprintf_P( PSTR("ERROR\r\nUSO: reset memory {hard|soft}\r\n"));
			return;
		}
    }

    reset();
}
//------------------------------------------------------------------------------
static void cmdReadFunction(void)
{
    
    FRTOS_CMD_makeArgv();       
        
    // SENS3V3
    // read bat3v3
	if (!strcmp_P( strupr(argv[1]), PSTR("BAT3V3")) ) {
        u_read_bat3v3(true);
        pv_snprintfP_OK();
		return;
	} 

    // SENS12V
    // read bat12v
	if (!strcmp_P( strupr(argv[1]), PSTR("BAT12V")) ) {
        u_read_bat12v(true);
        pv_snprintfP_OK();
		return;
	} 
    
    // CONTADOR
    // read cnt
	if (!strcmp_P( strupr(argv[1]), PSTR("CNT")) ) {
		xprintf_P(PSTR("CNT=%d\r\n"), counter_read());
        pv_snprintfP_OK();
		return;
	} 

    // read cnt_pin
	if (!strcmp_P( strupr(argv[1]), PSTR("CNT_PIN")) ) {
		xprintf_P(PSTR("CNT=%d\r\n"), counter_read_pin());
        pv_snprintfP_OK();
		return;
	} 
    
    // AINPUT
    // read ainput {n}
    if (!strcmp_P( strupr(argv[1]), PSTR("AINPUT"))  ) {
        ainputs_test_read_channel( atoi(argv[2]) ) ? pv_snprintfP_OK(): pv_snprintfP_ERR();
		return;
	}
    
    // NVMEE
	// read nvmee address length
	if (!strcmp_P( strupr(argv[1]), PSTR("NVMEE")) ) {
		NVMEE_test_read ( argv[2], argv[3] );
		return;
	}

	// AVRID
	// read avrid
	if (!strcmp_P( strupr(argv[1]), PSTR("AVRID"))) {
        xprintf_P(PSTR("ID: %s\r\n"), NVM_id2str() );
        xprintf_P(PSTR("SIGNATURE: %s\r\n"), NVM_signature2str() );
		return;
	}

    // INA
	// read ina regName
	if (!strcmp_P( strupr(argv[1]), PSTR("INA"))  ) {
		INA_test_read ( argv[2], argv[3] );
		return;
	}
    
    // RTC
	// read rtc { long | short } 
    if (!strcmp_P( strupr(argv[1]), PSTR("RTC")) ) {
        if (!strcmp_P( strupr(argv[2]), PSTR("LONG")) ) {
            RTC_read_time(FORMAT_LONG);
            pv_snprintfP_OK();
            return;
        }
        if (!strcmp_P( strupr(argv[2]), PSTR("SHORT")) ) {
            RTC_read_time(FORMAT_SHORT);
            pv_snprintfP_OK();
            return;
        }
        pv_snprintfP_ERR();
        return;
    }
    
    // EE
	// read ee address length
	if (!strcmp_P( strupr(argv[1]), PSTR("EE")) ) {
		EE_test_read ( argv[2], argv[3], argv[4] );
		return;
	}
     
    // CMD NOT FOUND
	xprintf("ERROR\r\nCMD NOT DEFINED\r\n\0");
	return;
 
}
//------------------------------------------------------------------------------
static void cmdWriteFunction(void)
{

    FRTOS_CMD_makeArgv();
 
    // NVMEE
	// write nvmee pos string
	if ( (strcmp_P( strupr(argv[1]), PSTR("NVMEE")) == 0)) {
		NVMEE_test_write ( argv[2], argv[3] );
		pv_snprintfP_OK();
		return;
	}
    
    // INA
	// write ina rconfValue
	// Solo escribimos el registro 0 de configuracion.
	if ((strcmp_P( strupr(argv[1]), PSTR("INA")) == 0) ) {
		( INA_test_write ( argv[2], argv[3] ) > 0)?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}
    
    // RTC
	// write rtc YYMMDDhhmm
	if ( strcmp_P( strupr(argv[1]), PSTR("RTC")) == 0 ) {
		( RTC_write_time( argv[2]) > 0)?  pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

    // EE
	// write ee pos string
	if ((strcmp_P( strupr(argv[1]), PSTR("EE")) == 0) ) {
		( EE_test_write ( argv[2], argv[3], argv[4] ) > 0)?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

    // CMD NOT FOUND
	xprintf("ERROR\r\nCMD NOT DEFINED\r\n\0");
	return;
 
}
//------------------------------------------------------------------------------
static void cmdStatusFunction(void)
{

    // https://stackoverflow.com/questions/12844117/printing-defined-constants

TickType_t xRemainingTime;
fat_s l_fat;

    xprintf("Spymovil %s %s TYPE=%s, VER=%s %s \r\n" , HW_MODELO, FRTOS_VERSION, FW_TYPE, FW_REV, FW_DATE);
    xprintf_P(PSTR("id: %s\r\n"), NVM_id2str());
    xprintf_P(PSTR("sign: %s\r\n"), NVM_signature2str());
    
    xprintf_P(PSTR("Config:\r\n"));
    xprintf_P(PSTR(" date: %s\r\n"), RTC_logprint(FORMAT_LONG));
    
    xRemainingTime = xTimerGetExpiryTime( cmd_xTimer ) - xTaskGetTickCount();
    xprintf_P(PSTR("Remain time: %lu s\r\n"), pdTICS_TO_SECS(xRemainingTime) );
    
        // Stats de memoria
    FAT_read(&l_fat);
    xprintf_P( PSTR(" memory: wrPtr=%d,rdPtr=%d,count=%d\r\n"),l_fat.head,l_fat.tail, l_fat.count );

    base_print_configuration();
    ainputs_print_configuration();
    counter_print_configuration();
    
}
//------------------------------------------------------------------------------
static void cmdTestFunction(void)
{

    FRTOS_CMD_makeArgv();
    
    // pwr_sens3v3, pwr_sens12V, pwr_sensors{enable|disable}
    if (!strcmp_P( strupr(argv[1]), PSTR("PWR_SENS12V"))  ) {
               
        if (!strcmp_P( strupr(argv[2]), PSTR("ON"))  ) {
            SET_EN_SENS12V();
            pv_snprintfP_OK();
            return;
        }        

        if (!strcmp_P( strupr(argv[2]), PSTR("OFF"))  ) {
            CLEAR_EN_SENS12V();
            pv_snprintfP_OK();
            return;
        } 
        
        pv_snprintfP_ERR();
        return;
    }
    
    if (!strcmp_P( strupr(argv[1]), PSTR("PWR_SENS3V3"))  ) {
               
        if (!strcmp_P( strupr(argv[2]), PSTR("ON"))  ) {
            SET_EN_SENS3V3();
            pv_snprintfP_OK();
            return;
        }        

        if (!strcmp_P( strupr(argv[2]), PSTR("OFF"))  ) {
            CLEAR_EN_SENS3V3();
            pv_snprintfP_OK();
            return;
        } 
        
        pv_snprintfP_ERR();
        return;
    }
    
    if (!strcmp_P( strupr(argv[1]), PSTR("PWR_SENSORS"))  ) {
               
        if (!strcmp_P( strupr(argv[2]), PSTR("ON"))  ) {
            ainputs_prender_sensores();
            pv_snprintfP_OK();
            return;
        }        

        if (!strcmp_P( strupr(argv[2]), PSTR("OFF"))  ) {
            ainputs_apagar_sensores();
            pv_snprintfP_OK();
            return;
        } 
        
        pv_snprintfP_ERR();
        return;
    }

    pv_snprintfP_ERR();
    return;
       
}
//------------------------------------------------------------------------------
static void cmdConfigFunction(void)
{
    
    FRTOS_CMD_makeArgv();

    // COUNTER
    // counter enable cname magPP modo(PULSO/CAUDAL)
	if (!strcmp_P( strupr(argv[1]), PSTR("COUNTER")) ) {
        counter_config_channel( argv[2], argv[3], argv[4], argv[5] );
        pv_snprintfP_OK();
		return;
	}
    
    // POWER
    // pwr_modo {continuo,discreto,mixto}
    if (!strcmp_P( strupr(argv[1]), PSTR("PWRMODO"))) {
        base_config_pwrmodo(argv[2]) ? pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}
    
    // pwr_on {hhmm}
     if (!strcmp_P( strupr(argv[1]), PSTR("PWRON"))) {
        base_config_pwron(argv[2]) ? pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}  
    
    // pwr_off {hhmm}
     if (!strcmp_P( strupr(argv[1]), PSTR("PWROFF"))) {
        base_config_pwroff(argv[2]) ? pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}   
            
    // DLGID
	if (!strcmp_P( strupr(argv[1]), PSTR("DLGID"))) {
        base_config_dlgid(argv[2]) ? pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
    }
    
    // TIMERPOLL
    // config timerpoll val
	if (!strcmp_P( strupr(argv[1]), PSTR("TIMERPOLL")) ) {
        base_config_timerpoll(argv[2]) ? pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}
    
    // TIMERDIAL
    // config timerdial val
	if (!strcmp_P( strupr(argv[1]), PSTR("TIMERDIAL")) ) {
		base_config_timerdial(argv[2]) ? pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

    // SAVE
	// config save
	if (!strcmp_P( strupr(argv[1]), PSTR("SAVE"))) {       
		u_save_config_in_NVM();
		pv_snprintfP_OK();
		return;
	}
    
    // LOAD
	// config load
	if (!strcmp_P( strupr(argv[1]), PSTR("LOAD"))) {
		u_load_config_from_NVM();
		pv_snprintfP_OK();
		return;
	}
    
    // DEFAULT
	// config default
	if (!strcmp_P( strupr(argv[1]), PSTR("DEFAULT"))) {
		u_config_default(argv[2]);
		pv_snprintfP_OK();
		return;
	}
    
    // DEBUG
    // none,config debug (ainput, counter, wan, consigna) (true,false)
    if (!strcmp_P( strupr(argv[1]), PSTR("DEBUG")) ) {
        u_config_debug( argv[2], argv[3]) ? pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
    }
    
    // AINPUT
	// ainput {0..%d} enable aname imin imax mmin mmax offset
	if (!strcmp_P( strupr(argv[1]), PSTR("AINPUT")) ) {
		ainputs_config_channel ( atoi(argv[2]), argv[3], argv[4], argv[5], argv[6], argv[7], argv[8], argv[9]);
        pv_snprintfP_OK();
		return;
	}

    
    // MODO COMANDO:
    // config modocomando
    if ( strcmp_P ( strupr( argv[1]), PSTR("MODOCOMANDO")) == 0 ) {
        // Cambio el periodo a 60 mins.
        xTimerChangePeriod( cmd_xTimer, pdSECS_TO_TICKS(3600), 10 );
        xTimerStart(cmd_xTimer, 10); 
        pv_snprintfP_OK();
        return;
    }
    
    // CMD NOT FOUND
	xprintf("ERROR\r\nCMD NOT DEFINED\r\n\0");
	return;
 
}
//------------------------------------------------------------------------------
static void pv_snprintfP_OK(void )
{
	xprintf("ok\r\n\0");
}
//------------------------------------------------------------------------------
static void pv_snprintfP_ERR(void)
{
	xprintf("error\r\n\0");
}
//------------------------------------------------------------------------------
