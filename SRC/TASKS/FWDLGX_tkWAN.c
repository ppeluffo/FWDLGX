#include "FWDLGX.h"
#include "frtos_cmd.h"

SemaphoreHandle_t sem_WAN;
StaticSemaphore_t WAN_xMutexBuffer;
#define MSTOTAKEWANSEMPH ((  TickType_t ) 10 )

static bool f_debug_comms = false;

typedef enum { WAN_APAGADO=0, WAN_OFFLINE, WAN_ONLINE_CONFIG, WAN_ONLINE_DATA } wan_states_t;

typedef enum { DATA=0, BLOCK, BLOCKEND } tx_type_t;

static uint8_t wan_state;

struct {
    dataRcd_s dr;
    bool dr_ready;
} drWanBuffer;

bool link_up4data;

static void wan_state_apagado(void);
static void wan_state_offline(void);
static void wan_state_online_config(void);
static void wan_state_online_data(void);

static bool wan_process_frame_ping(void);
static bool wan_process_rsp_ping(void);

static int8_t wan_process_frame_recoverId(void);
static int8_t wan_process_rsp_recoverId(void);

static int8_t wan_process_frame_configAll(void);
static int8_t wan_process_rsp_configAll(void);

struct {
    bool conf_base;
    bool conf_ainputs;
    bool conf_counter;
    bool conf_modbus;
    bool conf_presion;
} conf_all_flags;

static int8_t wan_process_frame_configBase(void);
static int8_t wan_process_rsp_configBase(void);
static int8_t wan_process_frame_configAinputs(void);
static int8_t wan_process_rsp_configAinputs(void);
static int8_t wan_process_frame_configCounters(void);
static int8_t wan_process_rsp_configCounters(void);
static int8_t wan_process_frame_configModbus(void);
static int8_t wan_process_rsp_configModbus(void);
static int8_t wan_process_frame_configConsigna(void);
static int8_t wan_process_rsp_configConsigna(void);

static bool wan_send_from_memory(void);
//static bool wan_bulk_send_from_memory(void);

static bool wan_process_frame_data(dataRcd_s *dr, bool response);
static void wan_load_dr_in_txbuffer(dataRcd_s *dr, uint8_t *buff, uint16_t buffer_size, bool response);

static bool wan_process_rsp_data(void);

bool wan_process_from_dump(char *buff, bool ultimo );

void wan_prender_modem(void);
void wan_apagar_modem(void);

#define DEBUG_WAN       true
#define NO_DEBUG_WAN    false

bool f_inicio;

static bool wan_check_response ( const char *s);
static void wan_xmit_out(void);
static void wan_print_RXbuffer(void);

fat_s l_fat1;

uint16_t uxHighWaterMark;

char tmpLocalStr[64] = { 0 };

#define PING_TRYES              5
#define RECOVERID_TRYES         3
#define CONFIGBASE_TRYES        3
#define CONFIGAIPUTS_TRYES      3
#define CONFIGCOUNTERS_TRYES    3
#define CONFIGCONSIGNA_TRYES    3
#define CONFIGPILOTO_TRYES      3
#define CONFIGMODBUS_TRYES      3
#define DATA_TRYES              3


//------------------------------------------------------------------------------
void tkWan(void * pvParameters)
{

	/*
     * Esta tarea el la FSM de la WAN.
     * Cuando hay datos para transmitir recibe una se�al y los transmite.
     * De otro modo, espera datos ( respuestas ) y las procesa.
     * El servidor solo envia respuestas: no trasmite por iniciativa.
     */

( void ) pvParameters;

    while ( ! starting_flag )
        vTaskDelay( ( TickType_t)( 100 / portTICK_PERIOD_MS ) );
    
    SYSTEM_ENTER_CRITICAL();
    tk_running[TK_WAN] = true;
    SYSTEM_EXIT_CRITICAL();
    
    sem_WAN = xSemaphoreCreateMutexStatic( &WAN_xMutexBuffer );
   
#ifdef TESTING_MODEM
    xprintf_P(PSTR("WAN::<%s>: Starting tkWan..\r\n"), RTC_logprint(true));
#else
    xprintf_P(PSTR("Starting tkWan..\r\n"));
#endif
    
    vTaskDelay( ( TickType_t)( 500 / portTICK_PERIOD_MS ) );
    
    wan_state = WAN_APAGADO;

    f_inicio = true;
    
	// loop
	for( ;; )
	{       
        switch(wan_state) {
            case WAN_APAGADO:
                wan_state_apagado();
                break;
            case WAN_OFFLINE:
                wan_state_offline();
                break;
            case WAN_ONLINE_CONFIG:
                wan_state_online_config();
                break;
            case WAN_ONLINE_DATA:
                wan_state_online_data();
                break;
            default:
                xprintf_P(PSTR("ERROR: WAN STATE ??\r\n"));
                wan_state = WAN_APAGADO;
                break;
        }
        vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
	}    
}
//------------------------------------------------------------------------------
static void wan_state_apagado(void)
{
    /*
     * Apago el modem y espero.
     * Al salir siempre prendo el modem.
     */
    
uint32_t waiting_secs;

//    uxHighWaterMark = SPYuxTaskGetStackHighWaterMark( NULL );
//    xprintf_P(PSTR("STACK apagado_in = %d\r\n"), uxHighWaterMark );
    
    u_kick_wdt(TK_WAN, T120S);
    
#ifdef TESTING_MODEM
    xprintf_P(PSTR("WAN::<%s>: State APAGADO\r\n"), RTC_logprint(true));
#else
    xprintf_P(PSTR("WAN:: State APAGADO\r\n"));
#endif
    
    // Siempre al entrar debo apagar el modem.
    wan_apagar_modem();
    vTaskDelay( ( TickType_t)( 5000 / portTICK_PERIOD_MS ) );
    
    // Cuando inicio me conecto siempre sin esperar para configurarme y vaciar la memoria.
    if ( f_inicio ) {
        f_inicio = false;
        goto exit;
    }
       
    // Veo cuanto esperar con el modem apagado.
    waiting_secs = u_get_sleep_time(f_debug_comms);
    
    // Modo continuo
    if ( waiting_secs == 0 ) {
        vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
        goto exit;        
    }
    
    // Espero intervalos de 60 secs monitoreando las se�ales
    while ( waiting_secs > 60 ) {
        u_kick_wdt(TK_WAN, T120S);
        vTaskDelay( ( TickType_t)(60000 / portTICK_PERIOD_MS ) );
        waiting_secs -= 60;
    }
       
    // Espero el saldo
    u_kick_wdt(TK_WAN, T120S);
    vTaskDelay( ( TickType_t)( waiting_secs * 1000 / portTICK_PERIOD_MS ) );

exit:
    
    // Salimos prendiendo el modem 
    wan_prender_modem();
    wan_state = WAN_OFFLINE;
       
//    uxHighWaterMark = SPYuxTaskGetStackHighWaterMark( NULL );
//    xprintf_P(PSTR("STACK apagado_out = %d\r\n"), uxHighWaterMark );

    return;
  
}
//------------------------------------------------------------------------------
static void wan_state_offline(void)
{
    /*
     * Espera que exista link.
     * Para esto envia cada 10s un PING hacia el servidor.
     * Cuando recibe respuesta (PONG), pasa a la fase de enviar
     * los frames de re-configuracion.
     * Las respuestas del server las maneja el callback de respuestas.
     * Este prende flags para indicar al resto que tipo de respuesta llego.
     */
 
uint8_t i;
bool atmode = false;
//bool save_dlg_config = false;
//bool retS = false;

//    uxHighWaterMark = SPYuxTaskGetStackHighWaterMark( NULL );
//    xprintf_P(PSTR("STACK offline_in = %d\r\n"), uxHighWaterMark );
    
    u_kick_wdt(TK_WAN, T120S);
   
#ifdef TESTING_MODEM
    xprintf_P(PSTR("WAN::<%s>: State OFFLINE\r\n"), RTC_logprint(true));
#else
    xprintf_P(PSTR("WAN:: State OFFLINE\r\n"));
#endif
    
    // Entro en modo AT
    for (i=0; i<3; i++) {
        vTaskDelay( ( TickType_t)( 5000 / portTICK_PERIOD_MS ) );
        xprintf_P(PSTR("WAN:: State OFFLINE modem enter AT (%d)\r\n"),i);
        atmode = MODEM_enter_mode_at(false);
        if (atmode)
            break;
    }
    
    // Si en 3 intentos no entre en modo AT, apago.
    if ( ! atmode ) {
        wan_state = WAN_APAGADO;
        goto quit;
    }
    
    
    // Estoy en modo AT
   
// -----------------------------------------------------------------------------
/*
 * Elimino esto para acelerar el proceso 
 */
    //if ( ! modem_check_and_reconfig(true, save_dlg_config) ) {
        /*
         * EL chequeo de la configuracion del modem indica que algo anda mal
         * y no puede seguir.
         */
    //    xprintf_P(PSTR("MODEM config ERROR. Reconfigure modem and datalogger. STOP\r\n"));
    //    wan_state = WAN_APAGADO;
    //    goto quit; 
    //}
    
    //if ( save_dlg_config ) {
    //    u_save_config_in_NVM();
    //}
    
    //vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
// -----------------------------------------------------------------------------

            
    // Leo el CSQ, IMEI, ICCID:
    modem_atcmd_read_iccid(true);
    modem_atcmd_read_imei(true);
    modem_atcmd_read_csq(true);
    modem_atcmd_read_localip(true);
    
    MODEM_exit_mode_at(false);
    
    if ( ! wan_process_frame_ping() ) {
        wan_state = WAN_APAGADO;
        goto quit;
    }
      
    wan_state = WAN_ONLINE_CONFIG;
    
quit:
   
    return;
}
//------------------------------------------------------------------------------
static void wan_state_online_config(void)
{
    /*
     * Se encarga de la configuracion.
     * Solo lo hace una sola vez por lo que tiene una flag estatica que
     * indica si ya se configuro.
     */
  
uint16_t i;
int8_t saveInMem = 0;
int8_t rsp = -1;

    u_kick_wdt(TK_WAN, T120S);
    
#ifdef TESTING_MODEM
    xprintf_P(PSTR("WAN::<%s>: State ONLINE_CONFIG\r\n"), RTC_logprint(true));
#else
    xprintf_P(PSTR("WAN:: State ONLINE_CONFIG\r\n"));
#endif
    
    if ( wan_process_frame_recoverId() == -1 ) {
       
        // Espero 1H y reintento.
        // Apago la tarea del system para no llenar la memoria al pedo
        
        vTaskSuspend( xHandle_tkSys );
        xprintf_P(PSTR("WAN:: ERROR RECOVER ID:Espero 1 h.\r\n"));
        for (i=0; i < 60; i++) {
             vTaskDelay( ( TickType_t)( 60000 / portTICK_PERIOD_MS ) );
             u_kick_wdt(TK_WAN, T120S);
        }
        
        xprintf("Reset..\r\n");
        reset();
        
    }
   
    rsp = wan_process_frame_configAll();
    if (  rsp == -1 ) {
        // No puedo configurarse o porque el servidor no responde
        // o porque da errores. Espero 60mins.
        xprintf_P(PSTR("WAN:: Errores en configuracion. Espero 60mins..!!\r\n"));
        base_conf.timerdial = 3600;
        base_conf.timerpoll = 3600;
        base_conf.pwr_modo = PWR_DISCRETO;
        wan_state = WAN_APAGADO;
        return;
    }
    
    if ( conf_all_flags.conf_base ) {
        rsp = wan_process_frame_configBase();
        if (  rsp == -1 ) {
            // No puedo configurarse o porque el servidor no responde
        // o porque da errores. Espero 60mins.
            xprintf_P(PSTR("WAN:: Errores en configuracion. Espero 60mins..!!\r\n"));
            base_conf.timerdial = 3600;
            base_conf.timerpoll = 3600;
            base_conf.pwr_modo = PWR_DISCRETO;
            wan_state = WAN_APAGADO;
            return;
        }
        saveInMem += rsp;
    }
    
    if ( conf_all_flags.conf_ainputs ) {
        rsp = wan_process_frame_configAinputs(); 
        if (rsp >= 0 ) {
            saveInMem += rsp;
        }
    }
    
    if ( conf_all_flags.conf_counter ) {
        rsp = wan_process_frame_configCounters();
        if (rsp >= 0 ) {
            saveInMem += rsp;
        }
    }
    
#ifdef HW_AVRDA
    
    if ( conf_all_flags.conf_modbus ) {
        rsp = wan_process_frame_configModbus();
        if (rsp >= 0 ) {
            saveInMem += rsp;
        } 
    }
    
    if ( conf_all_flags.conf_presion ) {
        rsp = wan_process_frame_configConsigna();
        if (rsp >= 0 ) {
            saveInMem += rsp;
        } 
    }
    
#endif
        
    if ( saveInMem > 0 ) {
        if ( ! u_save_config_in_NVM() ) {
            xprintf_P(PSTR("WAN ERROR saving NVM !!!\r\n"));
        }
    }
    
    wan_state = WAN_ONLINE_DATA;
    return;

}
//------------------------------------------------------------------------------
static void wan_state_online_data(void)
{
    /*
     * Este estado es cuando estamos en condiciones de enviar frames de datos
     * y procesar respuestas.
     * Al entrar, si hay datos en memoria los transmito todos.
     * Luego, si estoy en modo continuo, no hago nada: solo espero.
     * Si estoy en modo discreto, salgo
     */

bool res;

    u_kick_wdt(TK_WAN, T120S);

#ifdef TESTING_MODEM
    xprintf_P(PSTR("WAN::<%s>: State ONLINE_DATA\r\n"),RTC_logprint(true));
#else
    xprintf_P(PSTR("WAN:: State ONLINE_DATA\r\n"));
#endif
    
    link_up4data = true;
   
// ENTRY:
    
    // Si hay datos en memoria los transmito todos y los borro en bloques de a 8
    xprintf_P(PSTR("WAN:: ONLINE: dump memory...\r\n"));  
    // Vacio la memoria.
    wan_send_from_memory();  
    //wan_bulk_send_from_memory();
    // 
    // En modo discreto, apago y salgo
    if ( u_get_sleep_time(f_debug_comms) > 0 ) {
       wan_state = WAN_APAGADO;
       goto quit;
    }
    
    // En modo continuo me quedo esperando por datos para transmitir. 
    while( u_get_sleep_time(f_debug_comms) == 0 ) {
                      
        if ( drWanBuffer.dr_ready ) {
            // Hay datos para transmitir. Transmito y espero respuesta
            res =  wan_process_frame_data( &drWanBuffer.dr, true);
            if (res) {
                drWanBuffer.dr_ready = false;
            } else {
                // Cayo el enlace ???
                wan_state = WAN_OFFLINE;
                goto quit;
            }
        }
        
        //xprintf_P(PSTR("WAN:: Loop\r\n"));
        
        // Espero que hayan mas datos
        // Vuelvo a chequear el enlace cada 1 min( tickeless & wdg ).
        u_kick_wdt(TK_WAN, T120S);
        ulTaskNotifyTake( pdTRUE, ( TickType_t)( (60000) / portTICK_PERIOD_MS) );
    }
   
quit:
    
    link_up4data = false;
    return;        
}
//------------------------------------------------------------------------------
static bool wan_process_frame_ping(void)
{
    /*
     * Envia los PING para determinar si el enlace est� establecido
     * Intento durante 2 minutos mandando un ping cada 10s.
     */
    
uint16_t tryes;
uint16_t timeout;
bool retS = false;

#ifdef TESTING_MODEM
    xprintf_P(PSTR("WAN::<%s>: State LINK\r\n"), RTC_logprint(true));
#else
    xprintf_P(PSTR("WAN:: State LINK\r\n"));
#endif
 
    // Armo el frame
    while ( xSemaphoreTake( sem_WAN, MSTOTAKEWANSEMPH ) != pdTRUE )
        vTaskDelay( ( TickType_t)( 1 ) );
    
    memset(wan_tx_buffer, '\0', WAN_TX_BUFFER_SIZE);
    sprintf_P( wan_tx_buffer, PSTR("ID=%s&HW=%s&TYPE=%s&VER=%s&CLASS=PING"), base_conf.dlgid, HW, FW_TYPE, FW_REV );
        
    // Proceso
    tryes = PING_TRYES;
    while (tryes-- > 0) {
        
        // Envio el frame
        wan_xmit_out();
        // Espero respuesta chequeando cada 1s durante 15s.
        timeout = 15;
        while ( timeout-- > 0) {
            vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
            if ( wan_check_response("</html>") ) {
                wan_print_RXbuffer();
                if ( wan_check_response("CLASS=PONG")) {        
                    wan_process_rsp_ping();  
                    retS = true;
                    goto exit_;
                }
            }
        }
    }
    
    // Expiro el tiempo sin respuesta del server.
    xprintf_P(PSTR("WAN:: Link up timeout !!\r\n"));
    retS = false;
    
exit_:
               
    xSemaphoreGive( sem_WAN );
    return(retS);
 
}
//------------------------------------------------------------------------------
static bool wan_process_rsp_ping(void)
{

#ifdef TESTING_MODEM
    xprintf_P(PSTR("WAN::<%s>: Link up\r\n"), RTC_logprint(true));
#else
    xprintf_P(PSTR("WAN:: Link up\r\n"));
#endif
 
    
    return(true);
}
//------------------------------------------------------------------------------
static int8_t wan_process_frame_recoverId(void)
{
    /*
     * Este proceso es para recuperar el ID cuando localmente est� en DEFAULT.
     * Intento 2 veces mandar el frame.
     */
    
uint8_t tryes = 0;
uint8_t timeout = 0;
int8_t ret = -1;

#ifdef TESTING_MODEM
    xprintf_P(PSTR("WAN::<%s>: RECOVERID\r\n"), RTC_logprint(true));
#else
    xprintf_P(PSTR("WAN:: RECOVERID\r\n"));
#endif
    
    // Si el nombre es diferente de DEFAULT no tengo que hacer nada
    if ( strcmp_P( strupr( base_conf.dlgid ), PSTR("DEFAULT")) != 0 ) {
        xprintf_P(PSTR("WAN:: RECOVERID OK.\r\n"));
        ret = 0;
        return(ret);
    }

    // Armo el buffer
    while ( xSemaphoreTake( sem_WAN, MSTOTAKEWANSEMPH ) != pdTRUE )
        vTaskDelay( ( TickType_t)( 1 ) );
    
    memset(wan_tx_buffer, '\0', WAN_TX_BUFFER_SIZE);
    sprintf_P( wan_tx_buffer, PSTR("ID=%s&HW=%s&TYPE=%s&VER=%s&CLASS=RECOVER&UID=%s"), base_conf.dlgid, HW, FW_TYPE, FW_REV, NVM_signature2str());

    // Proceso. Envio hasta 2 veces el frame y espero hasta 10s la respuesta
    tryes = RECOVERID_TRYES;
    while (tryes-- > 0) {
        
        wan_xmit_out();
        // Espero respuesta chequeando cada 1s durante 10s.
        timeout = 15;
        while ( timeout-- > 0) {
            vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
            if ( wan_check_response("</html>") ) {
                
                wan_print_RXbuffer();
                
                if ( wan_check_response("CLASS=RECOVER")) {
                    ret = wan_process_rsp_recoverId();
                    goto exit_;
                }
                
                if ( wan_check_response("CONFIG=ERROR")) {
                    xprintf_P(PSTR("WAN:: RECOVERID ERROR: El servidor no reconoce al datalogger !!\r\n"));
                    ret = -1;
                    goto exit_;
                }
            }
        }
    }
    
    // Expiro el tiempo (3 envios) sin respuesta del server.
    xprintf_P(PSTR("WAN:: RECOVERID ERROR:Timeout en server rsp.!!\r\n"));
    ret = -1;
    
exit_:
               
    xSemaphoreGive( sem_WAN );

    return(ret);
}
//------------------------------------------------------------------------------
static int8_t wan_process_rsp_recoverId(void)
{
    /*
     * Extraemos el DLGID del frame y lo reconfiguramos
     * RXFRAME: <html><body><h1>CLASS=RECOVER&ID=xxxx</h1></body></html>
     *          <html><body><h1>CLASS=RECOVER&ID=DEFAULT</h1></body></html>
     */
    
char *stringp = NULL;
char *token = NULL;
char *delim = "&,;:=><";
char *ts = NULL;
char *p;
int8_t ret = -1;

    p = MODEM_get_buffer_ptr();
    //p = lBchar_get_buffer(&wan_rx_lbuffer);

    vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
	memset(tmpLocalStr,'\0',sizeof(tmpLocalStr));
	ts = strstr( p, "RECOVER");
	strncpy(tmpLocalStr, ts, sizeof(tmpLocalStr));
	stringp = tmpLocalStr;
	token = strsep(&stringp,delim);	    // RECOVER
	token = strsep(&stringp,delim);	 	// ID
	token = strsep(&stringp,delim);	 	// TEST01
	// Copio el dlgid recibido al systemConf.
	memset(base_conf.dlgid,'\0', DLGID_LENGTH );
	strncpy( base_conf.dlgid, token, DLGID_LENGTH);
	
    //save_config_in_NVM();
	xprintf_P( PSTR("WAN:: Reconfig DLGID to %s\r\n\0"), base_conf.dlgid );
    ret = 1;
	return(ret);
}
//------------------------------------------------------------------------------
static int8_t wan_process_frame_configAll(void)
{
     /*
      * Envo un frame con todos los hashes. El servidor me devuelve cual
      * modulo debo reconfigurar.
      * El server me puede mandar OK o la nueva configuracion que debo tomar.
      * Lo reintento 2 veces
      * 
      * RET:
      * -1: ERROR
      *  0: No reconfiguration
      * +1: reconfiguration
      * 
      */
    
uint8_t tryes = 0;
uint8_t timeout = 0;
int8_t ret = -1;
uint8_t b_hash = 0; 
uint8_t a_hash = 0;
uint8_t c_hash = 0;
uint8_t m_hash = 0;
uint8_t p_hash = 0;

#ifdef TESTING_MODEM
    xprintf_P(PSTR("WAN::<%s>: CONFIG_ALL\r\n"), RTC_logprint(true));
#else
    xprintf_P(PSTR("WAN:: CONFIG_ALL\r\n"));
#endif
    
    // Armo el buffer
    while ( xSemaphoreTake( sem_WAN, MSTOTAKEWANSEMPH ) != pdTRUE )
        vTaskDelay( ( TickType_t)( 1 ) );
    
    memset(wan_tx_buffer, '\0', WAN_TX_BUFFER_SIZE);
   
    conf_all_flags.conf_base = false;
    
    b_hash = base_hash();  
    a_hash = ainputs_hash();
    c_hash = counter_hash();
    m_hash = modbus_hash();
    p_hash = consigna_hash();     
    //snprintf( wan_tx_buffer, WAN_TX_BUFFER_SIZE,"ID=%s&TYPE=%s&VER=%s&CLASS=CONF_BASE&UID=%s&HASH=0x%02X", systemConf.ptr_base_conf->dlgid, FW_TYPE, FW_REV, NVM_signature2str(), hash );
    snprintf( wan_tx_buffer, WAN_TX_BUFFER_SIZE,
                        "ID=%s&HW=%s&TYPE=%s&VER=%s&CLASS=CONF_ALL&UID=%s&IMEI=%s&ICCID=%s&CSQ=%d&WDG=%d&BH=0x%02X&AH=0x%02X&CH=0x%02X&MH=0x%02X&PH=0x%02X", 
                        base_conf.dlgid, 
                        HW,
                        FW_TYPE, 
                        FW_REV, 
                        NVM_signature2str(), 
                        modem_atcmd_get_imei(),
                        modem_atcmd_get_iccid(),
                        modem_atcmd_get_csq(),
                        wdg_resetCause,
                        b_hash,
                        a_hash,
                        c_hash,
                        m_hash,
                        p_hash
                        );
   
    // Proceso. Envio hasta 3 veces el frame y espero hasta 10s la respuesta
    tryes = CONFIGBASE_TRYES;
    while (tryes-- > 0) {
        
        // Transmito y espero la respuesta </html>
        wan_xmit_out();  
        // Espero respuesta chequeando cada 1s durante 10s.
        timeout = 10;
        while ( timeout-- > 0) {
            vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
            if ( wan_check_response("</html>") ) {
               
                wan_print_RXbuffer();
            
                if ( wan_check_response("CONFIG=ERROR")) {
                    xprintf_P(PSTR("WAN:: CONF_ALL ERROR: El servidor no reconoce al datalogger !!\r\n"));
                    ret = -1;
                    goto exit_;
                }
                
                if ( wan_check_response("CONF_ALL&CONFIG=OK")) {
                    ret = 0;
                    goto exit_;
                }
                
                if ( wan_check_response("CLASS=CONF_ALL")) {
                    ret = wan_process_rsp_configAll();
                    goto exit_;
                } 
            }
        }
    }
    
    // Expiro el tiempo sin respuesta del server.
    xprintf_P(PSTR("WAN:: CONFIG_ALL ERROR: Timeout en server rsp.!!\r\n"));
    ret = -1;
    
exit_:
               
    xSemaphoreGive( sem_WAN );
    return(ret);   
}
//------------------------------------------------------------------------------
static int8_t wan_process_rsp_configAll(void)
{
    /*
     * Recibe la configuracion BASE.
     * RXFRAME: <html><body><h1>CLASS=CONF_ALL&BASE&COUNTER&AINPUT&MODBUS&PRESION</h1></body></html>                        
     *                          CLASS=CONF_ALL&CONFIG=OK
     *
     * RET:
     * -1: ERROR
     *  0: No reconfiguration
     * +1: reconfiguration
     *                           
     */
    
//char *stringp = NULL;
//char *token = NULL;
//char *delim = "&,;:=><";
//char *ts = NULL;
bool ret = false;
char *p;

    p = MODEM_get_buffer_ptr();
    //p = lBchar_get_buffer(&wan_rx_lbuffer);
    
    if  ( strstr( p, "CONFIG=OK") != NULL ) {
        ret = true;
        goto exit_;
    }
    
    if  ( strstr( p, "FAIL") != NULL ) {
        xprintf_P(PSTR("WAN:: CONF ERROR: El servidor no reconoce al frame !!\r\n"));
        // Reconfiguro para espera 1h.!!!
        xprintf_P(PSTR("WAN:: Reconfigurado para reintentar en 1H !!\r\n"));
        base_conf.pwr_modo = PWR_DISCRETO;
        base_conf.timerdial = 3600;
        base_conf.timerpoll = 3600;
        ret = false;
        goto exit_; 
    }
     
    if  ( strstr( p, "CONFIG=ERROR") != NULL ) {
        xprintf_P(PSTR("WAN:: CONF ERROR: El servidor no reconoce al datalogger !!\r\n"));
        // Reconfiguro para espera 1h.!!!
        xprintf_P(PSTR("WAN:: Reconfigurado para reintentar en 1H !!\r\n"));
        base_conf.pwr_modo = PWR_DISCRETO;
        base_conf.timerdial = 3600;
        base_conf.timerpoll = 3600;
        ret = false;
        goto exit_;    
    }
         
    vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
	memset(tmpLocalStr,'\0',sizeof(tmpLocalStr));
    
    if  ( strstr( p, "BASE") != NULL ) { 
        conf_all_flags.conf_base = true;
    }
    
    if  ( strstr( p, "AINPUT") != NULL ) { 
        conf_all_flags.conf_ainputs = true;
    }
    
    if  ( strstr( p, "MODBUS") != NULL ) { 
        conf_all_flags.conf_modbus = true;
    }
 
    if  ( strstr( p, "COUNTER") != NULL ) { 
        conf_all_flags.conf_counter = true;
    }
    
    if  ( strstr( p, "PRESION") != NULL ) { 
        conf_all_flags.conf_presion = true;
    }
    

    ret = true;
    
exit_:
       
    wdg_resetCause = 0x00;
    return(ret);      
}
//------------------------------------------------------------------------------
static int8_t wan_process_frame_configBase(void)
{
     /*
      * Envo un frame con el hash de la configuracion BASE.
      * El server me puede mandar OK o la nueva configuracion que debo tomar.
      * Lo reintento 2 veces
      * 
      * RET:
      * -1: ERROR
      *  0: No reconfiguration
      * +1: reconfiguration
      * 
      */
    
uint8_t tryes = 0;
uint8_t timeout = 0;
int8_t ret = -1;
uint8_t hash = 0;
  
#ifdef TESTING_MODEM
    xprintf_P(PSTR("WAN::<%s>: CONFIG_BASE\r\n"), RTC_logprint(true));
#else
    xprintf_P(PSTR("WAN:: CONFIG_BASE\r\n"));
#endif
    
    // Armo el buffer
    while ( xSemaphoreTake( sem_WAN, MSTOTAKEWANSEMPH ) != pdTRUE )
        vTaskDelay( ( TickType_t)( 1 ) );
    
    memset(wan_tx_buffer, '\0', WAN_TX_BUFFER_SIZE);
   
    hash = base_hash();      
    //snprintf( wan_tx_buffer, WAN_TX_BUFFER_SIZE,"ID=%s&TYPE=%s&VER=%s&CLASS=CONF_BASE&UID=%s&HASH=0x%02X", systemConf.ptr_base_conf->dlgid, FW_TYPE, FW_REV, NVM_signature2str(), hash );
    snprintf( wan_tx_buffer, WAN_TX_BUFFER_SIZE,
                        "ID=%s&HW=%s&TYPE=%s&VER=%s&CLASS=CONF_BASE&UID=%s&IMEI=%s&ICCID=%s&CSQ=%d&WDG=%d&HASH=0x%02X", 
                        base_conf.dlgid, 
                        HW,
                        FW_TYPE, 
                        FW_REV, 
                        NVM_signature2str(), 
                        modem_atcmd_get_imei(),
                        modem_atcmd_get_iccid(),
                        modem_atcmd_get_csq(),
                        wdg_resetCause,
                        hash );
   
    // Proceso. Envio hasta 3 veces el frame y espero hasta 10s la respuesta
    tryes = CONFIGBASE_TRYES;
    while (tryes-- > 0) {
        
        // Transmito y espero la respuesta </html>
        wan_xmit_out();  
        // Espero respuesta chequeando cada 1s durante 10s.
        timeout = 10;
        while ( timeout-- > 0) {
            vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
            if ( wan_check_response("</html>") ) {
               
                wan_print_RXbuffer();
            
                if ( wan_check_response("CONFIG=ERROR")) {
                    xprintf_P(PSTR("WAN:: CONF_BASE ERROR: El servidor no reconoce al datalogger !!\r\n"));
                    ret = -1;
                    goto exit_;
                }
                
                if ( wan_check_response("CONF_BASE&CONFIG=OK")) {
                    ret = 0;
                    goto exit_;
                }
                
                if ( wan_check_response("CLASS=CONF_BASE")) {
                    ret = wan_process_rsp_configBase();
                    goto exit_;
                } 
            }
        }
    }
    
    // Expiro el tiempo sin respuesta del server.
    xprintf_P(PSTR("WAN:: CONFIG_BASE ERROR: Timeout en server rsp.!!\r\n"));
    ret = -1;
    
exit_:
               
    xSemaphoreGive( sem_WAN );
    return(ret);   
}
//------------------------------------------------------------------------------
static int8_t wan_process_rsp_configBase(void)
{
    /*
     * Recibe la configuracion BASE.
     * RXFRAME: <html><body><h1>CLASS=CONF_BASE&TPOLL=30&TDIAL=900&PWRMODO=CONTINUO&PWRON=1800&PWROFF=1440</h1></body></html>                        
     *                          CLASS=CONF_BASE&CONFIG=OK
     *
     * RET:
     * -1: ERROR
     *  0: No reconfiguration
     * +1: reconfiguration
     *                           
     */
    
char *stringp = NULL;
char *token = NULL;
char *delim = "&,;:=><";
char *ts = NULL;
bool ret = -1;
char *p;

    p = MODEM_get_buffer_ptr();
    //p = lBchar_get_buffer(&wan_rx_lbuffer);
    
    if  ( strstr( p, "CONFIG=OK") != NULL ) {
        ret = 0;
        goto exit_;
    }
     
    if  ( strstr( p, "CONFIG=ERROR") != NULL ) {
        xprintf_P(PSTR("WAN:: CONF ERROR: El servidor no reconoce al datalogger !!\r\n"));
        // Reconfiguro para espera 1h.!!!
        xprintf_P(PSTR("WAN:: Reconfigurado para reintentar en 1H !!\r\n"));
        base_conf.pwr_modo = PWR_DISCRETO;
        base_conf.timerdial = 3600;
        base_conf.timerpoll = 3600;
        ret = -1;
        goto exit_;    
    }
         
    vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
	memset(tmpLocalStr,'\0',sizeof(tmpLocalStr));
	ts = strstr( p, "TPOLL=");
    if  ( ts != NULL ) {
        strncpy(tmpLocalStr, ts, sizeof(tmpLocalStr));
        stringp = tmpLocalStr;
        token = strsep(&stringp,delim);	 	// TPOLL
        token = strsep(&stringp,delim);	 	// timerpoll
        base_config_timerpoll(token);
        xprintf_P( PSTR("WAN:: Reconfig TIMERPOLL to %d\r\n\0"), base_conf.timerpoll );
    }
    //
    vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
    memset(tmpLocalStr,'\0',sizeof(tmpLocalStr));
	ts = strstr( p, "TDIAL=");
    if  ( ts != NULL ) {
        strncpy(tmpLocalStr, ts, sizeof(tmpLocalStr));
        stringp = tmpLocalStr;
        token = strsep(&stringp,delim);	 	// TDIAL
        token = strsep(&stringp,delim);	 	// timerdial
        base_config_timerdial(token);
        xprintf_P( PSTR("WAN:: Reconfig TIMERDIAL to %d\r\n\0"), base_conf.timerdial );
    }
    //
    vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
    memset(tmpLocalStr,'\0',sizeof(tmpLocalStr));
	ts = strstr( p, "PWRMODO=");
	if  ( ts != NULL ) {
        strncpy(tmpLocalStr, ts, sizeof(tmpLocalStr));
        stringp = tmpLocalStr;
        token = strsep(&stringp,delim);	 	// PWRMODO
        token = strsep(&stringp,delim);	 	// pwrmodo_string
        base_config_pwrmodo(token);
        xprintf_P( PSTR("WAN:: Reconfig PWRMODO to %s\r\n\0"), token );
    }
    //
    vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
    memset(tmpLocalStr,'\0',sizeof(tmpLocalStr));
	ts = strstr( p, "PWRON=");
	if  ( ts != NULL ) {
        strncpy(tmpLocalStr, ts, sizeof(tmpLocalStr));
        stringp = tmpLocalStr;
        token = strsep(&stringp,delim);	 	// PWRON
        token = strsep(&stringp,delim);	 	// pwron
        base_config_pwron(token);
        xprintf_P( PSTR("WAN:: Reconfig PWRON to %d\r\n\0"), base_conf.pwr_hhmm_on );
    }
    //
    vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
    memset(tmpLocalStr,'\0',sizeof(tmpLocalStr));
	ts = strstr( p, "PWROFF=");
	if  ( ts != NULL ) {
        strncpy(tmpLocalStr, ts, sizeof(tmpLocalStr));
        stringp = tmpLocalStr;
        token = strsep(&stringp,delim);	 	// PWROFF
        token = strsep(&stringp,delim);	 	// pwroff
        base_config_pwroff(token);
        xprintf_P( PSTR("WAN:: Reconfig PWROFF to %d\r\n\0"), base_conf.pwr_hhmm_off );
    }
    // 
    ret = 1;
    
exit_:
       
    wdg_resetCause = 0x00;
    return(ret);      
}
//------------------------------------------------------------------------------
static int8_t wan_process_frame_configAinputs(void)
{
     /*
      * Envo un frame con el hash de la configuracion de entradas analogicas.
      * El server me puede mandar OK o la nueva configuracion que debo tomar.
      * Lo reintento 2 veces
      */
    
uint8_t tryes = 0;
uint8_t timeout = 0;
int8_t ret = -1;
uint8_t hash = 0;

#ifdef TESTING_MODEM
    xprintf_P(PSTR("WAN::<%s>: CONFIG_AINPUTS\r\n"), RTC_logprint(true));
#else
    xprintf_P(PSTR("WAN:: CONFIG_AINPUTS\r\n"));
#endif
    
    // Armo el buffer
    while ( xSemaphoreTake( sem_WAN, MSTOTAKEWANSEMPH ) != pdTRUE )
        vTaskDelay( ( TickType_t)( 1 ) );
    
    memset(wan_tx_buffer, '\0', WAN_TX_BUFFER_SIZE);
    
    hash = ainputs_hash(); 
    snprintf( (char*)&wan_tx_buffer, WAN_TX_BUFFER_SIZE, "ID=%s&HW=%s&TYPE=%s&VER=%s&CLASS=CONF_AINPUTS&HASH=0x%02X", base_conf.dlgid, HW, FW_TYPE, FW_REV, hash );
    // Proceso. Envio hasta 2 veces el frame y espero hasta 10s la respuesta
    tryes = CONFIGAIPUTS_TRYES;
    while (tryes-- > 0) {
        
        //xprintf_P(PSTR("DEBUG [%s]\r\n"), wan_tx_buffer);
        
        wan_xmit_out();
        // Espero respuesta chequeando cada 1s durante 10s.
        timeout = 10;
        while ( timeout-- > 0) {
            vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
            if ( wan_check_response("</html>") ) {
              
                wan_print_RXbuffer();
            
                if ( wan_check_response("CONFIG=ERROR")) {
                    xprintf_P(PSTR("WAN:: CONF_AIPUTS ERROR: El servidor no reconoce al datalogger !!\r\n"));
                    ret = -1;
                    goto exit_;
                }
                
                if ( wan_check_response("CONF_AINPUTS&CONFIG=OK")) {
                    ret = 0;
                    goto exit_;
                }
                
                if ( wan_check_response("CLASS=CONF_AINPUTS")) {
                    ret = wan_process_rsp_configAinputs();
                    goto exit_;
                } 
            }
        }
    }
    
    // Expiro el tiempo sin respuesta del server.
    xprintf_P(PSTR("WAN:: CONFIG_AINPUTS ERROR: Timeout en server rsp.!!\r\n"));
    ret = -1;
    
exit_:
               
    xSemaphoreGive( sem_WAN );
    return(ret);      
}
//------------------------------------------------------------------------------
static int8_t wan_process_rsp_configAinputs(void)
{
   /*
     * Procesa la configuracion de los canales analogicos
     * RXFRAME: <html><body><h1>CLASS=CONF_AINPUTS&A0=true,pA,4,20,0.0,10.0,0.0&A1=true,pB,4,20,0.0,10.0,0.0&A2=false,X,4,20,0.0,10.0,0.0</h1></body></html>
     *                          CLASS=CONF_AINPUTS&CONFIG=OK
     */
    
char *ts = NULL;
char *stringp = NULL;
char *tk_name= NULL;
char *tk_enable= NULL;
char *tk_iMin= NULL;
char *tk_iMax = NULL;
char *tk_mMin = NULL;
char *tk_mMax = NULL;
char *tk_offset = NULL;
char *delim = "&,;:=><";
uint8_t ch;
char str_base[8];
char *p;
int8_t ret = -1;

    p = MODEM_get_buffer_ptr();
    //p = lBchar_get_buffer(&wan_rx_lbuffer);
       
    if  ( strstr( p, "CONFIG=OK") != NULL ) {
        ret = 0;
       goto exit_;
    }

	// A?
	for (ch=0; ch < NRO_ANALOG_CHANNELS; ch++ ) {
        
        vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
        
		memset( &str_base, '\0', sizeof(str_base) );
		snprintf_P( str_base, sizeof(str_base), PSTR("A%d=\0"), ch );

		if ( strstr( p, str_base) != NULL ) {
			memset(tmpLocalStr,'\0',sizeof(tmpLocalStr));
            ts = strstr( p, str_base);
			strncpy( tmpLocalStr, ts, sizeof(tmpLocalStr));
			stringp = tmpLocalStr;
			tk_name = strsep(&stringp,delim);		//A0
            tk_enable = strsep(&stringp,delim);     // enable
			tk_name = strsep(&stringp,delim);		//name
			tk_iMin = strsep(&stringp,delim);		//iMin
			tk_iMax = strsep(&stringp,delim);		//iMax
			tk_mMin = strsep(&stringp,delim);		//mMin
			tk_mMax = strsep(&stringp,delim);		//mMax
			tk_offset = strsep(&stringp,delim);		//offset

			ainputs_config_channel( ch, tk_enable, tk_name , tk_iMin, tk_iMax, tk_mMin, tk_mMax, tk_offset );
			xprintf_P( PSTR("WAN:: Reconfig A%d\r\n"), ch);
		}
	}
    ret = 1;
   
exit_:
                
    return(ret);
}
//------------------------------------------------------------------------------
static int8_t wan_process_frame_configCounters(void)
{
     /*
      * Envo un frame con el hash de la configuracion de contadores.
      * El server me puede mandar OK o la nueva configuracion que debo tomar.
      * Lo reintento 2 veces
      */
    
uint8_t tryes = 0;
uint8_t timeout = 0;
int8_t ret = -1;
uint8_t hash = 0;

#ifdef TESTING_MODEM
    xprintf_P(PSTR("WAN::<%s>: CONFIG_COUNTERS\r\n"), RTC_logprint(true));
#else
    xprintf_P(PSTR("WAN:: CONFIG_COUNTERS\r\n"));
#endif
    
    // Armo el buffer
    while ( xSemaphoreTake( sem_WAN, MSTOTAKEWANSEMPH ) != pdTRUE )
        vTaskDelay( ( TickType_t)( 1 ) );
    
    memset(wan_tx_buffer, '\0', WAN_TX_BUFFER_SIZE);
    
    hash = counter_hash();
    sprintf_P( (char*)&wan_tx_buffer, PSTR("ID=%s&HW=%s&TYPE=%s&VER=%s&CLASS=CONF_COUNTERS&HASH=0x%02X"), base_conf.dlgid, HW, FW_TYPE, FW_REV, hash );

    // Proceso. Envio hasta 2 veces el frame y espero hasta 10s la respuesta
    tryes = CONFIGCOUNTERS_TRYES;
    while (tryes-- > 0) {
        
        wan_xmit_out();
        // Espero respuesta chequeando cada 1s durante 10s.
        timeout = 10;
        while ( timeout-- > 0) {
            vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
            if ( wan_check_response("</html>") ) {
                
                wan_print_RXbuffer();
            
                if ( wan_check_response("CONFIG=ERROR")) {
                    xprintf_P(PSTR("WAN:: CONF_COUNTERS ERROR: El servidor no reconoce al datalogger !!\r\n"));
                    ret = -1;
                    goto exit_;
                }
                
                if ( wan_check_response("CONF_COUNTERS&CONFIG=OK")) {
                    ret = 0;
                    goto exit_;
                }
                
                if ( wan_check_response( "CLASS=CONF_COUNTERS")) {
                    ret = wan_process_rsp_configCounters();
                    goto exit_;
                } 
            }
        }
    }
 
    // Expiro el tiempo sin respuesta del server.
    xprintf_P(PSTR("WAN:: CONFIG_COUNTERS ERROR: Timeout en server rsp.!!\r\n"));
    ret = -1;
    
exit_:
               
    xSemaphoreGive( sem_WAN );
    return(ret);   
  
}
//------------------------------------------------------------------------------
static int8_t wan_process_rsp_configCounters(void)
{
    /*
     * Procesa la configuracion de los canales contadores
     * RXFRAME: <html><body><h1>CLASS=CONF_COUNTERS&C0=TRUE,CAU0,5.3,CAUDAL</h1></body></html>
     * 
     */

char *ts = NULL;
char *stringp = NULL;
char *tk_name = NULL;
char *tk_enable= NULL;
char *tk_magpp = NULL;
char *tk_modo = NULL;
char *delim = "&,;:=><";
char str_base[8];
char *p;
int8_t ret = -1;

    p = MODEM_get_buffer_ptr();
    //p = lBchar_get_buffer(&wan_rx_lbuffer);
    
    if  ( strstr( p, "CONFIG=OK") != NULL ) {
       ret = 0;
       goto exit_;
    }
 
	memset( &str_base, '\0', sizeof(str_base) );
	snprintf_P( str_base, sizeof(str_base), PSTR("C0") );

	if ( strstr( p, str_base) != NULL ) {
		memset(tmpLocalStr,'\0',sizeof(tmpLocalStr));
		ts = strstr( p, str_base);
		strncpy(tmpLocalStr, ts, sizeof(tmpLocalStr));
		stringp = tmpLocalStr;
		tk_enable = strsep(&stringp,delim);		//C0
        tk_enable = strsep(&stringp,delim);     //enable
		tk_name = strsep(&stringp,delim);		//name
		tk_magpp = strsep(&stringp,delim);		//magpp
		tk_modo = strsep(&stringp,delim);       //modo

        //xprintf_P(PSTR("DEBUG: ch=%d,enable=%s,name=%s magpp=%s,modo=%s,rbsize=%s\r\n"), ch, tk_enable, tk_name, tk_magpp, tk_modo, tk_rbsize);
        counter_config_channel( tk_enable, tk_name , tk_magpp, tk_modo );         
		xprintf_P( PSTR("WAN:: Reconfig C0\r\n"));
	}
    
    ret = 1;
    
exit_:
               
	return(ret);

}
//------------------------------------------------------------------------------
static int8_t wan_process_frame_configModbus(void)
{
     /*
      * Envo un frame con el hash de la configuracion de los canales modbus.
      * El server me puede mandar OK o la nueva configuracion que debo tomar.
      * Lo reintento 2 veces
      * 
      * RET:
      * -1: ERROR
      *  0: No reconfiguration
      * +1: reconfiguration
      * 
      */
    
uint8_t tryes = 0;
uint8_t timeout = 0;
int8_t ret = -1;
uint8_t hash = 0;

#ifdef TESTING_MODEM
    xprintf_P(PSTR("WAN::<%s>: CONFIG_MODBUS\r\n"), RTC_logprint(true));
#else
    xprintf_P(PSTR("WAN:: CONFIG_MODBUS\r\n"));
#endif
    
    // Armo el buffer
    while ( xSemaphoreTake( sem_WAN, MSTOTAKEWANSEMPH ) != pdTRUE )
        vTaskDelay( ( TickType_t)( 1 ) );
    memset(wan_tx_buffer, '\0', WAN_TX_BUFFER_SIZE);
    
    hash = modbus_hash();
    sprintf_P( (char*)&wan_tx_buffer, PSTR("ID=%s&HW=%s&TYPE=%s&VER=%s&CLASS=CONF_MODBUS&HASH=0x%02X"), base_conf.dlgid, HW, FW_TYPE, FW_REV, hash );
    
    // Proceso. Envio hasta 2 veces el frame y espero hasta 10s la respuesta
    tryes = CONFIGMODBUS_TRYES;
    while (tryes-- > 0) {
        
        wan_xmit_out();
        // Espero respuesta chequeando cada 1s durante 10s.
        timeout = 10;
        while ( timeout-- > 0) {
            vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
            if ( wan_check_response("</html>") ) {
                
                wan_print_RXbuffer();
                
                if ( wan_check_response("CONFIG=ERROR")) {
                    xprintf_P(PSTR("WAN:: CONF_MODBUS ERROR: El servidor no reconoce al datalogger !!\r\n"));
                    ret = -1;
                    goto exit_;
                }
                
                if ( wan_check_response("CONF_MODBUS&CONFIG=OK")) {
                    ret = 0;
                    goto exit_;
                }
                
                if ( wan_check_response( "CLASS=CONF_MODBUS" )) {
                    wan_process_rsp_configModbus();
                    ret = +1;
                    goto exit_;
                } 
            }
        }

    }
 
    // Expiro el tiempo sin respuesta del server.
    xprintf_P(PSTR("WAN:: CONFIG_MODBUS ERROR: Timeout en server rsp.!!\r\n"));
    ret = -1;
    
exit_:
               
    xSemaphoreGive( sem_WAN );
    return(ret);      
}
//------------------------------------------------------------------------------
static int8_t wan_process_rsp_configModbus(void)
{
   /*
     * Procesa la configuracion de los canales Modbus
     * RXFRAME: <html><body><h1>CLASS=CONF_MODBUS&ENABLE=TRUE&LOCALADDR=2&
     *                                             M0=TRUE,CAU1,8,100,2,3,U16,C0123,0&
     *                                             M1=TRUE,AUX,9,200,2,3,I16,C0123,0&
     *                                             M2=TRUE,PRE,7,1021,2,3,FLOAT,C3210,1&
     *                                             M3=TRUE,BOM,6,1022,2,3,U16,C0123,0&
     *                                             M4=TRUE,CAU2,5,1023,2,3,U16,C0123,0</h1></body></html>
     *                          CLASS:CONF_MODBUS;CONFIG:OK
     * 
     * CLASS=CONF_MODBUS&ENABLE=TRUE&LOCALADDR=2&M0=TRUE,CAU0,2,2069,2,3,FLOAT,C1032,0&M1=FALSE,X,2,2069,2,3,FLOAT,C1032,0&
     */

    
char *ts = NULL;
char *stringp = NULL;
char *tk_address = NULL;
char *tk_enable= NULL;
char *tk_name= NULL;
char *tk_sla= NULL;
char *tk_regaddr = NULL;
char *tk_nroregs = NULL;
char *tk_fcode = NULL;
char *tk_mtype = NULL;
char *tk_codec = NULL;
char *tk_pow10 = NULL;
char *delim = "&,;:=><";
uint8_t ch;
char str_base[8];
char *p;
int8_t ret = -1;

    p = MODEM_get_buffer_ptr();
    //p = lBchar_get_buffer(&wan_rx_lbuffer);
    
    if  ( strstr( p, "CONFIG=OK") != NULL ) {
        ret = 0;
       goto exit_;
    }
     
    if  ( strstr( p, "CONFIG=ERROR") != NULL ) {
        xprintf_P(PSTR("WAN:: CONF ERROR !!\r\n"));
        ret = -1;
        goto exit_;    
    }

    vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
    memset(tmpLocalStr,'\0',sizeof(tmpLocalStr));
	ts = strstr( p, "ENABLE=");
    if  ( ts != NULL ) {
        strncpy(tmpLocalStr, ts, sizeof(tmpLocalStr));
        stringp = tmpLocalStr;
        tk_enable = strsep(&stringp,delim);	 	// ENABLE
        tk_enable = strsep(&stringp,delim);	 	// TRUE/FALSE
        modbus_config_enable( tk_enable);
        xprintf_P( PSTR("WAN:: Reconfig MODBUS ENABLE to %s\r\n"), tk_enable );
    }

    vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
    memset(tmpLocalStr,'\0',sizeof(tmpLocalStr));
	ts = strstr( p, "LOCALADDR=");
    if  ( ts != NULL ) {
        strncpy(tmpLocalStr, ts, sizeof(tmpLocalStr));
        stringp = tmpLocalStr;
        tk_address = strsep(&stringp,delim);	 	// ENABLE
        tk_address = strsep(&stringp,delim);	 	// TRUE/FALSE
        modbus_config_localaddr( tk_address);
        xprintf_P( PSTR("WAN:: Reconfig MODBUS LOCALADDR to %s\r\n"), tk_address );
    }
    
    //
	// MB? M0=TRUE,CAU0,2,2069,2,3,FLOAT,C1032,0
	for (ch=0; ch < NRO_MODBUS_CHANNELS; ch++ ) {
        
        vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
		memset( &str_base, '\0', sizeof(str_base) );
		snprintf_P( str_base, sizeof(str_base), PSTR("M%d="), ch );

		if ( strstr( p, str_base) != NULL ) {
			memset(tmpLocalStr,'\0',sizeof(tmpLocalStr));
            ts = strstr( p, str_base);
			strncpy( tmpLocalStr, ts, sizeof(tmpLocalStr));
			stringp = tmpLocalStr;
			tk_enable = strsep(&stringp,delim);	    //M0
            tk_enable = strsep(&stringp,delim);     //enable
			tk_name = strsep(&stringp,delim);		//name
			tk_sla = strsep(&stringp,delim);		//sla_address
			tk_regaddr = strsep(&stringp,delim);	//reg_ddress
			tk_nroregs = strsep(&stringp,delim);	//nro_regs
			tk_fcode = strsep(&stringp,delim);		//fcode
			tk_mtype = strsep(&stringp,delim);		//mtype
			tk_codec = strsep(&stringp,delim);		//codec
			tk_pow10 = strsep(&stringp,delim);		//pow10
                    
            modbus_config_channel( ch,
                    tk_enable, 
                    tk_name, 
                    tk_sla, 
                    tk_regaddr, 
                    tk_nroregs, 
                    tk_fcode, 
                    tk_mtype, 
                    tk_codec, 
                    tk_pow10 );
			xprintf_P( PSTR("WAN:: Reconfig M%d\r\n"), ch);
		}
	}
    ret = +1;
   
exit_:
                
    return(ret);
}
//------------------------------------------------------------------------------
static int8_t wan_process_frame_configConsigna(void)
{
    /*
      * Envo un frame con el hash de la configuracion del consignas.
      * El server me puede mandar OK o la nueva configuracion que debo tomar.
      * Lo reintento 2 veces
      */
    
uint8_t tryes = 0;
uint8_t timeout = 0;
int8_t ret = -1;
uint8_t hash = 0;

#ifdef TESTING_MODEM
    xprintf_P(PSTR("WAN::<%s>: CONFIG_CONSIGNA\r\n"), RTC_logprint(true));
#else
    xprintf_P(PSTR("WAN:: CONFIG_CONSIGNA\r\n"));
#endif
    
    // Armo el buffer
    while ( xSemaphoreTake( sem_WAN, MSTOTAKEWANSEMPH ) != pdTRUE )
        vTaskDelay( ( TickType_t)( 1 ) );
    memset(wan_tx_buffer, '\0', WAN_TX_BUFFER_SIZE);
    
    hash = consigna_hash();
    sprintf_P( (char*)&wan_tx_buffer, PSTR("ID=%s&HW=%s&TYPE=%s&VER=%s&CLASS=CONF_CONSIGNA&HASH=0x%02X"), base_conf.dlgid, HW, FW_TYPE, FW_REV, hash );

    // Proceso. Envio hasta 2 veces el frame y espero hasta 10s la respuesta
    tryes = CONFIGCONSIGNA_TRYES;
    while (tryes-- > 0) {
        
        wan_xmit_out();
        // Espero respuesta chequeando cada 1s durante 10s.
        timeout = 10;
        while ( timeout-- > 0) {
            vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
            if ( wan_check_response("</html>") ) {
                
                wan_print_RXbuffer();
                
                if ( wan_check_response("CONFIG=ERROR")) {
                    xprintf_P(PSTR("WAN:: CONF_CONSIGNA ERROR: El servidor no reconoce al datalogger !!\r\n"));
                    ret = -1;
                    goto exit_;
                }
                
                if ( wan_check_response("CONF_CONSIGNA&CONFIG=OK")) {
                    ret = 0;
                    goto exit_;
                }
                
                if ( wan_check_response( "CLASS=CONF_CONSIGNA" )) {
                    wan_process_rsp_configConsigna();
                    ret = +1;
                    goto exit_;
                } 
            }
        }
    }
    
    // Expiro el tiempo sin respuesta del server.
    xprintf_P(PSTR("WAN:: CONFIG_CONSIGNA ERROR: Timeout en server rsp.!!\r\n"));
    ret = -1;
    
exit_:
               
    xSemaphoreGive( sem_WAN );
    return(ret);       
}
//------------------------------------------------------------------------------
static int8_t wan_process_rsp_configConsigna(void)
{
   /*
     * Procesa la configuracion de los canales Modbus
     * RXFRAME: <html><body><h1>CLASS=CONF_CONSIGNA&ENABLE=TRUE&DIURNA=700&NOCTURNA=2300</h1></body></html>
     *                          CLASS:CONF_CONSIGNA;CONFIG:OK
     * 
     */
  
char *token = NULL;
char *tk_enable = NULL;
char *tk_diurna = NULL;
char *tk_nocturna = NULL;
char *delim = "<>/&=";
char *ts = NULL;
char *p;
int8_t ret = -1;

    p = MODEM_get_buffer_ptr();
    //p = lBchar_get_buffer(&wan_rx_lbuffer);
    
    if  ( strstr( p, "CONFIG=OK") != NULL ) {
        ret = 0;
       goto exit_;
    }
     
    if  ( strstr( p, "CONFIG=ERROR") != NULL ) {
        xprintf_P(PSTR("WAN:: CONF ERROR !!\r\n"));
        ret = -1;
        goto exit_;    
    }
         
    vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
	memset(tmpLocalStr,'\0',sizeof(tmpLocalStr));
	ts = strstr( p, "ENABLE=");
    if  ( ts != NULL ) {
        strncpy(tmpLocalStr, ts, sizeof(tmpLocalStr));
        token = strtok (tmpLocalStr, delim);       
        while ( token != NULL ) {
            //xprintf_P(PSTR("TOK=%s\r\n"), token ); //printing each token
            if (strstr(token,"ENABLE") != NULL ) {
                token = strtok(NULL, delim); 
                tk_enable = token;
                continue;
            } else if (strstr(token,"DIURNA") != NULL ) {
                token = strtok(NULL, delim); 
                tk_diurna = token;
                continue;
            } else if (strstr(token,"NOCTURNA") != NULL ) {
                token = strtok(NULL, delim); 
                tk_nocturna = token;
                continue;
            }
            token = strtok(NULL, delim);
        }
	 	// 
    }
    //xprintf_P( PSTR("WAN:: Reconfig CONSIGNA to: %s,%s,%s\r\n"), tk_enable, tk_diurna, tk_nocturna);
    xprintf_P( PSTR("WAN:: Reconfig CONSIGNA\r\n"));
    consigna_config( tk_enable, tk_diurna, tk_nocturna );
    ret = +1;
   
exit_:
                
    return(ret);
}
//------------------------------------------------------------------------------
static bool wan_send_from_memory(void)
{
    /*
     * Lee el FS y transmite los registros acumulados.
     * Al recibir un OK los borra.
     * Envia de a 1.
     */
    
dataRcd_s dr;
bool retS = false;
uint8_t frames_counter;

    xprintf_P(PSTR("WAN:: Dump memory...\r\n"));
    
    FAT_read(&l_fat1);
    xprintf_P( PSTR("WAN:: wrPtr=%d,rdPtr=%d,count=%d\r\n"), l_fat1.head, l_fat1.tail, l_fat1.count );
    
    // Mando Sin confirmacion
    frames_counter = 1;
    while ( l_fat1.count > 0 ) {
        
        u_kick_wdt(TK_WAN, T120S);
        
        xprintf_P( PSTR("WAN:: wrPtr=%d,rdPtr=%d,count=%d\r\n"),l_fat1.head, l_fat1.tail, l_fat1.count );
        
        if ( ! FS_readRcd( &dr, sizeof(dataRcd_s) ) ) {
            goto quit;
        }
        
        if ( ( frames_counter == 0 ) || ( l_fat1.count == 1) ) {
            // Transmito y espero respuesta
            retS = wan_process_frame_data(&dr, true);
        } else {
            // Transmito y no espero respuesta
            retS = wan_process_frame_data(&dr, false);
        }
        if ( ! retS) {
            goto quit;
        }

        if ( frames_counter++ == 10 ) {
            frames_counter = 0;
        }
        
        FAT_read(&l_fat1);
    }
    
    
quit:
                
    xprintf_P(PSTR("WAN:: Memory Empty\r\n"));
    //FAT_flush();
    
    return (retS);


}
//------------------------------------------------------------------------------
static bool wan_send_from_memory_001(void)
{
    /*
     * Lee el FS y transmite los registros acumulados.
     * Al recibir un OK los borra.
     * Envia de a 1.
     */
    
dataRcd_s dr;
bool retS = false;

    xprintf_P(PSTR("WAN:: Dump memory...\r\n"));
    
    FAT_read(&l_fat1);
    xprintf_P( PSTR("WAN:: wrPtr=%d,rdPtr=%d,count=%d\r\n"), l_fat1.head, l_fat1.tail, l_fat1.count );
    
    while ( l_fat1.count > 0 ) {
        
        u_kick_wdt(TK_WAN, T120S);
        
        xprintf_P( PSTR("WAN:: wrPtr=%d,rdPtr=%d,count=%d\r\n"),l_fat1.head, l_fat1.tail, l_fat1.count );
        
        if ( FS_readRcd( &dr, sizeof(dataRcd_s) ) ) {
            retS = wan_process_frame_data(&dr, true);
            if ( ! retS) {
                goto quit;
            }
        } else {
            goto quit;
        }
        
        FAT_read(&l_fat1);
    }
    
quit:
                
    xprintf_P(PSTR("WAN:: Memory Empty\r\n"));
    //FAT_flush();
    
    return (retS);


}
//------------------------------------------------------------------------------
static bool wan_process_frame_data(dataRcd_s *dr, bool response)
{
   /*
    * Armo el wan_tx_buffer.
    * Transmite los datos pasados en la estructura dataRcd por el
    * puerto definido como WAN
    * Espero o no la respuesta
    */ 

uint8_t tryes = 0;
uint8_t timeout = 0;
bool retS = false;

#ifdef TESTING_MODEM    
    if (response) {
        xprintf_P(PSTR("WAN::<%s>: DATA\r\n"), RTC_logprint(true));
    } else {
        xprintf_P(PSTR("WAN::<%s>: DATANR\r\n"), RTC_logprint(true));
    }
#else
    if (response) {
        xprintf_P(PSTR("WAN:: DATA\r\n"));
    } else {
        xprintf_P(PSTR("WAN:: DATANR\r\n"));
    }
#endif
    
    while ( xSemaphoreTake( sem_WAN, MSTOTAKEWANSEMPH ) != pdTRUE )
        vTaskDelay( ( TickType_t)( 1 ) );
    
    // Formateo(escribo) el dr en el wan_tx_buffer
    wan_load_dr_in_txbuffer(dr, (uint8_t *)&wan_tx_buffer,WAN_TX_BUFFER_SIZE, response );
        
    if ( response == false ) {
        // Transmito y no espero respueta
        wan_xmit_out();
        retS = true;
        goto exit_;
        
    } else {
        
        // Proceso. Envio hasta 2 veces el frame y espero hasta 10s la respuesta   
        tryes = DATA_TRYES;
        while (tryes-- > 0) {
        
            wan_xmit_out();
            // Espero respuesta chequeando cada 1s durante 10s.
            timeout = 10;
            while ( timeout-- > 0) {
                vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
                if ( wan_check_response("</html>") ) {
               
                    wan_print_RXbuffer();
           
                    if ( wan_check_response("CLASS=DATA")) {
                        wan_process_rsp_data();
                        retS = true;
                        goto exit_;
                    }
                }
            }       
        
            xprintf_P(PSTR("WAN:: DATA RETRY\r\n"));       
        }
 
        // Expiro el tiempo sin respuesta del server.
        xprintf_P(PSTR("WAN:: DATA ERROR: Timeout en server rsp.!!\r\n"));
        retS = false;
    }
    
exit_:

    xSemaphoreGive( sem_WAN );
    return(retS);
    
}
//------------------------------------------------------------------------------
static void wan_load_dr_in_txbuffer(dataRcd_s *dr, uint8_t *buff, uint16_t buffer_size, bool response)
{
    /*
     * Toma un puntero a un dr y el wan_tx_buffer y escribe el dr en este
     * con el formato para transmitir.
     * En semaforo lo debo tomar en la funcion que invoca !!!.
     */

uint8_t i;
int16_t fptr;

   // Armo el buffer
    memset(buff, '\0', buffer_size);
    fptr = 0;
    if (response) {
        fptr = sprintf_P( (char *)&buff[fptr], PSTR("ID=%s&HW=%s&TYPE=%s&VER=%s&CLASS=DATA"), base_conf.dlgid, HW, FW_TYPE, FW_REV);   
    } else {
        fptr = sprintf_P( (char *)&buff[fptr], PSTR("ID=%s&HW=%s&TYPE=%s&VER=%s&CLASS=DATANR"), base_conf.dlgid, HW, FW_TYPE, FW_REV);  
    } 
    // Clock
    fptr += sprintf_P( (char *)&buff[fptr], PSTR("&DATE=%02d%02d%02d"), dr->rtc.year,dr->rtc.month, dr->rtc.day );
    fptr += sprintf_P( (char *)&buff[fptr], PSTR("&TIME=%02d%02d%02d"), dr->rtc.hour,dr->rtc.min, dr->rtc.sec);
    
    // Analog Channels:
    for ( i=0; i < NRO_ANALOG_CHANNELS; i++) {
        if ( ainputs_conf.channel[i].enabled ) {
            fptr += sprintf_P( (char*)&buff[fptr], PSTR("&%s=%0.2f"), ainputs_conf.channel[i].name, dr->ainputs[i]);
        }
    }
        
    // Counter:
    if ( counter_conf.enabled ) {
        fptr += sprintf_P( (char*)&buff[fptr], PSTR("&%s=%0.3f"), counter_conf.name, dr->contador);

    }
    
#ifdef HW_AVRDA
    // Modbus Channels:
    if (modbus_conf.enabled) {
        for ( i=0; i < NRO_MODBUS_CHANNELS; i++) {
            if (  modbus_conf.mbch[i].enabled ) {
                fptr += sprintf_P( (char*)&buff[fptr], PSTR("&%s=%0.3f"), modbus_conf.mbch[i].name, dr->modbus[i]);
            }
       }
    }
#endif
    
    // Battery
    fptr += sprintf_P( (char*)&buff[fptr], PSTR("&bt3v3=%0.3f"), dr->bt3v3);
    fptr += sprintf_P( (char*)&buff[fptr], PSTR("&bt12v=%0.3f"), dr->bt12v);
    
    //fptr += sprintf_P( (char*)&buff[fptr], PSTR("\r\n"));
    
}
//------------------------------------------------------------------------------
static bool wan_process_rsp_data(void)
{
   /*
     * Procesa las respuestas a los frames DATA
     * Podemos recibir CLOCK, RESET, PILOTO
     */
    
char *ts = NULL;
char *stringp = NULL;
char *token = NULL;
char *delim = "&,;:=><";
char *p;
//BaseType_t xHigherPriorityTaskWoken = pdTRUE;

    p = MODEM_get_buffer_ptr();
    //p = lBchar_get_buffer(&wan_rx_lbuffer);

    vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
    //xprintf_P(PSTR("WAN:: response\r\n") );
    
    if ( strstr( p, "CLOCK") != NULL ) {
        memset(tmpLocalStr,'\0',sizeof(tmpLocalStr));
        ts = strstr( p, "CLOCK=");
        strncpy(tmpLocalStr, ts, sizeof(tmpLocalStr));
        stringp = tmpLocalStr;
        token = strsep(&stringp,delim);			// CLOCK
        token = strsep(&stringp,delim);			// 1910120345

        // Error en el string recibido
        if ( strlen(token) < 10 ) {
            // Hay un error en el string que tiene la fecha.
            // No lo reconfiguro
            xprintf_P(PSTR("WAN:: data_resync_clock ERROR:[%s]\r\n"), token );
            return(false);
        } else {
            RTC_resync( token, false );
        }
    }
        
    if ( strstr( p, "RESET") != NULL ) {
        xprintf_P(PSTR("WAN:: RESET order from Server !!\r\n"));
        vTaskDelay( ( TickType_t)( 2000 / portTICK_PERIOD_MS ) );
        reset();
    }
    
    if ( strstr( p, "VOPEN") != NULL ) {
        xprintf_P(PSTR("WAN:: OPEN VALVE from server !!\r\n"));
        xTaskNotify( xHandle_tkCtlPres, SIGNAL_OPEN_VALVE, eSetBits );
        vTaskDelay( ( TickType_t)( 2000 / portTICK_PERIOD_MS ) );
        //VALVE_open();
    }

    if ( strstr( p, "VCLOSE") != NULL ) {
        xprintf_P(PSTR("WAN:: CLOSE VALVE from server !!\r\n"));
        xTaskNotify( xHandle_tkCtlPres, SIGNAL_CLOSE_VALVE, eSetBits );
        vTaskDelay( ( TickType_t)( 2000 / portTICK_PERIOD_MS ) );
        //VALVE_close();
    }
    
    
/*   
 *      if ( strstr( p, "RESMEM") != NULL ) {
        xprintf_P(PSTR("WAN:: RESET MEMORY order from Server !!\r\n"));
        vTaskDelay( ( TickType_t)( 2000 / portTICK_PERIOD_MS ) );
        u_reset_memory_remote();
    }

    if ( strstr( p, "PILOTO") != NULL ) {
        memset(tmpLocalStr,'\0',sizeof(tmpLocalStr));
        ts = strstr( p, "PILOTO=");
        strncpy(tmpLocalStr, ts, sizeof(tmpLocalStr));
        stringp = tmpLocalStr;
        token = strsep(&stringp,delim);			// PILOTO
        token = strsep(&stringp,delim);			// 3.45
        PILOTO_productor_handler_online(atof(token));
    }
 */
    
    return(true);  
}
//------------------------------------------------------------------------------
static void wan_xmit_out(void )
{
    /* 
     * Transmite el buffer de tx por el puerto comms configurado para wan.
     * 2024-08-13: Hay un problema con el modem que si mando mas de 150 caracters
     * los recorta !!.
     * Por ahora solo hago un control.
     */
    
    //if ( strlen(wan_tx_buffer) > 150 ) {
    //    xprintf_P( PSTR("ALERT WAN BUFFER SIZE > 150 !!!\r\n"));
    //}
    // Antes de trasmitir siempre borramos el Rxbuffer
    //lBchar_Flush(&wan_rx_lbuffer);
    
    
#ifdef TESTING_MODEM
    xfprintf_P( fdTERM, PSTR("Xmit->::<%s>: %s\r\n"), RTC_logprint(true), &wan_tx_buffer);
#else
    xfprintf_P( fdTERM, PSTR("Xmit-> %s\r\n"), &wan_tx_buffer);
#endif
    
    
    MODEM_flush_rx_buffer();
    //xfprintf_P( fdWAN, PSTR("%s"), wan_tx_buffer); 
    MODEM_txmit(&wan_tx_buffer[0]);
    
    //if (f_debug_comms ) {
        // xfprintf_P( fdTERM, PSTR("Xmit-> %s\r\n"), &wan_tx_buffer);
    //}
    
}
//------------------------------------------------------------------------------
static bool wan_check_response ( const char *s)
{
    /*
     * Retorna true/false si encuentra el patron *s en el buffer de rx wan.
     */
    
char *p;
    
    p = MODEM_get_buffer_ptr();
    //p = lBchar_get_buffer(&wan_rx_lbuffer);
        
    if ( strstr( p, s) != NULL ) {
        if (f_debug_comms) {
            xprintf_P( PSTR("rxbuff-> %s\r\n"), p);
        }
        return(true);
    }
    return (false);
}
//------------------------------------------------------------------------------
static void wan_print_RXbuffer(void)
{

char *p;
            
    p = MODEM_get_buffer_ptr();
    //p = lBchar_get_buffer(&wan_rx_lbuffer);
    //if (f_debug_comms) {
#ifdef TESTING_MODEM
    xprintf_P(PSTR("Rcvd::<%s> -> %s\r\n"),  RTC_logprint(true), p );
#else
    xprintf_P(PSTR("Rcvd-> %s\r\n"), p );    
#endif
    //}
}
//------------------------------------------------------------------------------
void wan_prender_modem(void)
{
    /*
     * Se prende el modem y se ve su configuracion. Si no es correcta
     * se intenta reconfigurarlo
     * Solo se hacen 3 intentos
     * 
     */
    xprintf_P(PSTR("WAN:: PRENDER MODEM\r\n"));
    
    MODEM_prender();
    MODEM_AWAKE(); 
    
    // Espero que se estabilize
    vTaskDelay( ( TickType_t)( 15000 / portTICK_PERIOD_MS ) );
    return;
             
}
//------------------------------------------------------------------------------
void wan_apagar_modem(void)
{
    MODEM_apagar();
    MODEM_SLEEP();
    
    xprintf_P(PSTR("WAN:: APAGAR MODEM\r\n"));
}
//------------------------------------------------------------------------------
//
// FUNCIONES PUBLICAS
//
//------------------------------------------------------------------------------
bool WAN_process_data_rcd( dataRcd_s *dataRcd)
{
    /*
     * El procesamiento implica transmitir si estoy con el link up, o almacenarlo
     * en memoria para luego transmitirlo.
     * Copio el registro a uno local y prendo la flag que indica que hay datos 
     * para transmitir.
     * Si hay un dato aun en el buffer, lo sobreescribo !!!
     * 
     */
   
bool retS = false;
fat_s l_fat;   

    // link up
    if ( link_up4data  ) {
        /*
         * Hay enlace. 
         * Guardo el dato en el buffer.
         * Indico que hay un dato listo para enviar
         * Aviso (despierto) para que se transmita.
         */
        if (f_debug_comms) {
            xprintf_P(PSTR("WAN:: New dataframe.\r\n"));
        }
        
        memcpy( &drWanBuffer.dr, dataRcd, sizeof(dataRcd_s));
        drWanBuffer.dr_ready = true;    
        // Aviso al estado online que hay un frame listo
        while ( xTaskNotify(xHandle_tkWAN, SIGNAL_FRAME_READY , eSetBits ) != pdPASS ) {
			vTaskDelay( ( TickType_t)( 100 / portTICK_PERIOD_MS ) );
		}
        retS = true;
        
    } else if (base_conf.pwr_modo == PWR_RTU ) {
        /*
         * No hay enlace: me voy, no guardo el dato.
         */
        retS = true;
        
    } else {
        // Guardo en memoria
        xprintf_P(PSTR("WAN:: Save frame in EE.\r\n"));
        retS = FS_writeRcd( dataRcd, sizeof(dataRcd_s) );
        if ( ! retS  ) {
            // Error de escritura o memoria llena ??
            xprintf_P(PSTR("WAN:: WR ERROR\r\n"));
        }
        // Stats de memoria
        FAT_read(&l_fat);
        xprintf_P( PSTR("wrPtr=%d,rdPtr=%d,count=%d\r\n"),l_fat.head,l_fat.tail, l_fat.count );
        
    }

    return(retS);
}
//------------------------------------------------------------------------------
void WAN_print_configuration(void)
{
    
    xprintf_P(PSTR("Comms:\r\n"));
    xprintf_P(PSTR(" debug: "));
    f_debug_comms ? xprintf_P(PSTR("on\r\n")) : xprintf_P(PSTR("off\r\n"));
    
}
//------------------------------------------------------------------------------
void WAN_config_debug(bool debug )
{
    if ( debug ) {
        f_debug_comms = true;
    } else {
        f_debug_comms = false;
    }
}
//------------------------------------------------------------------------------
bool WAN_read_debug(void)
{
    return (f_debug_comms);
}
//------------------------------------------------------------------------------
// DEPRECATED

static void __wan_state_online_config(void)
{
    /*
     * Se encarga de la configuracion.
     * Solo lo hace una sola vez por lo que tiene una flag estatica que
     * indica si ya se configuro.
     */
  
uint16_t i;
int8_t saveInMem = 0;
int8_t rsp = -1;

    u_kick_wdt(TK_WAN, T120S);
    
#ifdef TESTING_MODEM
    xprintf_P(PSTR("WAN::<%s>: State ONLINE_CONFIG\r\n"), RTC_logprint(true));
#else
    xprintf_P(PSTR("WAN:: State ONLINE_CONFIG\r\n"));
#endif
    
    if ( wan_process_frame_recoverId() == -1 ) {
       
        // Espero 1H y reintento.
        // Apago la tarea del system para no llenar la memoria al pedo
        
        vTaskSuspend( xHandle_tkSys );
        xprintf_P(PSTR("WAN:: ERROR RECOVER ID:Espero 1 h.\r\n"));
        for (i=0; i < 60; i++) {
             vTaskDelay( ( TickType_t)( 60000 / portTICK_PERIOD_MS ) );
             u_kick_wdt(TK_WAN, T120S);
        }
        
        xprintf("Reset..\r\n");
        reset();
        
    }
   
    rsp = wan_process_frame_configBase();
    if (  rsp == -1 ) {
        // No puedo configurarse o porque el servidor no responde
        // o porque da errores. Espero 60mins.
        xprintf_P(PSTR("WAN:: Errores en configuracion. Espero 60mins..!!\r\n"));
        base_conf.timerdial = 3600;
        base_conf.timerpoll = 3600;
        base_conf.pwr_modo = PWR_DISCRETO;
        wan_state = WAN_APAGADO;
        return;
    }
    
    saveInMem += rsp;
    
    rsp = wan_process_frame_configAinputs(); 
    if (rsp >= 0 ) {
        saveInMem += rsp;
    }
    
    rsp = wan_process_frame_configCounters();
    if (rsp >= 0 ) {
        saveInMem += rsp;
    }
    
#ifdef HW_AVRDA
    rsp = wan_process_frame_configModbus();
    if (rsp >= 0 ) {
        saveInMem += rsp;
    } 
    
    rsp = wan_process_frame_configConsigna();
    if (rsp >= 0 ) {
        saveInMem += rsp;
    } 
        
#endif
        
    if ( saveInMem > 0 ) {
        if ( ! u_save_config_in_NVM() ) {
            xprintf_P(PSTR("WAN ERROR saving NVM !!!\r\n"));
        }
    }
    
    wan_state = WAN_ONLINE_DATA;
    return;

}
//------------------------------------------------------------------------------
static bool __wan_bulk_send_from_memory(void)
{
    /*
     * Lee el FS y transmite los registros acumulados.
     * Al recibir un OK los borra.
     * Envia de a 1.
     */
    
dataRcd_s dr;
bool retS = false;
uint8_t frames_counter;
int16_t fptr;
uint16_t total_bytes;
uint8_t i;
uint8_t timeout = 0;

    xprintf_P(PSTR("WAN:: Bulk send...\r\n"));
    
    while ( xSemaphoreTake( sem_WAN, MSTOTAKEWANSEMPH ) != pdTRUE )
    vTaskDelay( ( TickType_t)( 1 ) );
    
    FAT_read(&l_fat1);
    xprintf_P( PSTR("WAN:: wrPtr=%d,rdPtr=%d,count=%d\r\n"), l_fat1.head, l_fat1.tail, l_fat1.count );
    if ( l_fat1.count <= 0 ) {
        // No hay datos en memoria para transmitir
        goto quit;
    }
    
    frames_counter = 0;
    // Send HEADER
    total_bytes = 0;
    memset((uint8_t *)&wan_tx_buffer, '\0', WAN_TX_BUFFER_SIZE);
    fptr = 0;
    fptr = sprintf_P( (char *)&wan_tx_buffer[fptr], PSTR("ID=%s&HW=%s&TYPE=%s&VER=%s&CLASS=DBULK"), base_conf.dlgid, HW, FW_TYPE, FW_REV);   
    total_bytes += fptr;
    wan_xmit_out();
      
    // Send PAYLOAD
    while(1) {

        u_kick_wdt(TK_WAN, T120S);  
        xprintf_P( PSTR("WAN:: ctl=%d, wrPtr=%d,rdPtr=%d,count=%d\r\n"),frames_counter, l_fat1.head, l_fat1.tail, l_fat1.count );
        // Leo un reg.de memoria
        if ( ! FS_readRcd( &dr, sizeof(dataRcd_s) ) ) {
            goto quit;
        }
        // Guardo el dataRecord en el txBuffer
        memset((uint8_t *)&wan_tx_buffer, '\0', WAN_TX_BUFFER_SIZE);
        fptr = 0;
        fptr = sprintf_P( (char *)&wan_tx_buffer[fptr], PSTR("CTL=%d"), frames_counter );     
        // Clock
        fptr += sprintf_P( (char *)&wan_tx_buffer[fptr], PSTR("&DATE=%02d%02d%02d"), dr.rtc.year,dr.rtc.month, dr.rtc.day );
        fptr += sprintf_P( (char *)&wan_tx_buffer[fptr], PSTR("&TIME=%02d%02d%02d"), dr.rtc.hour,dr.rtc.min, dr.rtc.sec);
    
        // Analog Channels:
        for ( i=0; i < NRO_ANALOG_CHANNELS; i++) {
            if ( ainputs_conf.channel[i].enabled ) {
                fptr += sprintf_P( (char*)&wan_tx_buffer[fptr], PSTR("&%s=%0.2f"), ainputs_conf.channel[i].name, dr.ainputs[i]);
            }
        }
        
        // Counter:
        if ( counter_conf.enabled ) {
            fptr += sprintf_P( (char*)&wan_tx_buffer[fptr], PSTR("&%s=%0.3f"), counter_conf.name, dr.contador);
        }
    
#ifdef HW_AVRDA
        // Modbus Channels:
        if (modbus_conf.enabled) {
            for ( i=0; i < NRO_MODBUS_CHANNELS; i++) {
                if (  modbus_conf.mbch[i].enabled ) {
                    fptr += sprintf_P( (char*)&wan_tx_buffer[fptr], PSTR("&%s=%0.3f"), modbus_conf.mbch[i].name, dr.modbus[i]);
                }
            }
        }
#endif
    
        // Battery
        fptr += sprintf_P( (char*)&wan_tx_buffer[fptr], PSTR("&bt3v3=%0.3f"), dr.bt3v3);
        fptr += sprintf_P( (char*)&wan_tx_buffer[fptr], PSTR("&bt12v=%0.3f"), dr.bt12v);
    
        // Envio
        wan_xmit_out();
        
        total_bytes += fptr;
        frames_counter++;
        FAT_read(&l_fat1);
        
        // Condiciones de salida
        if (frames_counter > 5) {
            goto quit;
        }
        
        if ( l_fat1.count <= 0 ) {
            goto quit;
        }
        
        if ( total_bytes > 2048 ) {
            goto quit;
        }
    }
     
    // Espero respuesta chequeando cada 1s durante 10s.
    timeout = 10;
    while ( timeout-- > 0) {
        vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
        if ( wan_check_response("</html>") ) {
               
            wan_print_RXbuffer();
           
            if ( wan_check_response("CLASS=DBULK")) {
                wan_process_rsp_data();
                retS = true;
                goto quit;
            }
        }
    } 

    retS = false;
    
    
quit:
    
    xSemaphoreGive( sem_WAN );
    xprintf_P(PSTR("WAN:: Memory Empty\r\n"));
    //FAT_flush();
    
    return (retS);


}
//------------------------------------------------------------------------------