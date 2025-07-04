
/*
 * File:   spxR2_main.c
 * Author: pablo
 *
 * Created on 25 de octubre de 2021, 11:55 AM
 * 
 * Debido a que es un micro nuevo, conviene ir a https://start.atmel.com/ y
 * crear un projecto con todos los perifericos que usemos y bajar el codigo
 * para ver como se inicializan y se manejan.
 *
 * https://www.professordan.com/avr/techlib/techlib8/appnotes/
 * 
 * FUSES:
 * avrdude -px256a3b -cavrispmkII -Pusb -p x256a3b -V -u -Ufuse0:w:0xff:m -Ufuse1:w:0xa:m -Ufuse2:w:0xfd:m -Ufuse4:w:0xf5:m -Ufuse5:w:0xd4:m
 * FIRMWARE
 * avrdude -v -Pusb -c avrispmkII -p x256a3 -F -e -U flash:w:FWDLGX.X.production.hex
 *
 * -----------------------------------------------------------------------------
 * Version 1.1.0 @ 20250625
 * A) En los pwr_modo_t agrego uno nuevo que es RTU.
 * En este caso el datalogger NO almacena datos en memoria sino que solo si
 * hay enlace lo transmite.
 * Esto es para los equipos que estan en tanques como esclavos de los PLC que no tiene
 * sentido que guarden datos. 
 * B) Acelero el proceso de conexion y envio de datos.
 *    Elimino modem_check_and_reconfig()
 *    Mando un frame inicial con CONFIG_ALL con todos los hashes
 * C) Los datos los mando en modo bulk hasta 10 frames juntos
 *    La idea es crear una clase DATANR de frame que no pida respuesta y otra que si
 *    Esto es porque el modem fragmenta entonces hay que mandar de un saque.
 * 
 * -----------------------------------------------------------------------------
 * Version 1.0.3 @ 20250528
 * Vemos que el equipo se resetea sin razon.
 * Luego de analizarlo encontramos que el problema es cuando entra en modo tickless.
 * Si no habilitamos este modo, NO SE RESETEA.
 * Luego de probar con el firmware TESTFWX encontramos que el problema esta en el 
 * modo sleep nativo (tickless=1).
 * Lo modificamos y lo ponemos como (tickless=2)
 * - porthardware.h
 * - port_DX.c
 * - FWDLGX_utils.c
 * 
 * En la arquitectura SPX no funciona a�n.
 * Incorporamos un #define TESTING_MODEM para poder probar los tiempos usados en la wan.
 * En idle_task ponemos un contador de 32 bits (rtcounter) para medir el tiempo con exactitud.
 * 
 * -----------------------------------------------------------------------------
 * Version 1.0.2 @ 20250421
 * 1- En tkCtl la rutina de watchdog solo queda en loop, pero esto implica que el
 *   watchdog este prendido cosa que veo que no pasa en los SPX.
 *   Agrego entonces un reset.
 * 2- Al vaciar la memoria si tiene muchos datos se resetea por TO. Agrego en wan_send_from_memory
 *   el resetear el watchdog c/frame que transmito
 * 3- Cuando el modem no tiene el APN ni el datalogger no lo puede resolver.Aunque no tengan
 *   el APN pero si la IP/PORT, el modem se puede conectar.
 * 
 * -----------------------------------------------------------------------------
 * Version 1.0.0 @ 20250313
 * Creamos la version FWDLGX que toma como base FWDLG.
 * Cambiamos el modo de operar la terminal. Solo se hace con modocomando.
 * CONSUMOS:
 * - SPQ_AVRDA_R1: 0.925 mA
 * - SPQ_AVRDA_R2: 0.43 mA
 * - SPX_XMEGA:  24 mA
 * 
 * -----------------------------------------------------------------------------
 * Version 1.3.9 @ 20250115
 * Pongo un nuevo define para CNTF y CNTE ya que las nuevas placas SPQ tienen el
 * contador en el diferente puerto. Los cambios van en contadores.c
 * Compilo siempre ambas versiones !!!
 * 
 * -----------------------------------------------------------------------------
 * Version 1.3.8 @ 20250102
 * Modifico la posicion de pines del CNT0 que pasa a E6
 * Idem con TERM_SENSE que pasa a A2. Esto trae problemas con los SPQ viejos.
 * Modificamos agregando un switch entre estados, cambiando solo cuando se
 * presionan teclas.
 * Agregamos que el modem lea la IP local y la muestre.
 * Agrego al config el modocomando para que entonces no saque a la termial a sleep.
 * 
 * -----------------------------------------------------------------------------
 * Version 1.3.7 @ 20240925
 * Cambiamos de nombre al proyecto a  FWDLG.
 * La idea es que el firmware sirva a los dataloggers SPX y SPQ.
 * En el directorio PORTABLE vamos a poner un include que se use para c/plataforma.
 * Agrego una flag con -D que indique un define global con el que identifico el tipo
 * de arquitectura
 * tkCMD: Modifico para que mida term_sense y entre en modo tickless (60s) y sino
 * continue en modo activo.
 * M3: OK. Consumo en reposo 0.450 mA
 * 
 * Version 1.3.6 BETA @ 20241011
 * 
 * https://devzone.nordicsemi.com/f/nordic-q-a/12926/freertos-tick-count-invalid-when-using-tickless-idle
 * 
 * Agrego el condicional DEBUG_COUNTERS_TICKLESSMODE para ver que pasa
 * con los pulsos en el modo TICKLESS.
 * El equipo UYCOL003 da errores en modo discreto y no los da en continuo.
 * -----------------------------------------------------------------------------
 * Version 1.3.6 @ 20240923
 * Agregamos todo lo referente a la confguracion del modem.
 * - modificamos el SystemConf, u_config_default, load_NVM, save_NVM, tkCMD
 * - modificamos modem_config_default.
 * - Agrego las opciones de modem config ip, port, apn.
 * - Chequeo la configuracion al prender el modem.
 * 
 * -----------------------------------------------------------------------------
 * Version 1.3.5 @ 20240918
 * Consumo standby = 0.590 mA
 * - Problema: hay veces que el modem se resetea a default.
 * - Solucion:
 * - Fijo la velocidad en 115200 de modo que aunque se vaya a default, puedo leerlo
 *   y reconfigurarlo
 * - El pin RELOAD si baja 3s y luego sube, lo resetea a default. Como no lo usamos
 *   eliminamos el hardware que lo usa. Con esto eliminamos la posibilidad que el 
 *   reset sea por esto.
 *   
 * -----------------------------------------------------------------------------
 * Version 1.3.4 @ 20240918
 * Consumo standby = 0.590 mA
 * 
 * - BUG: la tkSYS reseteaba al wdg porque esperaba 180 secs y el wdg solo 120.
 *        Se modifica para esperar 60s
 * - Mejoro la medida del ADC ( 16 lecturas + 64 lecturas )
 * - Corrijo forma de trasmitir x modbus (UART4) para no cortar al ultimo byte
 * - BUG: En open/close VALVE no apagabamos el CTL_PIN y quedaba consimiendo.
 *   Al inicializarla la dejo en standby.
 * - Deshabilito todo lo referente a VALVE.(No inicializo ni seteo inicial en 
 *   control de presion). Esto es porque aumenta el consumo. Cuando se vaya a usar
 *   lo atendemos.
 * - Agrego la tarea de control de presion que por ahora no hace nada hasta que
 *   incorporemos las consignas.
 * - Despues de 5mins, a veces sube el consumo por tkCMD. Cambio la forma de 
 *   manejarlo usando el pin TERMSENSE.
 * - Elimino el manejo de consignas.
 * 
 * 
 * -----------------------------------------------------------------------------
 * Version 1.3.3 @ 20240819
 * - Elimino la tkCtlPres porque no entra en sleep y consume.
 *   Cuando se haga todo lo relativo a doble consigna, lo incorporamos.
 *   Queda entonces consumiendo en sleep 0.6 mA
 * 
 * -----------------------------------------------------------------------------
 * Version 1.3.2 @ 20240814
 * - Al arrancar leemos el valor del vector de reset para saber la causa del reset.
 *   Este valor lo enviamos en el frame de CONF_BASE y luego lo borramos.
 * - Modificamos el frame de CONF_BASE paa que mande el valor del watchdog.
 *   Esto implica que el modem debe reconfigurarse para que el packaging time sea
 *   de 250 y permita frames m�s largos.
 * - Agregamos en el comando 'test modem queryall' que lea el packaging time, y 
 *   agregamos un comando para modificarlo: 'test modem set ftime'
 * - Agrego en el menu de test de valve operar sobre el pin enable y ctl.
 * - La incializacion de la valve a open la hago dentro del tkCtlPresion ya que 
 *   demora 10s y si la hago en tkCtl el watchdog reseteaba el micro. 
 * -----------------------------------------------------------------------------
 * Version 1.3.1 @ 20240813
 * - Cuando imprimo en pantalla en los menues uso strings cortos para no generar
 *   errores de caracteres.
 * - Controlo la causa del reset en el arranque y la mando en el primer frame
 * - El manejo de las tareas corriendo y los watchdows lo hago por medio de 
 *   vectores de bools.
 * -----------------------------------------------------------------------------
 * Version 1.2.5 @ 2024-06-04:
 * - Elimino el semaforo de modbus y creo uno global sem_RS485 que lo piden todos
 *   los que deban acceder al puerto.
 * ----------------------------------------------------------------------------- 
 * Version 1.2.1 @ 2024-04-10:
 * - cambio las funciones strncpy por strlcpy
 * - PENDIENTE: Controlar los SNPRINTF
 * -----------------------------------------------------------------------------
 * Version 1.2.0 @ 2024-04-08:
 * Aumento el tama�o de los buffers en las funciones que hacen hash
 * -----------------------------------------------------------------------------
 * Version 1.1.0 @ 2023-11-15:
 * Modbus: El caudalimentro Shinco tiene s/n 77 entonces el id es 119 !!!
 * -----------------------------------------------------------------------------
 * Version 1.1.0 @ 2023-07-21
 * Veo que hay veces que se cuelga o que tareas ocupan mas tiempo del esperable.
 * Esto se refleja en el flasheo del led e indica problemas con la asignacion de
 * tareas.
 * https://www.freertos.org/FreeRTOS_Support_Forum_Archive/December_2013/freertos_configUSE_TIME_SLICING_278999daj.html
 * https://www.freertos.org/FreeRTOS_Support_Forum_Archive/February_2018/freertos_Preemption_by_an_equal_priority_task_c8cb93d0j.html
 * 
 * #define configUSE_TIME_SLICING 1
 * 
 * El problema esta en la funcion FLASH_0_read_eeprom_block.
 * La soluci�n fue poner vTaskDelay( ( TickType_t)( 10 ) ) ente c/escritura de bytes.
 * 
 * Control del tama�o de las colas:
 * https://www.freertos.org/FAQMem.html#StackSize
 * https://www.freertos.org/FreeRTOS_Support_Forum_Archive/November_2015/freertos_Measuring_Stack_Usage_b5706bb4j.html
 * 
 * -----------------------------------------------------------------------------
 * Version 1.0.0 @ 2022-09-12
  * 
 * Para que imprima floats hay que poner en el linker options la opcion
 * -Wl,-u,vfprintf -lprintf_flt -lm
 * https://github.com/microchip-pic-avr-examples/avr128da48-cnano-printf-float-mplab-mcc
 * 
 * Log en el servidor:
 * hset 'SPCOMMS' 'DEBUG_DLGID' 'PPOTKIYU'
 * 
 * 
 * PENDIENTE:
 * 1- Transmitir en modo bloque al hacer un dump.
 * 2- Consumo: entrar en modo tickless
 *
 * -----------------------------------------------------------------------------
 * V1.1.0 @ 20230626
 * Hay muchos equipos que no pasan de la configuracion.
 * El problema es que se resetea x wdt. No queda claro porque pero con un
 * wdg_reset en  wan_state_online_data se arregla.
 * HAY QUE REVISAR TODO EL TEMA DE LOS WDGs. !!!!
 * 1,848,861-8
 * Patricia Cruz y Jos� Montejo
 * LITERATURA CRUZ BARISIONE, Patricia
 * Iris e In�s Montejo
 * http://www.CABRERA/MONS/ENRIQUE/Montevideo/Padron/2354U
 * 18488618
 * 22923645
 * CMC-20042
 * Circuito 625
 * -----------------------------------------------------------------------------
 * V1.1.0 @ 20230620
 * Si el canal analogico 2 esta configurado, entonces la bateria es 0.
 * El protocolo esta modificado para hablar con APICOMMSV3
 * -----------------------------------------------------------------------------
 * V1.0.8 @ 20230505
 * En los contadores incorporo el concepto de rb_size que sirve para determinar
 * cuanto quiero promediar los contadores.
 * El problema surge en perforaciones Colonia que no es util un promediado largo.
 * 
 * -----------------------------------------------------------------------------
 * V1.0.7 @ 20230306
 * Incorporamos el modulo de modbus.
 * Manejamos 5 canales (float) por lo que aumentamos el FS_BLOCKSIZE a 64.
 * Falta:
 *  - A veces da el pc timeout ? Es como si el frame se fragmentara
 *  - WAN
 * 
 * - BUG: No podia escribir el RTC
 * - BUG: Los canales analogicos, contadores, modbus por default deben ser 'X'
 * - BUG: Los contadores al medir caudal nunca llegaban a 0. Agrego en counters_clear()
 *        que converjan a 0.
 * - BUG: No calculaba bien el hash de conf_base y por eso se reconfigura siempre
 * 
 *  
 * -----------------------------------------------------------------------------
 * V1.0.6 @ 20230202
 * Solo manejo 2 canales y el 3o es la bateria.
 * 
 * V1.0.5 @ 20230202
 * Configuro 'samples_count' y 'alarm_level' por la wan.
 *  
 * V1.0.4 @ 20230118
 * Incorporo los conceptos de 'samples_count' y 'alarm_level'
 *  
 * 
 * V1.0.3 @ 20230109
 * Agrego logica de manejo de reles K1,K2. El modem esta en K1.
 * Revisar el reset memory hard que bloquea al dlg.
 * 
 * 
 * V1.0.2 @ 2023-01-02
 * Agrego en la funcion ainputs_test_read_channel() que prenda y apague los sensores.
 * 
 * V1.2.4 @ 2024-05-03:
 * BUG: La velocidad del bus I2C estaba mal seteada, suponiendo un clock de 4Mhz y no de 24Mhz
 * Lo soluciono cambiando el #define al ppio del archivo drv_i2c_avrDX.c
 * 
 * V1.2.6 @ 2024-06-24:
 * Cambio la velocidad del puerto TERM a 9600 para que quede compatible
 * con los bluetooh que usan los t�cnicos.
 *  
 */

#include "FWDLGX.h"


#ifdef HW_AVRDA

FUSES = {
	.WDTCFG = 0x0B, // WDTCFG {PERIOD=8KCLK, WINDOW=OFF}
	.BODCFG = 0x00, // BODCFG {SLEEP=DISABLE, ACTIVE=DISABLE, SAMPFREQ=128Hz, LVL=BODLEVEL0}
	.OSCCFG = 0xF8, // OSCCFG {CLKSEL=OSCHF}
	.SYSCFG0 = 0xD2, // SYSCFG0 {EESAVE=CLEAR, RSTPINCFG=GPIO, CRCSEL=CRC16, CRCSRC=NOCRC}
	.SYSCFG1 = 0xF8, // SYSCFG1 {SUT=0MS}
	.CODESIZE = 0x00, // CODESIZE {CODESIZE=User range:  0x0 - 0xFF}
	.BOOTSIZE = 0x00, // BOOTSIZE {BOOTSIZE=User range:  0x0 - 0xFF}
};

LOCKBITS = 0x5CC5C55C; // {KEY=NOLOCK}

#endif

//------------------------------------------------------------------------------
int main(void) {

uint8_t i;
    
// Leo la causa del reset para trasmitirla en el init.
#ifdef HW_AVRDA
	wdg_resetCause = RSTCTRL.RSTFR;
	RSTCTRL.RSTFR = 0xFF;
#endif
    
//    rtcounter = 0;
    
    // Init out of rtos !!!
    system_init();
    
    frtos_open(fdTERM, 9600 );
    frtos_open(fdWAN, 115200 );
    frtos_open(fdNVM, 0 );   
    frtos_open(fdI2C, 100 );
#ifdef HW_AVRDA
    frtos_open(fdRS485A, 9600 );
#endif
    
    // Creo el semaforo de systemVars
    SYSTEM_INIT();

    // Creo el semaforo del Fsystem
    FS_init();
    
    // Inicializaciones sin el RTOS corriendo
    ainputs_init_outofrtos();
    counter_init_outofrtos(&xHandle_tkCtl);
    modem_init_outofrtos(&xHandle_tkWANRX);
    rs485_init_outofrtos(&xHandle_tkRS485RX);
    
    // Inicializo el vector de tareas activas
    for (i=0; i< NRO_TASKS; i++) {
        tk_running[i] = false;
    }

    // Inicializo el vector de watchdogs ( 5 minutos para c/tarea)
    for (i=0; i< NRO_TASKS; i++) {
        tk_watchdog[i] = 60;
    }
    
    starting_flag = false;
    
    xHandle_tkCtl = xTaskCreateStatic( tkCtl, "CTL", tkCtl_STACK_SIZE, (void *)1, tkCtl_TASK_PRIORITY, tkCtl_Buffer, &tkCtl_Buffer_Ptr );
    xHandle_tkCmd = xTaskCreateStatic( tkCmd, "CMD", tkCmd_STACK_SIZE, (void *)1, tkCmd_TASK_PRIORITY, tkCmd_Buffer, &tkCmd_Buffer_Ptr );
    xHandle_tkSys = xTaskCreateStatic( tkSys, "SYS", tkSys_STACK_SIZE, (void *)1, tkSys_TASK_PRIORITY, tkSys_Buffer, &tkSys_Buffer_Ptr );
    xHandle_tkWANRX = xTaskCreateStatic( tkWanRX, "WANRX", tkWANRX_STACK_SIZE, (void *)1, tkWANRX_TASK_PRIORITY, tkWANRX_Buffer, &tkWANRX_Buffer_Ptr );
    xHandle_tkWAN = xTaskCreateStatic( tkWan, "WAN", tkWAN_STACK_SIZE, (void *)1, tkWAN_TASK_PRIORITY, tkWAN_Buffer, &tkWAN_Buffer_Ptr );
    xHandle_tkRS485RX = xTaskCreateStatic( tkRS485RX, "RS485", tkRS485RX_STACK_SIZE, (void *)1, tkRS485RX_TASK_PRIORITY, tkRS485RX_Buffer, &tkRS485RX_Buffer_Ptr );
    xHandle_tkCtlPres = xTaskCreateStatic( tkCtlPres, "CPRES", tkCtlPres_STACK_SIZE, (void *)1, tkCtlPres_TASK_PRIORITY, tkCtlPres_Buffer, &tkCtlPres_Buffer_Ptr );
    
    /* Arranco el RTOS. */
	vTaskStartScheduler();
  
	// En caso de panico, aqui terminamos.
	exit (1);
    
}
//------------------------------------------------------------------------------
/* configSUPPORT_STATIC_ALLOCATION is set to 1, so the application must provide an
   implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
   used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t ** ppxIdleTaskTCBBuffer,
                                        StackType_t ** ppxIdleTaskStackBuffer,
                                        configSTACK_DEPTH_TYPE * puxIdleTaskStackSize )

{
    /* If the buffers to be provided to the Idle task are declared inside this
       function then they must be declared static - otherwise they will be allocated on
       the stack and so not exists after this function exits. */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
       state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
       Note that, as the array is necessarily of type StackType_t,
       configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *puxIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/*-----------------------------------------------------------*/

/* configSUPPORT_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
   application must provide an implementation of vApplicationGetTimerTaskMemory()
   to provide the memory that is used by the Timer service task. */
 void vApplicationGetTimerTaskMemory( StaticTask_t ** ppxTimerTaskTCBBuffer,
                                         StackType_t ** ppxTimerTaskStackBuffer,
                                         configSTACK_DEPTH_TYPE * puxTimerTaskStackSize )
{
    /* If the buffers to be provided to the Timer task are declared inside this
       function then they must be declared static - otherwise they will be allocated on
       the stack and so not exists after this function exits. */
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

    /* Pass out a pointer to the StaticTask_t structure in which the Timer
       task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
       Note that, as the array is necessarily of type StackType_t,
      configTIMER_TASK_STACK_DEPTH is specified in words, not bytes. */
    *puxTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
//------------------------------------------------------------------------------
/*
void vApplicationIdleHook( void )
{

    rtcounter++;
    
}
*/
        
//------------------------------------------------------------------------------
