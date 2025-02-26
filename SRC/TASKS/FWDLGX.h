/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

// Arquitectura para la que compilamos

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "list.h"
#include "croutine.h"
#include "semphr.h"
#include "timers.h"
#include "limits.h"
#include "portable.h"
#include "portmacro.h"
#include "ccp.h"

#include <avr/wdt.h> 
#include <avr/pgmspace.h>
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "frtos-io.h"
#include "xprintf.h"
#include "xgetc.h"
#include "pines.h"
#include "linearBuffer.h"
#include "led.h"
#include "i2c.h"
#include "eeprom.h"
#include "rtc79410.h"
#include "ina3221.h"
#include "nvm.h"
#include "fileSystem.h"
#include "base.h"
#include "ainputs.h"
#include "contadores.h"
#include "modem_lte.h"

#ifdef MODEL_M1

    #ifndef F_CPU
        #define F_CPU 32000000
    #endif
    
    #define FW_TYPE "SPX_XMEGA"  
    #define SYSMAINCLK 32
    #define HW_MODELO "FWDLG M1 FWFRTOS R001 HW:AVR128DA64"

    #include "clksys_driver.h"

#endif

#ifdef MODEL_M3

#include "protected_io.h"
    #ifndef F_CPU
        #define F_CPU 24000000
    #endif
    
    #define FW_TYPE "SPQ_AVRDA"  
    #define SYSMAINCLK 24
    #define HW_MODELO "FWDLG M3 FWFRTOS R001 HW:AVR128DA64"

#endif
                
#define FW_REV "1.0.0"
#define FW_DATE "@ 20250225"
#define FRTOS_VERSION "FW:FreeRTOS 202212.01"

#define pdSECS_TO_TICKS(xTimeInSecs) ((TickType_t)(((uint32_t)(xTimeInSecs) * (uint32_t)configTICK_RATE_HZ) ))
#define pdTICS_TO_SECS(xTimeInTicks) ((TickType_t)(((uint32_t)(xTimeInTicks) / (uint32_t)configTICK_RATE_HZ) ))

#define tkCtl_TASK_PRIORITY	 	( tskIDLE_PRIORITY + 1 )
#define tkCmd_TASK_PRIORITY 	( tskIDLE_PRIORITY + 1 )
#define tkSys_TASK_PRIORITY 	( tskIDLE_PRIORITY + 1 )
#define tkWANRX_TASK_PRIORITY	( tskIDLE_PRIORITY + 1 )

#define tkCtl_STACK_SIZE		384
#define tkCmd_STACK_SIZE		512
#define tkSys_STACK_SIZE		512
#define tkWANRX_STACK_SIZE      384

StaticTask_t tkCtl_Buffer_Ptr;
StackType_t tkCtl_Buffer [tkCtl_STACK_SIZE];

StaticTask_t tkCmd_Buffer_Ptr;
StackType_t tkCmd_Buffer [tkCmd_STACK_SIZE];

StaticTask_t tkSys_Buffer_Ptr;
StackType_t tkSys_Buffer [tkSys_STACK_SIZE];

StaticTask_t tkWANRX_Buffer_Ptr;
StackType_t tkWANRX_Buffer [tkWANRX_STACK_SIZE];

TaskHandle_t xHandle_tkCtl, xHandle_tkCmd, xHandle_tkSys, xHandle_tkWANRX;

void tkCtl(void * pvParameters);
void tkCmd(void * pvParameters);
void tkSys(void * pvParameters);
void tkWanRX(void * pvParameters);

bool starting_flag;

void system_init(void);
void reset(void);

// Estructuras de datos del sistema
// -----------------------------------------------------------------------------

// Tipo que define la estrucutra de las medidas tomadas.
typedef struct {
    float ainputs[NRO_ANALOG_CHANNELS];
    float contador;
    //float modbus[NRO_MODBUS_CHANNELS];
    float bt3v3;
    float bt12v;
    RtcTimeType_t  rtc;	   
} dataRcd_s;

bool u_config_debug( char *tipo, char *valor);
void u_config_default( char *modo );
bool u_save_config_in_NVM(void);
bool u_load_config_from_NVM(void);
dataRcd_s *get_dataRcd_ptr(void);
bool u_poll_data(dataRcd_s *dataRcd);
void u_xprint_dr(dataRcd_s *dr);
void u_print_tasks_running(void);

//------------------------------------------------------------------------------
// Task running & watchdogs
#define NRO_TASKS  3

typedef enum { TK_CMD = 0, TK_SYS, TK_WANRX } t_wdg_ids;

void u_kick_wdt( t_wdg_ids wdg_id, uint16_t wdg_timer);

bool tk_running[NRO_TASKS];

int16_t tk_watchdog[NRO_TASKS];

uint8_t wdg_resetCause;


#endif	/* XC_HEADER_TEMPLATE_H */

