/* 
 * File:   utils.h
 * Author: pablo
 *
 * Created on 2 de junio de 2023, 03:27 PM
 */

#ifndef UTILS_H
#define	UTILS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "semphr.h"
    
#include <avr/pgmspace.h>
#include "stdint.h"
#include "stdlib.h"
#include "stdbool.h"
#include "xprintf.h"
#include "adc.h"
#include "pines.h"
#include "ainputs.h"
#include "rtc79410.h"
#include "base.h"
    
 
uint8_t u_hash(uint8_t seed, char ch );
uint8_t u_checksum( uint8_t *s, uint16_t size );
float u_read_bat3v3(bool debug);
float u_read_bat12v(bool debug);
uint16_t u_hhmm_to_mins(uint16_t hhmm);
uint32_t u_get_sleep_time(bool debug);

#define MSTOTAKESYSVARSSEMPH ((  TickType_t ) 10 )

SemaphoreHandle_t sem_SYSVars;
StaticSemaphore_t SYSVARS_xMutexBuffer;

void SYSTEM_INIT(void);
void SYSTEM_ENTER_CRITICAL(void);
void SYSTEM_EXIT_CRITICAL(void);

#ifdef	__cplusplus
}
#endif

#endif	/* UTILS_H */

