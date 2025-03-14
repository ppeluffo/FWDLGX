/* 
 * File:   cpres.h
 * Author: pablo
 *
 * Created on 7 de junio de 2024, 10:05 AM
 */

#ifndef CPRES_H
#define	CPRES_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "stdbool.h"
    
#include "modbus.h"
#include "xprintf.h"
#include "pines.h"
#include "rs485.h"
#include "bits.h"
    
typedef enum { CONSIGNA_DIURNA = 0, CONSIGNA_NOCTURNA } consigna_t;

void cpres_prender_sensor(void);
void cpres_apagar_sensor(void);
bool cpres_consigna_set( uint8_t consigna);
bool cpres_sensor_is_idle(void);
bool cpres_sensor_write( uint8_t consigna);

#define SENSOR_PRESION_ADDR  0x64
#define STATUS_BIT           7

mbus_CONTROL_BLOCK_t consigna_mbus_cb;



#ifdef	__cplusplus
}
#endif

#endif	/* CPRES_H */

