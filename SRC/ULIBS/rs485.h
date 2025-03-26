/* 
 * File:   rs485.h
 * Author: pablo
 *
 * Created on 14 de marzo de 2025, 08:57 AM
 */

#ifndef RS485_H
#define	RS485_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
    
#include "stdint.h"
#include "stdbool.h"
    
bool rs485_awake;

void rs485_AWAKE(void);
void rs485_SLEEP(void);

void rs485_init_outofrtos( TaskHandle_t *xHandle );
void rs485_ENTER_CRITICAL(void);
void rs485_EXIT_CRITICAL(void);

#ifdef	__cplusplus
}
#endif

#endif	/* RS485_H */

