/* 
 * File:   led.h
 * Author: pablo
 *
 * Created on 8 de febrero de 2022, 11:33 AM
 */

#ifndef LED_H
#define	LED_H

#ifdef	__cplusplus
extern "C" {
#endif


//#include <xc.h> 
#include <avr/io.h>
#include "FreeRTOS.h"
#include "task.h"
    
void LED_init(void);
void led_flash(void);
void led_prender(void);
void led_apagar(void);
void led_toggle(void);


#ifdef	__cplusplus
}
#endif

#endif	/* LED_H */

