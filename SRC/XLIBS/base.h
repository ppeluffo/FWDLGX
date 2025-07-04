/* 
 * File:   base.h
 * Author: pablo
 *
 * Created on 24 de febrero de 2025, 09:25 AM
 */

#ifndef BASE_H
#define	BASE_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "task.h"

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "math.h"
    
#include "xprintf.h"
#include "utils.h"
#include "nvm.h"
    
    
#define DLGID_LENGTH		12
    
#define TDIAL_MIN_DISCRETO  900
    
typedef enum { PWR_CONTINUO = 0, PWR_DISCRETO, PWR_MIXTO, PWR_RTU } pwr_modo_t;

typedef struct {
    char dlgid[DLGID_LENGTH];
    uint16_t timerpoll;
    uint16_t timerdial;
    pwr_modo_t pwr_modo;
    uint16_t pwr_hhmm_on;
    uint16_t pwr_hhmm_off;  
    uint8_t checksum;
} base_conf_t;

base_conf_t base_conf;

void base_config_defaults( void );
void base_print_configuration( void );
bool base_config_timerdial ( char *s_timerdial );
bool base_config_timerpoll ( char *s_timerpoll );
bool base_config_dlgid ( char *s_dlgid );
bool base_config_pwrmodo ( char *s_pwrmodo );
bool base_config_pwron ( char *s_pwron );
bool base_config_pwroff ( char *s_pwroff );
uint8_t base_hash(void);

int16_t base_save_config_in_NVM(uint16_t nvm_ptr);
int16_t base_load_config_from_NVM( uint16_t nvm_address);

#ifdef	__cplusplus
}
#endif

#endif	/* BASE_H */

