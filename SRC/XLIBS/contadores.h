/* 
 * File:   contadores.h
 * Author: pablo
 *
 * Created on July 19, 2023, 5:04 PM
 */

#ifndef CONTADORES_H
#define	CONTADORES_H

#ifdef	__cplusplus
extern "C" {
#endif


#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
    
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "math.h"
    
#include "xprintf.h"    
#include "pines.h"
#include "utils.h"
#include "nvm.h"
    
#define CNT_PARAMNAME_LENGTH	12
    
typedef enum { CAUDAL = 0, PULSOS } t_counter_modo;

// Configuracion de canales de contadores
typedef struct {
    bool enabled;
	char name[CNT_PARAMNAME_LENGTH];
	float magpp;
    t_counter_modo modo_medida;
    uint8_t checksum;
} counter_conf_t;

counter_conf_t counter_conf;

typedef struct {
    uint8_t fsm_ticks_count;
    uint16_t pulsos;
    float caudal;
    uint32_t start_pulse_ticks;
    uint32_t now_ticks;
    uint32_t T_ticks;
    float T_secs;
} counter_value_t;

counter_value_t contador;

StaticTimer_t counter_xTimerBuffer;
TimerHandle_t counter_xTimer;

void counter_init_outofrtos(TaskHandle_t *xHandle);
void counter_init( void );
void counter_config_defaults( void );
void counter_print_configuration( void );
bool counter_config_channel( char *s_enable, char *s_name, char *s_magpp, char *s_modo );
void counter_config_debug(bool debug );
bool counter_read_debug(void);

void counter_clear(void);
uint8_t counter_read_pin(void);
counter_value_t counter_read();
uint8_t counter_hash( void );

int16_t counter_save_config_in_NVM(uint16_t nvm_ptr);
int16_t counter_load_config_from_NVM( uint16_t nvm_address);

#ifdef	__cplusplus
}
#endif

#endif	/* CONTADORES_H */

