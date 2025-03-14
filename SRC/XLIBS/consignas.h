/* 
 * File:   consignas.h
 * Author: pablo
 *
 * Created on 12 de septiembre de 2023, 10:26 AM
 */

#ifndef CONSIGNAS_H
#define	CONSIGNAS_H

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
    
#include "xprintf.h"
#include "toyi_valves.h"
#include "rtc79410.h"
#include "utils.h"
#include "pines.h"
#include "modbus.h"
//#include "bits.h"
#include "cpres.h"
    
typedef struct {
    bool enabled;
	uint16_t consigna_diurna;
	uint16_t consigna_nocturna;
    uint8_t checksum;
} consigna_conf_t;

consigna_conf_t consigna_conf;

consigna_t consigna_aplicada;

//void consigna_init( int fd_consigna, int buffer_size, void (*f)(void), uint16_t (*g)(void), char *(*h)(void)  );
void consigna_config_defaults(void);
bool consigna_config( char *s_enable, char *s_cdiurna, char *s_cnocturna );
void consigna_print_configuration(void);
void consigna_config_debug(bool debug );
uint8_t consigna_hash(void);
bool consigna_debug_flag(void);

bool CONSIGNA_set_diurna(void);
bool CONSIGNA_set_nocturna(void);

int16_t consigna_save_config_in_NVM(uint16_t nvm_ptr);
int16_t consigna_load_config_from_NVM( uint16_t nvm_address);


#ifdef	__cplusplus
}
#endif

#endif	/* CONSIGNAS_H */

