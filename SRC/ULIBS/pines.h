    /* 
 * File:   pines.h
 * Author: pablo
 *
 * Created on 11 de febrero de 2022, 06:02 PM
 */

#ifndef PINES_H
#define	PINES_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <avr/io.h>
#include "stdbool.h"
    
#ifdef MODEL_M3

    // PWR_SENSORS (sensors de 4-20, output)
    #define PWR_SENSORS_PORT         PORTC
    #define PWR_SENSORS              1
    #define PWR_SENSORS_PIN_bm       PIN1_bm
    #define PWR_SENSORS_PIN_bp       PIN1_bp

    // EN_SENS3V3 (output)
    #define EN_SENS3V3_PORT         PORTD
    #define EN_SENS3V3              5
    #define EN_SENS3V3_PIN_bm       PIN5_bm
    #define EN_SENS3V3_PIN_bp       PIN5_bp
    
    // EN_SENS12V (output)
    #define EN_SENS12V_PORT         PORTD
    #define EN_SENS12V              4
    #define EN_SENS12V_PIN_bm       PIN4_bm
    #define EN_SENS12V_PIN_bp       PIN4_bp
    
    #define SET_EN_SENS3V3()        ( EN_SENS3V3_PORT.OUT |= EN_SENS3V3_PIN_bm )
    #define CLEAR_EN_SENS3V3()      ( EN_SENS3V3_PORT.OUT &= ~EN_SENS3V3_PIN_bm )
    #define CONFIG_EN_SENS3V3()     ( EN_SENS3V3_PORT.DIR |= EN_SENS3V3_PIN_bm )

    #define SET_EN_SENS12V()        ( EN_SENS12V_PORT.OUT |= EN_SENS12V_PIN_bm )
    #define CLEAR_EN_SENS12V()      ( EN_SENS12V_PORT.OUT &= ~EN_SENS12V_PIN_bm )
    #define CONFIG_EN_SENS12V()     ( EN_SENS12V_PORT.DIR |= EN_SENS12V_PIN_bm )

#endif

#ifdef MODEL_M1

    //--------------------------------------------------------------------------
    // TWI PE0(SDA)/PE1(SCL)
    #define SCL_PORT        PORTE
    #define SCL             1
    #define SCL_PIN_bm      PIN1_bm
    #define SCL_PIN_bp      PIN1_bp

    #define CONFIG_SCL()     SCL_PORT.DIR |= SCL_PIN_bm;
    #define SET_SCL()        ( SCL_PORT.OUT |= SCL_PIN_bm )
    #define CLEAR_SCL()		 ( SCL_PORT.OUT &= ~SCL_PIN_bm )
    //--------------------------------------------------------------------------
    
    #define PWR_SENSORS_PORT         PORTD
    #define PWR_SENSORS              1
    #define PWR_SENSORS_PIN_bm       PIN1_bm
    #define PWR_SENSORS_PIN_bp       PIN1_bp

#endif

#define CONFIG_PWR_SENSORS()    (PWR_SENSORS_PORT.DIR |= PWR_SENSORS_PIN_bm)
#define SET_PWR_SENSORS()       ( PWR_SENSORS_PORT.OUT |= PWR_SENSORS_PIN_bm )
#define CLEAR_PWR_SENSORS()     ( PWR_SENSORS_PORT.OUT &= ~PWR_SENSORS_PIN_bm )



//------------------------------------------------------------------------------

#ifdef	__cplusplus
}
#endif

#endif	/* PINES_H */

