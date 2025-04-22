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
    
#ifdef HW_AVRDA

    // RS485_RTS
    #define RTS_RS485_PORT         PORTD    
    #define RTS_RS485              7
    #define RTS_RS485_PIN_bm       PIN7_bm
    #define RTS_RS485_PIN_bp       PIN7_bp

    // EN_PWR_CPRES (output)
    #define EN_PWR_CPRES_PORT         PORTD
    #define EN_PWR_CPRES              6
    #define EN_PWR_CPRES_PIN_bm       PIN6_bm   
    #define EN_PWR_CPRES_PIN_bp       PIN6_bp

    // EN_PWR_QMBUS (output)
    #define EN_PWR_QMBUS_PORT         PORTF
    #define EN_PWR_QMBUS              1
    #define EN_PWR_QMBUS_PIN_bm       PIN1_bm
    #define EN_PWR_QMBUS_PIN_bp       PIN1_bp

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
    
#endif

#ifdef HW_XMEGA

    // TWI PE0(SDA)/PE1(SCL)
    #define SCL_PORT        PORTE
    #define SCL             1
    #define SCL_PIN_bm      PIN1_bm
    #define SCL_PIN_bp      PIN1_bp
    
    #define PWR_SENSORS_PORT         PORTA
    #define PWR_SENSORS              1
    #define PWR_SENSORS_PIN_bm       PIN1_bm
    #define PWR_SENSORS_PIN_bp       PIN1_bp

#endif

void SET_EN_PWR_CPRES(void);
void CLEAR_EN_PWR_CPRES(void);
void CONFIG_EN_PWR_CPRES(void);

void SET_EN_PWR_QMBUS(void);
void CLEAR_EN_PWR_QMBUS(void);
void CONFIG_EN_PWR_QMBUS(void);

void SET_RTS_RS485(void);
void CLEAR_RTS_RS485(void);
void CONFIG_RTS_485(void);

void CONFIG_SCL(void);
void SET_SCL(void);
void CLEAR_SCL(void);

void CONFIG_PWR_SENSORS(void);
void SET_PWR_SENSORS(void);
void CLEAR_PWR_SENSORS(void);

void SET_EN_SENS3V3(void);
void CLEAR_EN_SENS3V3(void);
void CONFIG_EN_SENS3V3(void);

void SET_EN_SENS12V(void);
void CLEAR_EN_SENS12V(void);
void CONFIG_EN_SENS12V(void);


//------------------------------------------------------------------------------

#ifdef	__cplusplus
}
#endif

#endif	/* PINES_H */

