/* 
 * File:   adc.h
 * Author: pablo
 *
 * Created on 27 de julio de 2022, 12:14 PM
 */

#ifndef ADC_H
#define	ADC_H

#ifdef	__cplusplus
extern "C" {
#endif

    
#include <avr/io.h>
#include "stdint.h"
#include "stdbool.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "pines.h"
#include "task.h"
#include <avr/pgmspace.h>
#include "xprintf.h" 
    
#define ADC_SHIFT_DIV16     (4)         /* where 4 is 2^4 = 16 */
#define ADC_SHIFT_DIV8      (3)         /* where 3 is 2^3 =  8 */
    
typedef void (*adc_irq_cb_t)(void);

/** Datatype for the result of the ADC conversion */
typedef uint16_t adc_result_t;
typedef int16_t diff_adc_result_t;

//* Analog channel selection */


int8_t ADC_init();
void ADC_enable();
void ADC_disable();
void ADC_stop_conversion();
bool ADC_is_conversion_done();
adc_result_t ADC_get_conversion_result(void);
uint8_t ADC_get_resolution();
uint16_t ADC_read_sens3v3(void);
uint16_t ADC_read_sens12v(void);

#ifdef HW_AVRDA
    typedef ADC_MUXPOS_t adc_0_channel_t;
    typedef ADC_MUXNEG_t adc_0_muxneg_channel_t;
#endif

#if defined HW_XMEGA
    typedef int16_t adc_0_channel_t;
    typedef int16_t adc_0_muxneg_channel_t;
#endif    
    
void ADC_start_conversion(adc_0_channel_t channel);
void ADC_start_diff_conversion(adc_0_channel_t channel, adc_0_muxneg_channel_t channel1);
adc_result_t ADC_get_conversion(adc_0_channel_t channel);
diff_adc_result_t ADC_get_diff_conversion(adc_0_channel_t channel, adc_0_muxneg_channel_t channel1);
uint16_t ADC_read_single(adc_0_channel_t channel, bool debug);
uint16_t ADC_read_multiple(adc_0_channel_t channel, uint8_t samples, bool debug);




#ifdef	__cplusplus
}
#endif

#endif	/* ADC_H */

