
#include "utils.h"

#define BAT3V3_FACTOR ( 2.5 * 2 / 4095 )
#define BAT12V_FACTOR ( 2.5 * 6.6 / 4095 )


const uint8_t hash_table[] PROGMEM =  {
		 93,  153, 124,  98, 233, 146, 184, 207, 215,  54, 208, 223, 254, 216, 162, 141,
		 10,  148, 232, 115,   7, 202,  66,  31,   1,  33,  51, 145, 198, 181,  13,  95,
		 242, 110, 107, 231, 140, 170,  44, 176, 166,   8,   9, 163, 150, 105, 113, 149,
		 171, 152,  58, 133, 186,  27,  53, 111, 210,  96,  35, 240,  36, 168,  67, 213,
		 12,  123, 101, 227, 182, 156, 190, 205, 218, 139,  68, 217,  79,  16, 196, 246,
		 154, 116,  29, 131, 197, 117, 127,  76,  92,  14,  38,  99,   2, 219, 192, 102,
		 252,  74,  91, 179,  71, 155,  84, 250, 200, 121, 159,  78,  69,  11,  63,   5,
		 126, 157, 120, 136, 185,  88, 187, 114, 100, 214, 104, 226,  40, 191, 194,  50,
		 221, 224, 128, 172, 135, 238,  25, 212,   0, 220, 251, 142, 211, 244, 229, 230,
		 46,   89, 158, 253, 249,  81, 164, 234, 103,  59,  86, 134,  60, 193, 109,  77,
		 180, 161, 119, 118, 195,  82,  49,  20, 255,  90,  26, 222,  39,  75, 243, 237,
		 17,   72,  48, 239,  70,  19,   3,  65, 206,  32, 129,  57,  62,  21,  34, 112,
		 4,    56, 189,  83, 228, 106,  61,   6,  24, 165, 201, 167, 132,  45, 241, 247,
		 97,   30, 188, 177, 125,  42,  18, 178,  85, 137,  41, 173,  43, 174,  73, 130,
		 203, 236, 209, 235,  15,  52,  47,  37,  22, 199, 245,  23, 144, 147, 138,  28,
		 183,  87, 248, 160,  55,  64, 204,  94, 225, 143, 175, 169,  80, 151, 108, 122
};

//------------------------------------------------------------------------------
uint8_t u_hash(uint8_t seed, char ch )
{
	/*
	 * Funcion de hash de pearson modificada.
	 * https://es.qwe.wiki/wiki/Pearson_hashing
	 * La función original usa una tabla de nros.aleatorios de 256 elementos.
	 * Ya que son aleatorios, yo la modifico a algo mas simple.
	 */

uint8_t h_entry = 0;
uint8_t h_val = 0;
uint8_t ord;

	ord = (int)ch;
	h_entry = seed ^ ord;
	h_val = (PGM_P)pgm_read_byte_far( &(hash_table[h_entry]));
	return(h_val);
}
//------------------------------------------------------------------------------
uint8_t u_checksum( uint8_t *s, uint16_t size )
{
	/*
	 * Recibe un puntero a una estructura y un tamaño.
	 * Recorre la estructura en forma lineal y calcula el checksum
	 */

uint8_t *p = NULL;
uint8_t cks = 0;
uint16_t i = 0;

	cks = 0;
	p = s;
	for ( i = 0; i < size ; i++) {
		 cks = (cks + (int)(p[i])) % 256;
	}

	return(cks);
}
//------------------------------------------------------------------------------
float u_read_bat3v3(bool debug)
{

uint16_t adc = 0;
float bat3v3 = 0.0;

#ifdef HW_AVRDA
    /*
     Leo el ADC con 64 muestras
     */

    SET_EN_SENS3V3();
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    SYSTEM_ENTER_CRITICAL();
    
    adc = ADC_read_sens3v3();
    bat3v3 += 1.0 * adc;

    SYSTEM_EXIT_CRITICAL();
    bat3v3 *= BAT3V3_FACTOR;
            
    CLEAR_EN_SENS3V3();
    
#endif
 
#ifdef HW_XMEGA
    // El model XMEGA no mide los 3V3. !!
    bat3v3 = -1.0;
 
#endif

    
    if(debug) {
        xprintf_P(PSTR("BAT3v3: adc=%d, bat3v3=%0.2f\r\n"), adc, bat3v3);
    }    
    return (bat3v3);
}
//------------------------------------------------------------------------------
float u_read_bat12v(bool debug)
{
uint16_t adc = 0;
float bat12v = 0.0;

#ifdef HW_AVRDA
    /*
     Como acumulo en el ADC 8 samples, el resultado debo dividirlo /8
     */

    SET_EN_SENS12V();
    vTaskDelay( 1000 / portTICK_PERIOD_MS );
    SYSTEM_ENTER_CRITICAL();
    
    adc = ADC_read_sens12v();
    bat12v += 1.0 * adc;
    
    SYSTEM_EXIT_CRITICAL();
    // Convierto a voltaje
    bat12v *= BAT12V_FACTOR;
    
    CLEAR_EN_SENS12V();
    
#endif
 
#ifdef HW_XMEGA
    
    float mag;
    uint16_t raw;

    ainputs_read_channel ( 99, &mag, &raw );
    bat12v = mag;
#endif
    
    if(debug) {
        xprintf_P(PSTR("BAT12v: adc=%d, bat12v=%0.2f\r\n"), adc, bat12v);
    }
    return (bat12v);
}
//------------------------------------------------------------------------------
uint32_t u_get_sleep_time(bool debug)
{
    /*
     * De acuerdo al pwrmodo determina cuanto debe estar 
     * apagado en segundos.
     * 
     */
    
RtcTimeType_t rtc;
uint16_t now;
uint16_t pwr_on;
uint16_t pwr_off;
uint32_t waiting_secs;

    switch(base_conf.pwr_modo) {
        case PWR_CONTINUO:
            if (debug) {
                xprintf_P(PSTR("SLEEP_TIME: pwr_continuo\r\n"));
            }
            waiting_secs = 0;
            break;
        case PWR_DISCRETO:
            if (debug) {
                xprintf_P(PSTR("SLEEP_TIME: pwr_discreto\r\n"));
            }
            waiting_secs = base_conf.timerdial; 
            break;
        case PWR_MIXTO:
            // Hay que ver en que intervalo del modo mixto estoy
            RTC_read_dtime(&rtc);
            now = rtc.hour * 100 + rtc.min;
            pwr_on = base_conf.pwr_hhmm_on;
            pwr_off = base_conf.pwr_hhmm_off;
            if (debug) {
                xprintf_P(PSTR("SLEEP_TIME A: now=%d, pwr_on=%d, pwr_off=%d\r\n"), now, pwr_on,pwr_off);
            }
            now = u_hhmm_to_mins(now);
            pwr_on = u_hhmm_to_mins( pwr_on );
            pwr_off = u_hhmm_to_mins( pwr_off );
            if (debug) {
                xprintf_P(PSTR("SLEEP_TIME B: now=%d, pwr_on=%d, pwr_off=%d\r\n"), now, pwr_on,pwr_off);
            }
                    
            if ( pwr_on < pwr_off) {
                // Caso A:
                // |----apagado-----|----continuo----|---apagado---|
                // 0     (A)      pwr_on   (B)    pwr_off  (C)   2400
                //
                if ( now < pwr_on ) {
                    // Estoy en A:mixto->apagado
                    if (debug) {
                        xprintf_P(PSTR("SLEEP_TIME: pwr_mixto_A\r\n"));
                    }                    
                    waiting_secs = (pwr_on - now)*60;
                } else if ( now < pwr_off) {
                    // Estoy en B:mixto->continuo
                    if (debug) { 
                        xprintf_P(PSTR("SLEEP_TIME: pwr_mixto_B\r\n"));
                    }
                    waiting_secs = 0;
                } else {
                    // Estoy en C:mixto->apagado
                    if (debug) {
                        xprintf_P(PSTR("SLEEP_TIME: pwr_mixto_C\r\n"));
                    }
                    waiting_secs = (1440 - now + pwr_on)*60;
                }
            
            } else {
                // Caso B:
                // |----continuo-----|----apagado----|---continuo---|
                // 0     (D)      pwr_off   (E)    pwr_on  (F)    2400
                //
                if ( now < pwr_off) {
                    // Estoy en D: mixto->continuo
                    if (debug) {
                        xprintf_P(PSTR("SLEEP_TIME: pwr_mixto_D\r\n"));
                    }
                    waiting_secs = 0;
                } else if (now < pwr_on) {
                    // Estoy en E: mixto->apagado
                    if (debug) {
                        xprintf_P(PSTR("SLEEP_TIME: pwr_mixto_E\r\n"));
                    }
                    waiting_secs = (pwr_on - now)*60;
                } else {
                    // Estoy en F:mixto->prendido
                    if (debug) {
                        xprintf_P(PSTR("SLEEP_TIME: pwr_mixto_F\r\n"));
                    }
                    waiting_secs = 0;
                }
            } 
            break;
        default:
            // En default, dormimos 30 minutos
            waiting_secs = 1800;
            break;
    }
    
    if (debug) {
        xprintf_P(PSTR("SLEEP_TIME: waiting_ticks=%lu secs\r\n"), waiting_secs);
    }
    return(waiting_secs);
}
//------------------------------------------------------------------------------
uint16_t u_hhmm_to_mins(uint16_t hhmm)
{
    /*
     * Convierte una hora (hhmm) en minutos (0..1440)
     */
    
uint16_t hh,mm, mins;

    hh = (uint16_t) (hhmm / 100);
    mm = hhmm - hh * 100;
    mins = hh*60 + mm;
    return (mins);
    
}
//------------------------------------------------------------------------------

void SYSTEM_INIT(void)
{
    sem_SYSVars = xSemaphoreCreateMutexStatic( &SYSVARS_xMutexBuffer );
    
}
//------------------------------------------------------------------------------
void SYSTEM_ENTER_CRITICAL(void)
{
    while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 5 ) != pdTRUE )
  		vTaskDelay( ( TickType_t)( 10 ) );   
}
//------------------------------------------------------------------------------
void SYSTEM_EXIT_CRITICAL(void)
{
    xSemaphoreGive( sem_SYSVars );
}
//------------------------------------------------------------------------------

