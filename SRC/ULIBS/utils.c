
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

#ifdef MODEL_M3
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
    if(debug) {
        xprintf_P(PSTR("BAT3v3: adc=%d, bat3v3=%0.2f\r\n"), adc, bat3v3);
    }
    
#endif
 
#ifdef MODEL_M1
    // El model XMEGA no mide los 3V3. !!
    bat3v3 = -1.0;
#endif
    
    return (bat3v3);
}
//------------------------------------------------------------------------------
float u_read_bat12v(bool debug)
{
uint16_t adc = 0;
float bat12v = 0.0;

#ifdef MODEL_M3
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
    if(debug) {
        xprintf_P(PSTR("BAT12v: adc=%d, bat12v=%0.2f\r\n"), adc, bat12v);
    }
    
#endif
 
#ifdef MODEL_M1
    
    float mag;
    uint16_t raw;

    ainputs_read_channel ( 99, &mag, &raw );
    bat12v = mag;
#endif
    
    return (bat12v);
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

