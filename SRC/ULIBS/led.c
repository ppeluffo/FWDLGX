

#include "led.h"

#ifdef HW_AVRDA
    #define LED_PORT	PORTC
    #define LED_PIN_bm	PIN4_bm
    #define LED_PIN_bp  PIN4_bp
#endif

#ifdef HW_XMEGA
    #define LED_PORT	PORTF
    #define LED_PIN_bm	PIN7_bm
    #define LED_PIN_bp  PIN7_bp
#endif
// -----------------------------------------------------------------------------
void LED_init(void)
{
	// Configura el pin del led como output
	LED_PORT.DIR |= LED_PIN_bm;	
	// Apago
    LED_PORT.OUT &= ~LED_PIN_bm;
}
// -----------------------------------------------------------------------------
void led_prender(void)
{
    LED_PORT.OUT |= LED_PIN_bm;
}
// -----------------------------------------------------------------------------
void led_apagar(void)
{
    LED_PORT.OUT &= ~LED_PIN_bm;
}
// -----------------------------------------------------------------------------
void led_toggle(void)
{
    LED_PORT.OUT ^= 1UL << LED_PIN_bp;
}
// -----------------------------------------------------------------------------
void led_flash(void)
{
	// Prende el led 50ms y lo apaga
 	LED_PORT.OUT |= LED_PIN_bm;
	vTaskDelay( (TickType_t)( 10 / portTICK_PERIOD_MS ) );
	LED_PORT.OUT &= ~LED_PIN_bm;
             
}
// -----------------------------------------------------------------------------

