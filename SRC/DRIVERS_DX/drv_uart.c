/*
 * drv_uart_spx.c
 *
 *  Created on: 11 jul. 2018
 *      Author: pablo
 * 
 * PLACA BASE: sensor_cloro.
 * 
 * El driver de las uart permite crear las uarts y 2 estructuras tipo 
 * ringbuffer (chars) para c/u.
 * Estos son las interfaces a la capa de FRTOS-IO.
 * Para transmitir se escribe en el ringBuffer de TX y para leer lo recibido
 * se lee del ringBuffer de RX.
 * La transmision / recepcion se hace por interrupcion. Estas ISR son 
 * provistas por el driver
 * Cada placa tiene diferente asignacion de puertos por lo tanto hay
 * que modificar el driver a c/placa.
 * 
 * 
 */

#include "drv_uart.h"

#ifndef F_CPU
    #define F_CPU 24000000
#endif

#define USART_SET_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5);

//------------------------------------------------------------------------------
// USART0:
//------------------------------------------------------------------------------
void drv_uart0_init(uint32_t baudrate )
{
    
    PORTA.DIR &= ~PIN1_bm;
    PORTA.DIR |= PIN0_bm;
    USART0.BAUD = (uint16_t)USART_SET_BAUD_RATE(baudrate);     
    USART0.CTRLC = USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_1BIT_gc;
    
    // Habilito el TX y el RX
    USART0.CTRLB |= USART_TXEN_bm;
    USART0.CTRLB |= USART_RXEN_bm;
    
    // Habilito las interrupciones por RX
    USART0.CTRLA |= USART_RXCIE_bm;
    
    // Las transmisiones son por poleo no INT.
    
    // RingBuffers
    rBchar_CreateStatic ( &TXRB_uart0, &uart0_txBuffer[0], UART0_TXSIZE  );
    rBchar_CreateStatic ( &RXRB_uart0, &uart0_rxBuffer[0], UART0_RXSIZE  );
}
//------------------------------------------------------------------------------
ISR(USART0_RXC_vect)
{
    // Driver ISR: Cuando se genera la interrupcion por RXIE, lee el dato
    // y lo pone en la cola (ringBuffer.)
char cChar = ' ';

	cChar = USART0.RXDATAL;
 	rBchar_PokeFromISR( &RXRB_uart0, cChar );
}
//------------------------------------------------------------------------------
// USART1:
//------------------------------------------------------------------------------
void drv_uart1_init(uint32_t baudrate )
{
    
    PORTC.DIR &= ~PIN1_bm;
    PORTC.DIR |= PIN0_bm;
    USART1.BAUD = (uint16_t)USART_SET_BAUD_RATE(baudrate);     
    USART1.CTRLC = USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_1BIT_gc;
    
    // Habilito el TX y el RX
    USART1.CTRLB |= USART_TXEN_bm;
    USART1.CTRLB |= USART_RXEN_bm;
    
    // Habilito las interrupciones por RX
    USART1.CTRLA |= USART_RXCIE_bm;
    
    // Las transmisiones son por poleo no INT.
    
    // RingBuffers
    rBchar_CreateStatic ( &TXRB_uart1, &uart1_txBuffer[0], UART1_TXSIZE  );
    rBchar_CreateStatic ( &RXRB_uart1, &uart1_rxBuffer[0], UART1_RXSIZE  );
}
//------------------------------------------------------------------------------
ISR(USART1_RXC_vect)
{
    // Driver ISR: Cuando se genera la interrupcion por RXIE, lee el dato
    // y lo pone en la cola (ringBuffer.)
char cChar = ' ';

	cChar = USART1.RXDATAL;
 	rBchar_PokeFromISR( &RXRB_uart1, cChar );
}
//------------------------------------------------------------------------------
// USART2:
//------------------------------------------------------------------------------
void drv_uart2_init(uint32_t baudrate )
{
    
    PORTF.DIR &= ~PIN1_bm;
    PORTF.DIR |= PIN0_bm;
    USART2.BAUD = (uint16_t)USART_SET_BAUD_RATE(baudrate);     
    USART2.CTRLC = USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_1BIT_gc;
    
    // Habilito el TX y el RX
    USART2.CTRLB |= USART_TXEN_bm;
    USART2.CTRLB |= USART_RXEN_bm;
    
    // Habilito las interrupciones por RX
    USART2.CTRLA |= USART_RXCIE_bm;
    
    // Las transmisiones son por poleo no INT.
    
    rBchar_CreateStatic ( &TXRB_uart2, &uart2_txBuffer[0], UART2_TXSIZE  );
    rBchar_CreateStatic ( &RXRB_uart2, &uart2_rxBuffer[0], UART2_RXSIZE  );

}
//------------------------------------------------------------------------------
ISR(USART2_RXC_vect)
{
    // Driver ISR: Cuando se genera la interrupcion por RXIE, lee el dato
    // y lo pone en la cola (ringBuffer.)
char cChar = ' ';

	cChar = USART2.RXDATAL;
 	rBchar_PokeFromISR( &RXRB_uart2, cChar );
}
//------------------------------------------------------------------------------
// USART3:
//------------------------------------------------------------------------------
void drv_uart3_init(uint32_t baudrate )
{
    
    PORTB.DIR &= ~PIN1_bm;
    PORTB.DIR |= PIN0_bm;
    USART3.BAUD = (uint16_t)USART_SET_BAUD_RATE(baudrate);     
    USART3.CTRLC = USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_1BIT_gc;
    
    // Habilito el TX y el RX
    USART3.CTRLB |= USART_TXEN_bm;
    USART3.CTRLB |= USART_RXEN_bm;
    
    // Habilito las interrupciones por RX
    USART3.CTRLA |= USART_RXCIE_bm;
    
    // Las transmisiones son por poleo no INT.
    
    // RingBuffers
    rBchar_CreateStatic ( &TXRB_uart3, &uart3_txBuffer[0], UART3_TXSIZE  );
    rBchar_CreateStatic ( &RXRB_uart3, &uart3_rxBuffer[0], UART3_RXSIZE  );
}
//------------------------------------------------------------------------------
ISR(USART3_RXC_vect)
{
    // Driver ISR: Cuando se genera la interrupcion por RXIE, lee el dato
    // y lo pone en la cola (ringBuffer.)
char cChar = ' ';

	cChar = USART3.RXDATAL;
 	rBchar_PokeFromISR( &RXRB_uart3, cChar );
}
//------------------------------------------------------------------------------
// USART4:
//------------------------------------------------------------------------------
void drv_uart4_init(uint32_t baudrate )
{
    
    PORTE.DIR &= ~PIN1_bm;
    PORTE.DIR |= PIN0_bm;
    USART4.BAUD = (uint16_t)USART_SET_BAUD_RATE(baudrate);     
    USART4.CTRLC = USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_1BIT_gc;
    
    // Habilito el TX y el RX
    USART4.CTRLB |= USART_TXEN_bm;
    USART4.CTRLB |= USART_RXEN_bm;
    
    // Habilito las interrupciones por RX
    USART4.CTRLA |= USART_RXCIE_bm;
    
    // Las transmisiones son por poleo no INT.
    
    // RingBuffers
    rBchar_CreateStatic ( &TXRB_uart4, &uart4_txBuffer[0], UART4_TXSIZE  );
    rBchar_CreateStatic ( &RXRB_uart4, &uart4_rxBuffer[0], UART4_RXSIZE  );
}
//------------------------------------------------------------------------------
ISR(USART4_RXC_vect)
{
    // Driver ISR: Cuando se genera la interrupcion por RXIE, lee el dato
    // y lo pone en la cola (ringBuffer.)
char cChar = ' ';

	cChar = USART4.RXDATAL;
 	rBchar_PokeFromISR( &RXRB_uart4, cChar );
}
//------------------------------------------------------------------------------
