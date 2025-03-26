/* 
 * File:   lte.h
 * Author: pablo
 *
 * Created on 25 de enero de 2024, 10:23 AM
 */

#ifndef LTE_H
#define	LTE_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <avr/io.h>
#include "stdbool.h"
#include "linearBuffer.h"
#include "frtos-io.h"
#include "xprintf.h"
#include "string.h"
#include "xgetc.h"
#include "ctype.h"
#include "utils.h"
//--------------------------------------------------------------------------
// PUSR WH-LTE-7S1
    
#ifdef HW_AVRDA
    // LTE_PWR
    #define LTE_PWR_PORT         PORTB
    #define LTE_PWR              4
    #define LTE_PWR_PIN_bm       PIN4_bm
    #define LTE_PWR_PIN_bp       PIN4_bp
    #define CLEAR_LTE_PWR()     ( LTE_PWR_PORT.OUT |= LTE_PWR_PIN_bm )
    #define SET_LTE_PWR()       ( LTE_PWR_PORT.OUT &= ~LTE_PWR_PIN_bm )
    #define CONFIG_LTE_PWR()     LTE_PWR_PORT.DIR |= LTE_PWR_PIN_bm;

    // LTE_EN_VCAP
    #define LTE_EN_VCAP_PORT         PORTB
    #define LTE_EN_VCAP              6
    #define LTE_EN_VCAP_PIN_bm       PIN6_bm
    #define LTE_EN_VCAP_PIN_bp       PIN6_bp
    #define SET_LTE_EN_VCAP()        ( LTE_EN_VCAP_PORT.OUT |= LTE_EN_VCAP_PIN_bm )
    #define CLEAR_LTE_EN_VCAP()      ( LTE_EN_VCAP_PORT.OUT &= ~LTE_EN_VCAP_PIN_bm )
    #define CONFIG_LTE_EN_VCAP()     LTE_EN_VCAP_PORT.DIR |= LTE_EN_VCAP_PIN_bm;

    //-------------------------------------------------------------------------
    // LTE_EN_DCIN
    #define LTE_EN_DCIN_PORT         PORTB
    #define LTE_EN_DCIN              5
    #define LTE_EN_DCIN_PIN_bm       PIN5_bm
    #define LTE_EN_DCIN_PIN_bp       PIN5_bp
    #define SET_LTE_EN_DCIN()        ( LTE_EN_DCIN_PORT.OUT |= LTE_EN_DCIN_PIN_bm )
    #define CLEAR_LTE_EN_DCIN()      ( LTE_EN_DCIN_PORT.OUT &= ~LTE_EN_DCIN_PIN_bm )
    #define CONFIG_LTE_EN_DCIN()     LTE_EN_DCIN_PORT.DIR |= LTE_EN_DCIN_PIN_bm;

    // LTE_RELOAD
    #define LTE_RELOAD_PORT         PORTB
    #define LTE_RELOAD              7
    #define LTE_RELOAD_PIN_bm       PIN7_bm
    #define LTE_RELOAD_PIN_bp       PIN7_bp
    #define CLEAR_LTE_RELOAD()      ( LTE_RELOAD_PORT.OUT |= LTE_RELOAD_PIN_bm )
    #define SET_LTE_RELOAD()        ( LTE_RELOAD_PORT.OUT &= ~LTE_RELOAD_PIN_bm )
    #define CONFIG_LTE_RELOAD()     LTE_RELOAD_PORT.DIR |= LTE_RELOAD_PIN_bm;
 
    // LTE_RESET
    #define LTE_RESET_PORT         PORTC
    #define LTE_RESET              0
    #define LTE_RESET_PIN_bm       PIN0_bm
    #define LTE_RESET_PIN_bp       PIN0_bp
    #define CLEAR_LTE_RESET()      ( LTE_RESET_PORT.OUT |= LTE_RESET_PIN_bm )
    #define SET_LTE_RESET()        ( LTE_RESET_PORT.OUT &= ~LTE_RESET_PIN_bm )
    #define CONFIG_LTE_RESET()     LTE_RESET_PORT.DIR |= LTE_RESET_PIN_bm;

#endif
    
    
#ifdef HW_XMEGA
    // LTE_PWR (GPRS_SW)
    #define LTE_PWR_PORT         PORTD
    #define LTE_PWR              5
    #define LTE_PWR_PIN_bm       PIN5_bm
    #define LTE_PWR_PIN_bp       PIN5_bp
    #define CLEAR_LTE_PWR()     ( LTE_PWR_PORT.OUT |= LTE_PWR_PIN_bm )
    #define SET_LTE_PWR()       ( LTE_PWR_PORT.OUT &= ~LTE_PWR_PIN_bm )
    #define CONFIG_LTE_PWR()     LTE_PWR_PORT.DIR |= LTE_PWR_PIN_bm;

    // LTE_EN_VCAP (GPRS_PWR)
    #define LTE_EN_VCAP_PORT         PORTD
    #define LTE_EN_VCAP              4
    #define LTE_EN_VCAP_PIN_bm       PIN4_bm
    #define LTE_EN_VCAP_PIN_bp       PIN4_bp
    #define SET_LTE_EN_VCAP()        ( LTE_EN_VCAP_PORT.OUT |= LTE_EN_VCAP_PIN_bm )
    #define CLEAR_LTE_EN_VCAP()      ( LTE_EN_VCAP_PORT.OUT &= ~LTE_EN_VCAP_PIN_bm )
    #define CONFIG_LTE_EN_VCAP()     LTE_EN_VCAP_PORT.DIR |= LTE_EN_VCAP_PIN_bm;

    //-------------------------------------------------------------------------
    // LTE_EN_DCIN
    #define LTE_EN_DCIN_PORT         PORTA
    #define LTE_EN_DCIN              4
    #define LTE_EN_DCIN_PIN_bm       PIN4_bm
    #define LTE_EN_DCIN_PIN_bp       PIN4_bp
    #define SET_LTE_EN_DCIN()        ( LTE_EN_DCIN_PORT.OUT |= LTE_EN_DCIN_PIN_bm )
    #define CLEAR_LTE_EN_DCIN()      ( LTE_EN_DCIN_PORT.OUT &= ~LTE_EN_DCIN_PIN_bm )
    #define CONFIG_LTE_EN_DCIN()     LTE_EN_DCIN_PORT.DIR |= LTE_EN_DCIN_PIN_bm;

    // LTE_RELOAD
    #define LTE_RELOAD_PORT         PORTA
    #define LTE_RELOAD              4
    #define LTE_RELOAD_PIN_bm       PIN4_bm
    #define LTE_RELOAD_PIN_bp       PIN4_bp
    #define CLEAR_LTE_RELOAD()      ( LTE_RELOAD_PORT.OUT |= LTE_RELOAD_PIN_bm )
    #define SET_LTE_RELOAD()        ( LTE_RELOAD_PORT.OUT &= ~LTE_RELOAD_PIN_bm )
    #define CONFIG_LTE_RELOAD()     LTE_RELOAD_PORT.DIR |= LTE_RELOAD_PIN_bm;
 
    // LTE_RESET
    #define LTE_RESET_PORT         PORTA
    #define LTE_RESET              4
    #define LTE_RESET_PIN_bm       PIN4_bm
    #define LTE_RESET_PIN_bp       PIN4_bp
    #define CLEAR_LTE_RESET()      ( LTE_RESET_PORT.OUT |= LTE_RESET_PIN_bm )
    #define SET_LTE_RESET()        ( LTE_RESET_PORT.OUT &= ~LTE_RESET_PIN_bm )
    #define CONFIG_LTE_RESET()     LTE_RESET_PORT.DIR |= LTE_RESET_PIN_bm;

#endif

#define MODEM_RX_BUFFER_SIZE 512
char modem_rx_buffer[MODEM_RX_BUFFER_SIZE];
lBuffer_s modem_rx_lbuffer;

#define WAN_TX_BUFFER_SIZE 255
char wan_tx_buffer[WAN_TX_BUFFER_SIZE];

typedef enum { MODEM_PWR_OFF=0, MODEM_PWR_ON} t_modem_pwr_status;
    
#define APN_LENGTH  24
#define SERVER_IP_LENGTH 24
#define SERVER_PORT_LENGTH 12

typedef struct {
    char apn[APN_LENGTH];
	char server_ip[SERVER_IP_LENGTH];
	char server_port[SERVER_PORT_LENGTH];
    uint8_t checksum;
} modem_conf_t;

modem_conf_t modem_conf;

bool modem_awake;

void MODEM_init(void);
void MODEM_prender(void);
void MODEM_apagar(void);
int MODEM_txmit( char *tx_buffer );
char *MODEM_get_buffer_ptr(void);
void MODEM_flush_rx_buffer(void);
bool MODEM_enter_mode_at(bool verbose);
void MODEM_exit_mode_at(bool verbose);
void MODEM_AWAKE(void);
void MODEM_SLEEP(void);

void modem_init_outofrtos( TaskHandle_t *xHandle );
char *modem_atcmd(char *s_cmd, bool verbose);
void modem_atcmd_save(bool verbose);
void modem_atcmd_queryall(void);
void modem_atcmd_read_iccid(bool verbose);
char *modem_atcmd_get_iccid(void);
void modem_atcmd_read_imei(bool verbose);
char *modem_atcmd_get_imei(void);
void modem_atcmd_read_csq(bool verbose);
uint8_t modem_atcmd_get_csq(void);
char *modem_atcmd_get_apn(void);
void modem_atcmd_read_apn(bool verbose);
void modem_atcmd_set_apn( char *apn, bool verbose);
void modem_atcmd_set_server( char *ip, char *port, bool verbose);
char *modem_atcmd_get_server_ip(void);
char *modem_atcmd_get_server_port(void);
void modem_atcmd_read_server(bool verbose);
void modem_atcmd_set_apiurl( char *apiurl, bool verbose);
void modem_atcmd_set_ftime( char *time_ms, bool verbose);
void modem_atcmd_read_localip(bool verbose);
char *modem_atcmd_get_localip(void);

bool modem_check_and_reconfig(bool verbose);

void modem_print_configuration( void );
bool modem_config( char *s_arg, char *s_value );
void modem_config_defaults( char *s_arg );

bool modem_test(char **argv);
int16_t modem_save_config_in_NVM(uint16_t nvm_ptr);
int16_t modem_load_config_from_NVM( uint16_t nvm_address);

#ifdef	__cplusplus
}
#endif

#endif	/* LTE_H */

