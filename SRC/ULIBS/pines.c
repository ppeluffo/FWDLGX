
#include "pines.h"

//------------------------------------------------------------------------------
void SET_EN_PWR_CPRES(void)
{
#ifdef HW_AVRDA
    EN_PWR_CPRES_PORT.OUT |= EN_PWR_CPRES_PIN_bm;
#endif 
}
//-----------------------------------------------------------------------------
void CLEAR_EN_PWR_CPRES(void)
{
#ifdef HW_AVRDA
    EN_PWR_CPRES_PORT.OUT &= ~EN_PWR_CPRES_PIN_bm;
#endif 
}
//-----------------------------------------------------------------------------
void CONFIG_EN_PWR_CPRES(void)
{
#ifdef HW_AVRDA
    EN_PWR_CPRES_PORT.DIR |= EN_PWR_CPRES_PIN_bm;
#endif 
}
//-----------------------------------------------------------------------------
void SET_EN_PWR_QMBUS(void)
{
#ifdef HW_AVRDA
    EN_PWR_QMBUS_PORT.OUT |= EN_PWR_QMBUS_PIN_bm;
#endif 
}
//-----------------------------------------------------------------------------
void CLEAR_EN_PWR_QMBUS(void)
{
#ifdef HW_AVRDA
    EN_PWR_QMBUS_PORT.OUT &= ~EN_PWR_QMBUS_PIN_bm;
#endif 
}
//-----------------------------------------------------------------------------
void CONFIG_EN_PWR_QMBUS(void)
{
#ifdef HW_AVRDA
    EN_PWR_QMBUS_PORT.DIR |= EN_PWR_QMBUS_PIN_bm;
#endif 
}
//------------------------------------------------------------------------------ 
void SET_RTS_RS485(void)
{
#ifdef HW_AVRDA
    RTS_RS485_PORT.OUT |= RTS_RS485_PIN_bm;
#endif 
}//-----------------------------------------------------------------------------
void CLEAR_RTS_RS485(void)
{
#ifdef HW_AVRDA
    RTS_RS485_PORT.OUT &= ~RTS_RS485_PIN_bm;
#endif
}
//------------------------------------------------------------------------------ 
void CONFIG_RTS_485(void)
{
#ifdef HW_AVRDA
    RTS_RS485_PORT.DIR |= RTS_RS485_PIN_bm;
#endif   
}
//------------------------------------------------------------------------------ 
void CONFIG_SCL(void)
{
#ifdef HW_XMEGA
    SCL_PORT.DIR |= SCL_PIN_bm;
#endif
}
//------------------------------------------------------------------------------ 
void SET_SCL(void)
{
#ifdef HW_XMEGA
    ( SCL_PORT.OUT |= SCL_PIN_bm );
#endif
}
//------------------------------------------------------------------------------ 
void CLEAR_SCL(void)
{
#ifdef HW_XMEGA
    ( SCL_PORT.OUT &= ~SCL_PIN_bm );
#endif
}
//------------------------------------------------------------------------------  
void CONFIG_PWR_SENSORS(void)
{ 
#ifdef HW_AVRDA
    ( PWR_SENSORS_PORT.DIR |= PWR_SENSORS_PIN_bm);
#endif
}
//------------------------------------------------------------------------------
void SET_PWR_SENSORS(void)
{
#ifdef HW_AVRDA
    ( PWR_SENSORS_PORT.OUT |= PWR_SENSORS_PIN_bm );
#endif
}
//------------------------------------------------------------------------------   
void CLEAR_PWR_SENSORS(void)
{
#ifdef HW_AVRDA
    ( PWR_SENSORS_PORT.OUT &= ~PWR_SENSORS_PIN_bm );
#endif
}
//------------------------------------------------------------------------------
void SET_EN_SENS3V3(void)
{
#ifdef HW_AVRDA
    ( EN_SENS3V3_PORT.OUT |= EN_SENS3V3_PIN_bm );
#endif
}
//------------------------------------------------------------------------------
void CLEAR_EN_SENS3V3(void)
{
#ifdef HW_AVRDA
    ( EN_SENS3V3_PORT.OUT &= ~EN_SENS3V3_PIN_bm );
#endif
}
//------------------------------------------------------------------------------
void CONFIG_EN_SENS3V3(void)
{
#ifdef HW_AVRDA
    ( EN_SENS3V3_PORT.DIR |= EN_SENS3V3_PIN_bm );
#endif
}
//------------------------------------------------------------------------------
void SET_EN_SENS12V(void)
{
#ifdef HW_AVRDA
    ( EN_SENS12V_PORT.OUT |= EN_SENS12V_PIN_bm );
#endif
}
//------------------------------------------------------------------------------            
void CLEAR_EN_SENS12V(void)
{
#ifdef HW_AVRDA
    ( EN_SENS12V_PORT.OUT &= ~EN_SENS12V_PIN_bm );
#endif
}
//------------------------------------------------------------------------------            
void CONFIG_EN_SENS12V(void)
{
#ifdef HW_AVRDA
    ( EN_SENS12V_PORT.DIR |= EN_SENS12V_PIN_bm );
#endif
}
// -----------------------------------------------------------------------------
