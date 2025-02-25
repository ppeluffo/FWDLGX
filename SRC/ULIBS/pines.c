
#include "pines.h"

//------------------------------------------------------------------------------ 
void CONFIG_SCL(void)
{
#ifdef MODEL_M1
    SCL_PORT.DIR |= SCL_PIN_bm;
#endif
}
//------------------------------------------------------------------------------ 
void SET_SCL(void)
{
#ifdef MODEL_M1
    ( SCL_PORT.OUT |= SCL_PIN_bm );
#endif
}
//------------------------------------------------------------------------------ 
void CLEAR_SCL(void)
{
#ifdef MODEL_M1
    ( SCL_PORT.OUT &= ~SCL_PIN_bm );
#endif
}
//------------------------------------------------------------------------------  
void CONFIG_PWR_SENSORS(void)
{ 
#ifdef MODEL_M3
    ( PWR_SENSORS_PORT.DIR |= PWR_SENSORS_PIN_bm);
#endif
}
//------------------------------------------------------------------------------
void SET_PWR_SENSORS(void)
{
#ifdef MODEL_M3
    ( PWR_SENSORS_PORT.OUT |= PWR_SENSORS_PIN_bm );
#endif
}
//------------------------------------------------------------------------------   
void CLEAR_PWR_SENSORS(void)
{
#ifdef MODEL_M3
    ( PWR_SENSORS_PORT.OUT &= ~PWR_SENSORS_PIN_bm );
#endif
}
//------------------------------------------------------------------------------
void SET_EN_SENS3V3(void)
{
#ifdef MODEL_M3
    ( EN_SENS3V3_PORT.OUT |= EN_SENS3V3_PIN_bm );
#endif
}
//------------------------------------------------------------------------------
void CLEAR_EN_SENS3V3(void)
{
#ifdef MODEL_M3
    ( EN_SENS3V3_PORT.OUT &= ~EN_SENS3V3_PIN_bm );
#endif
}
//------------------------------------------------------------------------------
void CONFIG_EN_SENS3V3(void)
{
#ifdef MODEL_M3
    ( EN_SENS3V3_PORT.DIR |= EN_SENS3V3_PIN_bm );
#endif
}
//------------------------------------------------------------------------------
void SET_EN_SENS12V(void)
{
#ifdef MODEL_M3
    ( EN_SENS12V_PORT.OUT |= EN_SENS12V_PIN_bm );
#endif
}
//------------------------------------------------------------------------------            
void CLEAR_EN_SENS12V(void)
{
#ifdef MODEL_M3
    ( EN_SENS12V_PORT.OUT &= ~EN_SENS12V_PIN_bm );
#endif
}
//------------------------------------------------------------------------------            
void CONFIG_EN_SENS12V(void)
{
#ifdef MODEL_M3
    ( EN_SENS12V_PORT.DIR |= EN_SENS12V_PIN_bm );
#endif
}
// -----------------------------------------------------------------------------
