
#include "rs485.h"

TaskHandle_t *l_xHandle_tkRS485;

SemaphoreHandle_t sem_RS485;
StaticSemaphore_t RS485_xMutexBuffer;

//------------------------------------------------------------------------------
void rs485_init_outofrtos( TaskHandle_t *xHandle )
{
    sem_RS485 = xSemaphoreCreateMutexStatic( &RS485_xMutexBuffer );
    l_xHandle_tkRS485 = xHandle;
}
// -----------------------------------------------------------------------------
void rs485_ENTER_CRITICAL(void)
{
    while ( xSemaphoreTake( sem_RS485, ( TickType_t ) 5 ) != pdTRUE )
  		vTaskDelay( ( TickType_t)( 10 ) );   
}
//------------------------------------------------------------------------------
void rs485_EXIT_CRITICAL(void)
{
    xSemaphoreGive( sem_RS485 );
}
//------------------------------------------------------------------------------
void rs485_AWAKE(void)
{
    
    //taskENTER_CRITICAL();
    rs485_awake = true;
    //taskEXIT_CRITICAL();
    
    // Despierto a la tarea.
    xTaskNotifyGive( *l_xHandle_tkRS485 );
    
    vTaskDelay( ( TickType_t)( 100 / portTICK_PERIOD_MS ) );
    
}
//------------------------------------------------------------------------------
void rs485_SLEEP(void)
{
    //taskENTER_CRITICAL();
    rs485_awake = false;
    //taskEXIT_CRITICAL();
    
}
//------------------------------------------------------------------------------
