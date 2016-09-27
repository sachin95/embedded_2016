#include "callback.h"
#include "debug.h"

void init_callback(){
    iteration_50ms = 0;
}  

void vTimerCallback(TimerHandle_t xtimer){
    iteration_50ms++;
    
    if ((iteration_50ms % 7) == 0){
        flash_LEDS();
    }
    appSendTimerValToMsgQ(iteration_50ms);
}

void vSensorTimerCallback(TimerHandle_t ytimer){
    PLIB_ADC_SampleAutoStartEnable(ADC_ID_1);
    
}