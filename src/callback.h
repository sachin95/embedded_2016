#include "system_config.h"
#include "system_definitions.h"
#include "timers.h"
#include "app1_public.h"

unsigned int iteration_50ms;

void init_callback();
void vTimerCallback(TimerHandle_t x_timer);
void vSensorTimerCallback(TimerHandle_t y_timer);