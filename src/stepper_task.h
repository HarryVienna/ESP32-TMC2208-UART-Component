#pragma once

#include "stepper_driver_tmc2208.h"

typedef struct stepper_conf_s {
    char* name;
    uint32_t speed;  
    stepper_driver_tmc2208_conf_t stepper_driver_conf;
} stepper_conf_t;

void stepper_task(void *pvParameter);

