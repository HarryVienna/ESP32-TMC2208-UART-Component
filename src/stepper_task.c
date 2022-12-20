#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "stepper_task.h"

#include "esp_attr.h"
#include "esp_log.h"


static const char* TAG = "stepper_task";



/**
 * @brief Main task 
 *
 */
void stepper_task(void *pvParameter){

  const stepper_conf_t *config = (stepper_conf_t *) pvParameter;
  char* task_name = config->name;

 
  ESP_LOGD(TAG, "%s: Init stepper driver", task_name); 

  // Create new driver
  stepper_driver_t *motor = stepper_driver_new_tmc2208(&config->stepper_driver_conf);
  // Init driver
  stepper_driver_init(motor);
  // Clear status
  stepper_driver_clear_gstat(motor);


  vTaskDelay(pdMS_TO_TICKS(1000));


  ESP_LOGD(TAG, "%s: Write settings to stepper driver", task_name);

  stepper_driver_set_stealthchop_thrs(motor, 0);
  stepper_driver_set_current(motor, 200, 50);
  stepper_driver_set_microsteps_per_step(motor, MICROSTEPS_1);

  stepper_driver_enable_pwm_autograd(motor);
  stepper_driver_enable_pwm_autoscale(motor);
  stepper_driver_set_pwm_reg(motor, 1);
  stepper_driver_set_pwm_freq(motor, FREQ_2_512);
  stepper_driver_enable(motor);

  vTaskDelay(pdMS_TO_TICKS(1000));

  ESP_LOGD(TAG, "%s: Move stepper per steps", task_name);

  for (int i = 0; i < 10; i++) {
    stepper_driver_direction(motor, 0);
    stepper_driver_steps(motor, 1000, config->speed);

    vTaskDelay(pdMS_TO_TICKS(100));

    stepper_driver_direction(motor, 1);
    stepper_driver_steps(motor, 1000, config->speed * 2);

    vTaskDelay(pdMS_TO_TICKS(100));
  }

  ESP_LOGD(TAG, "%s: Move stepper by register", task_name);
  stepper_driver_set_vactual(motor, 10000);

  vTaskDelay(pdMS_TO_TICKS(2000));

  stepper_driver_set_vactual(motor, -10000);

  vTaskDelay(pdMS_TO_TICKS(2000));

  stepper_driver_set_vactual(motor, 0);

  ESP_LOGD(TAG, "%s: Disbale motor", task_name);
  stepper_driver_disable(motor);

  vTaskDelete( NULL );

/*
  stepper_driver_set_pwm_grad(motor, 255);
  stepper_driver_set_pwm_offset(motor, 255);
  stepper_driver_disable_pwm_autograd(motor);
  stepper_driver_disable_pwm_autoscale(motor);
  stepper_driver_set_current(motor, 200, 100);
*/

  
}