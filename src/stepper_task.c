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

  ESP_LOGI(TAG, "%s: Task started", task_name); 

 
  ESP_LOGD(TAG, "%s: Init stepper driver", config->name); 
  // Create new driver
  stepper_driver_t *motor = stepper_driver_new_tmc2208(&config->stepper_driver_conf);
  // Init driver
  stepper_driver_init(motor);
  // Clear status
  stepper_driver_clear_gstat(motor);

  vTaskDelay(pdMS_TO_TICKS(1000));

  ESP_LOGD(TAG, "%s: Write settings to stepper driver", config->name);
  stepper_driver_set_stealthchop_thrs(motor, 0);
  stepper_driver_set_current(motor, 150, 100);
  stepper_driver_set_microsteps_per_step(motor, MICROSTEPS_1);

  stepper_driver_enable_pwm_autograd(motor);
  stepper_driver_enable_pwm_autoscale(motor);
  stepper_driver_set_pwm_reg(motor, 1);
  stepper_driver_set_pwm_freq(motor, FREQ_2_512);
  stepper_driver_enable(motor);

  vTaskDelay(pdMS_TO_TICKS(10000));

  stepper_driver_steps(motor, 30000, 200);


  stepper_driver_set_pwm_grad(motor, 255);
  stepper_driver_set_pwm_offset(motor, 255);
  stepper_driver_disable_pwm_autograd(motor);
  stepper_driver_disable_pwm_autoscale(motor);
  stepper_driver_set_current(motor, 200, 100);

  ESP_LOGD(TAG, "%s: Move steppers", config->name);
  stepper_driver_direction(motor, 1);
  //stepper_driver_move(motor, 2000);
  while(1) {
    stepper_driver_steps(motor, 1000, config->speed);
    //stepper_driver_dump_register_drv_status(motor);
    //stepper_driver_dump_register_tstep(motor);
    //stepper_driver_dump_register_ioin(motor);
    //stepper_driver_dump_register_gconf(motor);
    //stepper_driver_dump_register_otp_read(motor);
    //stepper_driver_dump_register_chopconf(motor);
    //stepper_driver_dump_register_pwmconf(motor);
    //stepper_driver_dump_register_factory_conf(motor);
    //stepper_driver_dump_register_mscuract(motor);
    stepper_driver_dump_register_pwm_scale(motor);
    stepper_driver_dump_register_pwm_auto(motor);
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  
}