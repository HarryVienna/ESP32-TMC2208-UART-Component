#pragma once

#include "esp_err.h"


typedef enum {
    MICROSTEPS_1 =   0,
    MICROSTEPS_2   = 1,
    MICROSTEPS_4   = 2,
    MICROSTEPS_8   = 3,
    MICROSTEPS_16  = 4,
    MICROSTEPS_32  = 5,
    MICROSTEPS_64  = 6,
    MICROSTEPS_128 = 7,
    MICROSTEPS_256 = 8
} stepper_driver_microsteps_t;


typedef enum {
    FREQ_2_1024 =   0,
    FREQ_2_683   = 1,
    FREQ_2_512   = 2,
    FREQ_2_410   = 3
} stepper_driver_pwm_freq_t;

/**
* @brief Step Motor Type
*
*/
typedef struct stepper_driver_s stepper_driver_t;



/**
* @brief Declare of Step motor Type
*
*/
struct stepper_driver_s {

    esp_err_t (*init)(stepper_driver_t *handle);
    esp_err_t (*enable)(stepper_driver_t *handle);
    esp_err_t (*disable)(stepper_driver_t *handle);
    esp_err_t (*direction)(stepper_driver_t *handle, uint32_t direction);
    esp_err_t (*steps)(stepper_driver_t *handle, uint32_t count, uint32_t delay);
    esp_err_t (*move)(stepper_driver_t *handle, int32_t speed);
    esp_err_t (*clear_gstat)(stepper_driver_t *handle);
    esp_err_t (*set_stealthchop_thrs)(stepper_driver_t *handle, uint32_t tpwmthrs);
    esp_err_t (*set_pwm_freq)(stepper_driver_t *handle, stepper_driver_pwm_freq_t pwm_freq);
    esp_err_t (*set_pwm_grad)(stepper_driver_t *handle, uint32_t grad);
    esp_err_t (*set_pwm_offset)(stepper_driver_t *handle, uint32_t offset);
    esp_err_t (*set_current)(stepper_driver_t *handle, uint32_t milliampere_run, uint32_t percent_hold);
    esp_err_t (*set_microsteps_per_step)(stepper_driver_t *handle, stepper_driver_microsteps_t microssteps);
    esp_err_t (*disable_pwm_autograd)(stepper_driver_t *handle);
    esp_err_t (*enable_pwm_autograd)(stepper_driver_t *handle); 
    esp_err_t (*disable_pwm_autoscale)(stepper_driver_t *handle);
    esp_err_t (*enable_pwm_autoscale)(stepper_driver_t *handle);

    esp_err_t (*dump_register_tstep)(stepper_driver_t *handle);
    esp_err_t (*dump_register_drv_status)(stepper_driver_t *handle);
    esp_err_t (*dump_register_ioin)(stepper_driver_t *handle);
    esp_err_t (*dump_register_gconf)(stepper_driver_t *handle);
    esp_err_t (*dump_register_otp_read)(stepper_driver_t *handle);
    esp_err_t (*dump_register_chopconf)(stepper_driver_t *handle);
    esp_err_t (*dump_register_pwmconf)(stepper_driver_t *handle);
    esp_err_t (*dump_register_factory_conf)(stepper_driver_t *handle);
    esp_err_t (*dump_register_mscuract)(stepper_driver_t *handle);
    esp_err_t (*dump_register_pwm_scale)(stepper_driver_t *handle);
    esp_err_t (*dump_register_pwm_auto)(stepper_driver_t *handle);
    esp_err_t (*dump_registers)(stepper_driver_t *handle);
};



/**
 * @brief Initialize step motor driver
 *
 * @param handle driver handle
 * @return
 *      - ESP_OK: successfully initialized
 *      - ESP_ERR_INVALID_ARG: wrong parameter
 */
esp_err_t stepper_driver_init(stepper_driver_t *handle);
esp_err_t stepper_driver_enable(stepper_driver_t *handle);
esp_err_t stepper_driver_disable(stepper_driver_t *handle);
esp_err_t stepper_driver_direction(stepper_driver_t *handle, uint32_t direction);
esp_err_t stepper_driver_steps(stepper_driver_t *handle, uint32_t count, uint32_t delay);
esp_err_t stepper_driver_move(stepper_driver_t *handle, int32_t speed);
esp_err_t stepper_driver_clear_gstat(stepper_driver_t *handle);
esp_err_t stepper_driver_set_stealthchop_thrs(stepper_driver_t *handle, uint32_t tpwmthrs);
esp_err_t stepper_driver_set_pwm_freq(stepper_driver_t *handle, stepper_driver_pwm_freq_t pwm_freq);
esp_err_t stepper_driver_set_pwm_grad(stepper_driver_t *handle, uint32_t grad);
esp_err_t stepper_driver_set_pwm_offset(stepper_driver_t *handle, uint32_t offset);
esp_err_t stepper_driver_set_current(stepper_driver_t *handle, uint32_t milliampere_run, uint32_t percent_hold);
esp_err_t stepper_driver_set_microsteps_per_step(stepper_driver_t *handle, stepper_driver_microsteps_t microssteps);
esp_err_t stepper_driver_disable_pwm_autograd(stepper_driver_t *handle);
esp_err_t stepper_driver_enable_pwm_autograd(stepper_driver_t *handle); 
esp_err_t stepper_driver_disable_pwm_autoscale(stepper_driver_t *handle);
esp_err_t stepper_driver_enable_pwm_autoscale(stepper_driver_t *handle);

esp_err_t stepper_driver_dump_register_tstep(stepper_driver_t *handle);
esp_err_t stepper_driver_dump_register_drv_status(stepper_driver_t *handle);
esp_err_t stepper_driver_dump_register_ioin(stepper_driver_t *handle);
esp_err_t stepper_driver_dump_register_gconf(stepper_driver_t *handle);
esp_err_t stepper_driver_dump_register_otp_read(stepper_driver_t *handle);
esp_err_t stepper_driver_dump_register_chopconf(stepper_driver_t *handle);
esp_err_t stepper_driver_dump_register_pwmconf(stepper_driver_t *handle);
esp_err_t stepper_driver_dump_register_factory_conf(stepper_driver_t *handle);
esp_err_t stepper_driver_dump_register_mscuract(stepper_driver_t *handle);
esp_err_t stepper_driver_dump_register_pwm_scale(stepper_driver_t *handle);
esp_err_t stepper_driver_dump_register_pwm_auto(stepper_driver_t *handle);

esp_err_t stepper_driver_dump_registers(stepper_driver_t *handle);


