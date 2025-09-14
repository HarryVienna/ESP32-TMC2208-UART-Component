#include "stepper_driver.h"

esp_err_t stepper_driver_init(stepper_driver_t *handle){
    return handle->init(handle);
}
esp_err_t stepper_driver_clear_gstat(stepper_driver_t *handle){
    return handle->clear_gstat(handle);
}


esp_err_t stepper_driver_enable(stepper_driver_t *handle){
    return handle->enable(handle);
}
esp_err_t stepper_driver_disable(stepper_driver_t *handle){
    return handle->disable(handle);
}
esp_err_t stepper_driver_direction(stepper_driver_t *handle, uint8_t direction){
    return handle->direction(handle, direction);
}
esp_err_t stepper_driver_steps(stepper_driver_t *handle, uint32_t count, uint32_t delay){
    return handle->steps(handle, count, delay);
}


esp_err_t stepper_driver_set_vactual(stepper_driver_t *handle, int32_t speed){
    return handle->set_vactual(handle, speed);
}
esp_err_t stepper_driver_set_tpowerdown(stepper_driver_t *handle, uint8_t tpowerdown){
    return handle->set_tpowerdown(handle, tpowerdown);
}
esp_err_t stepper_driver_set_stealthchop_thrs(stepper_driver_t *handle, uint32_t tpwmthrs){
    return handle->set_stealthchop_thrs(handle, tpwmthrs);
}
esp_err_t stepper_driver_set_current(stepper_driver_t *handle, uint16_t milliampere_run, uint8_t percent_hold){
    return handle->set_current(handle, milliampere_run, percent_hold);
}


esp_err_t stepper_driver_set_microsteps_per_step(stepper_driver_t *handle, stepper_driver_microsteps_t microssteps){
    return handle->set_microsteps_per_step(handle, microssteps);
}
esp_err_t stepper_driver_set_tbl(stepper_driver_t *handle, uint8_t tbl){
    return handle->set_tbl(handle, tbl);
}
esp_err_t stepper_driver_set_toff(stepper_driver_t *handle, uint8_t toff){
    return handle->set_toff(handle, toff);
}
esp_err_t stepper_driver_set_hysteresis(stepper_driver_t *handle, uint8_t hstrt, uint8_t hend){
    return handle->set_hysteresis(handle, hstrt, hend);
}


esp_err_t stepper_driver_set_pwm_lim(stepper_driver_t *handle, uint8_t pwm_lim){
    return handle->set_pwm_lim(handle, pwm_lim);
}
esp_err_t stepper_driver_set_pwm_reg(stepper_driver_t *handle, uint8_t pwm_reg){
    return handle->set_pwm_reg(handle, pwm_reg);
}
esp_err_t stepper_driver_set_pwm_freq(stepper_driver_t *handle, stepper_driver_pwm_freq_t pwm_freq){
    return handle->set_pwm_freq(handle, pwm_freq);
}
esp_err_t stepper_driver_set_pwm_grad(stepper_driver_t *handle, uint8_t grad){
    return handle->set_pwm_grad(handle, grad);
}
esp_err_t stepper_driver_set_pwm_offset(stepper_driver_t *handle, uint8_t offset){
    return handle->set_pwm_offset(handle, offset);
}
esp_err_t stepper_driver_disable_pwm_autograd(stepper_driver_t *handle){
    return handle->disable_pwm_autograd(handle);
}
esp_err_t stepper_driver_enable_pwm_autograd(stepper_driver_t *handle){
    return handle->enable_pwm_autograd(handle);
}
esp_err_t stepper_driver_disable_pwm_autoscale(stepper_driver_t *handle){
    return handle->disable_pwm_autoscale(handle);
}
esp_err_t stepper_driver_enable_pwm_autoscale(stepper_driver_t *handle){
    return handle->enable_pwm_autoscale(handle);
}


esp_err_t stepper_driver_read_register_gstat(stepper_driver_t *handle){
   return handle->read_register_gstat(handle);
}
esp_err_t stepper_driver_read_register_tstep(stepper_driver_t *handle){
   return handle->read_register_tstep(handle);
}
esp_err_t stepper_driver_read_register_drv_status(stepper_driver_t *handle){
   return handle->read_register_drv_status(handle);
}
esp_err_t stepper_driver_read_register_ioin(stepper_driver_t *handle){
   return handle->read_register_ioin(handle);
}
esp_err_t stepper_driver_read_register_otp_read(stepper_driver_t *handle){
   return handle->read_register_otp_read(handle);
}
esp_err_t stepper_driver_read_register_mscntd(stepper_driver_t *handle){
   return handle->read_register_mscntd(handle);
}
esp_err_t stepper_driver_read_register_mscuract(stepper_driver_t *handle){
   return handle->read_register_mscuract(handle);
}


esp_err_t stepper_driver_dump_register_tstep(stepper_driver_t *handle){
    return handle->dump_register_tstep(handle);
}
esp_err_t stepper_driver_dump_register_drv_status(stepper_driver_t *handle){
    return handle->dump_register_drv_status(handle);
}
esp_err_t stepper_driver_dump_register_ioin(stepper_driver_t *handle){
    return handle->dump_register_ioin(handle);
}
esp_err_t stepper_driver_dump_register_gconf(stepper_driver_t *handle){
    return handle->dump_register_gconf(handle);
}
esp_err_t stepper_driver_dump_register_otp_read(stepper_driver_t *handle){
    return handle->dump_register_otp_read(handle);
}
esp_err_t stepper_driver_dump_register_chopconf(stepper_driver_t *handle){
    return handle->dump_register_chopconf(handle);
}
esp_err_t stepper_driver_dump_register_pwmconf(stepper_driver_t *handle){
    return handle->dump_register_pwmconf(handle);
}
esp_err_t stepper_driver_dump_register_factory_conf(stepper_driver_t *handle){
    return handle->dump_register_factory_conf(handle);
}
esp_err_t stepper_driver_dump_register_mscuract(stepper_driver_t *handle){
    return handle->dump_register_mscuract(handle);
}
esp_err_t stepper_driver_dump_register_pwm_scale(stepper_driver_t *handle){
    return handle->dump_register_pwm_scale(handle);
}
esp_err_t stepper_driver_dump_register_pwm_auto(stepper_driver_t *handle){
    return handle->dump_register_pwm_auto(handle);
}

