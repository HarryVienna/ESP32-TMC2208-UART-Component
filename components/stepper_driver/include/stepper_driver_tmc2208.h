#pragma once


#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

#include "stepper_driver.h"
#include "stepper_driver_tmc2208_base.h"


// General
#define UART_MAX_DELAY    100
#define TMC2208_R_SENSE   110.0 // mOhm



typedef struct stepper_driver_s stepper_driver_t;

/**
* @brief Step Motor Configuration Type
*
*/
typedef struct stepper_driver_tmc2208_conf_s {
    uart_port_t uart_port;        /*!< UART port number */
    uint32_t rx_pin;              /*!< UART Rx Pin number */
    uint32_t tx_pin;              /*!< UART Tx Pin number */
    uint32_t baud_rate;           /*!< UART baud rate */

    gpio_num_t step_pin;
    gpio_num_t direction_pin;
    gpio_num_t enable_pin;
} stepper_driver_tmc2208_conf_t;




typedef struct {
    stepper_driver_t parent;

    // UART and ports
    stepper_driver_tmc2208_conf_t driver_config;

    // driver datagrams
    tmc2208_gconf_dgr_t gconf;
    tmc2208_gstat_dgr_t gstat;
    tmc2208_ifcnt_dgr_t ifcnt;
    tmc2208_slaveconf_dgr_t slaveconf;
    tmc2208_otp_prog_dgr_t otp_prog;
    tmc2208_otp_read_dgr_t otp_read;
    tmc2208_ioin_dgr_t ioin;
    tmc2208_factory_conf_dgr_t factory_conf;
    tmc2208_ihold_irun_dgr_t ihold_irun;
    tmc2208_tpowerdown_dgr_t tpowerdown;
    tmc2208_tstep_dgr_t tstep;
    tmc2208_tpwmthrs_dgr_t tpwmthrs;
    tmc2208_vactual_dgr_t vactual;
    tmc2208_mscnt_dgr_t mscnt;
    tmc2208_mscuract_dgr_t mscuract;
    tmc2208_chopconf_dgr_t chopconf;
    tmc2208_drv_status_dgr_t drv_status;
    tmc2208_pwmconf_dgr_t pwmconf;
    tmc2208_pwm_scale_dgr_t pwm_scale;
    tmc2208_pwm_auto_ctrl_dgr_t pwm_auto;
} stepper_driver_tmc2208_t;


/**
* @brief Install a new TMC2208 driver 
*
* @param config: step motor configuration
* @return
*      step motor instance or NULL
*/
stepper_driver_t *stepper_driver_new_tmc2208(const stepper_driver_tmc2208_conf_t *config);














