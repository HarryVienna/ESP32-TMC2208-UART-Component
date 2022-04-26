
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/cdefs.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "stepper_driver_tmc2208_base.h"
#include "stepper_driver_tmc2208.h"

static const char *TAG = "tmc2208";

// Forward declarations
static void byteswap (uint8_t data[4]);
static void calcCRC (uint8_t *datagram, uint8_t datagramLength);
static esp_err_t write_register_safe (stepper_driver_tmc2208_t *driver, tmc2208_datagram_t *reg);
static void write_register (stepper_driver_tmc2208_t *driver, tmc2208_datagram_t *reg);
static esp_err_t read_register (stepper_driver_tmc2208_t *driver, tmc2208_datagram_t *reg);

/**
 * @brief Init Stepper
 *
 * @param motor step_motor_t type object
 */
esp_err_t tmc2208_init(stepper_driver_t *handle)
{
    esp_err_t ret = ESP_OK;
    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);

    gpio_reset_pin(tmc2208->driver_config.step_pin);
    gpio_reset_pin(tmc2208->driver_config.direction_pin);
    gpio_reset_pin(tmc2208->driver_config.enable_pin);
    gpio_reset_pin(tmc2208->driver_config.rx_pin);
    gpio_reset_pin(tmc2208->driver_config.tx_pin);

    gpio_set_direction(tmc2208->driver_config.step_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(tmc2208->driver_config.direction_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(tmc2208->driver_config.enable_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(tmc2208->driver_config.rx_pin, GPIO_MODE_INPUT);
    gpio_set_direction(tmc2208->driver_config.tx_pin, GPIO_MODE_OUTPUT);

    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = tmc2208->driver_config.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ret = uart_driver_install(tmc2208->driver_config.uart_port, UART_FIFO_LEN * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install driver: %s (0x%x)", esp_err_to_name(ret), ret);
        return ret;
    }
    ret = uart_param_config(tmc2208->driver_config.uart_port, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to config param: %s (0x%x)", esp_err_to_name(ret), ret);
        return ret;
    }
    ret = uart_set_pin(tmc2208->driver_config.uart_port, tmc2208->driver_config.tx_pin, tmc2208->driver_config.rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set pins: %s (0x%x)", esp_err_to_name(ret), ret);
        return ret;
    }


    // Configure TMC2208
    gpio_set_level(tmc2208->driver_config.enable_pin, 1); // Disable stepper

    //ret = read_register(tmc2208, (tmc2208_datagram_t *)&tmc2208->gstat);
    //if (ret != ESP_OK) {
    //    ESP_LOGE(TAG, "Failed to read gstat register");
    //    return ret;
    //}
    //write_register_safe(tmc2208, (tmc2208_datagram_t *)&tmc2208->gstat);

    ret = read_register(tmc2208, (tmc2208_datagram_t *)&tmc2208->gconf);
    tmc2208->gconf.reg.pdn_disable = 1; // 0: PDN_UART controls standstill current reduction, 1: PDN_UART input function disabled
    tmc2208->gconf.reg.I_scale_analog = 0; // 0: Use internal reference derived from 5VOUT,  1: Use voltage supplied to VREF as current reference
    tmc2208->gconf.reg.mstep_reg_select = 1;  // 0: Microstep resolution selected by pins MS1, MS2, 1: Microstep resolution selected by MSTEP register 
    tmc2208->gconf.reg.en_spreadcycle = 0;  // 0: StealthChop PWM mode enabled, 1: SpreadCycle mode enabled 
    tmc2208->gconf.reg.multistep_filt = 1;  // 0: No filtering of STEP pulses, 1: Software pulse generator optimization enabled 
    write_register_safe(tmc2208, (tmc2208_datagram_t *)&tmc2208->gconf);

    return ret;
}

esp_err_t tmc2208_enable(stepper_driver_t *handle)
{
    esp_err_t ret = ESP_OK;
    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);

    gpio_set_level(tmc2208->driver_config.enable_pin, 0);

    return ret;
}

esp_err_t tmc2208_disable(stepper_driver_t *handle)
{
    esp_err_t ret = ESP_OK;
    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);

    gpio_set_level(tmc2208->driver_config.enable_pin, 1);

    return ret;
}

esp_err_t tmc2208_direction(stepper_driver_t *handle, uint32_t direction)
{
    esp_err_t ret = ESP_OK;
    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);

    gpio_set_level(tmc2208->driver_config.direction_pin, direction);

    return ret;
}

esp_err_t tmc2208_steps(stepper_driver_t *handle, uint32_t steps, uint32_t delay)
{
    esp_err_t ret = ESP_OK;
    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);

    for (int i=0; i<steps; i++) {
        gpio_set_level(tmc2208->driver_config.step_pin, 1);
        usleep(100);

        gpio_set_level(tmc2208->driver_config.step_pin, 0);
        usleep(delay);
    }

    return ret;
}

esp_err_t tmc2208_move(stepper_driver_t *handle, int32_t speed)
{
    esp_err_t ret = ESP_OK;
    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);

    tmc2208->vactual.reg.actual = speed; 
    write_register_safe(tmc2208, (tmc2208_datagram_t *)&tmc2208->vactual);

    return ret;
}

esp_err_t tmc2208_clear_gstat(stepper_driver_t *handle)
{
    esp_err_t ret = ESP_OK;
    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);

    ret = read_register(tmc2208, (tmc2208_datagram_t *)&tmc2208->gstat);
    ESP_LOGI(TAG, "Clear gstat. Values for  reset drv_err uv_cp: %d %d %d",
        tmc2208->gstat.reg.reset,
        tmc2208->gstat.reg.drv_err,
        tmc2208->gstat.reg.uv_cp);

    tmc2208->gstat.reg.reset = 1;
    tmc2208->gstat.reg.drv_err = 1;
    tmc2208->gstat.reg.uv_cp = 1;

    write_register_safe(tmc2208, (tmc2208_datagram_t *)&tmc2208->gstat);

    return ret;
}

esp_err_t tmc2208_set_stealthchop_thrs(stepper_driver_t *handle, uint32_t tpwmthrs) {

    esp_err_t ret = ESP_OK;
    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);

    tmc2208->tpwmthrs.reg.tpwmthrs = tpwmthrs; // PWM frequency selection,  // 0 = 1/1024, 1 = 2/683, 2 = 2/512, 3 = 2/410 fCLK

    write_register_safe(tmc2208, (tmc2208_datagram_t *)&tmc2208->tpwmthrs);

    return ret;
}

esp_err_t tmc2208_set_pwm_freq(stepper_driver_t *handle, stepper_driver_pwm_freq_t pwm_freq) {

    esp_err_t ret = ESP_OK;
    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);

    ret = read_register(tmc2208, (tmc2208_datagram_t *)&tmc2208->pwmconf);

    tmc2208->pwmconf.reg.pwm_freq = pwm_freq; // PWM frequency selection,  // 0 = 1/1024, 1 = 2/683, 2 = 2/512, 3 = 2/410 fCLK

    write_register_safe(tmc2208, (tmc2208_datagram_t *)&tmc2208->pwmconf);

    return ret;
}

esp_err_t tmc2208_set_pwm_grad(stepper_driver_t *handle, uint32_t grad) {

    esp_err_t ret = ESP_OK;
    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);

    ret = read_register(tmc2208, (tmc2208_datagram_t *)&tmc2208->pwmconf);

    tmc2208->pwmconf.reg.pwm_grad = grad;

    write_register_safe(tmc2208, (tmc2208_datagram_t *)&tmc2208->pwmconf);

    return ret;
}

esp_err_t tmc2208_set_pwm_offset(stepper_driver_t *handle, uint32_t offset) {

    esp_err_t ret = ESP_OK;
    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);

    ret = read_register(tmc2208, (tmc2208_datagram_t *)&tmc2208->pwmconf);

    tmc2208->pwmconf.reg.pwm_ofs = offset;

    write_register_safe(tmc2208, (tmc2208_datagram_t *)&tmc2208->pwmconf);

    return ret;
}

/**
 * @brief Set RMS current in mA
 * Equation from datahseet
 * I_rms = (CS+1)/32 * V_fs/(R_sense+30mohm) * 1/sqrt(2)      --- or 20??
 * Solve for CS ->
 * CS = 32*sqrt(2)*I_rms*(R_sense+30mOhm)/V_fs - 1
 *
 * @param handle stepper_driver_t type object
 * @param milliampere_run Current in milliampere for IRUN
 * @param percent_hold Current for IHOLD in percentage of IRUN
 */
esp_err_t tmc2208_set_current(stepper_driver_t *handle, uint32_t milliampere_run, uint32_t percent_hold) {

    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);

    read_register(tmc2208, (tmc2208_datagram_t *)&tmc2208->chopconf);

    uint32_t cs_run = 32.0 * 1.41421f * ((float)milliampere_run / 1000.0) * ((TMC2208_R_SENSE + 30.0)  / 325.0) - 1;
    uint32_t cs_hold = (cs_run * percent_hold) / 100;
    ESP_LOGD(TAG, "Calculated values for %d mA: IRUN=%d IHOLD=%d", milliampere_run, cs_run, cs_hold);
    if (cs_run < 16) { //  High sensitivity, low sense resistor voltage
        tmc2208->chopconf.reg.vsense = 1;
        cs_run = 32.0 * 1.41421f * ((float)milliampere_run / 1000.0) * ((TMC2208_R_SENSE + 30.0)  / 180.0) - 1;
        cs_hold = (cs_run * percent_hold) / 100;
        ESP_LOGD(TAG, "Recalculated values for %d mA: IRUN=%d IHOLD=%d", milliampere_run, cs_run, cs_hold);
    }
    else {
        tmc2208->chopconf.reg.vsense = 0;
    }

    write_register_safe(tmc2208, (tmc2208_datagram_t *)&tmc2208->chopconf);

    tmc2208->ihold_irun.reg.irun = cs_run;
    tmc2208->ihold_irun.reg.ihold = cs_hold;

    write_register_safe(tmc2208, (tmc2208_datagram_t *)&tmc2208->ihold_irun);

    esp_err_t ret = ESP_OK;
    return ret;
}


esp_err_t tmc2208_set_microsteps_per_step(stepper_driver_t *handle, stepper_driver_microsteps_t microssteps) {
    
    esp_err_t ret = ESP_OK;

    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);

    ret = read_register(tmc2208, (tmc2208_datagram_t *)&tmc2208->chopconf);
    tmc2208->chopconf.reg.mres = microssteps;   // Micro step resolution    
    write_register_safe(tmc2208, (tmc2208_datagram_t *)&tmc2208->chopconf);

    return ret;
}


esp_err_t tmc2208_disable_pwm_autograd(stepper_driver_t *handle) {
    
    esp_err_t ret = ESP_OK;

    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);

    ret = read_register(tmc2208, (tmc2208_datagram_t *)&tmc2208->pwmconf);
    tmc2208->pwmconf.reg.pwm_autograd = 0;  
    write_register_safe(tmc2208, (tmc2208_datagram_t *)&tmc2208->pwmconf);

    return ret;
}

esp_err_t tmc2208_enable_pwm_autograd(stepper_driver_t *handle) {
    
    esp_err_t ret = ESP_OK;

    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);

    ret = read_register(tmc2208, (tmc2208_datagram_t *)&tmc2208->pwmconf);
    tmc2208->pwmconf.reg.pwm_autograd = 1;  
    write_register_safe(tmc2208, (tmc2208_datagram_t *)&tmc2208->pwmconf);

    return ret;
}

esp_err_t tmc2208_disable_pwm_autoscale(stepper_driver_t *handle) {
    
    esp_err_t ret = ESP_OK;

    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);

    ret = read_register(tmc2208, (tmc2208_datagram_t *)&tmc2208->pwmconf);
    tmc2208->pwmconf.reg.pwm_autoscale = 0;  
    write_register_safe(tmc2208, (tmc2208_datagram_t *)&tmc2208->pwmconf);

    return ret;
}

esp_err_t tmc2208_enable_pwm_autoscale(stepper_driver_t *handle) {
    
    esp_err_t ret = ESP_OK;

    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);

    ret = read_register(tmc2208, (tmc2208_datagram_t *)&tmc2208->pwmconf);
    tmc2208->pwmconf.reg.pwm_autoscale = 1;  
    write_register_safe(tmc2208, (tmc2208_datagram_t *)&tmc2208->pwmconf);

    return ret;
}

esp_err_t tmc2208_dump_register_tstep(stepper_driver_t *handle)
{
    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);

    read_register(tmc2208, (tmc2208_datagram_t *)&tmc2208->tstep);
    ESP_LOGI(TAG, "TSTEP: \n"
                "  +-----------------+\n"
                "  | tstep: %7d  |\n"
                "  +-----------------+\n"
, 
        tmc2208->tstep.reg.tstep);    

    return ESP_OK; 
}

esp_err_t tmc2208_dump_register_drv_status(stepper_driver_t *handle)
{
    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);

    read_register(tmc2208, (tmc2208_datagram_t *)&tmc2208->drv_status);
    ESP_LOGI(TAG, "DRV_STATUS: \n"
                "  +---------------+------------+----------+----------+---------+---------+\n"
                "  | otpw: %d       | ot:   %d    | t120:  %d | t143:  %d | t150: %d | t157: %d |\n"
                //"  +---------------+------------+----------+----------+---------+---------+\n"
                "  | s2ga: %d       | s2gb: %d    | s2vsa: %d | s2vsa: %d | ola:  %d | olb:  %d |\n"
                //"  +---------------+------------+----------+----------+---------+---------+\n"
                "  | cs_actual: %2d | stealth: %d | stst: %d  |          |         |         |\n"
                "  +---------------+------------+----------+----------+---------+---------+\n"
, 
        tmc2208->drv_status.reg.otpw,
        tmc2208->drv_status.reg.ot,
        tmc2208->drv_status.reg.t120,
        tmc2208->drv_status.reg.t143,
        tmc2208->drv_status.reg.t150,
        tmc2208->drv_status.reg.t157,
        tmc2208->drv_status.reg.s2ga,
        tmc2208->drv_status.reg.s2gb,
        tmc2208->drv_status.reg.s2vsa,
        tmc2208->drv_status.reg.s2vsb,
        tmc2208->drv_status.reg.ola,
        tmc2208->drv_status.reg.olb,
        tmc2208->drv_status.reg.cs_actual,
        tmc2208->drv_status.reg.stealth,
        tmc2208->drv_status.reg.stst);  

    return ESP_OK; 
}

esp_err_t tmc2208_dump_register_ioin(stepper_driver_t *handle)
{
    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);

     read_register(tmc2208, (tmc2208_datagram_t *)&tmc2208->ioin);
    ESP_LOGI(TAG, "IOIN: \n"
                "  +-------------+----------+----------+---------+---------+--------+\n"
                "  | enn: %d      | ms1: %d   | ms2: %d   | diag: %d | step: %d | dir: %d |\n"
                "  | pdn_uart: %d | version: 0x%2x       | sel_a: %s             |\n"
                "  +-------------+----------+----------+---------+---------+--------+\n"
                , 
        tmc2208->ioin.reg.enn,
        tmc2208->ioin.reg.ms1,
        tmc2208->ioin.reg.ms2,
        tmc2208->ioin.reg.diag,
        tmc2208->ioin.reg.step,
        tmc2208->ioin.reg.dir,
        tmc2208->ioin.reg.pdn_uart,
        tmc2208->ioin.reg.version,
        tmc2208->ioin.reg.sel_a == 0 ? "TMC222x" : "TMC220x"); 

    return ESP_OK; 
}

esp_err_t tmc2208_dump_register_gconf(stepper_driver_t *handle)
{
    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);


    read_register(tmc2208, (tmc2208_datagram_t *)&tmc2208->gconf);
    ESP_LOGI(TAG, "GCONF: \n"
                "  +---------------------+--------------------+-------------------+----------+\n"
                "  | I_scale_analog: %d   | internal_Rsense: %d | en_spreadcycle: %d | shaft: %d |\n"
                "  | index_otpw: %d       | index_step: %d      | pdn_disable: %d    |          |\n"
                "  | mstep_reg_select: %d | multistep_filt: %d  | test_mode: %d      |          |\n"
                "  +---------------------+--------------------+-------------------+----------+\n"
                  , 
                tmc2208->gconf.reg.I_scale_analog,
                tmc2208->gconf.reg.internal_Rsense,
                tmc2208->gconf.reg.en_spreadcycle,
                tmc2208->gconf.reg.shaft,
                tmc2208->gconf.reg.index_otpw,
                tmc2208->gconf.reg.index_step,
                tmc2208->gconf.reg.pdn_disable,
                tmc2208->gconf.reg.mstep_reg_select,
                tmc2208->gconf.reg.multistep_filt,
                tmc2208->gconf.reg.test_mode);

    return ESP_OK; 
}

esp_err_t tmc2208_dump_register_otp_read(stepper_driver_t *handle)
{
    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);

    read_register(tmc2208, (tmc2208_datagram_t *)&tmc2208->otp_read);
    ESP_LOGI(TAG, "OTP_READ: \n"
                "  +---------------------------+------------------------------+-------------------------------+------------------------------+\n"
                "  | otp_en_spreadcycle: %d     | otp_ihold: %d                 | otp_iholddelay: %d             |                              |\n"
                "  | otp_pwm_freq: %d           | otp_pwm_reg: %d               |                               |                              |\n"
                "  | otp_pwm_ofs_chopconf8: %d  | otp_tpwmthrs_chopconf5_7: %d  | otp_pwm_autograd_chopconf4: %d | otp_pwm_grad_chopconf0_3: %2d |\n"
                "  | otp_tbl: %d                | otp_internalrsense: %d        | otp_ottrim: %d                 | otp_fclktrim: %d             |\n"
                "  +---------------------------+------------------------------+-------------------------------+------------------------------+\n"   
                , 
                tmc2208->otp_read.reg.otp_en_spreadcycle,
                tmc2208->otp_read.reg.otp_ihold,
                tmc2208->otp_read.reg.otp_iholddelay,
                tmc2208->otp_read.reg.otp_pwm_freq,
                tmc2208->otp_read.reg.otp_pwm_reg,
                tmc2208->otp_read.reg.otp_pwm_ofs_chopconf8,
                tmc2208->otp_read.reg.otp_tpwmthrs_chopconf5_7,
                tmc2208->otp_read.reg.otp_pwm_autograd_chopconf4,
                tmc2208->otp_read.reg.otp_pwm_grad_chopconf0_3,
                tmc2208->otp_read.reg.otp_tbl,
                tmc2208->otp_read.reg.otp_internalrsense,
                tmc2208->otp_read.reg.otp_ottrim,
                tmc2208->otp_read.reg.otp_fclktrim);

    return ESP_OK; 
}

esp_err_t tmc2208_dump_register_chopconf(stepper_driver_t *handle)
{
    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);

    read_register(tmc2208, (tmc2208_datagram_t *)&tmc2208->chopconf);
    ESP_LOGI(TAG, "CHOPCONF: \n"
                "  +----------+-----------+----------+-----------+------------+\n"
                "  | toff: %2d | hstrt: %d  | hend: %2d | tbl: %d    | vsense: %d  |\n"
                "  | mres: %2d | intpol: %d | dedge: %d | diss2g: %d | diss2vs: %d |\n"
                "  +----------+-----------+----------+-----------+------------+\n" 
                , 
        tmc2208->chopconf.reg.toff,
        tmc2208->chopconf.reg.hstrt,
        tmc2208->chopconf.reg.hend,
        tmc2208->chopconf.reg.tbl,
        tmc2208->chopconf.reg.vsense,
        tmc2208->chopconf.reg.mres,
        tmc2208->chopconf.reg.intpol,
        tmc2208->chopconf.reg.dedge,
        tmc2208->chopconf.reg.diss2g,
        tmc2208->chopconf.reg.diss2vs);

    return ESP_OK; 
}

esp_err_t tmc2208_dump_register_pwmconf(stepper_driver_t *handle)
{
    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);


    read_register(tmc2208, (tmc2208_datagram_t *)&tmc2208->pwmconf);
    ESP_LOGI(TAG, "PWMCONF: \n"
                "  +-----------------+---------------+--------------+------------------+\n"
                "  | pwm_ofs: %3d    | pwm_grad: %3d | pwm_freq: %d  | pwm_autoscale: %d |\n"
                "  | pwm_autograd: %d | freewheel: %d  | pwm_reg: %2d  | pwm_lim: %2d      |\n"
                "  +-----------------+---------------+--------------+------------------+\n"
                ,
        tmc2208->pwmconf.reg.pwm_ofs,
        tmc2208->pwmconf.reg.pwm_grad,
        tmc2208->pwmconf.reg.pwm_freq,
        tmc2208->pwmconf.reg.pwm_autoscale,
        tmc2208->pwmconf.reg.pwm_autograd,
        tmc2208->pwmconf.reg.freewheel,
        tmc2208->pwmconf.reg.pwm_reg,
        tmc2208->pwmconf.reg.pwm_lim);

    return ESP_OK; 
}

esp_err_t tmc2208_dump_register_factory_conf(stepper_driver_t *handle)
{
    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);

    read_register(tmc2208, (tmc2208_datagram_t *)&tmc2208->factory_conf);
    ESP_LOGI(TAG, "FACTORY_CONFIG: \n"
                "  +--------------+-----------+\n"
                "  | fclktrim: %2d | ottrim: %d |\n"
                "  +--------------+-----------+\n"
                , 
        tmc2208->factory_conf.reg.fclktrim,
        tmc2208->factory_conf.reg.ottrim );     

    return ESP_OK; 
}

esp_err_t tmc2208_dump_register_mscuract(stepper_driver_t *handle)
{
    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);

    read_register(tmc2208, (tmc2208_datagram_t *)&tmc2208->mscuract);
    struct { // Sign extend 9 Bit number
        signed int a:9;
        signed int b:9;
        } s;
    int16_t signed_cur_a = s.a = tmc2208->mscuract.reg.cur_a;
    int16_t signed_cur_b = s.b = tmc2208->mscuract.reg.cur_b;
    ESP_LOGI(TAG, "MSCURACT: \n"
                "  +-------------+-------------+\n"
                "  | cur_a: %4d | cur_b: %4d |\n"
                "  +-------------+-------------+\n"    
                ,
        signed_cur_a,
        signed_cur_b);   

    return ESP_OK; 
}

esp_err_t tmc2208_dump_register_pwm_scale(stepper_driver_t *handle)
{
    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);

    read_register(tmc2208, (tmc2208_datagram_t *)&tmc2208->pwm_scale);
    struct {signed int x:9;} s; // Sign extend 9 Bit number
    int16_t signed_pwm_scale_auto = s.x = tmc2208->pwm_scale.reg.pwm_scale_auto;
    ESP_LOGI(TAG, "PWM_SCALE: \n"
                "  +--------------------+----------------------+\n"
                "  | pwm_scale_sum: %3d | pwm_scale_auto: %4d |\n"
                "  +--------------------+----------------------+\n"    
                ,    
        tmc2208->pwm_scale.reg.pwm_scale_sum,
        signed_pwm_scale_auto);     

    return ESP_OK; 
}

esp_err_t tmc2208_dump_register_pwm_auto(stepper_driver_t *handle)
{
    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);

    read_register(tmc2208, (tmc2208_datagram_t *)&tmc2208->pwm_auto);
    ESP_LOGI(TAG, "PWM_AUTO: \n"
                "  +-------------------+--------------------+\n"
                "  | pwm_ofs_auto: %3d | pwm_grad_auto: %3d |\n"
                "  +-------------------+--------------------+\n"    
                ,        
        tmc2208->pwm_auto.reg.pwm_ofs_auto,
        tmc2208->pwm_auto.reg.pwm_grad_auto);    

    return ESP_OK; 
}

esp_err_t tmc2208_dump_registers(stepper_driver_t *handle)
{
    esp_err_t ret = ESP_OK;

    stepper_driver_tmc2208_t *tmc2208 = __containerof(handle, stepper_driver_tmc2208_t, parent);

    read_register(tmc2208, (tmc2208_datagram_t *)&tmc2208->gstat);
    ESP_LOGI(TAG, "GSTAT: \n"
                  "                        drv_err: %d \n"
                  "                        reset: %d \n"
                  "                        uv_cp: %d \n", tmc2208->gstat.reg.drv_err,  tmc2208->gstat.reg.reset,  tmc2208->gstat.reg.uv_cp);

    return ret;
}


stepper_driver_t *stepper_driver_new_tmc2208(const stepper_driver_tmc2208_conf_t *config)
{
    stepper_driver_tmc2208_t *tmc2208 = calloc(1, sizeof(stepper_driver_tmc2208_t));
 
    tmc2208->parent.init = tmc2208_init;
    tmc2208->parent.enable = tmc2208_enable;
    tmc2208->parent.disable = tmc2208_disable;
    tmc2208->parent.direction = tmc2208_direction;
    tmc2208->parent.steps= tmc2208_steps;
    tmc2208->parent.move= tmc2208_move;
    tmc2208->parent.clear_gstat= tmc2208_clear_gstat;
    tmc2208->parent.set_stealthchop_thrs = tmc2208_set_stealthchop_thrs;
    tmc2208->parent.set_pwm_freq = tmc2208_set_pwm_freq;
    tmc2208->parent.set_pwm_grad = tmc2208_set_pwm_grad;
    tmc2208->parent.set_pwm_offset = tmc2208_set_pwm_offset;
    tmc2208->parent.set_current = tmc2208_set_current;
    tmc2208->parent.set_microsteps_per_step = tmc2208_set_microsteps_per_step;
    tmc2208->parent.disable_pwm_autograd = tmc2208_disable_pwm_autograd;
    tmc2208->parent.enable_pwm_autograd = tmc2208_enable_pwm_autograd;
    tmc2208->parent.disable_pwm_autoscale = tmc2208_disable_pwm_autoscale;
    tmc2208->parent.enable_pwm_autoscale = tmc2208_enable_pwm_autoscale;

    tmc2208->parent.dump_register_tstep = tmc2208_dump_register_tstep;
    tmc2208->parent.dump_register_drv_status = tmc2208_dump_register_drv_status;
    tmc2208->parent.dump_register_ioin = tmc2208_dump_register_ioin;
    tmc2208->parent.dump_register_gconf = tmc2208_dump_register_gconf;
    tmc2208->parent.dump_register_otp_read = tmc2208_dump_register_otp_read;
    tmc2208->parent.dump_register_chopconf = tmc2208_dump_register_chopconf;
    tmc2208->parent.dump_register_pwmconf = tmc2208_dump_register_pwmconf;
    tmc2208->parent.dump_register_factory_conf = tmc2208_dump_register_factory_conf;
    tmc2208->parent.dump_register_mscuract = tmc2208_dump_register_mscuract;
    tmc2208->parent.dump_register_pwm_scale = tmc2208_dump_register_pwm_scale;
    tmc2208->parent.dump_register_pwm_auto = tmc2208_dump_register_pwm_auto;
    tmc2208->parent.dump_registers = tmc2208_dump_registers;


    tmc2208->driver_config.uart_port = (uart_port_t)config->uart_port;
    tmc2208->driver_config.rx_pin = (uint32_t)config->rx_pin;
    tmc2208->driver_config.tx_pin = (uint32_t)config->tx_pin;
    tmc2208->driver_config.baud_rate = (uint32_t)config->baud_rate;
    tmc2208->driver_config.enable_pin = (gpio_num_t)config->enable_pin;
    tmc2208->driver_config.step_pin = (gpio_num_t)config->step_pin;
    tmc2208->driver_config.direction_pin = (gpio_num_t)config->direction_pin;

    tmc2208->gconf.addr.idx = TMC2208_REG_GCONF;
    tmc2208->gstat.addr.idx = TMC2208_REG_GSTAT;
    tmc2208->ifcnt.addr.idx = TMC2208_REG_IFCNT;
    tmc2208->slaveconf.addr.idx = TMC2208_REG_SLAVECONF;
    tmc2208->otp_prog.addr.idx = TMC2208_REG_OTP_PROG;
    tmc2208->otp_read.addr.idx = TMC2208_REG_OTP_READ;
    tmc2208->ioin.addr.idx = TMC2208_REG_IOIN;
    tmc2208->factory_conf.addr.idx = TMC2208_REG_FACTORY_CONF;
    tmc2208->ihold_irun.addr.idx = TMC2208_REG_IHOLD_IRUN;
    tmc2208->tpowerdown.addr.idx = TMC2208_REG_TPOWERDOWN;
    tmc2208->tstep.addr.idx = TMC2208_REG_TSTEP;
    tmc2208->tpwmthrs.addr.idx = TMC2208_REG_TPWMTHRS;
    tmc2208->vactual.addr.idx = TMC2208_REG_VACTUAL;
    tmc2208->mscnt.addr.idx = TMC2208_REG_MSCNT;
    tmc2208->mscuract.addr.idx = TMC2208_REG_MSCURACT;
    tmc2208->chopconf.addr.idx = TMC2208_REG_CHOPCONF;
    tmc2208->drv_status.addr.idx = TMC2208_REG_DRV_STATUS;
    tmc2208->pwmconf.addr.idx = TMC2208_REG_PWMCONF;
    tmc2208->pwm_scale.addr.idx = TMC2208_REG_PWM_SCALE;
    tmc2208->pwm_auto.addr.idx = TMC2208_REG_PWM_AUTO;

    return &tmc2208->parent;
}


static esp_err_t write_register_safe(stepper_driver_tmc2208_t *tmc2208, tmc2208_datagram_t *reg)
{
    uint8_t ifcnt_pre;
    uint8_t ifcnt_after;

    read_register(tmc2208, (tmc2208_datagram_t *)&tmc2208->ifcnt);
    ifcnt_pre = tmc2208->ifcnt.reg.count;

    write_register(tmc2208, reg);

    read_register(tmc2208, (tmc2208_datagram_t *)&tmc2208->ifcnt);
    ifcnt_after = tmc2208->ifcnt.reg.count;

    bool ok = ifcnt_after - ifcnt_pre == 1;

    if (ok) {
        return ESP_OK;
    }
    else {
        ESP_LOGE(TAG, "Write failed: %d %d", ifcnt_pre, ifcnt_after);
        return ESP_FAIL;
    }
}

static void write_register(stepper_driver_tmc2208_t *tmc2208, tmc2208_datagram_t *reg)
{
    ESP_LOGD(TAG, "Write register 0x%02x", reg->addr.idx);

    tmc2208_write_datagram_t datagram;

    datagram.msg.sync = 0x05;
    datagram.msg.slave = 0x00;
    datagram.msg.addr.idx = reg->addr.idx;
    datagram.msg.addr.write = 1;
    datagram.msg.payload.value = reg->payload.value;

    byteswap(datagram.msg.payload.data);

    calcCRC(datagram.data, sizeof(datagram.data));

    uart_write_bytes(tmc2208->driver_config.uart_port, &datagram, sizeof(datagram));
    uart_wait_tx_done(tmc2208->driver_config.uart_port, UART_MAX_DELAY);
    uart_flush_input(tmc2208->driver_config.uart_port);
}

static esp_err_t read_register(stepper_driver_tmc2208_t *tmc2208, tmc2208_datagram_t *reg)
{
    esp_err_t ret = ESP_OK;

    ESP_LOGD(TAG, "Read register 0x%02x", reg->addr.idx);

    tmc2208_read_request_datagram_t request_datagram;
    tmc2208_read_request_datagram_t request_echo;
    tmc2208_read_reply_datagram_t reply_datagram;

    request_datagram.msg.sync = 0x05;
    request_datagram.msg.slave = 0x00;
    request_datagram.msg.addr.idx = reg->addr.idx;
    request_datagram.msg.addr.write = 0;

    calcCRC(request_datagram.data, sizeof(request_datagram.data));
    
    uart_write_bytes_with_break(tmc2208->driver_config.uart_port, &request_datagram, sizeof(request_datagram), 8 + 1); // default=8 bit times plus a little bit
    uart_wait_tx_done(tmc2208->driver_config.uart_port, UART_MAX_DELAY);

    uart_read_bytes(tmc2208->driver_config.uart_port, &request_echo, sizeof(request_echo), UART_MAX_DELAY);
    uart_read_bytes(tmc2208->driver_config.uart_port, &reply_datagram, sizeof(reply_datagram), UART_MAX_DELAY);

    if(reply_datagram.msg.slave == 0xff && reply_datagram.msg.addr.idx == request_datagram.msg.addr.idx) {
        uint8_t reply_crc = reply_datagram.msg.crc;
        calcCRC(reply_datagram.data, sizeof(reply_datagram.data));
        if((reply_crc == reply_datagram.msg.crc)) {
            reg->payload.value = reply_datagram.msg.payload.value;
            byteswap(reg->payload.data);
        }
        else {
            ESP_LOGE(TAG, "CRC failed: %d %d", reply_crc, reply_datagram.msg.crc);
            ret = ESP_FAIL;
        }
    }
    else {
        ESP_LOGE(TAG, "Reply datagram corrupt: slave %d addr %d", reply_datagram.msg.slave, reply_datagram.msg.addr.value);
        ret = ESP_FAIL;
    }

    return ret;
}



static void byteswap (uint8_t data[4])
{
    uint8_t tmp;

    tmp = data[0];
    data[0] = data[3];
    data[3] = tmp;
    tmp = data[1];
    data[1] = data[2];
    data[2] = tmp;
}

static void calcCRC (uint8_t *datagram, uint8_t datagramLength)
{
    int i,j;
    uint8_t *crc = datagram + (datagramLength - 1); // CRC located in last byte of message
    uint8_t currentByte;
    *crc = 0;
    for (i = 0; i < (datagramLength - 1); i++) {    // Execute for all bytes of a message
        currentByte = datagram[i];                  // Retrieve a byte to be sent from Array
        for (j = 0; j < 8; j++) {
            if ((*crc >> 7) ^ (currentByte & 0x01)) // update CRC based result of XOR operation
                *crc = (*crc << 1) ^ 0x07;
            else
                *crc = (*crc << 1);
            currentByte = currentByte >> 1;
        } // for CRC bit
    } // for message byte 
}

