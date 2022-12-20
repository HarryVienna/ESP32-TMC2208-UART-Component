#pragma once


#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/rmt.h"

#include "esp_err.h"

#include "stepper_driver.h"


// General
#define UART_MAX_DELAY    100
#define TMC2208_R_SENSE   110.0 // mOhm

#pragma pack(push, 1)

typedef uint8_t tmc2208_regaddr_t;

enum tmc2208_regaddr_t {
    TMC2208_REG_GCONF        = 0x00,
    TMC2208_REG_GSTAT        = 0x01,
    TMC2208_REG_IFCNT        = 0x02,
    TMC2208_REG_SLAVECONF    = 0x03,
    TMC2208_REG_OTP_PROG     = 0x04,
    TMC2208_REG_OTP_READ     = 0x05,
    TMC2208_REG_IOIN         = 0x06,
    TMC2208_REG_FACTORY_CONF = 0x07,

    TMC2208_REG_IHOLD_IRUN   = 0x10,
    TMC2208_REG_TPOWERDOWN   = 0x11,
    TMC2208_REG_TSTEP        = 0x12,
    TMC2208_REG_TPWMTHRS     = 0x13,
    TMC2208_REG_VACTUAL      = 0x22,

    TMC2208_REG_MSCNT        = 0x6A,
    TMC2208_REG_MSCURACT     = 0x6B,
    TMC2208_REG_CHOPCONF     = 0x6C,
    TMC2208_REG_DRV_STATUS   = 0x6F,
    TMC2208_REG_PWMCONF      = 0x70,
    TMC2208_REG_PWM_SCALE    = 0x71,
    TMC2208_REG_PWM_AUTO     = 0x72
};


typedef union {
    uint8_t value;
    struct {
        uint8_t
        idx   :7,
        write :1;
    };
} tmc2208_addr_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t
        reset_flag   :1,
        driver_error :1,
        sg2          :1,
        standstill   :1,
        unused       :4;
    };
} tmc2208_status_t;

// --- register definitions ---

// GCONF : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        I_scale_analog   :1,
        internal_Rsense  :1,
        en_spreadcycle   :1,
        shaft            :1,
        index_otpw       :1,
        index_step       :1,
        pdn_disable      :1,
        mstep_reg_select :1,
        multistep_filt   :1,
        test_mode        :1,
        reserved         :22;
    };
} tmc2208_gconf_reg_t;

// GSTAT : R+C
typedef union {
    uint32_t value;
    struct {
        uint32_t
        reset    :1,
        drv_err  :1,
        uv_cp    :1,
        reserved :29;
    };
} tmc2208_gstat_reg_t;

// IFCNT : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        count    :8,
        reserved :24;
    };
} tmc2208_ifcnt_reg_t;

// SLAVECONF : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        reserved0 :8,
        conf      :4,
        reserved1 :20;
    };
} tmc2208_slaveconf_reg_t;

// OTP_PROG : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        otpbit   :2,
        otpbyte  :2,
        otpmagic :28;
    };
} tmc2208_otp_prog_reg_t;

// OTP_READ : R
typedef union {
    uint32_t value;
    struct {   
        uint32_t
        otp_fclktrim               :5,
        otp_ottrim                 :1,
        otp_internalrsense         :1,
        otp_tbl                    :1,
        otp_pwm_grad_chopconf0_3   :4,
        otp_pwm_autograd_chopconf4 :1,
        otp_tpwmthrs_chopconf5_7   :3,
        otp_pwm_ofs_chopconf8      :1,
        otp_pwm_reg                :1,
        otp_pwm_freq               :1,
        otp_iholddelay             :2,
        otp_ihold                  :2,
        otp_en_spreadcycle         :1,
        reserved                   :8;
    };  
} tmc2208_otp_read_reg_t;

// IOIN : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        enn       :1,
        unused0   :1,
        ms1       :1,
        ms2       :1,
        diag      :1,
        unused1   :1,
        pdn_uart  :1,
        step      :1,
        sel_a     :1,
        dir       :1,
        reserved  :14,
        version   :8;
    };
} tmc2208_ioin_reg_t;

// FACTORY_CONF : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        fclktrim  :4,
        reserved1 :3,
        ottrim    :2,
        reserved :23;
    };
} tmc2208_factory_conf_reg_t;

// IHOLD_IRUN : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        ihold      :5,
        reserved1  :3,
        irun       :5,
        reserved2  :3,
        iholddelay :4,
        reserved3  :12;
    };
} tmc2208_ihold_irun_reg_t;

// TPOWERDOWN : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        tpowerdown :8,
        reserved   :24;
    };
} tmc2208_tpowerdown_reg_t;

// TSTEP : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        tstep    :20,
        reserved :12;
    };
} tmc2208_tstep_reg_t;

// TPWMTHRS : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        tpwmthrs :20,
        reserved :12;
    };
} tmc2208_tpwmthrs_reg_t;

// VACTUAL : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        actual   :24,
        reserved :8;
    };
} tmc2208_vactual_reg_t;

// MSCNT : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        mscnt    :10,
        reserved :22;
    };
} tmc2208_mscnt_reg_t;

// MSCURACT : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        cur_a     :9,
        reserved1 :7,
        cur_b     :9,
        reserved2 :7;
    };
} tmc2208_mscuract_reg_t;

// CHOPCONF : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        toff      :4,
        hstrt     :3,
        hend      :4,
        reserved0 :4,
        tbl       :2,
        vsense    :1,
        reserved1 :6,
        mres      :4,
        intpol    :1,
        dedge     :1,
        diss2g    :1,
        diss2vs   :1;
    };
} tmc2208_chopconf_reg_t;

// DRV_STATUS : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        otpw       :1,
        ot         :1,
        s2ga       :1,
        s2gb       :1,
        s2vsa      :1,
        s2vsb      :1,
        ola        :1,
        olb        :1,
        t120       :1,
        t143       :1,
        t150       :1,
        t157       :1,
        reserved1  :4,
        cs_actual  :5,
        reserved2  :3,
        reserved3  :6,
        stealth    :1,
        stst       :1;
    };
} tmc2208_drv_status_reg_t;

// PWMCONF : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        pwm_ofs       :8,
        pwm_grad      :8,
        pwm_freq      :2,
        pwm_autoscale :1,
        pwm_autograd  :1,
        freewheel     :2,
        reserved      :2,
        pwm_reg       :4,
        pwm_lim       :4;
    };
} tmc2208_pwmconf_reg_t;

// PWM_SCALE : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        pwm_scale_sum  :8,
        reserved1      :8,
        pwm_scale_auto :9, // signed 9 Bit value!
        reserved2      :7;
    };
} tmc2208_pwm_scale_reg_t;

// PWM_AUTO : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        pwm_ofs_auto  :8,
        unused0       :8,
        pwm_grad_auto :8,
        unused1       :8;
    };
} tmc2208_pwm_auto_reg_t;

// --- end of register definitions ---


// --- datagrams ---

typedef struct {
    tmc2208_addr_t addr;
    tmc2208_gconf_reg_t reg;
} tmc2208_gconf_dgr_t;

typedef struct {
    tmc2208_addr_t addr;
    tmc2208_gstat_reg_t reg;
} tmc2208_gstat_dgr_t;

typedef struct {
    tmc2208_addr_t addr;
    tmc2208_tpowerdown_reg_t reg;
} tmc2208_tpowerdown_dgr_t;

typedef struct {
    tmc2208_addr_t addr;
    tmc2208_ifcnt_reg_t reg;
} tmc2208_ifcnt_dgr_t;

typedef struct {
    tmc2208_addr_t addr;
    tmc2208_slaveconf_reg_t reg;
} tmc2208_slaveconf_dgr_t;

typedef struct {
    tmc2208_addr_t addr;
    tmc2208_otp_prog_reg_t reg;
} tmc2208_otp_prog_dgr_t;

typedef struct {
    tmc2208_addr_t addr;
    tmc2208_otp_read_reg_t reg;
} tmc2208_otp_read_dgr_t;

typedef struct {
    tmc2208_addr_t addr;
    tmc2208_ioin_reg_t reg;
} tmc2208_ioin_dgr_t;

typedef struct {
    tmc2208_addr_t addr;
    tmc2208_factory_conf_reg_t reg;
} tmc2208_factory_conf_dgr_t;

typedef struct {
    tmc2208_addr_t addr;
    tmc2208_ihold_irun_reg_t reg;
} tmc2208_ihold_irun_dgr_t;

typedef struct {
    tmc2208_addr_t addr;
    tmc2208_tstep_reg_t reg;
} tmc2208_tstep_dgr_t;

typedef struct {
    tmc2208_addr_t addr;
    tmc2208_tpwmthrs_reg_t reg;
} tmc2208_tpwmthrs_dgr_t;

typedef struct {
    tmc2208_addr_t addr;
    tmc2208_vactual_reg_t reg;
} tmc2208_vactual_dgr_t;

typedef struct {
    tmc2208_addr_t addr;
    tmc2208_mscnt_reg_t reg;
} tmc2208_mscnt_dgr_t;

typedef struct {
    tmc2208_addr_t addr;
    tmc2208_mscuract_reg_t reg;
} tmc2208_mscuract_dgr_t;

typedef struct {
    tmc2208_addr_t addr;
    tmc2208_chopconf_reg_t reg;
} tmc2208_chopconf_dgr_t;

typedef struct {
    tmc2208_addr_t addr;
    tmc2208_drv_status_reg_t reg;
} tmc2208_drv_status_dgr_t;

typedef struct {
    tmc2208_addr_t addr;
    tmc2208_pwmconf_reg_t reg;
} tmc2208_pwmconf_dgr_t;

typedef struct {
    tmc2208_addr_t addr;
    tmc2208_pwm_scale_reg_t reg;
} tmc2208_pwm_scale_dgr_t;

typedef struct {
    tmc2208_addr_t addr;
    tmc2208_pwm_auto_reg_t reg;
} tmc2208_pwm_auto_dgr_t;

// -- end of datagrams

typedef union {
    uint32_t value;
    uint8_t data[4];
    tmc2208_gconf_reg_t gconf;
    tmc2208_gstat_reg_t gstat;
    tmc2208_ifcnt_reg_t ifcnt;
    tmc2208_slaveconf_reg_t slaveconf;
    tmc2208_otp_prog_reg_t otp_prog;
    tmc2208_otp_read_reg_t otp_read;
    tmc2208_ioin_reg_t ioin;
    tmc2208_factory_conf_reg_t factory_conf;
    tmc2208_ihold_irun_reg_t ihold_irun;
    tmc2208_tpowerdown_reg_t tpowerdown;
    tmc2208_tstep_reg_t tstep;
    tmc2208_tpwmthrs_reg_t tpwmthrs;
    tmc2208_vactual_reg_t vactual;
    tmc2208_mscnt_reg_t mscnt;
    tmc2208_mscuract_reg_t mscuract;
    tmc2208_chopconf_reg_t chopconf;
    tmc2208_drv_status_reg_t drv_status;
    tmc2208_pwmconf_reg_t pwmconf;
    tmc2208_pwm_scale_reg_t pwm_scale;
    tmc2208_pwm_auto_reg_t pwm_auto_ctrl;
} tmc2208_payload_t;

typedef struct {
     tmc2208_addr_t addr;
     tmc2208_payload_t payload;
} tmc2208_datagram_t;

typedef union {
    uint8_t data[4];
    struct {
        uint8_t sync;
        uint8_t slave;
        tmc2208_addr_t addr;
        uint8_t crc;
    } msg;
} tmc2208_read_request_datagram_t;

typedef union {
    uint8_t data[8];
    struct {
        uint8_t sync;
        uint8_t slave;
        tmc2208_addr_t addr;
        tmc2208_payload_t payload;
        uint8_t crc;
    } msg;
} tmc2208_read_reply_datagram_t;

typedef union {
    uint8_t data[8];
    struct {
        uint8_t sync;
        uint8_t slave;
        tmc2208_addr_t addr;
        tmc2208_payload_t payload;
        uint8_t crc;
    } msg;
} tmc2208_write_datagram_t;








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

    rmt_channel_t channel;        /*!< RMT channel */

    gpio_num_t step_pin;          /*!< Step port number */
    gpio_num_t direction_pin;     /*!< Direction port number */
    gpio_num_t enable_pin;        /*!< Enable port number */
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
    tmc2208_pwm_auto_dgr_t pwm_auto;
} stepper_driver_tmc2208_t;

#pragma pack(pop)

/**
* @brief Install a new TMC2208 driver 
*
* @param config: step motor configuration
* @return
*      step motor instance or NULL
*/
stepper_driver_t *stepper_driver_new_tmc2208(const stepper_driver_tmc2208_conf_t *config);














