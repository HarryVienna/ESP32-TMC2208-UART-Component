set(component_srcs "src/stepper_driver.c"
                   "src/stepper_driver_tmc2208.c"
)

idf_component_register(SRCS "${component_srcs}"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS ""
                       PRIV_REQUIRES "driver"
                       REQUIRES "")
