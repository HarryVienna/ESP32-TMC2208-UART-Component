    # components/stepper_driver/CMakeLists.txt

    idf_component_register(SRCS "stepper_driver.c" "stepper_driver_tmc2208.c"
                        INCLUDE_DIRS "include"
                        REQUIRES driver)
					   
