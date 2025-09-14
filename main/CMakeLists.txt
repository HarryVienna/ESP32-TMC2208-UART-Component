# main/CMakeLists.txt

idf_component_register(SRCS "main.c" "stepper_task.c"
                       INCLUDE_DIRS "."
                       REQUIRES stepper_driver)