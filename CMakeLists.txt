# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16.0)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
list(APPEND EXTRA_COMPONENT_DIRS "components/hall_sensor" "components/step_motor" "components/stepper_driver" "components/gps_modul" "components/tz_database" "components/wifi")
project(Zyklochron)


