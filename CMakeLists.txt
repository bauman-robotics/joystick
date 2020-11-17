cmake_minimum_required(VERSION 3.5.1)
#project(full_featured_RC C)
project(full_featured_RC)


#set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pthread")

include_directories(.)

add_executable(full_featured_RC
        360_controller_map.h
        cobs.c
        cobs.h
        easy_serial.h
        joystick.h
        odroid_test_analog.c
        robot_defines.h
        sensors.h
        xv11.h
        xv11_1.c)
