cmake_minimum_required(VERSION 2.6)

project(Estudo)

#Make possible debug with GDB :) 
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

add_executable(main ${CMAKE_CURRENT_SOURCE_DIR}/main.c ${CMAKE_CURRENT_SOURCE_DIR}/gpio_api.c) 
