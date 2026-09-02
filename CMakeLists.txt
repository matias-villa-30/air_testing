cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(co2_sensor)

target_sources(app PRIVATE 
   src/main.cpp
   src/bme690.cpp
   src/scd41.cpp
   src/task.cpp
   src/util.cpp
)
