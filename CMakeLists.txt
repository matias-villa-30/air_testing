cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(co2_sensor)

target_sources(app PRIVATE 
   src/main.c
   src/bme690.c
   src/scd41.c
)
