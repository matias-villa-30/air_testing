/*    AirObserver
 *    util.hpp
 *    Matias Villa
 */
#pragma once

#ifndef UTIL_HPP
#define UTIL_HPP

#include <zephyr/kernel.h>
#include <stdint.h>


namespace UTIL {
   
   constexpr uint8_t I2C_WAIT_MS =  5;
   // MUTEX
   extern struct k_mutex            air_mutex;
   // BME690
   extern struct k_work_delayable   gas_work;
   extern struct k_work_delayable   meas_steps;
   extern struct k_sem              sem_gas;
   extern struct k_sem              sem_meas;
   // SCD41
   extern struct k_work_delayable   meas_scd41;
   extern struct k_sem              sem_scd41;
}

#endif
