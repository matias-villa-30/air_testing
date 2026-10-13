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

   struct Context_util {
      // MUTEX 
      k_mutex           air_mutex;  // FAKE MUTEX BEFORE REAL IMPLEMENTATION
      // BME690 
      k_work_delayable  meas_steps; // WORKQUEUE FOR HEATING GAS SENSOR
      k_work_delayable  gas_work;   // WORKQUQUE FOR FETCHING REGISTER VALUES NEEDED FOR GAS SENSOR CALIBRATION
      k_sem             sem_gas;    // SEMAPHORE FOR REGISTER FETCHING WORK 
      k_sem             sem_meas;   // SEMAPHORE FOR HEATING SENSOR WORK 
      // SCD41 
      k_work_delayable  meas_scd41; // WORKQUEUE FOR FETCHING READINGS OF CO2, TEMP AND HUMIDITY
      k_sem             sem_scd41;  // SEMAPHORRE FOR CO2, TEMP AND HUMIDITY READINGS
   };

   Context_util& util_ctx(void);
}

#endif
