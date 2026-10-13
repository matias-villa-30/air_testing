/*
 *  AirObserver 
 *  scd41.hpp  
 *  Created by Matias Villa
 *
 *  INITIAL TESTING OF THE SCD41: CO2, TEMPERATURE AND HUMIDITY SENSOR.
*/

#pragma once

#ifndef SCD41_HPP
#define SCD41_HPP

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/sensor/scd4x.h>

namespace SCD41
{
   constexpr uint8_t RETRY_I2C   = 3;
   constexpr uint8_t QUE_SIZE    = 8;

   struct scd41_readings {
      struct sensor_value co2;
      struct sensor_value temp;
      struct sensor_value humidity;
   };

   struct recalib_values {
      uint16_t correction;
      uint16_t target_ppm;
   };

   struct que_item {
      void *reserved;
      int value;
   };
     
   typedef enum {
      CO2,
      TEMP,
      HUMIDITY
   } step_readings;

   struct Context_scd41 {
      const struct device *dev;
      scd41_readings enviroment_data;
      // queues 
      k_queue co2_queue;
      k_queue temp_queue;
      k_queue humid_queue;
      // item queues 
      que_item sample_pool[QUE_SIZE];
      que_item co2_pool[QUE_SIZE];
      que_item temp_pool[QUE_SIZE];
      que_item humid_pool[QUE_SIZE];
   };

   Context_scd41& ctx_scd41(void);
   bool  force_scd41_recalib(void);
   void  fetch_scd41_readings(struct k_work *read_scd41);
   void  run_scd41_readings(void);
   void  queue_sample_push(int item_queue, struct que_item *pool, uint8_t *idx, struct k_queue *que);
   float avg_measured(struct k_queue *que);


}   


#endif // SCD41_H
