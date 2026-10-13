/*
 *  AirObserver 
 *  scd41.cpp
 *  Created by Matias Villa
 *
 *  INITIAL TESTING OF THE SCD41: CO2, TEMPERATURE AND HUMIDITY SENSOR.
*/

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor/scd4x.h>
#include "scd41.hpp"
#include "util.hpp"


namespace SCD41
{

   struct recalib_values   recalibration;
   step_readings           scd41_steps;

   struct Context_scd41 scd_ctx {
      .dev = DEVICE_DT_GET(DT_NODELABEL(scd41))
   };
   // FUNCTIONS
   Context_scd41& ctx_scd41(void) {return scd_ctx;}
   
   void queue_sample_push(int item_queue, struct que_item *pool, uint8_t *idx, struct k_queue *que){
      // PUSH SAMPLE MEASURED TO QUEUE 
      struct que_item *item = &pool[*idx];
      *idx = (*idx + 1) % QUE_SIZE; // MAKE SURE THAT ONLY 8 SAMPLES ARE PUSHED TO THE QUEUE 

      item->value = item_queue;
      k_queue_append(que, item);
   }
   
   float avg_measured(struct k_queue *que) {
      // RETURNS THE AVERAGE OF 8 SAMPLES
      float sum = 0.0f;
      int count = 0;

      while (!k_queue_is_empty(que)) {
         void *data = k_queue_get(que, K_MSEC(UTIL::QUE_DELAY));
         if (data == NULL) {
            break;
         }
//       struct que_item *item = (struct que_item *)data;
         auto *item = static_cast<que_item *>(data);
         sum += item->value;
         count++;
      }

      return (count > 0) ? (sum / count) : 0.0f;
   }

   bool force_scd41_recalib(void){

      // Force recalibration and put the device to sleep for 3 minutes
      auto &c = ctx_scd41();
      recalibration.target_ppm = 430; // 430 ppm as target value for calibration
      printk("Forced recalibration about to start. Duration time: 3 minutes.\n"); 
      k_sleep(K_MINUTES(3));
      // Stop measurement before triggering forced recalibration 
      int recalib = scd4x_forced_recalibration(c.dev, recalibration.target_ppm, &recalibration.correction);
      
      if (recalib == 0){
         printk("Calibration succesfull\n");
         k_sleep(K_MSEC(400)); // wait 400 miliseconds before
         return true;
      }
      else {
         printk("Forced calibration failed. Error code [%d]\n", recalib);
         return false;
      }
   }

   void fetch_scd41_readings(struct k_work *read_scd41){
      // GET THE SAMPLES FROM THE SENSOR
      auto &c = ctx_scd41();
      auto &util_c = UTIL::util_ctx();
      switch (scd41_steps){
         case CO2:
            sensor_channel_get(c.dev, SENSOR_CHAN_CO2, &c.enviroment_data.co2);
            scd41_steps = TEMP;
            k_work_schedule(&util_c.meas_scd41, K_MSEC(UTIL::I2C_WAIT_MS));
            break;

         case TEMP:
            sensor_channel_get(c.dev, SENSOR_CHAN_AMBIENT_TEMP, &c.enviroment_data.temp);
            scd41_steps = HUMIDITY;
            k_work_schedule(&util_c.meas_scd41, K_MSEC(UTIL::I2C_WAIT_MS));
            break;

         case HUMIDITY:
            sensor_channel_get(c.dev, SENSOR_CHAN_HUMIDITY, &c.enviroment_data.humidity);
            k_sem_give(&util_c.sem_scd41);
            break;
      }
   }

   void run_scd41_readings(void){
      auto &util_c = UTIL::util_ctx();
      scd41_steps = CO2;
      k_work_schedule(&util_c.meas_scd41, K_NO_WAIT);
      k_sem_take(&util_c.sem_scd41, K_FOREVER);

   }
   
}
