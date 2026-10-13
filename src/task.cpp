/*    AirObserver
 *    task.cpp
 *    Matias Villa
 *
 */

#include <zephyr/kernel.h>
#include "scd41.hpp"
#include "bme690.hpp"
#include "task.hpp"
#include "util.hpp"

namespace AirObserver {
   
   void bme690_thread(void){
      // THREAD FOR GAS SENSOR AKA TASK IN FREERTOS
      auto &c = BME690::ctx(); // INITIALIZE REFERENCE TO CONTEXT OBJECT 
      auto &util_context = UTIL::util_ctx(); // INITIALIZE REFERENCE TO UTIL CONTEXT OBJECT
      uint8_t avg_index = 0;   // INDEX FOR TAKING THE AVERAGE OF THE SAMPLES
      
      // WORKQUEUE
      k_work_init_delayable(&util_context.gas_work, BME690::fetch_work_handler);
      k_work_init_delayable(&util_context.meas_steps, BME690::measurement_work_handler);
      // INITITALIZE QUEUE 
      k_queue_init(&c.gas_queue);              
      // SEMAPHORES
      k_sem_init(&util_context.sem_gas, 0, 1);  // INITIALIZE SEMAPHORE FOR REGISTER FETCHING 
      k_sem_init(&util_context.sem_meas, 0, 1); // INITIALIZE SEMAPHORE FOR HEATING SENSOR STEPS
      
      while (1){
         
         k_mutex_lock(&UTIL::air_mutex, K_FOREVER); // lock the mutex to perform reaings and take the average of the samples

         BME690::run_bme690_readings();

         bool is_ready = BME690::new_gas_readout(); // check that there is a valid gas reading
         
         if (is_ready) {
            BME690::parse_gas_readings();
            BME690::gas_sample_push(c.gas_calib.gas_ohms);
            avg_index++;
            if (avg_index == BME690::GAS_TOTAL_QUE){
               // PRINT THE RESISTANCE MEASURED TO CONSOLE AND RESET THE INDEX
               float avg = BME690::avg_gas_measured(&c.gas_queue);
               printk("Gas resistance level is: [%.3f] ohms\n", (double)avg);
               avg_index = 0;
            }
         } 
         else printk("Measurement not ready this cycle, skipping.\n");

         k_mutex_unlock(&UTIL::air_mutex); // UNLOCK MUTEX FOR SCD41 SENSOR
      }

   }

   void scd41_thread(void){
      // THREAD FOR CO2, TEMP AND HUMIDITY SENSOR AKA TASK IN FREERTOS 
      auto &c = SCD41::ctx_scd41(); // INITIALIZE REFERENCE TO CONTEXT OBJECT
      
      auto &util_context = UTIL::util_ctx(); // INITIALIZE REFERENCE TO UTIL CONTEXT OBJECT
      // WORKQUEUE 
      k_work_init_delayable(&util_context.meas_scd41, SCD41::fetch_scd41_readings);
      // SEMAPHORE 
      k_sem_init(&util_context.sem_scd41, 0, 1);
      // INITIALIZE ALL QUEUES 
      k_queue_init(&c.co2_queue);
      k_queue_init(&c.temp_queue);
      k_queue_init(&c.humid_queue);
      
      // INDEXES FOR QUEUES 
      uint8_t tmp_idx      = 0;
      uint8_t co2_idx      = 0;
      uint8_t hmd_idx      = 0;
      uint8_t sample_count = 0;

      while (1){
     
         k_mutex_lock(&UTIL::air_mutex, K_FOREVER); // LOCK THE MUTEX WHILE PERFORMING READINGS

         if (sensor_sample_fetch(c.dev) < 0) {
            printk("Failed to fetch sample\n");
            k_mutex_unlock(&UTIL::air_mutex); // UNLOCK MUTEX FOR BME690 SENSOR
            continue;
         }
         
         // RUN READINGS AND PUSH THEM TO ITS OWN QUEUE 
         SCD41::run_scd41_readings();
         SCD41::queue_sample_push(c.enviroment_data.co2.val1,      c.co2_pool,   &co2_idx, &c.co2_queue);
         SCD41::queue_sample_push(c.enviroment_data.temp.val1,     c.temp_pool,  &tmp_idx, &c.temp_queue);
         SCD41::queue_sample_push(c.enviroment_data.humidity.val1, c.humid_pool, &hmd_idx, &c.humid_queue);
         
         sample_count++;

         if (sample_count == SCD41::QUE_SIZE){
            // PRINT READINGS TO CONSOLE AND RESET SAMPLE COUNT INDEX 
            float co2_avg     = SCD41::avg_measured(&c.co2_queue);
            float temp_avg    = SCD41::avg_measured(&c.temp_queue);
            float humid_avg   = SCD41::avg_measured(&c.humid_queue);

            printk("CO2: %.2f ppm | Temp: %.2f C | Humidity: %.2f/100\n", co2_avg, temp_avg, humid_avg);
            
            sample_count = 0;
         }
         
         k_mutex_unlock(&UTIL::air_mutex); // UNLOCK MUTEX FOR BME690 SENSOR
      }
   }
 
}
