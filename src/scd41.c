/*
 *  AirObserver 
 *
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
#include "scd41.h"

struct recalib_values recalibration;
struct scd41_readings enviroment_data;
const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(scd41));

bool force_scd41_recalib(void){

   // Force recalibration and put the device to sleep for 3 minutes

   recalibration.target_ppm = 430; // 430 ppm as target value for calibration
   printk("Forced recalibration about to start. Duration time: 3 minutes.\n"); 
   k_sleep(K_MINUTES(3));
   // Stop measurement before triggering forced recalibration 
   int recalib = scd4x_forced_recalibration(dev, recalibration.target_ppm, &recalibration.correction);
   
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

void fetch_scd41_readings(void){
   sensor_channel_get(dev, SENSOR_CHAN_CO2, &enviroment_data.co2);
   sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &enviroment_data.temp);
   sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &enviroment_data.humidity);

   // Print data 
   
   printk("CO2: %d ppm | Temp: %d C | Humidity: %d/100\n", enviroment_data.co2.val1, enviroment_data.temp.val1, enviroment_data.humidity.val1);
}
