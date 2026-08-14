/*
 * AirObserver 
 *
 * Created by Matias Villa
 *
 * INITIAL TESTING OF SCD41 AND BME690 CONNECTED TOGETHER IN PARALLEL ON BREADBOARD
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
#include "bme690.h"
#include "scd41.h"

#define I2C0_NODE DT_NODELABEL(bme690)

const struct i2c_dt_spec bme_dev = I2C_DT_SPEC_GET(I2C0_NODE);
struct gas_info gas_calib;
struct gas_parsed curated_gas;

int main(void) {

   const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(scd41));
   
   k_sleep(K_SECONDS(2));
   printk("|BOOT|\n");

   // Make sure the driver initialized successfully on boot
   if (!device_is_ready(dev) || !device_is_ready(bme_dev.bus)) {
      printk("Sensors not ready!\n");
      return 0;
   } else printk("Sensors ready!\n");
   
   k_sleep(K_MSEC(200)); // Give some time to initialize the next sensor
  
   run_readings();

   while (true){
      printk("Performing gas resistance measurement...\n");

      start_gas_measurement();       // trigger a NEW measurement each cycle
      k_sleep(K_MSEC(1000));         // give heater + conversion time to complete

      bool is_ready = new_gas_readout();
      if (is_ready) {
         parse_gas_readings();
         printk("Gas resistance level is: [%.3f] ohms\n", (double)gas_calib.gas_ohms);
      } else printk("Measurement not ready this cycle, skipping.\n");
      

      k_sleep(K_MSEC(2000));        // spacing between measurement cycles
      if (sensor_sample_fetch(dev) < 0) {
         printk("Failed to fetch sample\n");
         continue;
      }

      fetch_scd41_readings();       
      // Measure every 5 seconds.
      k_sleep(K_MSEC(5000));
   }
} 
