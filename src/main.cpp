/*
 * AirObserver 
 * main.cpp
 * Created by Matias Villa
 *
 * INITIAL TESTING OF SCD41 AND BME690 CONNECTED TOGETHER IN PARALLEL ON BREADBOARD
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor/scd4x.h>
#include "bme690.hpp"
#include "task.hpp"
#include "scd41.hpp"
#include "zephyr/sys/clock.h"

#define STACKSIZE          1024
#define BME690_PRIORITY    7
#define SCD41_PRIORITY     7

#define I2C0_NODE DT_NODELABEL(bme690)

K_THREAD_DEFINE(bme690_id, STACKSIZE, AirObserver::bme690_thread, NULL, NULL, NULL, BME690_PRIORITY, 0, K_TICKS_FOREVER);

K_THREAD_DEFINE(scd41_id, STACKSIZE, AirObserver::scd41_thread, NULL, NULL, NULL, SCD41_PRIORITY, 0, K_TICKS_FOREVER);

int main(void) {

//   const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(scd41));
   auto c = BME690::ctx(); 
   auto scd = SCD41::ctx_scd41();
   printk("|BOOT|\n");

   // Make sure the driver initialized successfully on boot
   if (!device_is_ready(scd.dev) || !device_is_ready(c.bme_dev.bus)) {
      printk("Sensors not ready!\n");
      return 0;
   } else printk("Sensors ready!\n");

   k_thread_start(bme690_id);
   k_thread_start(scd41_id);
   
} 
