/*
 *  AirObserver 
 *
 *  Created by Matias Villa
 *
 *  INITIAL TESTING OF THE SCD41: CO2, TEMPERATURE AND HUMIDITY SENSOR.
*/

#ifndef SCD41_H
#define SCD41_H

#include <stdint.h>
#include <zephyr/drivers/sensor/scd4x.h>
#include <stdbool.h>

bool force_scd41_recalib(void);
void fetch_scd41_readings(void);

struct scd41_readings {
   struct sensor_value co2;
   struct sensor_value temp;
   struct sensor_value humidity;
};

struct recalib_values {
   uint16_t correction;
   uint16_t target_ppm;
};


extern const struct device *dev;
extern struct scd41_readings enviroment_data;

#endif // SCD41_H
