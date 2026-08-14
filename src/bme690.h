/*
 *  AirObserver 
 *
 *  Created by Matias Villa
 *
 *  INITIAL TESTING OF THE BME690: GAS AND AIR PRESSURE SENSOR
*/

#ifndef BME690_H
#define BME690_H

#include <stdint.h>
#include <stdbool.h>

struct gas_info {
   uint8_t  par_g1;
   int16_t  par_g2;
   int8_t   par_g3;
   int8_t   res_heat;
   uint8_t  res_range;
   uint8_t  is_calib;
   float    gas_ohms;
};

struct gas_parsed {
   uint16_t    adc_gas;
   uint8_t     range_gas;
   uint8_t     heat_stab_reg;
};

void fetch_gas_values(void);
void calc_res_heat(void);
void start_gas_measurement(void);
void print_gas_values(void);
void read_meas_regs(void);
void soft_reset(void);
void configure_oversampling(void);
void parse_gas_readings(void);
void run_readings(void);
bool new_gas_readout(void);

#endif // BME690_H
