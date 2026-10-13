/*
 *  AirObserver 
 *  bme690.hpp
 *  Created by Matias Villa
 *
*/
#pragma once

#ifndef BME690_HPP
#define BME690_HPP

#include <stdint.h>
#include <stdbool.h>
#include "zephyr/drivers/i2c.h"
#include "zephyr/kernel.h"

namespace BME690
{

   namespace REGISTERS
   {
      // REGISTER ADDRESSES FOR CALCULATING GAS STUFF
      constexpr uint8_t PAR_G1            = 0xED; // CALIBRATION PARAMETER 
      constexpr uint8_t PAR_G2            = 0xEC; // CALIBRATION PARAMETER
      constexpr uint8_t PAR_G2_B          = 0xEC; // CALIBRATION PARAMETER
      constexpr uint8_t PAR_G3            = 0xEE; // CALIBRATION PARAMETER
      constexpr uint8_t RES_HEAT_RANGE    = 0x02; // HEATER RANGE IN BIT <5:4>
      constexpr uint8_t RES_HEAT_VALUE    = 0x00; // HEATER RESISTANCE CORRECTION FACTOR
      constexpr uint8_t AMB_TEMP          = 26;   // ESTIMATED ENVIROMENTAL TEMPERATURE
      constexpr uint8_t TARGET_TEMP       = 200;  // TEMPERATURE TO HEAT SENSOR (CELSIUS)
      constexpr uint8_t CTRL_MEAS         = 0x74; // SELECT SENSOR POWER MODE 
      constexpr uint8_t CTRL_GAS_1        = 0x71; // bit = run_gas, bit <3:0> = nb_conv 
      constexpr uint8_t RES_HEAT_0        = 0x5A; // TARGET HEATER RESISTANCE
      constexpr uint8_t GAS_WAIT_0        = 0x64; // SETS THE WAIT TIME FOR GAS MEASUREMENT IN SECONDS 
      constexpr uint8_t RESET_REG         = 0xE0; // SOFTWARE RESET REGISTER 
      constexpr uint8_t RESET_CMD         = 0xB6; // SOFTWARE RESET COMMAND 
      constexpr uint8_t CTRL_MEAS_OSRS    = 0x24; // TEMP X1, PRESSURE X1, MODE BITS UNTOUCHED HERE 
      constexpr uint8_t GAS_ADC           = 0x2D; // GAS ADC DATA REGISTER 
   }

   constexpr uint8_t I2C_RETRY         = 3;    // RETRY UPON UNSUCCESFUL I2C TRANSACTION
   constexpr uint8_t GAS_TOTAL_QUE     = 8;    // SIZE OF QUEUE 


   // GAS INFO STRUCT STORES ALL THE RAW VALUES FETCHED FROM REGISTERS
   struct gas_info {
      uint8_t  par_g1;     // STORE VALUE FROM PARTIAL GAS DATA REGISTER 1
      int16_t  par_g2;     // STORE VALUE FROM PARTIAL GAS DATA REGISTER 2
      int8_t   par_g3;     // STORE VALUE FROM PARTIAL GAS DATA REGISTER 3
      int8_t   res_heat;   // IS THE DECIMAL VALUE THAT NEEDS TO BE STORED IN REGISTER, WHERE X CORRESPONDS TO THE TEMPERATURE PROFILE NUMBER BETWEEN 0 AND 9
      uint8_t  res_range;  // HEATER RANGE STORED IN REGISTERED ADDRESS 0x02 <5:4>,
      uint8_t  is_calib;   // IF is_calib == 0 -> SENSOR NOT CALIBRATED, IF is_calib == 1 -> SENSOR IS CALIBRATED
      float    gas_ohms;   // VALUE OF GAS RESISTANCE
      uint8_t  raw_range;
      int8_t   raw_val;
   };
   // VALIDATE THAT EACH REGISTER FROM GAS_INFO GOT THE CORRECT VALUE FROM I2C TRANSACTION

   struct gas_fetched {
      uint8_t  reg;              // VALUE FROM REGISTER 
      void     *dest;            // POINTER TO VARIABLA THAT WILL HOLD THE VALUE COMING FROM THE REGISTER 
      size_t   len;              // SIZE OF DATA 
      bool     *fetched_flag;    // TRUE FOR SUCCESFUL I2C TRANSACTION
   };

   struct fetch_flag {
      // FLAGAS FOR SUCCESFUL I2C TRANSACTIONS WHEN FETCHING FROM REGISTERS BEFORE PERFORMING CALIBRATION
      bool par_g1;
      bool par_g2;
      bool par_g3;
      bool res_range;
      bool res_heat;
   };

   // GAS PARSED STORES ALL THE DATA RESULTING FROM CALCULATIONS ON THE RAW REGISTER DATA
   struct gas_parsed {
      uint16_t    adc_gas;          // RAW RESISTANCE OUTPUT DATA 
      uint8_t     range_gas;        // ADC RAMGE OF THE MEASURED GAS RESISTANCE 
      uint8_t     heat_stab_reg;    // IF HEAT_STAB_R IS ZERO, IT INDICATES THAT EITHER THE HEATING TIME WAS NOT ENOUGH TO ALLOW THE SENSOR TO REACH TO CONFIGURED TARGET TEMPERATURE OR THAT THE TARGET TEMPERATURE WAS TOO HIGH FOR THE SENSOR TO REACH.
   };

   struct gas_que_item {
      void *reserved;
      float value;
   };

   typedef enum {
      GAS_WAIT_X,
      RES_HEAT_X,
      RUN_GAS,
      NB_CONV,
   } measurement_steps;
   
   struct Context {
      i2c_dt_spec    bme_dev;       // I2C 
      gas_info       gas_calib;     // CALIBRATION VALUES 
      gas_parsed     curated_gas;   // VALUES AFTER CALCULATIONS 
      k_queue        gas_queue;     // QUUEUE FOR GAS RESISTANCE READINGS 
      gas_que_item   gas_pool[BME690::GAS_TOTAL_QUE];
   };
 
   Context& ctx(void);

   void fetch_gas_values(void);           // FETCH VALUES FROM REGISTERS TO START CALCULATIOS AND CONVERSIONS FOR MEASURING GAS IN ENVIROMENT
   void calc_res_heat(void);              // CALCULATE HEAT RESISTANCE 
   void start_gas_measurement(void);      // PERFORM GAS MEASUREMENTS
   void soft_reset(void);                 // PERFORM A SOFT RESET, HAS THE SAME EFFECT AS POWER-ON RESET
   void configure_oversampling(void);     // READS MULTIPLE SAMPLES AND AVERAGES THEM TO IMPROVE MEASUREMENT STABILITY. 
   void parse_gas_readings(void);         // CONVERT RAW ADC GAS DATA TO OHMS
   void run_bme690_readings(void);        // READ RESISTANCE DATA ALREADY CONVERTED TO OHMS 
   void fetch_work_handler(struct k_work *work);
   void measurement_work_handler(struct k_work *measurements);
   bool new_gas_readout(void);            // CHECK IF THERE IS A NEW VALID READ
   void gas_sample_push(float gas_item);
   float avg_gas_measured(struct k_queue *gas_queue);

}

#endif // BME690_HPP
