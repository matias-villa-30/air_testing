/*
 * AirObserver
 * bme690.cpp
 * Created by Matias Villa
 *
 */

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include "bme690.hpp"
#include "util.hpp"

#define I2C0_NODE DT_NODELABEL(bme690)

namespace BME690
{

   measurement_steps          current_meas_step;      // mini statemachine for heating sensor 
   struct fetch_flag          gas_was_fetched;        // bool flags to verify succesful i2c transactions

   static uint8_t             gas_pool_idx;           // index for elements in gas queue 
   static size_t              step_idx         = 0;   // index for elements in the registers array
   static int                 retry_count      = 0;   // index for i2c retries when transaction fails on fetch work handler 
   static int                 meas_retry_count = 0;   // index for i2c retries when transaction fails on measurement work handler

   struct Context bme_ctx{
      .bme_dev = I2C_DT_SPEC_GET(I2C0_NODE),
   };
   
   Context& ctx() {return bme_ctx;}

   void fetch_work_handler(struct k_work *work) {
      // FETCH REGISTERS VALUES NECESSARY TO PERFORM SENSOR HEATING AND CALIBRATION
      auto &c = ctx();
      auto &c_util = UTIL::util_ctx();

      struct gas_fetched steps[] = {
         { BME690_REGISTERS::PAR_G1,         &c.gas_calib.par_g1,    sizeof(c.gas_calib.par_g1),    &gas_was_fetched.par_g1    },
         { BME690_REGISTERS::PAR_G2,         &c.gas_calib.par_g2,    sizeof(c.gas_calib.par_g2),    &gas_was_fetched.par_g2    },
         { BME690_REGISTERS::PAR_G3,         &c.gas_calib.par_g3,    sizeof(c.gas_calib.par_g3),    &gas_was_fetched.par_g3    },
         { BME690_REGISTERS::RES_HEAT_RANGE, &c.gas_calib.raw_range, sizeof(c.gas_calib.raw_range), &gas_was_fetched.res_range },
         { BME690_REGISTERS::RES_HEAT_VALUE, &c.gas_calib.raw_val,   sizeof(c.gas_calib.raw_val),   &gas_was_fetched.res_heat  },
      };

      struct gas_fetched *s = &steps[step_idx];

      int rc = i2c_write_read_dt(&c.bme_dev, &s->reg, 1, s->dest, s->len);

      if (rc == 0) {
         *s->fetched_flag = true;
         step_idx++;
         retry_count = 0;
      } else {
         printk("I2C error on step %u (reg 0x%02x): [%d]\n", step_idx, s->reg, rc);
         retry_count++;
         if (retry_count >= BME690_REGISTERS::I2C_RETRY) {
            *s->fetched_flag = false;   // give up on this one, move on so we don't hang forever
            step_idx++;
            retry_count = 0;
         }
         // else: step_idx unchanged -> same step gets retried on next schedule
      }

      if (step_idx < ARRAY_SIZE(steps)) {
         k_work_schedule(&c_util.gas_work, K_MSEC(UTIL::I2C_WAIT_MS));
      } else {
         c.gas_calib.res_range = (c.gas_calib.raw_range >> 4) & 0x03;   // bits <5:4>
         c.gas_calib.res_heat  = c.gas_calib.raw_val;
         step_idx = 0;   // reset for next fetch cycle
         k_sem_give(&c_util.sem_gas);
      }
   }

   void configure_oversampling(void) {
      auto &c = ctx();
      // configure sensor oversampling rate for gas sensor
      uint8_t ctrl_meas_val[2] = {BME690_REGISTERS::CTRL_MEAS, BME690_REGISTERS::CTRL_MEAS_OSRS};
      i2c_write_dt(&c.bme_dev, ctrl_meas_val, sizeof(ctrl_meas_val));
   }

   void soft_reset(void) {
      auto &c = ctx();
      // PERORM A SOFTWARE RESET
      uint8_t buf[2] = {BME690_REGISTERS::RESET_REG, BME690_REGISTERS::RESET_CMD};
      i2c_write_dt(&c.bme_dev, buf, sizeof(buf));
   }

   void calc_res_heat(void) {
      // Formula provided by Bosch in the datasheet
      // Calculate the values necessary for sensor calibration
      // Heat resistance calculation
      auto &c = ctx();
      double var1       = ((double)c.gas_calib.par_g1 / 16.0) + 49.0;
      double var2       = (((double)c.gas_calib.par_g2 / 32768.0) * 0.0005) + 0.00235;
      double var3       = (double)c.gas_calib.par_g3 / 1024.0;
      double var4       = var1 * (1.0 + (var2 * (double)BME690_REGISTERS::TARGET_TEMP));
      double var5       = var4 + (var3 * (double)BME690_REGISTERS::AMB_TEMP);
      double res_heat_x = (uint8_t)(3.4 * ((var5 * (4.0 / (4.0 + (double)c.gas_calib.res_range))) *
                           (1.0 / (1.0 + (double)c.gas_calib.res_heat * 0.002))) - 25);
      // sensor is calibrated
      c.gas_calib.is_calib = res_heat_x;
   }

   void measurement_work_handler(struct k_work *measurements) {
      // HEAT GAS SENSOR
      auto &c = ctx();
      auto &util_c = UTIL::util_ctx();

      uint8_t gas_wait_0[2] = {BME690_REGISTERS::GAS_WAIT_0, 0x59};
      uint8_t res_heat_0[2] = {BME690_REGISTERS::RES_HEAT_0, (uint8_t)c.gas_calib.is_calib};
      uint8_t ctrl_gas_1    = BME690_REGISTERS::CTRL_GAS_1;
      uint8_t ctrl_meas     = BME690_REGISTERS::CTRL_MEAS;
      uint8_t run_gas;
      uint8_t set_mode;
      int rc = 0;

      switch (current_meas_step) {

      case GAS_WAIT_X:
         rc = i2c_write_dt(&c.bme_dev, gas_wait_0, sizeof(gas_wait_0));
         if (rc != 0) {
            printk("Error writing gas_wait_0: [%d]\n", rc);
            if (++meas_retry_count < BME690_REGISTERS::I2C_RETRY) {
               k_work_schedule(&util_c.meas_steps, K_MSEC(UTIL::I2C_WAIT_MS));
               return;   // retry same step
            }
            printk("Giving up on gas_wait_0 after %d retries\n", BME690_REGISTERS::I2C_RETRY);
         }
         meas_retry_count  = 0;
         current_meas_step = RES_HEAT_X;
         k_work_schedule(&util_c.meas_steps, K_MSEC(UTIL::I2C_WAIT_MS));
         break;

      case RES_HEAT_X:
         rc = i2c_write_dt(&c.bme_dev, res_heat_0, sizeof(res_heat_0));
         if (rc != 0) {
            printk("Error writing res_heat_0: [%d]\n", rc);
            if (++meas_retry_count < BME690_REGISTERS::I2C_RETRY) {
               k_work_schedule(&util_c.meas_steps, K_MSEC(UTIL::I2C_WAIT_MS));
               return;
            }
            printk("Giving up on res_heat_0 after %d retries\n", BME690_REGISTERS::I2C_RETRY);
         }
         meas_retry_count  = 0;
         current_meas_step = RUN_GAS;
         k_work_schedule(&util_c.meas_steps, K_MSEC(UTIL::I2C_WAIT_MS));
         break;

      case RUN_GAS:
         rc = i2c_write_read_dt(&c.bme_dev, &ctrl_gas_1, 1, &run_gas, 1);
         if (rc != 0) {
            printk("Error reading ctrl_gas_1: [%d]\n", rc);
            if (++meas_retry_count < BME690_REGISTERS::I2C_RETRY) {
               k_work_schedule(&util_c.meas_steps, K_MSEC(UTIL::I2C_WAIT_MS));
               return;
            }
            printk("Giving up on ctrl_gas_1 after %d retries\n", BME690_REGISTERS::I2C_RETRY);
            meas_retry_count  = 0;
            current_meas_step = NB_CONV;   // don't hang forever, but run_gas bit is unset
            k_work_schedule(&util_c.meas_steps, K_MSEC(UTIL::I2C_WAIT_MS));
            break;
         }
         run_gas &= ~0x0F;     // clear nb_conv index to 0
         run_gas |= (1 << 5);  // set run_gas bit
         {
            uint8_t buf[2] = {BME690_REGISTERS::CTRL_GAS_1, run_gas};
            i2c_write_dt(&c.bme_dev, buf, sizeof(buf));
         }
         meas_retry_count  = 0;
         current_meas_step = NB_CONV;
         k_work_schedule(&util_c.meas_steps, K_MSEC(UTIL::I2C_WAIT_MS));
         break;

      case NB_CONV:
         rc = i2c_write_read_dt(&c.bme_dev, &ctrl_meas, 1, &set_mode, 1);
         if (rc != 0) {
            printk("Error reading ctrl_meas register: [%d]\n", rc);
            if (++meas_retry_count < BME690_REGISTERS::I2C_RETRY) {
               k_work_schedule(&util_c.meas_steps, K_MSEC(UTIL::I2C_WAIT_MS));
               return;
            }
            printk("Giving up on ctrl_meas after %d retries\n", BME690_REGISTERS::I2C_RETRY);
            meas_retry_count = 0;
            k_sem_give(&util_c.sem_meas);   // don't leave bme690_thread blocked forever
            break;
         }
         set_mode &= ~0x03;   // Clear bits 1:0
         set_mode |= 0x01;    // Set Forced Mode (0b01)
         {
            uint8_t buf2[2] = {BME690_REGISTERS::CTRL_MEAS, set_mode};
            i2c_write_dt(&c.bme_dev, buf2, sizeof(buf2));
         }
         meas_retry_count = 0;
         k_sem_give(&util_c.sem_meas);
         break;
      }
   }

   bool new_gas_readout(void) {
      // IF HEAT_STAB_R IS EQUAL TO 1, SENSOR IS READY TO MEASURE GAS IN ENVIROEMENT
      auto &c = ctx();
      uint8_t gas_r_lsb = 0x2D;
      uint8_t heat_stab_r;

      int rc = i2c_write_read_dt(&c.bme_dev, &gas_r_lsb, 1, &heat_stab_r, 1);

      if (rc != 0) {
         printk("Error writing to register\n");
         return false;
      } else {
         if ((heat_stab_r & 16) != 16) {
            // CHECK IF VALUE OF BIT NUMBER 4 IS 1, IF 0 THERE IS NO VALID DATA.
            printk("There is no valid data. Value of bit 4 is: 0\n");
            return false;
         }
         //printk("Sensor ready to perform readings!\n");
         c.curated_gas.heat_stab_reg = heat_stab_r;
         return true;
      }
   }

   void parse_gas_readings(void) {
      // CONVERT GAS RESISTANCE MEASUREMENT TO OHMS.
      // ELI5 EXPLANATION: THE GREATER THE RESISTANCE, CLEANER THE AIR.
      // YET TO IMPLEMENT A MORE INTUITIVE READING OF GAS IN ENVIROEMENT.
      auto &c = ctx();

      uint8_t  msb_reg = 0x2C;
      uint8_t  lsb_reg = 0x2D;
      uint8_t  gas_r_msb;
      uint8_t  gas_r_lsb;
      uint16_t gas_adc;
      uint8_t  gas_range;

      int err = i2c_write_read_dt(&c.bme_dev, &msb_reg, 1, &gas_r_msb, 1);
      if (err != 0) printk("Error reading gas_r_msb\n");

      err = i2c_write_read_dt(&c.bme_dev, &lsb_reg, 1, &gas_r_lsb, 1);
      if (err != 0) printk("Error reading gas_r_lsb\n");

      gas_adc   = ((uint16_t)gas_r_msb << 2) | (gas_r_lsb >> 6);   // "Lives" on bit <7:6>
      gas_range = gas_r_lsb & 0x0F;                                // "Lives" on bits <3:0>

      c.curated_gas.adc_gas   = gas_adc;
      c.curated_gas.range_gas = gas_range;

      // FORMULA PROVIDED BY BOSCH DATASHEET ->
      // CONVERT ANALOG READS INTO OHMS
      uint32_t var1 = UINT32_C(262144) >> gas_range;
      int32_t  var2 = (int32_t)gas_adc - INT32_C(512);
      var2 *= INT32_C(3);
      var2 = INT32_C(4096) + var2;
      float gas_res = 1000000.0f * (float)var1 / (float)var2;
      c.gas_calib.gas_ohms = gas_res;
   }
   
   void gas_sample_push(float value) {
      // PUSH GAS RESISTANCE FUNCTION TO ITS QUEUE 
      auto &c = ctx(); 
      auto *item = &c.gas_pool[gas_pool_idx];
      gas_pool_idx = (gas_pool_idx + 1) % BME690_REGISTERS::GAS_TOTAL_QUE;

       item->value = value;
       k_queue_append(&c.gas_queue, item);
   }

   float avg_gas_measured(struct k_queue *gas_que) {
      // RETURNS THE AVERAGE OF 8 SAMPLES 
      float sum = 0.0f;
      int count = 0;

      while (!k_queue_is_empty(gas_que)) {
         void *data = k_queue_get(gas_que, K_MSEC(250));
         if (data == NULL) {
            break;
         }
         auto *item = static_cast<gas_que_item *>(data);
         sum += item->value;
         count++;
      }

      return (count > 0) ? (sum / count) : 0.0f;
   }
   
   void run_bme690_readings(void) {
      // EXPERIMENTAL GAS SENSOR SEQUENCE (FIELD 0)
      current_meas_step = GAS_WAIT_X;

      auto &ctx_util = UTIL::util_ctx();
      
      // SCHEDULE WORKS
      k_work_schedule(&ctx_util.gas_work, K_NO_WAIT);
      k_work_schedule(&ctx_util.meas_steps, K_NO_WAIT);
      
      k_sem_take(&ctx_util.sem_gas, K_FOREVER);
      k_sem_take(&ctx_util.sem_meas, K_FOREVER);

      calc_res_heat();

      configure_oversampling();
   }
     
   
}
