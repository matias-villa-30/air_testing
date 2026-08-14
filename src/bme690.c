/*
 * AirObserver 
 *
 * Created by Matias Villa
 *
 * INITIAL TESTING OF THE BOSCH BME690 -> ONLY USING GAS SENSORS.
 */

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include "bme690.h"

#define I2C0_NODE DT_NODELABEL(bme690)
// REGISTER ADDRESSES FOR CALCULATING GAS STUFF
#define     PAR_G1          0xED
#define     PAR_G2          0xEB
#define     PAR_G2_B        0xEC
#define     PAR_G3          0xEE
#define     RES_HEAT_RANGE  0x02
#define     RES_HEAT_VALUE  0x00
#define     AMB_TEMP        26
#define     TARGET_TEMP     200  
#define     CTRL_MEAS       0x74
#define     CTRL_GAS_1      0x71 // bit 5 = run_gas, bit <3:0> nb_conv 
#define     RES_HEAT_0      0x5a                                   
#define     GAS_WAIT_0      0x64  
#define     RESET_REG       0xE0
#define     RESET_CMD       0xB6
#define     CTRL_MEAS_OSRS  0x24  // temp x1, pressure x1, mode bits untouched here
#define     GAS_ADC         0x2D

extern const struct i2c_dt_spec           bme_dev;
extern struct gas_info                    gas_calib;
extern struct gas_parsed                  curated_gas;

void configure_oversampling(void){
   uint8_t ctrl_meas_val[2] = {CTRL_MEAS, CTRL_MEAS_OSRS};
   i2c_write_dt(&bme_dev, ctrl_meas_val, sizeof(ctrl_meas_val));
   k_sleep(K_MSEC(10));
}

void soft_reset(void){
   uint8_t buf[2] = {RESET_REG, RESET_CMD};
   i2c_write_dt(&bme_dev, buf, sizeof(buf));
   k_sleep(K_MSEC(100)); // datasheet: allow ~2ms, use 100ms super margin
}

void fetch_gas_values(void){
   // Fetch register values from registers and save to struct before performing heating of gas sensor
   uint8_t reg_1     =     PAR_G1;
   uint8_t reg_2     =     PAR_G2;
   uint8_t reg_3     =     PAR_G3;
   uint8_t range_reg =     RES_HEAT_RANGE;
   uint8_t val_reg   =     RES_HEAT_VALUE;
   uint8_t raw_range;
   int8_t  raw_val;

   i2c_write_read_dt(&bme_dev, &reg_1, 1, &gas_calib.par_g1, sizeof(gas_calib.par_g1));
   k_sleep(K_MSEC(50));
   i2c_write_read_dt(&bme_dev, &reg_2, 1, &gas_calib.par_g2, sizeof(gas_calib.par_g2));
   k_sleep(K_MSEC(50));
   i2c_write_read_dt(&bme_dev, &reg_3, 1, &gas_calib.par_g3, sizeof(gas_calib.par_g3));
   k_sleep(K_MSEC(50));

   i2c_write_read_dt(&bme_dev, &range_reg, 1, &raw_range, 1);
   gas_calib.res_range = (raw_range >> 4) & 0x03;   // res_heat_range bits <5:4>

   k_sleep(K_MSEC(50));
   i2c_write_read_dt(&bme_dev, &val_reg, 1, (uint8_t *)&raw_val, 1);
   gas_calib.res_heat = raw_val;                     
}

void calc_res_heat(void){
   // Formula provided by Bosch in the datasheet
   // Calculate the values necessary for sensor calibration
   double var1 = ((double)gas_calib.par_g1 / 16.0) + 49.0;
   double var2 = (((double)gas_calib.par_g2 / 32768.0) * 0.0005) + 0.00235;
   double var3 = (double)gas_calib.par_g3 / 1024.0;
   double var4 = var1 * (1.0 + (var2 * (double)TARGET_TEMP)); 
   double var5 = var4 + (var3 * (double)AMB_TEMP);
   double res_heat_x = (uint8_t)(3.4 * ((var5 * (4.0 / (4.0 + (double)gas_calib.res_range))) * (1.0 / (1.0 + (double)gas_calib.res_heat * 0.002))) - 25);
   // sensor is calibrated
   gas_calib.is_calib = res_heat_x;
}

void start_gas_measurement(void){
    uint8_t gas_wait_0[2] = {GAS_WAIT_0, 0x59};
    uint8_t res_heat_0[2] = {RES_HEAT_0, (uint8_t)gas_calib.is_calib};
    uint8_t ctrl_gas_1 = CTRL_GAS_1;
    uint8_t ctrl_meas = CTRL_MEAS;
    uint8_t run_gas;
    uint8_t set_mode;
    
    // 1) Set gas_wait_0 duration
    int err = i2c_write_dt(&bme_dev, gas_wait_0, sizeof(gas_wait_0));
    if (err != 0) printk("Error writing gas_wait_0\n");
    
    k_sleep(K_MSEC(10));

    // 2) Write target heater resistance to res_heat_0
    err = i2c_write_dt(&bme_dev, res_heat_0, sizeof(res_heat_0));
    if (err != 0) printk("Error writing res_heat_0\n");
    
    k_sleep(K_MSEC(10));

    // 3 & 4) Enable gas measurement bit (run_gas)
    err = i2c_write_read_dt(&bme_dev, &ctrl_gas_1, 1, &run_gas, 1);
    if (err != 0) {
        printk("Error reading ctrl_gas_1\n");
    } else {
        run_gas &= ~0x0F;     // clear nb_conv index to 0
        run_gas |= (1 << 5);  // set run_gas bit
        uint8_t buf[2] = {CTRL_GAS_1, run_gas};
        i2c_write_dt(&bme_dev, buf, sizeof(buf));
    }

    k_sleep(K_MSEC(10));

    // 5) Set mode <1:0> to 0b01 to trigger Forced Mode measurement
    err = i2c_write_read_dt(&bme_dev, &ctrl_meas, 1, &set_mode, 1);
    if (err != 0) {
        printk("Error reading ctrl_meas\n");
    } else {
        set_mode &= ~0x03; // Clear bits 1:0
        set_mode |= 0x01;  // Set Forced Mode (0b01)

        uint8_t buf2[2] = {CTRL_MEAS, set_mode};
        i2c_write_dt(&bme_dev, buf2, sizeof(buf2));
    }
}

void print_gas_values(){
   printk("*---CHECK VALUES FETCHED FROM REGISTERS---*\n");
   printk("PAR_G1:            [%d]\n", gas_calib.par_g1);
   printk("PAR_G2:            [%d]\n", gas_calib.par_g2);
   printk("PAR_G3:            [%d]\n", gas_calib.par_g3);
   printk("HEAT RESISTANCE:   [%d]\n", gas_calib.res_heat);
   printk("RESISTANCE RANGE:  [%d]\n", gas_calib.res_range);
}

void read_meas_regs(void){
   // Helper for debug prints
   uint8_t read_data_0;
   uint8_t meas_status_0 = 0x1D;

   // CHECK REGISTER CTRL_MEAS_0 TO SEE DATA
   int new_data = i2c_write_read_dt(&bme_dev, &meas_status_0, 1, &read_data_0, 1);
   if (new_data != 0) printk("Error writing to register\n");
   else printk("Data gathered from meas_status[0]: [%d]\n", read_data_0);
}

bool new_gas_readout(void){
   // IF HEAT_STAB_R IS EQUAL TO 1, SENSOR IS READY TO MEASURE GAS IN ENVIROEMENT
   uint8_t gas_r_lsb = 0x2D;
   uint8_t heat_stab_r;

   int err = i2c_write_read_dt(&bme_dev, &gas_r_lsb, 1, &heat_stab_r, 1);

   if (err != 0) {
      printk("Error writing to register\n");
      return false;
   }
   else{
      if ((heat_stab_r & 16) != 16){
         // CHECK IF VALUE OF BIT NUMBER 4 IS 1, IF 0 THERE IS NO VALID DATA.
         printk("There is no valid data. Value of bit 4 is: 0\n");
         return false;
      }
      printk("Sensor ready to perform readings!\n");
      curated_gas.heat_stab_reg = heat_stab_r;
      return true;
   }
}

void parse_gas_readings(void){
   // CONVERT GAS RESISTANCE MEASUREMENT TO OHMS. 
   // ELI5 EXPLANATION: THE GREATER THE RESISTANCE, CLEANER THE AIR.
   // YET TO IMPLEMENT A MORE INTUITIVE READING OF GAS IN ENVIROEMENT.
   uint8_t msb_reg = 0x2C;
   uint8_t lsb_reg = 0x2D;
   uint8_t gas_r_msb;
   uint8_t gas_r_lsb;
   uint16_t gas_adc;
   uint8_t gas_range;

   int err = i2c_write_read_dt(&bme_dev, &msb_reg, 1, &gas_r_msb, 1);
   if (err != 0) printk("Error reading gas_r_msb\n");

   err = i2c_write_read_dt(&bme_dev, &lsb_reg, 1, &gas_r_lsb, 1);
   if (err != 0) printk("Error reading gas_r_lsb\n");
   
   gas_adc   = ((uint16_t)gas_r_msb << 2) | (gas_r_lsb >> 6); // "Lives" on bit <7:6>
   gas_range = gas_r_lsb & 0x0F;                              // "Lives" on bits <3:0> 
   
   // DEBUG/CURIOSITY PRINT
   printk("gas_adc: [%d], gas_range: [%d]\n", gas_adc, gas_range);
   
   curated_gas.adc_gas = gas_adc;
   curated_gas.range_gas = gas_range;
   
   // FORMULA PROVIDED BY BOSCH DATASHEET ->
   // CONVERT ANALOG READS INTO OHMS
   uint32_t var1 = UINT32_C(262144) >> gas_range;
   int32_t var2 = (int32_t) gas_adc - INT32_C(512);
   var2 *= INT32_C(3);
   var2 = INT32_C(4096) + var2;
   float gas_res = 1000000.0f * (float)var1 / (float)var2;
   gas_calib.gas_ohms = gas_res;
}

void run_readings(void){
   uint8_t reg_addr = 0xD0;
   uint8_t chip_id = 0;

   //Fetch Chip ID to confirm a succesful I2C transaction
   int ret = i2c_write_read_dt(&bme_dev, &reg_addr, 1, &chip_id, 1);
   printk("Get CHIP ID from I2C bus to verify a succesful transaction: \n");
   if (ret != 0) {
       printk("Error reading Chip ID! Error code: %d\n", ret);
   } else {
       printk("Data gathered (Chip ID): [0x%02X]\n", chip_id);
   }
   k_sleep(K_MSEC(500)); 
   
   // EXPERIMENTAL GAS SENSOR SEQUENCE (FIELD 0)
   printk("Fetching values to perform calibration...\n");
   fetch_gas_values();
   print_gas_values();
   printk("Calculating res_heat_x\n");
   k_sleep(K_SECONDS(5));
   calc_res_heat();
   printk("res_heat_x: [%d]\n", gas_calib.is_calib);
   printk("Enabling gas measurment...\n");
   configure_oversampling();
   k_sleep(K_SECONDS(2));
   start_gas_measurement();
   k_sleep(K_SECONDS(2));
   read_meas_regs();
}

