
/*    AirObserver
 *    util.cpp
 *    Matias Villa
 */

#include "util.hpp"
#include <zephyr/kernel.h>
#include "bme690.hpp"
#include "scd41.hpp"

namespace UTIL {
   K_MUTEX_DEFINE(air_mutex);
   // BME690 
   K_WORK_DELAYABLE_DEFINE(gas_work, BME690::fetch_work_handler);
   K_WORK_DELAYABLE_DEFINE(meas_steps, BME690::measurement_work_handler);

   K_SEM_DEFINE(sem_gas, 0, 1);
   K_SEM_DEFINE(sem_meas, 0, 1);
   
   // SCD41
   K_WORK_DELAYABLE_DEFINE(meas_scd41, SCD41::fetch_scd41_readings);
   K_SEM_DEFINE(sem_scd41, 0, 1);

}
