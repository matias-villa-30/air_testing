
/*    AirObserver
 *    util.cpp
 *    Matias Villa
 */

#include <stdint.h>
#include <zephyr/kernel.h>
#include "util.hpp"

namespace UTIL {
   K_MUTEX_DEFINE(air_mutex);
   

   Context_util CTX_UTIL;
   Context_util& util_ctx(void) {return CTX_UTIL;}
}
