/*    AirObserver
 *    task.hpp
 *    Matias Villa
 *
 */
#pragma once

#ifndef TASK_HPP
#define TASK_HPP

#include <zephyr/kernel.h>

namespace AirObserver{

   void bme690_thread(void);
   void scd41_thread(void);
}

#endif
