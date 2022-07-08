/*
 * visca_conf.h
 *
 *  Created on: Dec 29, 2021
 *      Author: miftakur
 */

#ifndef DRIVER_LIBVISCA_120_VISCA_CONF_H_
#define DRIVER_LIBVISCA_120_VISCA_CONF_H_

#include "main.h"

/*
 * how to use this lib:
 * 1. create VISCAInterface_t & VISCACamera_t struct
 * 2. set serial port, iface.port_fd
 * 3. set iface.broadcast = 0
 * 4. set camera.address = 1
 *
 */

#define DEBUG_VISCA						1
#define VISCA_USE_RTOS					0

#if VISCA_USE_RTOS==0
#define VISCA_USE_WDT					1
#endif	//if VISCA_USE_RTOS==0

#endif /* DRIVER_LIBVISCA_120_VISCA_CONF_H_ */
