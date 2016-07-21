/*
 * INS_UserInteract.h
 *
 *  Created on: Jun 26, 2016
 *      Author: chris
 */

#ifndef INS_USERINTERACT_H_
#define INS_USERINTERACT_H_

#include <AP_Progmem.h>
#include <AP_InertialSensor.h>
#include <AP_InertialSensor_UserInteract.h>

#include <stdarg.h>

class INS_UserInteract : public AP_InertialSensor_UserInteract {
public:
    bool blocking_read();
    void _printf_P(const prog_char *str, ...);
};


#endif /* INS_USERINTERACT_H_ */
