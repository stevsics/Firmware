//
// File: attitudeController.h
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 16-Dec-2014 15:07:04
//
#ifndef __ATTITUDECONTROLLER_H__
#define __ATTITUDECONTROLLER_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "attitudeController_types.h"

// Function Declarations
extern void attitudeController(const float R_IB[9], float yaw, float yawOffset,
  float F_des[3], const float Kp[3], const float Kd[3], const float Ki[3], const
  float omega[3], boolean_T intFreeze, boolean_T intReset, float rates_des[3],
  float *thrust);
extern void attitudeController_initialize();
extern void attitudeController_terminate();

#endif

//
// File trailer for attitudeController.h
//
// [EOF]
//
