/*
 * positionLQGC.h
 *
 * Code generation for function 'positionLQGC'
 *
 * C source code generated on: Mon Aug 18 14:58:54 2014
 *
 */

#ifndef __POSITIONLQGC_H__
#define __POSITIONLQGC_H__
/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"

#include "rtwtypes.h"
#include "positionLQGC_types.h"

/* Function Declarations */
extern void positionLQGC(float q, const float r_position[3], const float r_velocity[3], const float r_acceleration[3], const float k[6], const float x_apo_k[10], const float z[10], const boolean_T zFlag[4], const float F_global[3], boolean_T intfreeze, const float setpPosRaw[3], const float setpVelRaw[3], const float setpAccRaw[3], boolean_T intReset, float dxmax, const boolean_T useControlOutputForPrediction[2], float mass, float q_baro_offset, float r_baro, float deltaT, float local_altitude_offset, const float intLim[3], const float uLim[3], float x_apo[10], float u[3], float F_desProp[3], float *variance_horizontal, float *variance_vertical, float debugOutput[4]);
extern void positionLQGC_initialize();
extern void positionLQGC_terminate();
#endif
/* End of code generation (positionLQGC.h) */
