/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: @author Thomas Gubler <thomasgubler@gmail.com>
 *   Author: @author Tobias Naegeli <naegelit@student.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mc_pos_control_hcf_params.c
 * Multicopter position controller parameters specific to the HCF project.
 *
 *  @author Thomas Gubler <thomasgubler@gmail.com>
 *  @author Tobias Naegeli <naegelit@student.ethz.ch>
 */

#include <systemlib/param/param.h>


PARAM_DEFINE_FLOAT(HCF_LQG_CP, 1.0f);
PARAM_DEFINE_FLOAT(HCF_LQG_CV, 1.0f);
PARAM_DEFINE_FLOAT(HCF_LQG_CU, 1.0f);


/*
 *  Process noise variance
 */
PARAM_DEFINE_FLOAT(HCF_LQG_VP, 0.01f);

/*
 *  Enable manual gps measurement variance override
 *  If set to a value != 0, the variance information of the gps sensor will be ignored and the values from the params
 *  below are used
 */
PARAM_DEFINE_INT32(HCF_LQG_ENMV, 0);

/*
 *  Position measurement variance xy
 */
PARAM_DEFINE_FLOAT(HCF_LQG_VMXY, 0.01f);

/*
 *  Position measurement variance z
 */
PARAM_DEFINE_FLOAT(HCF_LQG_VMZ, 0.01f);

/*
 *  Velocity measurement variance xy
 */
PARAM_DEFINE_FLOAT(HCF_LQG_VMVXY, 0.01f);

/*
 *  Velocity measurement variance z
 */
PARAM_DEFINE_FLOAT(HCF_LQG_VMVZ, 0.01f);

/*
 *  Acceleration measurement variance xy
 */
PARAM_DEFINE_FLOAT(HCF_LQG_VMAXY, 0.2f);

/*
 *  Acceleration measurement variance z
 */
PARAM_DEFINE_FLOAT(HCF_LQG_VMAZ, 0.2f);

/*
 * Baroemeter offset process variance
 */
PARAM_DEFINE_FLOAT(HCF_LQG_VPBO, 0.00001f);

/*
 * Barometer measurement variance
 */
PARAM_DEFINE_FLOAT(HCF_LQG_VMB, 0.001f);

/*
 * Control gains
 */
PARAM_DEFINE_FLOAT(HCF_LQG_K0XY, 0.25f);
PARAM_DEFINE_FLOAT(HCF_LQG_K1XY, 0.2f);
PARAM_DEFINE_FLOAT(HCF_LQG_K2XY, 0.0f);
PARAM_DEFINE_FLOAT(HCF_LQG_K0Z, 0.25f);
PARAM_DEFINE_FLOAT(HCF_LQG_K1Z, 0.2f);
PARAM_DEFINE_FLOAT(HCF_LQG_K2Z, 0.0f);


PARAM_DEFINE_FLOAT(HCF_MASS, 0.435);

PARAM_DEFINE_FLOAT(HCF_MAXLIFTKG, 0.8);

/*
 *  Use pos control output for prediction, if set to 1, the control output will also be used in the prediction step
 */
PARAM_DEFINE_INT32(HCF_PREUSEU, 0);

/*
 *  Maximum velocity for setpoint shift in manual position controlled flight
 */
PARAM_DEFINE_FLOAT(HCF_SPVELSCA, 0.15);

/*
 *  Enable setting also the velocity setpoint
 */
PARAM_DEFINE_INT32(HCF_SPVEL_EN, 0);

/*
 *  Enable setting also the acceleration setpoint
 */
PARAM_DEFINE_INT32(HCF_SPACC_EN, 0);

/*
 *  Manual pressure offset in Pa added to the pressure measurement (for debugging only!)
 */
PARAM_DEFINE_FLOAT(HCF_PRES_OFF, 0.0f);

/*
 *  Integrator limit x y
 */
PARAM_DEFINE_FLOAT(HCF_ILIMXY, 1000.0f);

/*
 *  Integrator limit z
 */
PARAM_DEFINE_FLOAT(HCF_ILIMZ, 10000.0f);

/*
 *  Control output limit x y
 */
PARAM_DEFINE_FLOAT(HCF_ULIMXY, 10.0f);

/*
 *  Control output limit z
 */
PARAM_DEFINE_FLOAT(HCF_ULIMZ, 10.0f);
