/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: @author Anton Babushkin <anton.babushkin@me.com>
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
 * @file attitude_controller.cpp
 * Small class that performs attitude control
 *
 *  @author Thomas Gubler <thomasgubler@gmail.com>
 *  @author Tobias Naegeli <naegelit@student.ethz.ch>
 *
 */

#include "attitude_controller.h"

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <systemlib/err.h>
#include <arch/board/board.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>

AttitudeController::AttitudeController(SuperBlock *parent, const char *name) :
	SuperBlock(parent, name),
	/* publications*/
	_vehicle_rates_sp_pub(ORB_ID(vehicle_rates_setpoint), &getPublications()),
	/* Params */
	_Kp(this, "P"),
	_Ki(this, "I"),
	_Kd(this, "D"),
//	_KpPitch(this, "PP"),
//	_KiPitch(this, "PI"),
//	_KdPitch(this, "PD"),
	_KpYaw(this, "YP"),
	_KiYaw(this, "YI"),
	_KdYaw(this, "YD"),
	_rate_p(),
	_rate_d(),
	_attCtlArgs(),
	_rates_prev()
{
	/* call init function of matlab exported code */
	attitudeController_initialize();
}

AttitudeController::~AttitudeController() {};

void AttitudeController::update(const math::Vector<3> forceSetpointNED,float yawSp,float yawRateSp, const struct vehicle_attitude_s &att, bool intReset)
{

	/* Set arguments for call to matlab exported function */
	for (int i = 0; i < 9; i++) {
		_attCtlArgs.in.R_IB[i] = att.R[i];
	}

	_attCtlArgs.in.yaw = att.yaw;
	_attCtlArgs.in.yawOffset = 0.0f;
	_attCtlArgs.in.F_des[0] =  forceSetpointNED(0);
	_attCtlArgs.in.F_des[1] =  forceSetpointNED(1);
	_attCtlArgs.in.F_des[2] =  forceSetpointNED(2);
	_attCtlArgs.in.Kp[0] = _Kp.get();
	_attCtlArgs.in.Kp[1] = _Kp.get();
	_attCtlArgs.in.Kp[2] = _KpYaw.get();
	_attCtlArgs.in.Ki[0] = _Ki.get();
	_attCtlArgs.in.Ki[1] = _Ki.get();
	_attCtlArgs.in.Ki[2] = _KiYaw.get();
	_attCtlArgs.in.Kd[0] = _Kd.get();
	_attCtlArgs.in.Kd[1] = _Kd.get();
	_attCtlArgs.in.Kd[2] = _KdYaw.get();
	_attCtlArgs.in.e_Rd_v[0]=-att.rollspeed;
	_attCtlArgs.in.e_Rd_v[1]=-att.pitchspeed;
	_attCtlArgs.in.e_Rd_v[2]=0.0f;
	_attCtlArgs.in.intFreeze = (boolean_T)false;
	_attCtlArgs.in.intReset = (boolean_T)(intReset);

	/* Call actual attitude control function */
	attitudeController(
		_attCtlArgs.in.R_IB,
		_attCtlArgs.in.yaw,
		_attCtlArgs.in.yawOffset,
		_attCtlArgs.in.F_des,
		_attCtlArgs.in.Kp,
		_attCtlArgs.in.Kd,
		_attCtlArgs.in.Ki,
		_attCtlArgs.in.e_Rd_v,
		_attCtlArgs.in.intFreeze,
		_attCtlArgs.in.intReset,
		_attCtlArgs.out.rates_des,
		&_attCtlArgs.out.thrust
	);

	/* Scale to get value on range 0 to 1 */
	_attCtlArgs.out.thrust /= 7.848f;

	//[> roll is inverted in matlab attitude controller <]
	//_attCtlArgs.out.rates_des[0] = -_attCtlArgs.out.rates_des[0];


	 _attCtlArgs.out.rates_des[2]=yawRateSp; //testhack




	/* Publish rate sp */
	_vehicle_rates_sp_pub.roll = _attCtlArgs.out.rates_des[0];
	_vehicle_rates_sp_pub.pitch = _attCtlArgs.out.rates_des[1];
	_vehicle_rates_sp_pub.yaw = _attCtlArgs.out.rates_des[2];
	_vehicle_rates_sp_pub.thrust = _attCtlArgs.out.thrust;
//
//				static int counter = 0;
//				if (counter % 100 == 0) {
//					//warnx("kx %.3f ky %.3f kz %.3f ", (double)_attCtlArgs.out.rates_des[0], (double) _attCtlArgs.out.rates_des[1],(double)_attCtlArgs.out.rates_des[2]);
//					warnx("desforce %0.3f",(double)_attCtlArgs.out.thrust);
//				}
//				counter++;


//	static int counter = 0;
//	if (counter % 50 == 0) {
//		warnx("fx %.3f fy %.3f fz %.3f yaw %.3f", (double)forceSetpointNED(0), (double) forceSetpointNED(1),(double) forceSetpointNED(2), (double)_vehicle_rates_sp_pub.thrust);
//		warnx("kx %.3f ky %.3f kz %.3f ", (double)_attCtlArgs.in.Kp[0], (double) _attCtlArgs.in.Kp[1],(double)_attCtlArgs.in.Kp[2]);
//
//	}
//	counter++;
	_vehicle_rates_sp_pub.update();
}
