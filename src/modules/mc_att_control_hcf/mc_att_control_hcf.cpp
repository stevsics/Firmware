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
 * @file mc_att_control_hcf.cpp
 * Multicopter position controller for the HCF project.
 *
 *  @author Anton Babushkin <anton.babushkin@me.com>
 *  @author Thomas Gubler <thomasgubler@gmail.com>
 *  @author Tobias Naegeli <naegelit@student.ethz.ch>
 *
 */

#include "mc_att_control_hcf.h"

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <systemlib/err.h>
#include <arch/board/board.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <mavlink/mavlink_log.h>

#define SIGMA			0.000001f
/**
 * Multicopter position control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_att_control_hcf_main(int argc, char *argv[]);

MulticopterAttitudeControlHcf::MulticopterAttitudeControlHcf(SuperBlock *parent, const char *name) :
	SuperBlock(parent, name),
	_mavlink_fd(-1),
/* uORB publications */
/* uORB subscriptions */ //XXX: set correct rates
	_att(ORB_ID(vehicle_attitude), 5, &getSubscriptions()),
	_arming(ORB_ID(actuator_armed), 15, &getSubscriptions()),
	_control_mode(ORB_ID(vehicle_control_mode), 15, &getSubscriptions()),
/* params */

	t_prev(0),
	_was_armed(false),
	_was_attcontrol(false),
	_initialized(false),
	_debug(false),
/* Sub blocks */
	_inputLoader(this, "INP"),
	_attitudeController(this, "Att"),
	_rateController(this, "Rate")
{

	/* fetch initial parameter values */
	updateParams();

	/* get an initial update for all sensor and status data */
	updateSubscriptions();


	/* Initialize filter */

	/* initialize mavlink fd */
	_mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	mavlink_log_info(_mavlink_fd, "[mpc] started");


	/* initialize values of critical structs until first regular update */
	_arming.armed = false;

}

MulticopterAttitudeControlHcf::~MulticopterAttitudeControlHcf()
{
	perf_free(_perfCounterDt);
	perf_free(_perfCounterDuration);
}

void
MulticopterAttitudeControlHcf::update()
{
	/* init some variables */
	perf_begin(_perfCounterDuration);
	perf_count(_perfCounterDt);

	static int counter = 0;

	/* poll for new data */
	struct pollfd fds[1];
	fds[0].fd = _att.getHandle();
	fds[0].events = POLLIN;

	/* wait for up to 500ms for data */
	int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);

	/* timed out - periodic check for _task_should_exit */
	if (pret == 0) {
		return;
	}

	/* this is undesirable but not much we can do */
	if (pret < 0) {
		warn("poll error %d, %d", pret, errno);
		return;
	}

	/* update subscriptions and params */
	updateSubscriptions();
	updateParams();


	/* Measure Time */
	hrt_abstime t = hrt_absolute_time();
	float dt = t_prev != 0 ? (t - t_prev) * 0.000001f : 0.0f;
	t_prev = t;
	setDt(dt); //update the dt of all blocks element

	/* check armed status */
	if (_control_mode.flag_armed && !_was_armed) {
		/* reset setpoints and integrals on arming */
		//XXX do what the comment says
	}
	_was_armed = _control_mode.flag_armed;

	/* Load inputs */
	_inputLoader.update();

	/* determine if the controller should be run in this iteration */
	bool run_controller = _control_mode.flag_control_attitude_enabled || _control_mode.flag_control_force_enabled;

	if (run_controller) {
		/*Attitude Control */
		_attitudeController.update(_inputLoader.getForceSetpointNED(),_inputLoader.getSetpointYaw(),_inputLoader.getSetpointYawRate() , _att.getData(),
				(!_was_attcontrol || !_was_armed));

		/* Rate control */
		_rateController.update(_attitudeController.getRatesSp(), _attitudeController.getThrustSp(),
			math::Vector<3>({_att.rollspeed, _att.pitchspeed, _att.yawspeed}));

	}

	/* set flag if pos control was active in this iteration */
	_was_attcontrol = run_controller;

	counter++;
	perf_end(_perfCounterDuration);

}

int MulticopterAttitudeControlHcf::initStateIfNecessary()
{
	if (_initialized) {
		/* already initialized, nothing to do */
		return -1;
	}

	/* Not yet initialized: do initialization */
	//XXX do state initiialization
	_initialized = true;

	return 0;

}

void MulticopterAttitudeControlHcf::debug_print(const char *fmt, va_list args)
{
		fprintf(stderr, "%s: ", "[mc_att_control_hcf]");
		vfprintf(stderr, fmt, args);

		fprintf(stderr, "\n");
}

void MulticopterAttitudeControlHcf::debug(const char *fmt, ...) {

		if (!_debug) {
			return;
		}

		va_list args;

		va_start(args, fmt);
		debug_print(fmt, args);
}
