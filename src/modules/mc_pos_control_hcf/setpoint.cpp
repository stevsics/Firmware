
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
 * @file setpoint.cpp
 * Setpoint data structures for the mc position controller
 *
 *  @author Thomas Gubler <thomasgubler@gmail.com>
 *  @author Tobias Naegeli <naegelit@student.ethz.ch>
*/

/* Remove floating point error. */
#pragma GCC diagnostic ignored "-Wfloat-equal"

#include "setpoint.h"
#include <geo/geo.h>
#include <math.h>

void Setpoint::reset_xy_sp(float xNew, float yNew) {
	if (isfinite(xNew) && isfinite(yNew)) {
		this->x = xNew;
		this->y = yNew;
	}
}

void Setpoint::reset_z_sp(float zNew) {
	if (isfinite(zNew)) {
		this->z = zNew;
	}
}


void Setpoint::reset_xyz_sp(float xNew, float yNew, float zNew) {
	if (isfinite(xNew) && isfinite(yNew) && isfinite(zNew)) {
		reset_xy_sp(xNew, yNew);
		reset_z_sp(zNew);
	}
};

Setpoint Setpoint::operator=(const Setpoint& rhs)
{
	x = rhs.x;
	y = rhs.y;
	z = rhs.z;
	vx = rhs.vx;
	vy = rhs.vy;
	vz = rhs.vz;
	ax = rhs.ax;
	ay = rhs.ay;
	az = rhs.az;
	return *this;
}


bool GlobalSetpoint::update(double lat_new, double lon_new, float alt_new, bool force_transformation)
{
	bool ret = false;

	if (force_transformation || lat_new != lat || lon_new != lon || alt_new != alt) {
		/* Setpoint changed: transform new setpoint and store */
		ret = true;
		globallocalconverter_tolocal(lat_new, lon_new, alt_new, &x, &y, &z);
		lat = lat_new;
		lon = lon_new;
		alt = alt_new;

	}

	return ret;
}
