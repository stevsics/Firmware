
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
 * @file setpoint.h
 * Setpoint data structures for the mc position controller
 *
 *  @author Thomas Gubler <thomasgubler@gmail.com>
 *  @author Tobias Naegeli <naegelit@student.ethz.ch>
 */

/*
 * A class for storing a setpoint for the position controller
 */
class Setpoint
{
public:
	Setpoint() :
		x(0.0f),
		y(0.0f),
		z(0.0f),
		vx(0.0f),
		vy(0.0f),
		vz(0.0f),
		ax(0.0f),
		ay(0.0f),
		az(0.0f)

	{};
	~Setpoint() {};

	float x;
	float y;
	float z;
	float vx;
	float vy;
	float vz;
	float ax;
	float ay;
	float az;

	/**
	* Reset x/y to current position
	*/
	void		reset_xy_sp(float xNew = 0.0f, float yNew = 0.0f);

	/**
	* Reset z setpoint to current altitude
	*/
	void		reset_z_sp(float zNew = 0.0f);

	/**
	* Reset x,y,z setpoint to current position
	*/
	void reset_xyz_sp(float xNew = 0.0f, float yNew = 0.0f, float zNew = 0.0f);

	Setpoint operator=(const Setpoint& rhs);
};

/*
 * A class for storing a setpoint provided in lat/lon/alt (global pos/gps/WGS84) including conversion functions
 */
class GlobalSetpoint : public Setpoint
{
public:
	GlobalSetpoint() :
		Setpoint (),
		lat(0.0),
		lon(0.0),
		alt(0.0),
		initialized(false)
	{};

	~GlobalSetpoint() {};

	double lat;
	double lon;
	float alt;
	bool initialized;

	/*
	 * Update the setpoint: if the new lat/lon/alt values differ from the stored ones perform the transformation to
	 * the local frame.
	 * The transformation can be forced with the forece_transformation flag to be performed irrespictive of the
	 * comparison with the stored lat/lon/alt values (e.g. use when the coordinate frame reference changes)
	 */
	bool update(double lat_new, double lon_new, float alt_new, bool force_transformation = false);

};
