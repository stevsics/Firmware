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
 * @file mc_pos_control_hcf.cpp
 * Multicopter position controller for the HCF project.
 *
 *  @author Anton Babushkin <anton.babushkin@me.com>
 *  @author Thomas Gubler <thomasgubler@gmail.com>
 *  @author Tobias Naegeli <naegelit@student.ethz.ch>
 *
 */

#include "mc_pos_control_hcf.h"

#include <px4_config.h>
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
extern "C" __EXPORT int mc_pos_control_hcf_main(int argc, char *argv[]);

MulticopterPositionControlHcf::MulticopterPositionControlHcf(SuperBlock *parent, const char *name) :
	SuperBlock(parent, name),
	_mavlink_fd(-1),

/* uORB publications */
	_att_sp(ORB_ID(vehicle_attitude_setpoint), &getPublications()),
	_local_pos(ORB_ID(vehicle_local_position), &getPublications()),
	_global_pos(ORB_ID(vehicle_global_position), &getPublications()),
	_pos_sp_triplet_pub(ORB_ID(position_setpoint_triplet), &getPublications()),
	_force_sp(ORB_ID(vehicle_force_setpoint), &getPublications()),
/* uORB subscriptions */ //XXX: set correct rates
	_att(ORB_ID(vehicle_attitude), 5, &getSubscriptions()),
	_manual(ORB_ID(manual_control_setpoint), 15, &getSubscriptions()),
	_control_mode(ORB_ID(vehicle_control_mode), 15, &getSubscriptions()),
	_arming(ORB_ID(actuator_armed), 15, &getSubscriptions()),
	_vicon_pos(ORB_ID(vehicle_vicon_position), 15, &getSubscriptions()),
	_gps_pos(ORB_ID(vehicle_gps_position), 15, &getSubscriptions()), //XXX lower frequency
	_pos_sp_triplet(ORB_ID(position_setpoint_triplet), 15, &getSubscriptions()), //XXX lower frequency
	_sensor_combined(ORB_ID(sensor_combined), 15, &getSubscriptions()), //XXX lower frequency
	_sensor_accel(ORB_ID(sensor_accel), 5, &getSubscriptions()), //XXX sensor_accel, no more sensor_accel0 (Check if works)
/* params */
	_lqgK0XY(this, "LQG_K0XY"),
	_lqgK1XY(this, "LQG_K1XY"),
	_lqgK2XY(this, "LQG_K2XY"),
	_lqgK0Z(this, "LQG_K0Z"),
	_lqgK1Z(this, "LQG_K1Z"),
	_lqgK2Z(this, "LQG_K2Z"),
	_lqgCostPos(this, "LQG_CP"),
	_lqgCostVel(this, "LQG_CV"),
	_lqgCostU(this, "LQG_CU"),
	_varNoiseProcess(this, "LQG_VP"),
	_varNoiseMeasurementXY(this, "LQG_VMXY"),
	_varNoiseMeasurementZ(this, "LQG_VMZ"),
	_varNoiseMeasurementVXY(this, "LQG_VMVXY"),
	_varNoiseMeasurementVZ(this, "LQG_VMVZ"),
	_varNoiseMeasurementAXY(this, "LQG_VMAXY"),
	_varNoiseMeasurementAZ(this, "LQG_VMAZ"),
	_varNoiseProcessBaroOffset(this, "LQG_VPBO"),
	_varNoiseMeasurementBaro(this, "LQG_VMB"),
	_mass(this, "MASS"),
	_maxLiftKg(this, "MAXLIFTKG"),
	_useControlOutputForPrediction(this, "PREUSEU"),
	_setpointVelScale(this, "SPVELSCA"),
	_setVelocitySetpointEnabled(this, "SPVEL_EN"),
	_setAccelerationSetpointEnabled(this, "SPACC_EN"),
	_positionVarianceOverrideEnabled(this, "LQG_ENMV"),
	_pressureOffsetManual(this, "PRES_OFF"),
	_intLimXY(this, "ILIMXY"),
	_intLimZ(this, "ILIMZ"),
	_uLimXY(this, "ULIMXY"),
	_uLimZ(this, "ULIMZ"),

	setpoint(),
	_was_armed(false),
	_initialized(false),
	t_prev(0),
	t_lastvicon(0),
	dt_vicon(0),
	t_lastgps(0),
	dt_gps(0),
	_posCtlArgs(),
	_perfCounterDt(perf_alloc(PC_INTERVAL, "mc_pos_control_hcf dt")),
	_perfCounterDuration(perf_alloc(PC_ELAPSED, "mc_pos_control_hcf update duration")),
	_debug(false)
{

	/* fetch initial parameter values */
	updateParams();

	/* get an initial update for all sensor and status data */
	updateSubscriptions();


	/* Initialize filter */
	_position_measurement_inertial.zero();
	_velocity_measurement_inertial.zero();

	_posCtlArgs.in.dxmax = 1.0f; //XXX value?

	_posCtlArgs.in.q = _varNoiseProcess.get();
	_posCtlArgs.in.r_position[0] =  _varNoiseMeasurementXY.get();
	_posCtlArgs.in.r_position[1] =  _varNoiseMeasurementXY.get();
	_posCtlArgs.in.r_position[2] =  _varNoiseMeasurementZ.get();

	/* initialize mavlink fd */
	_mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	mavlink_log_info(_mavlink_fd, "[mpc] started");


	/* initialize values of critical structs until first regular update */
	_arming.armed = false;

	sp_move_rate.zero();
	thrust_int.zero();
	R.identity();

	for (size_t i = 0; i < sizeof(_dkv) / sizeof(_dkv[0]); i++) {
		_dkv[i] = new uORB::Publication<debug_key_value_s>(ORB_ID(debug_key_value), &getPublications());
	}

	strncpy(_dkv[0]->key, "dkv_0_u0  ", 10);
	strncpy(_dkv[1]->key, "dkv_1_u1  ", 10);
	strncpy(_dkv[2]->key, "dkv_2_u2  ", 10);
	strncpy(_dkv[3]->key, "dkv_3_dtvi", 10);
	strncpy(_dkv[4]->key, "dkv_4_Pres", 10);
	strncpy(_dkv[5]->key, "dkv_5_dtg ", 10);
	strncpy(_dkv[6]->key, "dkv_6_boff", 10);

	/* call init function of matlab exported code */
	positionLQGC_initialize();

}

MulticopterPositionControlHcf::~MulticopterPositionControlHcf()
{
	perf_free(_perfCounterDt);
	perf_free(_perfCounterDuration);
}

float
MulticopterPositionControlHcf::scale_control(float ctl, float end, float dz)
{
	if (ctl > dz) {
		return (ctl - dz) / (end - dz);

	} else if (ctl < -dz) {
		return (ctl + dz) / (end - dz);

	} else {
		return 0.0f;
	}
}

void
MulticopterPositionControlHcf::update()
{
	/* init some variables */
	perf_begin(_perfCounterDuration);
	perf_count(_perfCounterDt);

	static int counter = 0;
	static int viconCounter = 0;
	static int noPositionMeasurementUpdateCounter = 0;
	static int debugKeyCounter = 0;

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

	/* Check which position measurement topic was updated */
	bool vicon_pos_updated = _vicon_pos.updated();
	bool gps_updated = _gps_pos.updated();
	bool gps_updated_valid = gps_updated && globallocalconverter_initialized(); //map proj. needs to be initalized

	/* Set measurement_position_updated flag: true if we have a valid measurement. Accept vicon only if not already in
	 * global pos mode (if we have never received a gps message)
	 */
	bool measurement_position_updated = gps_updated_valid || (vicon_pos_updated && !_global_pos_mode_enabled);

	/* check if sensor combined topic (contains baro measurement got updated)
	 * global local converter needs to be initialized because we need a rough guess of the local frame altitude
	 * offset in order to predict the pressure
	 */
	bool sensor_combined_updated = _sensor_combined.updated() &&
		globallocalconverter_initialized();

	/* check if accelerometer updated */
	bool sensor_accel_updated = _sensor_accel.updated() && globallocalconverter_initialized();

	/* update subscriptions and params */
	updateSubscriptions();
	updateParams();

	/* flag that is true if there was an incoming velocity measuremen e for gps measurements, false for vicon measurements) */
	bool measurement_velocity_updated = false;

	if (gps_updated_valid) {
		/* Transform gps measurement to local frame using global map projection */
		math::Vector<3> p_g;
		globallocalconverter_tolocal((double)_gps_pos.lat * 1.0e-7, (double)_gps_pos.lon * 1.0e-7,
				(float)_gps_pos.alt * 1e-3f, &p_g(0), &p_g(1), &p_g(2));
		_position_measurement_inertial = p_g;

		/* Copy velocity measurement and set flag that velocity measurement is available */
		_velocity_measurement_inertial(0) =_gps_pos.vel_n_m_s;
		_velocity_measurement_inertial(1) =_gps_pos.vel_e_m_s;
		_velocity_measurement_inertial(2) =_gps_pos.vel_d_m_s;
		measurement_velocity_updated = true;

		/* set a flag that the global position should be published, also this disables the use of later vicon
		 * message */
		_global_pos_mode_enabled = true;

		/* If the filter was not initialized: set the state based on the first measurement */
		initStateIfNecessary(_position_measurement_inertial, _velocity_measurement_inertial);

		/* save time since last gps update */
		dt_gps = hrt_elapsed_time(&t_lastgps);

		noPositionMeasurementUpdateCounter = 0;


	} else if (vicon_pos_updated && !_global_pos_mode_enabled) {
		/* If vicon got updated and not in global pos mode: transform to local pos in inertial frame */

		/* Transform vicon frame to inertial frame
		 * 1) rotate vicon data to the B2 frame using the rotation matrix from the vicon: p_B2 = R_B2V * p_V
		 * 2) rotate from B2 to intertial frame: _position_measurement_inertial = p_I = R_IB2 * p_B2
		 *
		 * Note: frame B2 is a frame at the  origin  with the same orientation as the body fixed frame B
		 */
		math::Vector<3> p_v({_vicon_pos.x, _vicon_pos.y, _vicon_pos.z});
		math::Matrix<3, 3> R_VB2;
		R_VB2.from_euler(_vicon_pos.roll, _vicon_pos.pitch, _vicon_pos.yaw);
		math::Matrix<3, 3> R_IB2;
		R_IB2.set(_att.R);
		_position_measurement_inertial = R_IB2 * R_VB2.transposed() * p_v;

		/* Set flag that velocity measurement is not available */
		measurement_velocity_updated = false;

		/* If the filter was not initialized: set the state based on the first measurement */
		initStateIfNecessary(_position_measurement_inertial, _velocity_measurement_inertial);

		/* save deltat since last vicon update */
		dt_vicon = hrt_elapsed_time(&t_lastvicon);

		viconCounter++;
		noPositionMeasurementUpdateCounter = 0;

	} else {
		noPositionMeasurementUpdateCounter++; //XXX: use perf counter

		if (noPositionMeasurementUpdateCounter > 1000) {
			warnx("receiving no valid position measurement");
			noPositionMeasurementUpdateCounter = 0;
		}
	}

	//debug("pos meas %.3f %.3f %.3f, gps upd: %u", (double)_position_measurement_inertial(0),
			//(double)_position_measurement_inertial(1),
			//(double)_position_measurement_inertial(2),
			//gps_updated_valid);

	/* Prepare accelerometer measurements */
	if (sensor_accel_updated) {
		/* create vector with acc measurements in body frame */
		math::Vector<3> a({_sensor_accel.x, _sensor_accel.y, _sensor_accel.z});

		/* Rotate to a body fixed NED frame */
		math::Matrix<3, 3> R_IB;
		R_IB.set(_att.R);
		_acceleration_measurement_ned = R_IB * a;

		/* remove gravity */
		_acceleration_measurement_ned(2) += CONSTANTS_ONE_G;

	}

	/* Measure Time */
	hrt_abstime t = hrt_absolute_time();
	float dt = t_prev != 0 ? (t - t_prev) * 0.000001f : 0.0f;
	t_prev = t;
	setDt(dt); //update the dt of all blocks element

	/* check armed status */
	if (_control_mode.flag_armed && !_was_armed) {
		/* reset setpoints and integrals on arming */
		setpoint.reset_xyz_sp(_local_pos.x, _local_pos.y, _local_pos.z);
	}
	_was_armed = _control_mode.flag_armed;

	/* Store altitude difference between global and local frame: used for static pressure prediction in the
		* estimator */
	if (globallocalconverter_getref(NULL, NULL, &_posCtlArgs.in.local_altitude_offset) )
	{
		/* could not get reference --> set flags so that measurement is not taken into account in the
			* estimator */
		gps_updated_valid = measurement_position_updated = false;
	}

	/* determine if the controller should be run in this iteration: otherwise just the estimator part will be run */
	bool run_controller = _control_mode.flag_control_altitude_enabled ||
			      _control_mode.flag_control_position_enabled ||
			      _control_mode.flag_control_climb_rate_enabled ||
			      _control_mode.flag_control_velocity_enabled;

	if (run_controller) {

		if (!_was_poscontrol) {
			/* reset the manual setpoints */
			setpoint.reset_xyz_sp(_local_pos.x, _local_pos.y, _local_pos.z);
		}

		/* select control source */
		if (_control_mode.flag_control_manual_enabled && !_control_mode.flag_control_offboard_enabled) {
			/* manual control, but not offboard */
			/* XXX: as soon as the navigator app also provides a setpoint in
			 * manual mdoe this can be removed */

			if (_control_mode.flag_control_altitude_enabled) {
				/* manual control and altitude control flag set: seatbelt (or easy) mode: move altitude setpoint with throttle stick */
				sp_move_rate(2) = -scale_control(_manual.z - 0.5f, 0.5f, alt_ctl_dz);
			}

			if (_control_mode.flag_control_position_enabled) {
				/* Manual control and position control flag set: easy mode --> control position via RC sticks */

				/* move position setpoint with roll/pitch stick */
				sp_move_rate(0) = scale_control(_manual.x, 1.0f, pos_ctl_dz);
				sp_move_rate(1) = scale_control(_manual.y, 1.0f, pos_ctl_dz);
			}

			/* limit setpoint move rate */
			float sp_move_norm = sp_move_rate.length();
			if (sp_move_norm > 1.0f) {
				sp_move_rate /= sp_move_norm;
			}

			/* scale to max speed and rotate around yaw */
			math::Matrix<3, 3> R_yaw_sp;
			R_yaw_sp.from_euler(0.0f, 0.0f, _att_sp.yaw_body);
			sp_move_rate = R_yaw_sp * sp_move_rate * _setpointVelScale.get();
			//sp_move_rate = sp_move_rate * _setpointVelScale.get(); //version without rotation for steering in inertial frame

			/* move position setpoint */
			setpoint.x += sp_move_rate(0) * dt;
			setpoint.y += sp_move_rate(1) * dt;
			setpoint.z += sp_move_rate(2) * dt;

			/* Publish position sp to uorb (in global frame) */
			globallocalconverter_toglobal(setpoint.x, setpoint.y, setpoint.z, &_pos_sp_triplet_pub.current.lat,
					&_pos_sp_triplet_pub.current.lon, &_pos_sp_triplet_pub.current.alt);
			_pos_sp_triplet_pub.update();

		} else {
			/* Auto mode or offboard mode: follow setpoint from navigator (auto)
			 * or mavlink app (offboard mode) */
			_pos_sp_triplet.update();

			if (_pos_sp_triplet.current.valid) {

				if (_pos_sp_triplet.current.position_valid) {
					/* Local position setpoint is available. --> use it */
					setpoint.x = _pos_sp_triplet.current.x;
					setpoint.y = _pos_sp_triplet.current.y;
					setpoint.z = _pos_sp_triplet.current.z;

					/* copy force setpoint part to F_global if force setpoint is set */
					if (_pos_sp_triplet.current.acceleration_valid &&
							_pos_sp_triplet.current.acceleration_is_force) {
						_posCtlArgs.in.F_global[0] = _pos_sp_triplet.current.a_x;
						_posCtlArgs.in.F_global[1] = _pos_sp_triplet.current.a_y;
						_posCtlArgs.in.F_global[2] = -_pos_sp_triplet.current.a_z; //minus sign because of z up convention of estimator/controller, XXX correct here?
					}
				} else {
					/* use global position setpoint */
					externalSetpoint.update(_pos_sp_triplet.current.lat,
							_pos_sp_triplet.current.lon,
							_pos_sp_triplet.current.alt);
					/* update position setpoint */
					setpoint = (Setpoint)externalSetpoint;
				}

				if (_pos_sp_triplet.current.velocity_valid) {
					setpoint.vx = _pos_sp_triplet.current.vx;
					setpoint.vy = _pos_sp_triplet.current.vy;
					setpoint.vz = _pos_sp_triplet.current.vz;
				}

				/* update yaw setpoint if needed */
				if (isfinite(_pos_sp_triplet.current.yaw)) {
					_att_sp.yaw_body = _pos_sp_triplet.current.yaw;
				}

			}
		}

		/* yaw setpoint is current yaw */
		_att_sp.yaw_body = _att.yaw; //XXX

	}
	/* END select control source */

	/* LQGC Controller */
	/* determine if estimator should be run */
	bool run_estimator = globallocalconverter_initialized(); // run once the projection is initialized (== gps lock)
	if (run_estimator) {

		/* prepare values for positionLQGC */
		//XXX: don't hold a copy of all arguments
		_posCtlArgs.in.q = _varNoiseProcess.get();
		_posCtlArgs.in.q_baro_offset = _varNoiseProcessBaroOffset.get();
		/* set the measurement variance:
		* if HCF_LQG_ENMV is set to a value != 0 or we are in vicon mode the values from the parameters will be used,
		* otherwise the values given by the gps are used
		*/
		if (!_global_pos_mode_enabled || (bool)_positionVarianceOverrideEnabled.get()) {
			_posCtlArgs.in.r_position[0] =  _varNoiseMeasurementXY.get();
			_posCtlArgs.in.r_position[1] =  _varNoiseMeasurementXY.get();
			_posCtlArgs.in.r_position[2] =  _varNoiseMeasurementZ.get();
			_posCtlArgs.in.r_velocity[0] =  _varNoiseMeasurementVXY.get();
			_posCtlArgs.in.r_velocity[1] =  _varNoiseMeasurementVXY.get();
			_posCtlArgs.in.r_velocity[2] =  _varNoiseMeasurementVZ.get();
		} else {
			_posCtlArgs.in.r_position[0] =  _gps_pos.eph * _gps_pos.eph;
			_posCtlArgs.in.r_position[1] =  _gps_pos.eph * _gps_pos.eph;
			_posCtlArgs.in.r_position[2] =  _gps_pos.epv * _gps_pos.epv;
			_posCtlArgs.in.r_velocity[0] =  _gps_pos.s_variance_m_s * _gps_pos.s_variance_m_s;
			_posCtlArgs.in.r_velocity[1] =  _gps_pos.s_variance_m_s * _gps_pos.s_variance_m_s;
			_posCtlArgs.in.r_velocity[2] =  _gps_pos.s_variance_m_s * _gps_pos.s_variance_m_s;
		}

		_posCtlArgs.in.r_acceleration[0] =  _varNoiseMeasurementAXY.get();
		_posCtlArgs.in.r_acceleration[1] =  _varNoiseMeasurementAXY.get();
		_posCtlArgs.in.r_acceleration[2] =  _varNoiseMeasurementAZ.get();
		_posCtlArgs.in.r_baro = _varNoiseMeasurementBaro.get();
		_posCtlArgs.in.k[0] = _lqgK0XY.get();
		_posCtlArgs.in.k[1] = _lqgK1XY.get();
		_posCtlArgs.in.k[2] = _lqgK2XY.get();
		_posCtlArgs.in.k[3] = _lqgK0Z.get();
		_posCtlArgs.in.k[4] = _lqgK1Z.get();
		_posCtlArgs.in.k[5] = _lqgK2Z.get();
		_posCtlArgs.in.z[INDEX_MEAS_PX] = _position_measurement_inertial(0);
		_posCtlArgs.in.z[INDEX_MEAS_VX] = _velocity_measurement_inertial(0);
		_posCtlArgs.in.z[INDEX_MEAS_AX] = _acceleration_measurement_ned(0);
		_posCtlArgs.in.z[INDEX_MEAS_PY] = _position_measurement_inertial(1);
		_posCtlArgs.in.z[INDEX_MEAS_VY] = _velocity_measurement_inertial(1);
		_posCtlArgs.in.z[INDEX_MEAS_AY] = _acceleration_measurement_ned(1);
		_posCtlArgs.in.z[INDEX_MEAS_PZ] = _position_measurement_inertial(2); //this is converted to z up by the matlab code itself
		_posCtlArgs.in.z[INDEX_MEAS_VZ] = _velocity_measurement_inertial(2); //this is converted to z up by the matlab code itself
		_posCtlArgs.in.z[INDEX_MEAS_AZ] = _acceleration_measurement_ned(2);
		_posCtlArgs.in.z[INDEX_MEAS_PRESSURE] = _sensor_combined.baro_pres_mbar * 1e2f + _pressureOffsetManual.get();
		/* set zFlag vector which tells the estimator which sensors got updated */
		_posCtlArgs.in.zFlag[0] = (boolean_T)measurement_position_updated; // flag if position was updated
		_posCtlArgs.in.zFlag[1] = (boolean_T)measurement_velocity_updated; // flag if velocity was updated
		_posCtlArgs.in.zFlag[2] = (boolean_T)sensor_accel_updated; // flag if acceleration was updated
		_posCtlArgs.in.zFlag[3] = (boolean_T)sensor_combined_updated; // flag if barometer was updated
		_posCtlArgs.in.intfreeze = (boolean_T)!run_controller;
		if (_control_mode.flag_control_position_enabled) {
			_posCtlArgs.in.setpPosRaw[0] = setpoint.x;
			_posCtlArgs.in.setpPosRaw[1] = setpoint.y;
			_posCtlArgs.in.setpVelRaw[0] = setpoint.vx;
			_posCtlArgs.in.setpVelRaw[1] = setpoint.vy;
			_posCtlArgs.in.setpAccRaw[0] = setpoint.ax;
			_posCtlArgs.in.setpAccRaw[1] = setpoint.ay;
		} else {
			_posCtlArgs.in.setpPosRaw[0] = _posCtlArgs.in.x_apo_k[INDEX_STATE_PX];
			_posCtlArgs.in.setpPosRaw[1] = _posCtlArgs.in.x_apo_k[INDEX_STATE_PY];
			_posCtlArgs.in.setpVelRaw[0] = 0.0f;
			_posCtlArgs.in.setpVelRaw[1] = 0.0f;
			_posCtlArgs.in.setpAccRaw[0] = 0.0f;
			_posCtlArgs.in.setpAccRaw[1] = 0.0f;;
		}
		//if (counter % 10 == 0) {
			//warnx("sp z %.3f", (double)setpoint.z);
		//}
		if (_control_mode.flag_control_altitude_enabled) {
			_posCtlArgs.in.setpPosRaw[2] =  -setpoint.z; //Matlab code uses z up internally
			_posCtlArgs.in.setpVelRaw[2] =  -setpoint.vz; //Matlab code uses z up internally
			_posCtlArgs.in.setpAccRaw[2] =  -setpoint.az; //Matlab code uses z up internally
		} else {

			_posCtlArgs.in.setpPosRaw[2] = _posCtlArgs.in.x_apo_k[INDEX_STATE_PZ];
			_posCtlArgs.in.setpVelRaw[2] = 0.0f;
			_posCtlArgs.in.setpAccRaw[2] = 0.0f;
		}
		_posCtlArgs.in.intReset = (boolean_T)!run_controller;
		_posCtlArgs.in.deltaT = dt;
		_posCtlArgs.in.intLim[0] = _intLimXY.get();
		_posCtlArgs.in.intLim[1] = _intLimXY.get();
		_posCtlArgs.in.intLim[2] = _intLimZ.get();
		_posCtlArgs.in.uLim[0] = _uLimXY.get();
		_posCtlArgs.in.uLim[1] = _uLimXY.get();
		_posCtlArgs.in.uLim[2] = _uLimZ.get();
		/* Set flags which tell the estimator if it has to use the control output in the prediction
		* for altitude control _control_mode.flag_control_velocity_enabled == false and hence the filter does not
		* predict the horizontal movement with the control output
		*
		* In nay case, the estimator should not predict with the control output when the controller is not running or
		* if the feature is disabled via parameter
		*/
		_posCtlArgs.in.useControlOutputForPrediction[0] = (boolean_T)_useControlOutputForPrediction.get() &&
			run_controller && _control_mode.flag_control_velocity_enabled;

		_posCtlArgs.in.useControlOutputForPrediction[1] = (boolean_T)_useControlOutputForPrediction.get() &&
			run_controller;
			memcpy(_posCtlArgs.in.x_apo_k, _posCtlArgs.out.x_apo, sizeof(_posCtlArgs.in.x_apo_k));

		/* call positionLQGC */
		positionLQGC(
			/* inputs */
			_posCtlArgs.in.q,
			_posCtlArgs.in.r_position,
			_posCtlArgs.in.r_velocity,
			_posCtlArgs.in.r_acceleration,
			_posCtlArgs.in.k,
			_posCtlArgs.in.x_apo_k,
			_posCtlArgs.in.z,
			_posCtlArgs.in.zFlag,
			_posCtlArgs.in.F_global,
			_posCtlArgs.in.intfreeze,
			_posCtlArgs.in.setpPosRaw,
			_posCtlArgs.in.setpVelRaw,
			_posCtlArgs.in.setpAccRaw,
			_posCtlArgs.in.intReset,
			_posCtlArgs.in.dxmax,
			_posCtlArgs.in.useControlOutputForPrediction,
			_mass.get(),
			_posCtlArgs.in.q_baro_offset,
			_posCtlArgs.in.r_baro,
			_posCtlArgs.in.deltaT,
			_posCtlArgs.in.local_altitude_offset,
			_posCtlArgs.in.intLim,
			_posCtlArgs.in.uLim,
			/* outputs */
			_posCtlArgs.out.x_apo,
			_posCtlArgs.out.u,
			_posCtlArgs.out.F_desProp,
			&_posCtlArgs.out.variance_horizontal,
			&_posCtlArgs.out.variance_vertical,
			_posCtlArgs.out.debugOutput);
	}
	/* END LQGC Controller */

	/* debug plots */ //xxx rewrite with proper mapping
	switch (debugKeyCounter) {
	case 0:
		_dkv[debugKeyCounter]->value = _posCtlArgs.out.u[0];
		break;

	case 1:
		_dkv[debugKeyCounter]->value = _posCtlArgs.out.u[1];
		break;

	case 2:
		/* u[2] is positive for upward thrust due to the z up convention of the matlab script */
		_dkv[debugKeyCounter]->value = -_posCtlArgs.out.u[2];
		break;

	case 3:
		_dkv[debugKeyCounter]->value = dt_vicon*1e-6f;
		break;

	case 4:
		_dkv[debugKeyCounter]->value = _posCtlArgs.out.debugOutput[0];
		break;

	case 5:
		_dkv[debugKeyCounter]->value = dt_gps*1e-6f;
		break;

	case 6:
		_dkv[debugKeyCounter]->value = _posCtlArgs.out.x_apo[6];
		break;

	default:
		break;

	}
	_dkv[debugKeyCounter]->update();
	if (debugKeyCounter >= 6) {
		debugKeyCounter = -1;
	}
	debugKeyCounter++;
	/* END debug plots */


	/* Publish Estimate
	 * first set flags if the estimate should be published (only if estimator runs)
	 */
	bool publish_local_pos = run_estimator;
	bool publish_global_pos = publish_local_pos && _global_pos_mode_enabled;
	publishEstimates(publish_local_pos, publish_global_pos);

	/* Copy, convert and publish LQGC control output if control is enabled
	 * we publish to 2 different topics:
	 * vehicle_force_setpoint: used by the hcf att control app
	 * vehicle_attitude_setpoint: used by the px4 attitude control app (Anton)
	 * */
	if (run_controller) {
		if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid
		    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
			/* idle state, publish zero force */
			_force_sp.x = 0.0f;
			_force_sp.y = 0.0f;
			_force_sp.z = 0.0f;
			_force_sp.update();

			/* idle state, publish zero thrust and attitude */
			R.identity();
			memcpy(&_att_sp.R_body[0], R.data, sizeof(_att_sp.R_body));
			_att_sp.R_valid = true;

			_att_sp.roll_body = 0.0f;
			_att_sp.pitch_body = 0.0f;
			_att_sp.yaw_body = _att.yaw;
			_att_sp.thrust = 0.0f;

			_att_sp.timestamp = hrt_absolute_time();

			/* publish attitude setpoint */
			_att_sp.update();

		} else  {
			/* Add hover offset */
			float hover_offset = _mass.get() / _maxLiftKg.get();
			/* u[2] is positive for upward thrust due to the z up convention of the matlab script */
			math::Vector<3> thrust_sp = {_posCtlArgs.out.u[0],
				_posCtlArgs.out.u[1],
				-_posCtlArgs.out.u[2] - hover_offset};
			if (counter % 3 == 0) {
				debug("x_apo %6.3f\t%6.3f\t%6.3f", (double)_posCtlArgs.out.x_apo[0], (double)_posCtlArgs.out.x_apo[3], -(double)_posCtlArgs.out.x_apo[6]);
				debug("sp  pos %6.3f\t%6.3f\t%6.3f", (double)_posCtlArgs.in.setpPosRaw[0], (double)_posCtlArgs.in.setpPosRaw[1], -(double)_posCtlArgs.in.setpPosRaw[2]);
				debug("-u[2] %.3f -hover_offset %.3f, meas(2) %.3f", (double)-_posCtlArgs.out.u[2], (double)-hover_offset, (double)_position_measurement_inertial(2));
				//debug("mass %.3f lift %.3f offset %.3f thrust sp(2) %.3f u2 %.3f (zup) flag pos %u", (double)_mass.get(), (double)_maxLiftKg.get(), (double)hover_offset, (double)thrust_sp(2), (double)_posCtlArgs.out.u[2], _control_mode.flag_control_position_enabled);
			}

			/* Directly publish force setpoint */
			if (_control_mode.flag_control_position_enabled) {
				/* Horizontal and vertical control */
				_force_sp.x = thrust_sp(0);
				_force_sp.y = thrust_sp(1);
				_force_sp.z = thrust_sp(2);
				if (counter % 3 == 0) {
					debug("fsp %6.3f\t%6.3f\t%6.3f\n", (double)_force_sp.x, (double)_force_sp.y, (double)_force_sp.z);
				}
			} else {
				/* Altitude control only */
				_force_sp.x = 0.0f;
				_force_sp.y = 0.0f;
				_force_sp.z = thrust_sp(2);
			}
			_force_sp.update();


			/* Prepare an publish attitude setpoint */
			/* use thrust vector from lqg controller and calculate attitude setpoint which is forwarded to attitude setpoint */
			/* set absolute thrust value: in altitude control (no xy control by the controller) thrust_abs
			 * is the z component of the control output
			 */
			float thrust_abs = 0.0f;
			if (_control_mode.flag_control_position_enabled) {
				thrust_abs = thrust_sp.length();
			} else {
				thrust_abs = thrust_sp(2) < 0 ? -thrust_sp(2) : 0.0f;//-(-_posCtlArgs.out.u[2] - hover_offset);
			}


			/* calculate attitude setpoint from thrust vector */
			if (_control_mode.flag_control_velocity_enabled) {
				/* desired body_z axis = -normalize(thrust_vector) */
				math::Vector<3> body_x;
				math::Vector<3> body_y;
				math::Vector<3> body_z;

				if (thrust_abs > SIGMA) {
					body_z = -thrust_sp / thrust_abs;

				} else {
					/* no thrust, set Z axis to safe value */
					body_z.zero();
					body_z(2) = 1.0f;
				}

				/* vector of desired yaw direction in XY plane, rotated by PI/2 */
				math::Vector<3> y_C(-sinf(_att_sp.yaw_body), cosf(_att_sp.yaw_body), 0.0f);

				if (fabsf(body_z(2)) > SIGMA) {
					/* desired body_x axis, orthogonal to body_z */
					body_x = y_C % body_z;

					/* keep nose to front while inverted upside down */
					if (body_z(2) < 0.0f) {
						body_x = -body_x;
					}

					body_x.normalize();

				} else {
					/* desired thrust is in XY plane, set X downside to construct correct matrix,
					 * but yaw component will not be used actually */
					body_x.zero();
					body_x(2) = 1.0f;
				}

				/* desired body_y axis */
				body_y = body_z % body_x;

				/* fill rotation matrix */
				for (int i = 0; i < 3; i++) {
					R(i, 0) = body_x(i);
					R(i, 1) = body_y(i);
					R(i, 2) = body_z(i);
				}

				/* copy rotation matrix to attitude setpoint topic */
				memcpy(&_att_sp.R_body[0], R.data, sizeof(_att_sp.R_body));
				_att_sp.R_valid = true;

				/* calculate euler angles, for logging only, must not be used for control */
				math::Vector<3> euler = R.to_euler();
				_att_sp.roll_body = euler(0);
				_att_sp.pitch_body = euler(1);
				/* yaw already used to construct rot matrix, but actual rotation matrix can have different yaw near singularity */
			}
				//_att_sp.thrust = thrust_sp(2) < 0 ? thrust_abs : 0.0f;
				_att_sp.thrust = thrust_abs;

			_att_sp.timestamp = hrt_absolute_time();

			/* publish attitude setpoint */
			_att_sp.update();
		}

	} else {
		/* position controller disabled, reset setpoints */
		setpoint.reset_xyz_sp(_local_pos.x, _local_pos.y, _local_pos.z);
	}
	/* END copy and convert */


	/* set flag if pos control was active in this iteration */
	_was_poscontrol = run_controller;

	if (vicon_pos_updated) {
		t_lastvicon = hrt_absolute_time();
	}

	if (gps_updated_valid) {
		t_lastgps = hrt_absolute_time();
	}

	counter++;

	perf_end(_perfCounterDuration);

}

int MulticopterPositionControlHcf::initStateIfNecessary(const math::Vector<3> &positionLocal, const math::Vector<3> &velocityLocal)
{
	if (_initialized) {
		/* already initialized, nothing to do */
		return -1;
	}

	/* Not yet initialized: do initialization */
	_posCtlArgs.out.x_apo[INDEX_STATE_PX] =  positionLocal(0);
	_posCtlArgs.out.x_apo[INDEX_STATE_PY] =  positionLocal(1);
	_posCtlArgs.out.x_apo[INDEX_STATE_PZ] =  -positionLocal(2); //filter/matlab uses z up
	_posCtlArgs.out.x_apo[INDEX_STATE_VX] =  velocityLocal(0);
	_posCtlArgs.out.x_apo[INDEX_STATE_VY] =  velocityLocal(1);
	_posCtlArgs.out.x_apo[INDEX_STATE_VZ] =  -velocityLocal(2); //filter/matlab uses z up

	_initialized = true;

	return 0;

}

void MulticopterPositionControlHcf::debug_print(const char *fmt, va_list args)
{
		fprintf(stderr, "%s: ", "[mc_pos_control_hcf]");
		vfprintf(stderr, fmt, args);

		fprintf(stderr, "\n");
}

void MulticopterPositionControlHcf::debug(const char *fmt, ...) {

		if (!_debug) {
			return;
		}

		va_list args;

		va_start(args, fmt);
		debug_print(fmt, args);
}


void MulticopterPositionControlHcf::publishEstimates(bool publishLocalPos, bool publishGlobalPos)
{

	if (publishLocalPos) {
		//debug("*** zflag %u %u %u %u meas %.3f  state %.3f state prio %.3f", _lqg_zFlag[0],  _lqg_zFlag[1], _lqg_zFlag[2], _lqg_zFlag[3], _lqg_z[INDEX_MEAS_PZ], _lqg_x_apo[INDEX_STATE_PZ], _lqg_x_apo_k[INDEX_STATE_PZ]);
		//if (_lqg_zFlag[3]) {
			//debug("x_pre[offeset] %.3f x_pre[z] %.3f predicted pressure %.3f alt offset %.3f", _lqg_x_apo_k[INDEX_STATE_PRESSUREOFFSET], _lqg_x_apo_k[INDEX_STATE_PZ], _lqg_debugOutput[0], _lqg_local_altitude_offset);
			//debug("z[pressure] %.3f x[offeset] %.3f", _lqg_z[INDEX_MEAS_PRESSURE], _lqg_x_apo[INDEX_STATE_PRESSUREOFFSET]);
		//}

		_local_pos.timestamp = hrt_absolute_time();
		_local_pos.x = _posCtlArgs.out.x_apo[INDEX_STATE_PX];
		_local_pos.vx = _posCtlArgs.out.x_apo[INDEX_STATE_VX];
		_local_pos.y = _posCtlArgs.out.x_apo[INDEX_STATE_PY];
		_local_pos.vy = _posCtlArgs.out.x_apo[INDEX_STATE_VY];
		_local_pos.z = -_posCtlArgs.out.x_apo[INDEX_STATE_PZ];
		_local_pos.vz = -_posCtlArgs.out.x_apo[INDEX_STATE_VZ];
		_local_pos.xy_valid = true;
		_local_pos.v_xy_valid = true;
		//_local_pos.landed = false; TODO: is necessary

		_local_pos.update();
	}

	if (publishGlobalPos) {
		/* Reproject local pos to global pos */
		globallocalconverter_toglobal(_local_pos.x, _local_pos.y, _local_pos.z, &_global_pos.lat,
				&_global_pos.lon, &_global_pos.alt);
		_global_pos.vel_n = _local_pos.vx;
		_global_pos.vel_e = _local_pos.vy;
		_global_pos.vel_d = _local_pos.vz;
		_global_pos.eph = sqrtf(_posCtlArgs.out.variance_horizontal);
		_global_pos.epv = sqrtf(_posCtlArgs.out.variance_vertical);
		_global_pos.timestamp = hrt_absolute_time();

		/* Publish global pos */
		_global_pos.update();
	}
}
