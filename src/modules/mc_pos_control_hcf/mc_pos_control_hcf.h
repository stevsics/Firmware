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
 * @file mc_pos_control_hcf.h
 * Multicopter position controller for the HCF project.
 *
 *  @author Thomas Gubler <thomasgubler@gmail.com>
 *  @author Tobias Naegeli <naegelit@student.ethz.ch>
 */

#include <mathlib/mathlib.h>
#include <systemlib/param/param.h>
#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/vehicle_vicon_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_force_setpoint.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/sensor_combined.h>
#include <drivers/drv_accel.h>
#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>
#include <drivers/drv_hrt.h>
#include <systemlib/perf_counter.h>

#include "setpoint.h"
#include "codegen/positionLQGC.h"

/**
 * Multicopter position control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_pos_control_hcf_main(int argc, char *argv[]);

class MulticopterPositionControlHcf : public control::SuperBlock
{
public:
	/**
	 * Constructor
	 */
	MulticopterPositionControlHcf(SuperBlock *parent, const char *name);

	/**
	 * Destructor, also kills task.
	 */
	~MulticopterPositionControlHcf();

	/**
	 * Main sensor collection task.
	 */
	void update();

	/**
	 * Publish estimates
	 */
	void publishEstimates(bool publishLocalPos, bool publishGlobalPos);

private:

	int		_mavlink_fd;			/**< mavlink fd */
	bool		_global_pos_mode_enabled;		/**< if set to true the reprojection of the estimation is
							  published as global pos */
	/* Publications */
	uORB::Publication<vehicle_attitude_setpoint_s> _att_sp;       /**< attitude setpoint publication */
	uORB::Publication<vehicle_local_position_s> _local_pos;	/**< vehicle local pos publication: for now from vicon */
	uORB::Publication<vehicle_global_position_s> _global_pos;	/**< vehicle local pos publication: for now from vicon */
	uORB::Publication<debug_key_value_s> *_dkv[7];
	uORB::Publication<position_setpoint_triplet_s>		_pos_sp_triplet_pub;	/**< position setpoint triplet
											  publication, only active
											  when position is
											  controlled with stick */
	uORB::Publication<vehicle_force_setpoint_s> _force_sp;	/**< force setpoint */

	/* Subscriptions */
	uORB::Subscription<vehicle_attitude_s> 			_att;			/**< vehicle attitude */
	uORB::Subscription<manual_control_setpoint_s>		_manual;		/**< r/c channel data */
	uORB::Subscription<vehicle_control_mode_s>		_control_mode;		/**< vehicle control mode */
	uORB::Subscription<actuator_armed_s>			_arming;		/**< actuator arming status */
	uORB::Subscription<vehicle_vicon_position_s>		_vicon_pos;		/**< vehicle vicon position */
	uORB::Subscription<vehicle_gps_position_s>		_gps_pos;		/**< vehicle gps position */
	uORB::Subscription<position_setpoint_triplet_s>		_pos_sp_triplet;	/**< position setpoint triplet */
	uORB::Subscription<sensor_combined_s>			_sensor_combined;	/**< all onboard sensor data */
	uORB::Subscription<accel_report>			_sensor_accel;		/**< acceleromter data */

	/* Params */
	control::BlockParamFloat _lqgK0XY;
	control::BlockParamFloat _lqgK1XY;
	control::BlockParamFloat _lqgK2XY;
	control::BlockParamFloat _lqgK0Z;
	control::BlockParamFloat _lqgK1Z;
	control::BlockParamFloat _lqgK2Z;
	control::BlockParamFloat _lqgCostPos;
	control::BlockParamFloat _lqgCostVel;
	control::BlockParamFloat _lqgCostU;
	control::BlockParamFloat _varNoiseProcess;
	control::BlockParamFloat _varNoiseMeasurementXY;
	control::BlockParamFloat _varNoiseMeasurementZ;
	control::BlockParamFloat _varNoiseMeasurementVXY;
	control::BlockParamFloat _varNoiseMeasurementVZ;
	control::BlockParamFloat _varNoiseMeasurementAXY;
	control::BlockParamFloat _varNoiseMeasurementAZ;
	control::BlockParamFloat _varNoiseProcessBaroOffset;
	control::BlockParamFloat _varNoiseMeasurementBaro;
	control::BlockParamFloat _mass;
	control::BlockParamFloat _maxLiftKg;
	control::BlockParamInt _useControlOutputForPrediction;
	control::BlockParamFloat _setpointVelScale;
	control::BlockParamInt _setVelocitySetpointEnabled;
	control::BlockParamInt _setAccelerationSetpointEnabled;
	control::BlockParamInt _positionVarianceOverrideEnabled;
	control::BlockParamFloat _pressureOffsetManual;
	control::BlockParamFloat _intLimXY;
	control::BlockParamFloat _intLimZ;
	control::BlockParamFloat _uLimXY;
	control::BlockParamFloat _uLimZ;

	Setpoint setpoint;		/**< The current setpoint in use for the controller */
	GlobalSetpoint externalSetpoint; /**< The lat/lon/alt setpoint given by the navigator */
	bool _was_armed;
	bool _was_poscontrol;
	bool _initialized;		/**< Set to true after the first valid position measruement (position
					  measurements are only accepted once the globallocal converter was initialized)
					  */

	math::Vector<3> sp_move_rate;
	math::Vector<3> thrust_int;
	math::Matrix<3, 3> R;

	const float alt_ctl_dz = 0.2f;
	const float pos_ctl_dz = 0.05f;

	hrt_abstime t_prev;	    /**< Timestamp of last iteration */
	hrt_abstime t_lastvicon;
	hrt_abstime dt_vicon;
	hrt_abstime t_lastgps;
	hrt_abstime dt_gps;

	/* Const indices to be used with the matlab arguments */
	static const size_t NSTATES = 10;
	static const size_t NMEASUREMENTS = 10;
	const size_t INDEX_STATE_PX = 0;
	const size_t INDEX_STATE_VX = 1;
	const size_t INDEX_STATE_AX = 2;
	const size_t INDEX_STATE_PY = 3;
	const size_t INDEX_STATE_VY = 4;
	const size_t INDEX_STATE_AY = 5;
	const size_t INDEX_STATE_PZ = 6;
	const size_t INDEX_STATE_VZ = 7;
	const size_t INDEX_STATE_AZ = 8;
	const size_t INDEX_STATE_PRESSUREOFFSET = 9;
	const size_t INDEX_MEAS_PX = 0;
	const size_t INDEX_MEAS_VX = 1;
	const size_t INDEX_MEAS_AX = 2;
	const size_t INDEX_MEAS_PY = 3;
	const size_t INDEX_MEAS_VY = 4;
	const size_t INDEX_MEAS_AY = 5;
	const size_t INDEX_MEAS_PZ = 6;
	const size_t INDEX_MEAS_VZ = 7;
	const size_t INDEX_MEAS_AZ = 8;
	const size_t INDEX_MEAS_PRESSURE = 9;

	/* Variables needed for the call to the matlab generated pos control function */
	struct PositionControllerArguments {
		struct inputs {
			float q;
			float r_position[3];
			float r_velocity[3];
			float r_acceleration[3];
			float k[6];
			float x_apo_k[NSTATES];
			float z[NMEASUREMENTS];
			boolean_T zFlag[4];
			float F_global[3];
			boolean_T intfreeze;
			float setpPosRaw[3];
			float setpVelRaw[3];
			float setpAccRaw[3];
			boolean_T intReset;
			boolean_T useControlOutputForPrediction[2];
			float dxmax;
			float q_baro_offset;
			float r_baro;
			float deltaT;
			float intLim[3];
			float uLim[3];
			float local_altitude_offset;
		} in;

	/* LQG outputs */
		struct outputs {
			float x_apo[NSTATES];
			float u[3];
			float F_desProp[3];
			float variance_horizontal;
			float variance_vertical;
			float debugOutput[4];
		} out;
	} _posCtlArgs;

	math::Vector<3> _position_measurement_inertial;	/**< vehicle position measurement transformed into
							  inertial frame (measurement/ground truth) */
	math::Vector<3> _velocity_measurement_inertial;	/**< vehicle velocity transformed into
							  inertial frame (measurement/ground truth) */
	math::Vector<3> _acceleration_measurement_ned;	/**< vehicle acceleration transformed into
							  ned frame at vehicle location (measurement/ground truth) */

	/* Performance counters */
	perf_counter_t _perfCounterDt;
	perf_counter_t _perfCounterDuration;

	bool _debug;			/**< Set true to enable debug output */

	static float	scale_control(float ctl, float end, float dz);

	/**
	 * Initialize state of the filter if not yet initialized
	 */
	int initStateIfNecessary(const math::Vector<3> &positionLocal, const math::Vector<3> &velocityLocal);


	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	static void debug_print(const char *fmt, va_list args);
	void	    debug(const char *fmt, ...);
};
