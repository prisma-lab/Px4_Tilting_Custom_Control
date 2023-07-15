/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#pragma once

//#include "PositionControl/PositionControl.hpp"
//#include "Takeoff/Takeoff.hpp"

#include "PositionControlBase/PositionControlPass.hpp"
#include "PositionControlBase/PositionControlGeom.hpp"
#include "PositionControlBase/PositionControlTilt.hpp"

#include <modules/mc_pos_control/Takeoff/Takeoff.hpp>

#include <drivers/drv_hrt.h>

#include <matrix/matrix/math.hpp>
#include <lib/controllib/blocks.hpp>
#include <lib/slew_rate/SlewRateYaw.hpp>
#include <lib/hysteresis/hysteresis.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/takeoff_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/prisma_virtual_acc_setpoint.h>
#include <uORB/topics/prisma_geom_pos_out.h>
#include <uORB/topics/prisma_tilt_pos_out.h>

#include <uORB/topics/debug_vect.h>
#include <uORB/topics/debug_key_value.h>
#include <cstring>

// #define TILT_CONTROL
#define GEOM_CONTROL
// #define PASS_CONTROL

#if defined GEOM_CONTROL
	#define POS_OUT_S prisma_geom_pos_out_s
	#define POS_OUT_ORB_ID ORB_ID(prisma_geom_pos_out)
	#define CONTROL_TYPE PositionControlGeom
#elif defined PASS_CONTROL
	#define POS_OUT_S prisma_virtual_acc_setpoint_s
	#define POS_OUT_ORB_ID ORB_ID(prisma_virtual_acc_setpoint)
	#define CONTROL_TYPE PositionControlPass
#elif defined TILT_CONTROL
	#define POS_OUT_S prisma_tilt_pos_out_s
	#define POS_OUT_ORB_ID ORB_ID(prisma_tilt_pos_out)
	#define CONTROL_TYPE PositionControlTilt
#endif

using namespace time_literals;

extern "C" __EXPORT int prisma1control_main(int argc, char *argv[]);

class Prisma1Control : public ModuleBase<Prisma1Control>, public control::SuperBlock,
	public ModuleParams, public px4::ScheduledWorkItem
{
public:
	Prisma1Control();
	virtual ~Prisma1Control() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	bool _is_active;
	bool _flying;

	/** @see ModuleBase::run() */
	void Run() override;

	long _counter;

	Takeoff _takeoff;

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);

	PositionControlState set_vehicle_state(const vehicle_local_position_s &local_pos);
	PositionControlInput set_control_input(const vehicle_local_position_setpoint_s &setpoint);

	// Publications
	uORB::Publication<POS_OUT_S> _pos_out_pub{POS_OUT_ORB_ID};
	uORB::Publication<vehicle_local_position_setpoint_s> _local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::PublicationData<takeoff_status_s>              _takeoff_status_pub {ORB_ID(takeoff_status)};

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _local_pos_sub {this, ORB_ID(vehicle_local_position)};	/**< vehicle local position */
	
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _hover_thrust_estimate_sub {ORB_ID(hover_thrust_estimate)};
	uORB::Subscription _trajectory_setpoint_sub {ORB_ID(trajectory_setpoint)};
	uORB::Subscription _vehicle_constraints_sub {ORB_ID(vehicle_constraints)};
	uORB::Subscription _vehicle_control_mode_sub {ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_land_detected_sub {ORB_ID(vehicle_land_detected)};

	vehicle_local_position_setpoint_s _setpoint {};
	vehicle_control_mode_s _vehicle_control_mode {};

	vehicle_constraints_s _vehicle_constraints {
		.timestamp = 0,
		.speed_up = NAN,
		.speed_down = NAN,
		.want_takeoff = false,
	};

	vehicle_land_detected_s _vehicle_land_detected {
		.timestamp = 0,
		.freefall = false,
		.ground_contact = true,
		.maybe_landed = true,
		.landed = true,
	};

	hrt_abstime	_time_stamp_last_loop{0};		/**< time stamp of last loop iteration */


	control::BlockDerivative _vel_x_deriv; /**< velocity derivative in x */
	control::BlockDerivative _vel_y_deriv; /**< velocity derivative in y */
	control::BlockDerivative _vel_z_deriv; /**< velocity derivative in z */

	CONTROL_TYPE _control;
	
	hrt_abstime _last_warn{0}; /**< timer when the last warn message was sent out */

	bool _in_failsafe{false};  /**< true if failsafe was entered within current cycle */

	bool _hover_thrust_initialized{false};

	/** Timeout in us for trajectory data to get considered invalid */
	static constexpr uint64_t TRAJECTORY_STREAM_TIMEOUT_US = 500_ms;

	/** If Flighttask fails, keep 0.2 seconds the current setpoint before going into failsafe land */
	static constexpr uint64_t LOITER_TIME_BEFORE_DESCEND = 200_ms;

	/** During smooth-takeoff, below ALTITUDE_THRESHOLD the yaw-control is turned off and tilt is limited */
	static constexpr float ALTITUDE_THRESHOLD = 0.3f;

	static constexpr float MAX_SAFE_TILT_DEG = 89.f; // Numerical issues above this value due to tanf

	systemlib::Hysteresis _failsafe_land_hysteresis{false}; /**< becomes true if task did not update correctly for LOITER_TIME_BEFORE_DESCEND */
	SlewRate<float> _tilt_limit_slew_rate;

	uint8_t _vxy_reset_counter{0};
	uint8_t _vz_reset_counter{0};
	uint8_t _xy_reset_counter{0};
	uint8_t _z_reset_counter{0};
	uint8_t _heading_reset_counter{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MPC_SPOOLUP_TIME>) _param_mpc_spoolup_time, /**< time to let motors spool up after arming */
		(ParamFloat<px4::params::MPC_TKO_RAMP_T>)   _param_mpc_tko_ramp_t,   /**< time constant for smooth takeoff ramp */
		(ParamFloat<px4::params::MPC_Z_VEL_P_ACC>)  _param_mpc_z_vel_p_acc,
		(ParamFloat<px4::params::MPC_TILTMAX_LND>)  _param_mpc_tiltmax_lnd,  /**< maximum tilt for landing and smooth takeoff */
		(ParamFloat<px4::params::MPC_TILTMAX_AIR>)  _param_mpc_tiltmax_air,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_mpc_z_vel_max_up,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_mpc_z_vel_max_dn,
		(ParamFloat<px4::params::MPC_LAND_SPEED>)   _param_mpc_land_speed,
		(ParamFloat<px4::params::PRISMA_THR>) 			_param_thr,
		(ParamFloat<px4::params::PRISMA_KP_XY>)     _param_xy_p,
		(ParamFloat<px4::params::PRISMA_KP_Z>)      _param_z_p,
		(ParamFloat<px4::params::PRISMA_KD_XY>)     _param_xy_v,
		(ParamFloat<px4::params::PRISMA_KD_Z>)      _param_z_v,
		(ParamFloat<px4::params::PRISMA_KI_XY>)     _param_xy_i,
		(ParamFloat<px4::params::PRISMA_KI_Z>)      _param_z_i,
		(ParamFloat<px4::params::PRISMA_MASS>)      _param_mass,
		(ParamFloat<px4::params::PRISMA_C1>)        _param_c1,
		(ParamFloat<px4::params::PRISMA_SIGMA>)     _param_sigma,
		(ParamFloat<px4::params::PRISMA_Z_INT_S>)   _param_start_z_int
	);
	
	void reset_setpoint_to_nan(vehicle_local_position_setpoint_s &setpoint);
};

