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

#include "prisma1control.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>

using namespace matrix;


Prisma1Control::Prisma1Control() :
	SuperBlock(nullptr, "P1C"),
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_vel_x_deriv(this, "VELD"),
	_vel_y_deriv(this, "VELD"),
	_vel_z_deriv(this, "VELD")
{
	parameters_update(true);
	_tilt_limit_slew_rate.setSlewRate(.2f);
	_takeoff_status_pub.advertise();

	PX4_WARN("Prisma1Control started!!");

	_counter = 0;
	_is_active = false;
	_flying = false;
}

bool Prisma1Control::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_time_stamp_last_loop = hrt_absolute_time();
	ScheduleNow();

	return true;
}

void Prisma1Control::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		ModuleParams::updateParams();
		SuperBlock::updateParams();


		// Update control parameters
		_control.setPosGains(Vector3f(_param_xy_p.get(), _param_xy_p.get(), _param_z_p.get()));
		_control.setVelGains(Vector3f(_param_xy_v.get(), _param_xy_v.get(), _param_z_v.get()));
		_control.setIntGains(Vector3f(_param_xy_i.get(), _param_xy_i.get(), _param_z_i.get()));
		_control.setMass(_param_mass.get());
		_control.setStartZInt(_param_start_z_int.get());
		#ifdef GEOM_CONTROL
		_control.setSigma(_param_sigma.get());
		_control.setC1(_param_c1.get());
		#endif

		_takeoff.setSpoolupTime(_param_mpc_spoolup_time.get());
		_takeoff.setTakeoffRampTime(_param_mpc_tko_ramp_t.get());
		_takeoff.generateInitialRampValue(_param_mpc_z_vel_p_acc.get());
	}
}

PositionControlState Prisma1Control::set_vehicle_state(const vehicle_local_position_s &local_pos)
{
	PositionControlState state;

	// only set position states if valid and finite
	if (PX4_ISFINITE(local_pos.x) && PX4_ISFINITE(local_pos.y) && local_pos.xy_valid) {
		state.position(0) = local_pos.x;
		state.position(1) = local_pos.y;

	} else {
		state.position(0) = NAN;
		state.position(1) = NAN;
	}

	if (PX4_ISFINITE(local_pos.z) && local_pos.z_valid) {
		state.position(2) = local_pos.z;

	} else {
		state.position(2) = NAN;
	}

	if (PX4_ISFINITE(local_pos.vx) && PX4_ISFINITE(local_pos.vy) && local_pos.v_xy_valid) {
		state.velocity(0) = local_pos.vx;
		state.velocity(1) = local_pos.vy;

	} else {
		state.velocity(0) = NAN;
		state.velocity(1) = NAN;

		// reset derivatives to prevent acceleration spikes when regaining velocity
		_vel_x_deriv.reset();
		_vel_y_deriv.reset();
	}

	if (PX4_ISFINITE(local_pos.vz) && local_pos.v_z_valid) {
		state.velocity(2) = local_pos.vz;

	} else {
		state.velocity(2) = NAN;

		// reset derivative to prevent acceleration spikes when regaining velocity
		_vel_z_deriv.reset();
	}
	return state;
}

PositionControlInput Prisma1Control::set_control_input(const vehicle_local_position_setpoint_s &setpoint) {
	PositionControlInput input;

	input.position_sp = Vector3f(setpoint.x, setpoint.y, setpoint.z);
	input.velocity_sp = Vector3f(setpoint.vx, setpoint.vy, setpoint.vz);
	input.acceleration_sp = Vector3f(setpoint.acceleration[0], setpoint.acceleration[1], setpoint.acceleration[2]);
	input.yaw_sp = setpoint.yaw;
	input.yaw_dot_sp = setpoint.yawspeed;
	input.yaw_ddot_sp = 0.0f;

	return input;
}

int Prisma1Control::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int Prisma1Control::task_spawn(int argc, char *argv[])
{
	Prisma1Control *instance = new Prisma1Control();
		
	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}


void Prisma1Control::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	_counter = (_counter+1)%100;

	// reschedule backup
	ScheduleDelayed(100_ms);

	parameters_update(false);

	vehicle_local_position_s local_pos;

	if (_local_pos_sub.update(&local_pos)) {
		const hrt_abstime time_stamp_now = local_pos.timestamp_sample;
		const float dt = math::constrain(((time_stamp_now - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);
		_time_stamp_last_loop = time_stamp_now;

		// set _dt in controllib Block for BlockDerivative
		setDt(dt);
		
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);
		_vehicle_land_detected_sub.update(&_vehicle_land_detected);

		PositionControlState state{set_vehicle_state(local_pos)};

		if (_vehicle_control_mode.flag_control_prisma_enabled) {
			const bool is_trajectory_setpoint_updated = (_trajectory_setpoint_sub.update(&_setpoint));

			if(!_is_active || !_vehicle_control_mode.flag_armed){
				_is_active = true;
				_control.resetIntegral();
				// PX4_WARN("Resetting position integral");
			}		

			// adjust existing (or older) setpoint with any EKF reset deltas
			if (_setpoint.timestamp < local_pos.timestamp) {
				if (local_pos.vxy_reset_counter != _vxy_reset_counter) {
					_setpoint.vx += local_pos.delta_vxy[0];
					_setpoint.vy += local_pos.delta_vxy[1];
				}

				if (local_pos.vz_reset_counter != _vz_reset_counter) {
					_setpoint.vz += local_pos.delta_vz;
				}

				if (local_pos.xy_reset_counter != _xy_reset_counter) {
					_setpoint.x += local_pos.delta_xy[0];
					_setpoint.y += local_pos.delta_xy[1];
				}

				if (local_pos.z_reset_counter != _z_reset_counter) {
					_setpoint.z += local_pos.delta_z;
				}

				if (local_pos.heading_reset_counter != _heading_reset_counter) {
					_setpoint.yaw += local_pos.delta_heading;
				}
			}

			// update vehicle constraints and handle smooth takeoff
			_vehicle_constraints_sub.update(&_vehicle_constraints);
			
			// fix to prevent the takeoff ramp to ramp to a too high value or get stuck because of NAN
			// TODO: this should get obsolete once the takeoff limiting moves into the flight tasks
			if (!PX4_ISFINITE(_vehicle_constraints.speed_up) || (_vehicle_constraints.speed_up > _param_mpc_z_vel_max_up.get())) {
				_vehicle_constraints.speed_up = _param_mpc_z_vel_max_up.get();
			}

			if (_vehicle_control_mode.flag_control_offboard_enabled) {

				bool want_takeoff = _vehicle_control_mode.flag_armed && _vehicle_land_detected.landed
						    && hrt_elapsed_time(&_setpoint.timestamp) < 1_s;

				if (want_takeoff && PX4_ISFINITE(_setpoint.z)
				    && (_setpoint.z < state.position(2))) {

					_vehicle_constraints.want_takeoff = true;

				} else if (want_takeoff && PX4_ISFINITE(_setpoint.vz)
					   && (_setpoint.vz < 0.f)) {

					_vehicle_constraints.want_takeoff = true;

				} else if (want_takeoff && PX4_ISFINITE(_setpoint.acceleration[2])
					   && (_setpoint.acceleration[2] < 0.f)) {

					_vehicle_constraints.want_takeoff = true;

				} else {
					_vehicle_constraints.want_takeoff = false;
				}

				// override with defaults
				_vehicle_constraints.speed_up = _param_mpc_z_vel_max_up.get();
				_vehicle_constraints.speed_down = _param_mpc_z_vel_max_dn.get();
			}

			// handle smooth takeoff
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed,
						    _vehicle_constraints.want_takeoff,
						    _vehicle_constraints.speed_up, false, time_stamp_now);

			const bool flying = (_takeoff.getTakeoffState() >= TakeoffState::flight);

			if (is_trajectory_setpoint_updated) {
				// make sure takeoff ramp is not amended by acceleration feed-forward
				if (!flying) {
					_setpoint.acceleration[2] = NAN;
					// hover_thrust maybe reset on takeoff
					// _control.setHoverThrust(_param_mpc_thr_hover.get());
				}

				const bool not_taken_off             = (_takeoff.getTakeoffState() < TakeoffState::rampup);
				const bool flying_but_ground_contact = (flying && _vehicle_land_detected.ground_contact);

				if (not_taken_off || flying_but_ground_contact) {
					reset_setpoint_to_nan(_setpoint);

					Vector3f(0.f, 0.f, 100.f).copyTo(_setpoint.acceleration); // High downwards acceleration to make sure there's no thrust

					// prevent any integrator windup
					_control.resetIntegral();
				}
			}
			
			const float tilt_limit_deg = (_takeoff.getTakeoffState() < TakeoffState::flight)
			     ? _param_mpc_tiltmax_lnd.get() : _param_mpc_tiltmax_air.get();
			_tilt_limit_slew_rate.update(math::radians(tilt_limit_deg), dt);

			// const float speed_up = _takeoff.updateRamp(dt,
			// 		       PX4_ISFINITE(_vehicle_constraints.speed_up) ? _vehicle_constraints.speed_up : _param_mpc_z_vel_max_up.get());
			// const float speed_down = PX4_ISFINITE(_vehicle_constraints.speed_down) ? _vehicle_constraints.speed_down :
			// 			 _param_mpc_z_vel_max_dn.get();

			// Allow ramping from zero thrust on takeoff
			// const float minimum_thrust = flying ? _param_mpc_thr_min.get() : 0.f;

			// _control.setThrustLimits(minimum_thrust, _param_mpc_thr_max.get());

			// _control.setVelocityLimits(
			// 	_param_mpc_xy_vel_max.get(),
			// 	math::min(speed_up, _param_mpc_z_vel_max_up.get()), // takeoff ramp starts with negative velocity limit
			// 	math::max(speed_down, 0.f));

			
			// Here I set the input to the controller
			PositionControlInput input{set_control_input(_setpoint)};
			
			_control.setInputSetpoint(input);

			if(!_counter) {
				// PX4_INFO("----------------------------");
				// PX4_INFO("Setpoint position:        %f, %f, %f",
				// 	(double)_setpoint.x, (double)_setpoint.y, (double)_setpoint.z);
				// PX4_INFO("Setpoint velocity:        %f, %f, %f",
				// 	(double)_setpoint.vx, (double)_setpoint.vy, (double)_setpoint.vz);
				// PX4_INFO("Setpoint acceleration:    %f, %f, %f",
				// 	(double)_setpoint.acceleration[0], (double)_setpoint.acceleration[1], (double)_setpoint.acceleration[2]);
				// PX4_INFO("Setpoint yaw and yaw_dot: %f, %f",
				// 	(double)_setpoint.yaw, (double)_setpoint.yawspeed);
			}

			/*const float speed_up = */_takeoff.updateRamp(dt,
				PX4_ISFINITE(_vehicle_constraints.speed_up) ? _vehicle_constraints.speed_up : _param_mpc_z_vel_max_up.get());

			// PX4_INFO("speed_up = %f", (double)speed_up);
			
			// update states
			if (!PX4_ISFINITE(_setpoint.z)
			    && PX4_ISFINITE(_setpoint.vz) && (fabsf(_setpoint.vz) > FLT_EPSILON)
			    && PX4_ISFINITE(local_pos.z_deriv) && local_pos.z_valid && local_pos.v_z_valid) {
				// A change in velocity is demanded and the altitude is not controlled.
				// Set velocity to the derivative of position
				// because it has less bias but blend it in across the landing speed range
				//  <  MPC_LAND_SPEED: ramp up using altitude derivative without a step
				//  >= MPC_LAND_SPEED: use altitude derivative
				float weighting = fminf(fabsf(_setpoint.vz) / _param_mpc_land_speed.get(), 1.f);
				state.velocity(2) = local_pos.z_deriv * weighting + local_pos.vz * (1.f - weighting);
			}

			// Here I set the state of the UAV in the controller
			_control.setState(state);

			// Run position control
			if (_control.update(dt)) {
				_failsafe_land_hysteresis.set_state_and_update(false, time_stamp_now);
			} else {
				// Failsafe				
			}
			
			// Publish internal position control setpoints
			vehicle_local_position_setpoint_s local_pos_sp{};
			_control.getLocalPositionSetpoint(local_pos_sp);
			local_pos_sp.timestamp = hrt_absolute_time();
			for(int i=0; i<3; i++) {
				local_pos_sp.thrust[i] /= _param_thr.get();
			}
			_local_pos_sp_pub.publish(local_pos_sp);

			// PX4_INFO("--------------------");
			// PX4_INFO("Input z velocity: %f", (double)_setpoint.vz);
			// PX4_INFO("Thrust:           %f", (double)local_pos_sp.thrust[2]);

			// Publish the output of the position controller
			POS_OUT_S pos_out;
			_control.getControlOutput(&pos_out);
			_pos_out_pub.publish(pos_out);


			// Publish takeoff status
			const uint8_t takeoff_state = static_cast<uint8_t>(_takeoff.getTakeoffState());

			if ((takeoff_state != _takeoff_status_pub.get().takeoff_state
				|| !isEqualF(_tilt_limit_slew_rate.getState(), _takeoff_status_pub.get().tilt_limit))
				&& _vehicle_control_mode.flag_control_prisma_enabled) {
				// PX4_WARN("Publishing takeoff status: %u", takeoff_state);
				_takeoff_status_pub.get().takeoff_state = takeoff_state;
				_takeoff_status_pub.get().tilt_limit = _tilt_limit_slew_rate.getState();
				_takeoff_status_pub.get().timestamp = hrt_absolute_time();
				_takeoff_status_pub.update();
			}

		} else {
			// an update is necessary here because otherwise the takeoff state doesn't get skiped with non-altitude-controlled modes
			_is_active = false;
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed, false, 10.f, true,
						    time_stamp_now);
		}


		// save latest reset counters
		_vxy_reset_counter = local_pos.vxy_reset_counter;
		_vz_reset_counter = local_pos.vz_reset_counter;
		_xy_reset_counter = local_pos.xy_reset_counter;
		_z_reset_counter = local_pos.z_reset_counter;
		_heading_reset_counter = local_pos.heading_reset_counter;
	}

}

int Prisma1Control::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Custom controller

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

void Prisma1Control::reset_setpoint_to_nan(vehicle_local_position_setpoint_s &setpoint)
{
	setpoint.x = setpoint.y = setpoint.z = NAN;
	setpoint.vx = setpoint.vy = setpoint.vz = NAN;
	setpoint.yaw = setpoint.yawspeed = NAN;
	setpoint.acceleration[0] = setpoint.acceleration[1] = setpoint.acceleration[2] = NAN;
	setpoint.thrust[0] = setpoint.thrust[1] = setpoint.thrust[2] = NAN;
}


int prisma1control_main(int argc, char *argv[])
{
	return Prisma1Control::main(argc, argv);
}
