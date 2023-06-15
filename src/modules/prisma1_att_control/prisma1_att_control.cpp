#include "prisma1_att_control.hpp"

#include <px4_platform_common/posix.h>
#include <uORB/topics/parameter_update.h>

using namespace matrix;

Prisma1AttitudeControl::Prisma1AttitudeControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl) {
	
	parameters_update(true);

	PX4_WARN("Prisma Attitude Control started!!");

	_counter = 0;
	_is_active = false;
}

bool Prisma1AttitudeControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void Prisma1AttitudeControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		ModuleParams::updateParams();

		_control.setMass(_param_mass.get());
		_control.setIb(_param_ibx.get(), _param_iby.get(), _param_ibz.get());
		_control.setThrust(_param_thr.get());
		_control.setXYZTorque(_param_x_tor.get(), _param_y_tor.get(), _param_z_tor.get());
		#if defined GEOM_CONTROL
		_control.setKr(Vector3f(_param_xy_kr.get(), _param_xy_kr.get(), _param_z_kr.get()));
		_control.setKom(Vector3f(_param_xy_kom.get(), _param_xy_kom.get(), _param_z_kom.get()));
		_control.setKi(Vector3f(_param_xy_ki.get(), _param_xy_ki.get(), _param_z_ki.get()));
		_control.setC2(_param_c2.get());
		#elif defined TILT_CONTROL
		_control.setKr(Vector3f(_param_xy_kr.get(), _param_xy_kr.get(), _param_z_kr.get()));
		_control.setKq(Vector3f(_param_xy_kq.get(), _param_xy_kq.get(), _param_z_kq.get()));
		_control.setAngleInputMode(_param_angleInputMode.get());
		#elif defined PASS_CONTROL
		#endif
	}
}

int Prisma1AttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int Prisma1AttitudeControl::task_spawn(int argc, char *argv[])
{
	Prisma1AttitudeControl *instance = new Prisma1AttitudeControl();
		
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

void Prisma1AttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	_counter = (_counter+1)%500;

	parameters_update(false);

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		const hrt_abstime time_stamp_now = angular_velocity.timestamp_sample;
		const float dt = math::constrain(((time_stamp_now - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);
		_time_stamp_last_loop = time_stamp_now;

		_vehicle_control_mode_sub.update(&_vehicle_control_mode);
		if(_vehicle_attitude_sub.updated())
			_vehicle_attitude_sub.update(&_attitude);

		AttitudeControlState state{set_vehicle_state(_attitude, angular_velocity)};

		if (_vehicle_control_mode.flag_control_prisma_enabled) {
			

			if(_trajectory_setpoint_sub.updated())
				_trajectory_setpoint_sub.update(&_traj_sp);

			hover_thrust_estimate_s hte;

			if (_hover_thrust_estimate_sub.update(&hte)) {
				if (hte.valid) {
					//_control.updateHoverThrust(hte.hover_thrust);
				}
			}

			_setpoint.yaw_sp = _traj_sp.yaw;
			_setpoint.yaw_dot_sp = _traj_sp.yawspeed;
			_setpoint.yaw_ddot_sp = 0.0f;
			_control.setInputSetpoint(_setpoint);

			_control.setState(state);

			if(!_is_active){
				_control.resetIntegral();
				_is_active = true;

				#ifdef TILT_CONTROL
				//_control.resetBuffers();
				#endif
			}

			// Run position control
			if (_control.update(dt)) {
				_failsafe_land_hysteresis.set_state_and_update(false, time_stamp_now);

			} else {
				// Failsafe
				
			}

			vehicle_thrust_setpoint_s thrust_sp{};
			vehicle_torque_setpoint_s torque_sp{};
			actuator_controls_s actuators{};

			_control.getThrustSetpoint(thrust_sp);
			_control.getTorqueSetpoint(torque_sp);

			actuators.control[0] = torque_sp.xyz[0];
			actuators.control[1] = torque_sp.xyz[1];
			actuators.control[2] = torque_sp.xyz[2];
			actuators.control[3] = -thrust_sp.xyz[2];
			actuators.timestamp = thrust_sp.timestamp;


			if(_counter == 0) {
				//PX4_WARN("Max Thrust = %f", static_cast<double>(_param_mpc_thr_hover.get()));
				//PX4_WARN("Thrust = %f", static_cast<double>(actuators.control[3]));
			}

			_actuators_0_pub.publish(actuators);

			_vehicle_thrust_setpoint_pub.publish(thrust_sp);
			_vehicle_torque_setpoint_pub.publish(torque_sp);

		} else {
			// an update is necessary here because otherwise the takeoff state doesn't get skiped with non-altitude-controlled modes
			_is_active = false;	
		}
	}

}

AttitudeControlState Prisma1AttitudeControl::set_vehicle_state(const vehicle_attitude_s &att, const vehicle_angular_velocity_s &att_dot)
{
	AttitudeControlState state;

	state.attitude = matrix::Quaternionf(att.q[0], att.q[1], att.q[2], att.q[3]);

	state.angular_velocity = matrix::Vector3f(att_dot.xyz[0], att_dot.xyz[1], att_dot.xyz[2]);

	return state;
}

int Prisma1AttitudeControl::print_usage(const char *reason)
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

int prisma1_att_control_main(int argc, char *argv[])
{
	return Prisma1AttitudeControl::main(argc, argv);
}
