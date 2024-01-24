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

	// Custom
	_p_adm(0) = 0.0f;
	_p_adm(1) = 0.0f;
	_p_adm(2) = 0.0f;
	_pd_adm(0) = 0.0f;
	_pd_adm(1) = 0.0f;
	_pd_adm(2) = 0.0f;
	_pdd_adm(0) = 0.0f;
	_pdd_adm(1) = 0.0f;
	_pdd_adm(2) = 0.0f;
	// END Custom
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

		// Custom
		setAdmGains(Vector3f(_param_m_adm.get(), _param_kd_adm.get(), _param_kp_adm.get()));
		// END CUSTOM

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
		_control.setKiw(Vector3f(_param_xy_ki.get(), _param_xy_ki.get(), _param_z_ki.get()));
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

// Custom
void Prisma1AttitudeControl::setAdmGains(Vector3f adm) {
	_Kp(0) = adm(2);
	_Kp(1) = adm(2);
	_Kp(2) = adm(2);

	_Kd(0) = adm(1);
	_Kd(1) = adm(1);
	_Kd(2) = adm(1);

	_M(0) = adm(0);
	_M(1) = adm(0);
	_M(2) = adm(0);
}

void Prisma1AttitudeControl::adm_filter(double dt){
	matrix::Vector3f f_fb;
	matrix::Matrix3f R_rot;
	R_rot(0,0) = 0.0f;
	R_rot(0,1) = 0.0f;
	R_rot(0,2) = -1.0f;
    R_rot(1,0) = 0.0f;
	R_rot(1,1) = -1.0f;
	R_rot(1,2) = 0.0f;
    R_rot(2,0) = -1.0f;
	R_rot(2,1) = 0.0f;
	R_rot(2,2) = 0.0f;
	f_fb = inv(R_rot)*Vector3f(_ft_fb.torque[0], _ft_fb.torque[1], _ft_fb.torque[2]);
    _pdd_adm = inv(diag(_M))*(f_fb - diag(_Kd)*_pd_adm - diag(_Kp)*_p_adm);
	_pd_adm = _pd_adm + _pdd_adm*dt;
	_p_adm = _p_adm + _pd_adm*dt;
	_setpoint_temp.yaw_sp = _setpoint.yaw_sp + _p_adm(2);
	_setpoint_temp.yaw_dot_sp = _setpoint.yaw_dot_sp + _pd_adm(2);
}
// End Custom

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

			// Custom
			_ft_sensor_sub.update(&_ft_fb);
			// PX4_INFO("force fb: %f, %f, %f", (double)_ft_fb.force[0], (double)_ft_fb.force[1], (double)_ft_fb.force[2]);

            adm_filter(dt);
		    // PX4_INFO("pos correction: %f, %f, %f", (double)_setpoint.x, (double)_setpoint.y, (double)_setpoint.z);
			// End Custom

			_control.setInputSetpoint(_setpoint_temp);
            	// PX4_INFO("Setpoint yaw and yaw_dot: %f, %f", (double)_setpoint_temp.yaw_sp, (double)_setpoint_temp.yaw_dot_sp);
			_control.setState(state);

			#ifdef TILT_CONTROL
			_control.setOffboard(_vehicle_control_mode.flag_control_offboard_enabled);
			#endif

			if(!_is_active || !_vehicle_control_mode.flag_armed){
				_control.resetIntegral();
				_is_active = true;

				// PX4_WARN("Resetting attitude integral");

				#ifdef TILT_CONTROL
				_control.resetBuffers();
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
			// actuator_controls_s actuators{};

			_control.getThrustSetpoint(thrust_sp);
			_control.getTorqueSetpoint(torque_sp);

			// actuators.control[0] = torque_sp.xyz[0];
			// actuators.control[1] = torque_sp.xyz[1];
			// actuators.control[2] = torque_sp.xyz[2];
			// actuators.control[3] = -thrust_sp.xyz[2];
			// actuators.control[7] = -1;
			// actuators.timestamp = thrust_sp.timestamp;


			if(_counter == 0) {
				//PX4_WARN("Max Thrust = %f", static_cast<double>(_param_mpc_thr_hover.get()));
				//PX4_WARN("Thrust = %f", static_cast<double>(actuators.control[3]));
			}

			// _actuators_0_pub.publish(actuators);

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
