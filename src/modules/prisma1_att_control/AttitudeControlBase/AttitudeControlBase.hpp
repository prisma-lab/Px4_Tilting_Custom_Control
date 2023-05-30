#pragma once

#include <drivers/drv_hrt.h>

#include <matrix/matrix/math.hpp>

#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

#define G 9.81f

struct AttitudeControlState {
	matrix::Vector3f velocity;
	matrix::Quaternionf attitude;
	matrix::Vector3f angular_velocity;
};

struct AttitudeControlInput {
	matrix::Vector3f velocity_sp;
	matrix::Vector3f acceleration_sp;
	float yaw_sp;
	float yaw_dot_sp;
	float yaw_ddot_sp;
};

class AttitudeControlBase {
public:
	AttitudeControlBase() = default;
	~AttitudeControlBase() = default;

	virtual void setState(const AttitudeControlState &state){
		_state = state;
	};
	virtual void setInputSetpoint(const AttitudeControlInput &setpoint){
		_input = setpoint;
	};

	virtual bool update(const float dt) {
		_dt = dt;
		return true;
	};

	void getThrustSetpoint(vehicle_thrust_setpoint_s &thrust_sp) {
		thrust_sp.xyz[0] = _thrust_sp(0);
		thrust_sp.xyz[1] = _thrust_sp(1);
		thrust_sp.xyz[2] = _thrust_sp(2);

		thrust_sp.timestamp = hrt_absolute_time();
		thrust_sp.timestamp_sample = thrust_sp.timestamp;
	};

	void getTorqueSetpoint(vehicle_torque_setpoint_s &torque_sp) {
		torque_sp.xyz[0] = _torque_sp(0);
		torque_sp.xyz[1] = _torque_sp(1);
		torque_sp.xyz[2] = _torque_sp(2);

		torque_sp.timestamp = hrt_absolute_time();
		torque_sp.timestamp_sample = torque_sp.timestamp;
	};

	virtual void resetIntegral() = 0;

protected:

	float _dt;

	// State
	AttitudeControlState _state;

	// Setpoints
	AttitudeControlInput _input;

	// Control variables
	matrix::Vector3f _torque_sp; /**< desired torque */
	matrix::Vector3f _thrust_sp; /**< desired thrust */

	// Constants
	const float _THRUST_MAX = 28.2656;
	const float _TORQUE_X_MAX = 2.968;
	const float _TORQUE_Y_MAX = 1.837;
	const float _TORQUE_Z_MAX = 0.848;
};