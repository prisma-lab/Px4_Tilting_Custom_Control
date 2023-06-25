#pragma once

#include <drivers/drv_hrt.h>

#include <lib/controllib/blocks.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>
#include <matrix/matrix/math.hpp>
#include <uORB/topics/vehicle_local_position_setpoint.h>

#define G 9.81f

struct PositionControlState {
	matrix::Vector3f position;
	matrix::Vector3f velocity;
};

struct PositionControlInput {
	matrix::Vector3f position_sp;
	matrix::Vector3f velocity_sp;
	matrix::Vector3f acceleration_sp;
	float yaw_sp;
	float yaw_dot_sp;
	float yaw_ddot_sp;
};

class PositionControlBase {
public:
	PositionControlBase() = default;
	~PositionControlBase() = default;

	/**
	 * Pass the current vehicle state to the controller
	 * @param ControlStates structure
	 */
	virtual void setState(const PositionControlState &state) {
		_state = state;
	};

	/**
	 * Pass the desired setpoints
	 * Note: NAN value means no feed forward/leave state uncontrolled if there's no higher order setpoint.
	 * @param setpoint a vehicle_local_position_setpoint_s structure
	 */
	void setInputSetpoint(const PositionControlInput &setpoint) {
		_input = setpoint;
	};

	/**
	 * Returns the control out
	 * @param out a <custom_msg>_s structure, where <custom_msg> is the message created
	 * to communicate between position and attitude control
	 */
	virtual void getControlOutput( void *out ) = 0;

	/**
	 * Apply controller that updates the internal state of the controller
	 * @return true if update succeeded and output setpoint is executable, false if not
	 */
	virtual bool update(const float dt) {
		_dt = dt;
		return true;
	};

	/**
	 * Returns the local position setpoint used inside the controller
	 * It is needed for the use of the flight tasks that contain smoothing
	 * @param out position setpoint to be set
	 */
	virtual void getLocalPositionSetpoint(vehicle_local_position_setpoint_s &setpoint) = 0;


	/**
	 * Resets any integral contained in the controller
	*/
	virtual void resetIntegral() = 0;

protected:

	float _dt;

	// States
	PositionControlState _state;

	// Setpoints
	PositionControlInput _input;

};
