#pragma once

#include "AttitudeControlBase.hpp"

#include <lib/mathlib/mathlib.h>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>

#include <lib/controllib/blocks.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/topics/prisma_virtual_acc_setpoint.h>

class AttitudeControlPass : public AttitudeControlBase {
public:
	AttitudeControlPass();
	~AttitudeControlPass() = default;

	virtual bool update(const float dt) override;

	void setInputSetpoint(const AttitudeControlInput &setpoint) override;
	void setState(const AttitudeControlState &state) override;

	virtual void resetIntegral() override {};


private:
	uORB::Subscription _virtual_acc_sp_sub {ORB_ID(prisma_virtual_acc_sp)};

	void _thrustAndAttitude();
	void _attitudeController();

	void _updateDynamicMatrices();
	void _normalization();

	float _mass;
	matrix::SquareMatrix3f _Ib;
	float _hover_thrust;

	matrix::SquareMatrix3f _C;
	matrix::SquareMatrix3f _M;
	matrix::SquareMatrix3f _Q;
	matrix::SquareMatrix3f _R;

	matrix::Vector3f _gain_Ko; ///< Velocity control proportional gain
	matrix::Vector3f _gain_Do; ///< Velocity control proportional gain
	float _gain_nu; ///< Velocity control proportional gain

	// States
	matrix::Vector3f _att_eul; /**< Euler angles */
	matrix::Vector3f _att_eul_dot; /**< Euler angles derivative */

	// Setpoints
	matrix::Vector3f _mu; /**< desired virtual acceleration */

	// Control variables
	float _roll_sp{}; /**< desired roll */
	float _rollspeed_sp{}; /** desired roll-speed */
	float _rollaccel_sp{}; /** desired roll-acceleration */
	float _pitch_sp{}; /**< desired pitch */
	float _pitchspeed_sp{}; /** desired pitch-speed */
	float _pitchaccel_sp{}; /** desired pitch-acceleration */

	matrix::Vector3f _att_sp; /**< attitude setpoint */
	matrix::Vector3f _att_dot_sp; /**< attitude velocity setpoint */
	matrix::Vector3f _att_ddot_sp; /**< attitude acceleration setpoint */


	// Filters and derivatives
	math::LowPassFilter2p<float> _roll_filt;

	math::LowPassFilter2p<float> _pitch_filt;

	control::BlockDerivative _roll_deriv;
	control::BlockDerivative _roll_dot_deriv;

	control::BlockDerivative _pitch_deriv;
	control::BlockDerivative _pitch_dot_deriv;

	// Variable used for logging
	long _counter;
};