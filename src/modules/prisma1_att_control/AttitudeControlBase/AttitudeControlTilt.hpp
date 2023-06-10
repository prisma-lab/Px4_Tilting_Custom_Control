#pragma once

#include "AttitudeControlBase.hpp"

#include <lib/mathlib/mathlib.h>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>

#include <lib/controllib/blocks.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/topics/prisma_tilt_pos_out.h>

class AttitudeControlTilt : public AttitudeControlBase {
public:
	AttitudeControlTilt();
	~AttitudeControlTilt() = default;

	virtual bool update(const float dt) override;

	void setInputSetpoint(const AttitudeControlInput &setpoint) override;
	void setState(const AttitudeControlState &state) override;

	void setKr(const matrix::Vector3f K);
	void setKq(const matrix::Vector3f K);
	void setMass(const float m);
	void setIb(const float Ibx, const float Iby, const float Ibz);

	virtual void resetIntegral() override {};

private:
	uORB::Subscription _tilt_pos_out_sub {ORB_ID(prisma_tilt_pos_out)};

	void _attitudeController();

	void _normalization();

	float _mass;
	matrix::SquareMatrix3f _Ib;

	matrix::SquareMatrix3f _R;
	matrix::SquareMatrix3f _R_dot;

	matrix::Vector3f _Kr; ///< Velocity control proportional gain
	matrix::Vector3f _Kq; ///< Velocity control proportional gain

	// States

	// Setpoints
	float _roll_sp;
	float _pitch_sp;
	matrix::Vector3f _w_sp; ///< Angular velocity setpoint
	matrix::Vector3f _f_w; ///< Desired force in the world frame = thrust

	// Control variables	

	// Filters and derivatives

	// Integral

	// Variable used for logging
	long _counter;

};