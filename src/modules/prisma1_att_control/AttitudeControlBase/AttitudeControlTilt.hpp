#pragma once

#include "AttitudeControlBase.hpp"

#include <lib/mathlib/mathlib.h>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>

#include <lib/controllib/blocks.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/topics/prisma_tilt_pos_out.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/tilting_attitude_setpoint.h>


class AttitudeControlTilt : public AttitudeControlBase {
public:
	AttitudeControlTilt();
	~AttitudeControlTilt() = default;

	virtual bool update(const float dt) override;

	void setInputSetpoint(const AttitudeControlInput &setpoint) override;
	void setState(const AttitudeControlState &state) override;
	void setOffboard(const bool offboard) { _offboard = offboard; };

	void setKr(const matrix::Vector3f K);
	void setKq(const matrix::Vector3f K);
	void setMass(const float m);
	void setIb(const float Ibx, const float Iby, const float Ibz);

	void setAngleInputMode(unsigned int mode); //< 0:Roll, 1:Pitch, 2:Yaw

	virtual void resetIntegral() override {_integral.setZero();};
	virtual void resetBuffers();

private:
	uORB::Subscription _tilt_pos_out_sub {ORB_ID(prisma_tilt_pos_out)};
	uORB::Subscription _tilt_att_sp_sub {ORB_ID(tilting_attitude_setpoint)};
	uORB::Subscription _pos_sp_sub {ORB_ID(vehicle_local_position_setpoint)};
	vehicle_local_position_setpoint_s _pos_sp;

	void _attitudeController();

	void _normalization();

	unsigned int _angleInputMode = 2;

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
	matrix::Vector3f _integral;	
	float _rpy_sp_buffer[3] = {0.0f, 0.0f, 0.0f};
	matrix::Quaternionf _q_buf;
	tilting_attitude_setpoint_s _att_sp{};
	matrix::Dcmf _rotmat_buf;
	bool _offboard{false};
	float _q_signum{1.0f};

	// Filters and derivatives

	// Integral

	// Variable used for logging
	long _counter;

};