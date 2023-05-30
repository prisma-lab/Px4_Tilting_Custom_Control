#pragma once

#include "AttitudeControlBase.hpp"

#include <lib/mathlib/mathlib.h>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>

#include <lib/controllib/blocks.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/topics/prisma_geom_pos_out.h>

class AttitudeControlGeom : public AttitudeControlBase {
public:
	AttitudeControlGeom();
	~AttitudeControlGeom() = default;

	virtual bool update(const float dt) override;

	void setInputSetpoint(const AttitudeControlInput &setpoint) override;
	void setState(const AttitudeControlState &state) override;

	void setKr(const matrix::Vector3f K);
	void setKom(const matrix::Vector3f K);
	void setKi(const matrix::Vector3f K);
	void setC2(const float c);
	void setMass(const float m);
	void setIb(const float Ibx, const float Iby, const float Ibz);

	virtual void resetIntegral() override {_integral.setZero();};

private:
	uORB::Subscription _geom_pos_out_sub {ORB_ID(prisma_geom_pos_out)};

	void _attitudeController();

	void _normalization();

	float _mass;
	matrix::SquareMatrix3f _Ib;

	matrix::SquareMatrix3f _R;
	matrix::SquareMatrix3f _R_dot;

	matrix::Vector3f _Kr; ///< Velocity control proportional gain
	matrix::Vector3f _Kom; ///< Velocity control proportional gain
	matrix::Vector3f _Ki; ///< Velocity control proportional gain
	float _c2;

	// States

	// Setpoints
	matrix::Matrix3f _Rc; ///< Computed rotation matrix
	matrix::Vector3f _Wc; ///< Computer angular velocity
	matrix::Vector3f _Wc_dot; ///< Computer angular velocity
	matrix::Vector3f _f_w; ///< Computer angular velocity

	// Control variables
	

	// Filters and derivatives

	// Integral
	matrix::Vector3f _integral;	

	// Variable used for logging
	long _counter;

};