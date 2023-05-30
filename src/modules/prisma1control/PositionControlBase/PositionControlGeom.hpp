#pragma once

#include "PositionControlBase.hpp"

#include <uORB/Subscription.hpp>
#include <uORB/topics/prisma_geom_pos_out.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>

void deriv_unit_vector( const matrix::Vector3f &A, const matrix::Vector3f &A_dot, const matrix::Vector3f &A_ddot, \
    matrix::Vector3f &q, matrix::Vector3f &q_dot, matrix::Vector3f &q_ddot );

class PositionControlGeom : public PositionControlBase {
public:
	PositionControlGeom();
	~PositionControlGeom() = default;

	virtual bool update(const float dt) override;

	virtual void getControlOutput(void *out) override;

	void setPosGains(matrix::Vector3f K);
	void setVelGains(matrix::Vector3f D);
	void setIntGains(matrix::Vector3f I);
	void setSigma(const float sigma);
	void setC1(const float c1);
	void setMass(const float m);
	
	virtual void getLocalPositionSetpoint(vehicle_local_position_setpoint_s &setpoint) override;
	virtual void resetIntegral() override;
	
private:
	void _positionController();

	uORB::Subscription _attitude_sub {ORB_ID(vehicle_attitude)};
	uORB::Subscription _angular_vel_sub {ORB_ID(vehicle_angular_velocity)};

	float _mass;

	matrix::Vector3f _Kx; ///< Position control proportional gain
	matrix::Vector3f _Kv; ///< Velocity control proportional gain
	matrix::Vector3f _Ki; ///< Integral control proportional gain
	float _c1;
	float _sigma;

	// Control variables
	matrix::Matrix3f _Rc;
	matrix::Vector3f _Wc;
	matrix::Vector3f _Wc_dot;
	matrix::Vector3f _f_w;
	matrix::Vector3f _thrust_sp;

	// Integral
	matrix::Vector3f _integral;

	long _counter;

};
