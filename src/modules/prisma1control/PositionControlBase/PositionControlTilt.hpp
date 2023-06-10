#pragma once

#include "PositionControlBase.hpp"

#include <uORB/Subscription.hpp>
#include <uORB/topics/prisma_tilt_pos_out.h>
#include <uORB/topics/vehicle_attitude.h>

class PositionControlTilt : public PositionControlBase {
public:
	PositionControlTilt();
	~PositionControlTilt() = default;

	virtual bool update(const float dt) override;

	virtual void getControlOutput(void *out) override;

	void setPosGains(matrix::Vector3f K);
	void setVelGains(matrix::Vector3f D);
	void setIntGains(matrix::Vector3f I);
	void setMass(const float m);
	
	virtual void getLocalPositionSetpoint(vehicle_local_position_setpoint_s &setpoint) override;
	virtual void resetIntegral() override;
	
private:
	void _positionController();

	uORB::Subscription _attitude_sub {ORB_ID(vehicle_attitude)};

	float _mass;

	matrix::Vector3f _Kx; ///< Position control proportional gain
	matrix::Vector3f _Kv; ///< Velocity control proportional gain
	matrix::Vector3f _Ki; ///< Integral control proportional gain

	// Control variables
	matrix::Vector3f _f_w; ///< Desired force in the world frame
	matrix::Vector3f _f_b; ///< Desired force in the body frame

	// Integral
	matrix::Vector3f _integral;

	long _counter;

};
