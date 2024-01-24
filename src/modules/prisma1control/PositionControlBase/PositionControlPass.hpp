#pragma once

#include "PositionControlBase.hpp"

#include <uORB/topics/prisma_virtual_acc_setpoint.h>

class PositionControlPass : public PositionControlBase {
public:
	PositionControlPass();
	~PositionControlPass() = default;

	virtual bool update(const float dt) override;

	virtual void getControlOutput(void *out) override;

	void setPosGains(matrix::Vector3f K);
	void setVelGains(matrix::Vector3f D);
	
	virtual void getLocalPositionSetpoint(vehicle_local_position_setpoint_s &setpoint) override;
	virtual void resetIntegral() override {};
	
private:
	void _positionController();

	float _mass;

	matrix::SquareMatrix3f _gain_pos_p; ///< Position control proportional gain
	matrix::SquareMatrix3f _gain_vel_p; ///< Velocity control proportional gain

	// Control variables
	matrix::Vector3f _mu; /**< desired virtual acceleration */

	long _counter;

};
