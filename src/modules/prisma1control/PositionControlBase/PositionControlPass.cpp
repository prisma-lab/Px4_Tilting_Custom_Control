#include "PositionControlPass.hpp"


PositionControlPass::PositionControlPass() {

	_mass = 1.5f;

	_gain_pos_p.setZero();
	_gain_pos_p(0,0) = 0.5f;
	_gain_pos_p(1,1) = 0.5f;
	_gain_pos_p(2,2) = 10.0f;

	_gain_vel_p.setZero();
	_gain_vel_p(0,0) = 0.1;
	_gain_vel_p(1,1) = 0.1;
	_gain_vel_p(2,2) = 1.0;

	_counter = 0;

}

bool PositionControlPass::update(const float dt){
	PositionControlBase::update(dt);

	_counter = (_counter+1)%200;

	_positionController();

	return true;
}

void PositionControlPass::_positionController(){

	matrix::Vector3f e = _state.position - _input.position_sp;
	matrix::Vector3f e_dot = _state.velocity - _input.velocity_sp;

	_mu = _input.acceleration_sp - (_gain_pos_p * e + _gain_vel_p * e_dot) / _mass;

	if(!_counter){
		//PX4_INFO("------------------------------");
		//PX4_INFO("Current Position: %f, %f, %f",
		//	(double)_state.position(0), (double)_state.position(1), (double)_state.position(2));
		//PX4_INFO("Position error: %f, %f, %f",
		//	(double)e(0), (double)e(1), (double)e(2));
		//PX4_INFO("Virtual acceleration: %f, %f, %f",
		//	(double)_mu(0), (double)_mu(1), (double)_mu(2));
	}

}

void PositionControlPass::getControlOutput(void *out) {
	
	prisma_virtual_acc_setpoint_s *mu = static_cast<prisma_virtual_acc_setpoint_s*>(out);

	mu->x = _mu(0);
	mu->y = _mu(1);
	mu->z = _mu(2);

	mu->timestamp = hrt_absolute_time();
}

void PositionControlPass::setPosGains(matrix::Vector3f K) {
	_gain_pos_p(0,0) = K(0);
	_gain_pos_p(1,1) = K(1);
	_gain_pos_p(2,2) = K(2);
}

void PositionControlPass::setVelGains(matrix::Vector3f D) {
	_gain_vel_p(0,0) = D(0);
	_gain_vel_p(1,1) = D(1);
	_gain_vel_p(2,2) = D(2);
}

void PositionControlPass::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &setpoint) {
	setpoint.x = _input.position_sp(0);
	setpoint.y = _input.position_sp(1);
	setpoint.z = _input.position_sp(2);
	setpoint.yaw = _input.yaw_sp;
	setpoint.yawspeed = _input.yaw_dot_sp;
	setpoint.vx = _input.velocity_sp(0);
	setpoint.vy = _input.velocity_sp(1);
	setpoint.vz = _input.velocity_sp(2);
	_input.acceleration_sp.copyTo(setpoint.acceleration);
	matrix::Vector3f(0.0f, 0.0f, 0.0f).copyTo(setpoint.thrust);
}