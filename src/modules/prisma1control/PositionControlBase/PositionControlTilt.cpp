#include "PositionControlTilt.hpp"
#include "ControlMath.hpp"

using namespace matrix;

PositionControlTilt::PositionControlTilt() {

	_mass = 1.5f;

	_integral.setZero();

	_Kx(0) = 10.0f;
	_Kx(1) = 10.0f;
	_Kx(2) = 10.0f;

	_Kv(0) = 4.0f;
	_Kv(1) = 4.0f;
	_Kv(2) = 4.0f;

	_Ki(0) = 1.0f;
	_Ki(1) = 1.0f;
	_Ki(2) = 1.0f;

	_counter = 0;

}

bool PositionControlTilt::update(const float dt){
	PositionControlBase::update(dt);
	
	_counter = (_counter+1)%100;

	_positionController();

	return true;
}

void PositionControlTilt::_positionController(){

	Vector3f e =  _input.position_sp - _state.position;
	Vector3f e_dot =  _input.velocity_sp - _state.velocity;

	if(!_counter){
		// PX4_INFO("------------------------------");
		// PX4_INFO("Input acceleration: %f, %f, %f",
		// 	(double)_input.acceleration_sp(0), (double)_input.acceleration_sp(1), (double)_input.acceleration_sp(2));
	}

	PrismaControlMath::setZeroIfNanVector3f(e);
	PrismaControlMath::setZeroIfNanVector3f(e_dot);
	PrismaControlMath::setZeroIfNanVector3f(_input.acceleration_sp);
	PrismaControlMath::setZeroIfNan(_input.yaw_dot_sp);
	PrismaControlMath::setZeroIfNan(_input.yaw_ddot_sp);

	_integral += e * _dt;

	vehicle_attitude_s att;
	_attitude_sub.update(&att);

	Quaternionf att_q(att.q[0], att.q[1], att.q[2], att.q[3]);
	Dcmf Rb(att_q);

	_f_w = e.emult(_Kx) + e_dot.emult(_Kv) + _integral.emult(_Ki) \
					+ _mass * Vector3f(0.0f, 0.0f, -G) + _mass * _input.acceleration_sp;
	_f_b = Rb * _f_w;

	if(!_counter){
		// PX4_INFO("------------------------------");
		// PX4_INFO("Thrust: %f, %f, %f",
		// 	(double)_f_b(0), (double)_f_b(1), (double)_f_b(2));
	}

}

void PositionControlTilt::getControlOutput(void *out) {
	
	prisma_tilt_pos_out_s *msg = static_cast<prisma_tilt_pos_out_s*>(out);

	msg->f_des[0] = _f_w(0);
	msg->f_des[1] = _f_w(1);
	msg->f_des[2] = _f_w(2);

	msg->timestamp = hrt_absolute_time();
}

void PositionControlTilt::setPosGains(Vector3f K) {
	_Kx(0) = K(0);
	_Kx(1) = K(1);
	_Kx(2) = K(2);
}

void PositionControlTilt::setVelGains(Vector3f D) {
	_Kv(0) = D(0);
	_Kv(1) = D(1);
	_Kv(2) = D(2);
}

void PositionControlTilt::setIntGains(Vector3f I) {
	_Ki(0) = I(0);
	_Ki(1) = I(1);
	_Ki(2) = I(2);
}

void PositionControlTilt::setMass(const float m) {
	_mass = m;
}

void PositionControlTilt::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &setpoint) {
	setpoint.x = PX4_ISFINITE(_input.position_sp(0)) ? _input.position_sp(0) : _state.position(0);
	setpoint.y = PX4_ISFINITE(_input.position_sp(1)) ? _input.position_sp(1) : _state.position(1);
	setpoint.z = PX4_ISFINITE(_input.position_sp(2)) ? _input.position_sp(2) : _state.position(2);
	setpoint.yaw = _input.yaw_sp;
	setpoint.yawspeed = _input.yaw_dot_sp;
	setpoint.vx = _input.velocity_sp(0);
	setpoint.vy = _input.velocity_sp(1);
	setpoint.vz = _input.velocity_sp(2);
	_input.acceleration_sp.copyTo(setpoint.acceleration);
	_f_b.copyTo(setpoint.thrust);
}

void PositionControlTilt::resetIntegral()
{
	_integral.setZero();
	// PX4_WARN("Resetting integral");
}
