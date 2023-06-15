#include "AttitudeControlTilt.hpp"
#include <modules/prisma1control/PositionControlBase/ControlMath.hpp>

using namespace matrix;

AttitudeControlTilt::AttitudeControlTilt() {

	_mass = 3.68f;

	_Ib.setZero();
	_Ib(0,0) = 0.15;
	_Ib(1,1) = 0.15;
	_Ib(2,2) = 0.15;

	_Kr(0) = 0.5f;
	_Kr(1) = 0.5f;
	_Kr(2) = 0.5f;

	_Kq(0) = 5.0f;
	_Kq(1) = 5.0f;
	_Kq(2) = 10.0f;

	_counter = 0;

	_integral.setZero();
	_f_w.setZero();

	_roll_sp = _pitch_sp = 0.0f;
}

bool AttitudeControlTilt::update(const float dt){
	AttitudeControlBase::update(dt);

	_counter = (_counter+1)%100;

	_attitudeController();

	_normalization();

	return true;
}

void AttitudeControlTilt::_attitudeController() {
	
	PrismaControlMath::setZeroIfNan(_input.yaw_dot_sp);
	PrismaControlMath::setZeroIfNan(_input.yaw_ddot_sp);

	if(isnan(_input.yaw_sp)) {
		_input.yaw_sp = _rpy_sp_buffer[_angleInputMode] + _input.yaw_dot_sp*_dt;
		_rpy_sp_buffer[_angleInputMode] = _input.yaw_sp;
	}

	_rpy_sp_buffer[0] = math::constrain(_rpy_sp_buffer[0], -0.3f, 0.3f);
	_rpy_sp_buffer[1] = math::constrain(_rpy_sp_buffer[1], -0.3f, 0.3f);

	_thrust_sp = _R.transpose() * _f_w;

	Quaternionf q_des(Eulerf(_rpy_sp_buffer[0], _rpy_sp_buffer[1], _rpy_sp_buffer[2]));
	// Quaternionf q_des(1.0f, 0.0f, 0.0f, 0.0f);
	Quaternionf q_err = q_des * _state.attitude.inversed();

	// q_err = Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);

	_w_sp = Vector3f(q_err(1), q_err(2), q_err(3)).emult(_Kq) * sign(q_err(0));

	Vector3f w_err = _w_sp - _state.angular_velocity;

	_torque_sp = w_err.emult(_Kr) + _state.angular_velocity.cross(_Ib*_state.angular_velocity);
	
	Eulerf att(_state.attitude);
	if(!_counter){
		PX4_INFO("--------------------");
		// PX4_INFO("thrust: %f, %f, %f",
		// 	(double)_thrust_sp(0), (double)_thrust_sp(1), (double)_thrust_sp(2));
		// PX4_INFO("Roll_dot setpoint:   %f",
		// 	(double)_input.yaw_dot_sp);
		PX4_INFO("Roll - setpoint - error:   %f, %f, %f",
			(double)att.phi(), (double)_rpy_sp_buffer[0], (double)Eulerf(q_err).phi());
		PX4_INFO("Pitch - setpoint - error:  %f, %f, %f",
			(double)att.theta(), (double)_rpy_sp_buffer[1], (double)Eulerf(q_err).theta());
		PX4_INFO("Yaw - setpoint - error:    %f, %f, %f",
			(double)att.psi(), (double)_rpy_sp_buffer[2], (double)Eulerf(q_err).psi());
		// PX4_INFO("w_sp: %f, %f, %f",
		// 	(double)_w_sp(0), (double)_w_sp(1), (double)_w_sp(2));
		// PX4_INFO("w_err: %f, %f, %f",
		// 	(double)w_err(0), (double)w_err(1), (double)w_err(2));
	}
}

void AttitudeControlTilt::_normalization() {

	for (int i = 0; i < 2; i++) {
		_thrust_sp(i) = PX4_ISFINITE(_thrust_sp(i)) ? _thrust_sp(i) / _THRUST_MAX : 0.0f;
		_thrust_sp(i) = math::constrain(_thrust_sp(i), -1.0f, 1.0f);
	}
	_thrust_sp(2) = PX4_ISFINITE(_thrust_sp(2)) ? _thrust_sp(2) / _THRUST_MAX : 0.0f;
	_thrust_sp(2) = math::constrain(_thrust_sp(2), -1.0f, 0.0f);
	

	_torque_sp(0) = PX4_ISFINITE(_torque_sp(0)) ? _torque_sp(0) / _TORQUE_X_MAX : 0.0f;
	_torque_sp(1) = PX4_ISFINITE(_torque_sp(1)) ? _torque_sp(1) / _TORQUE_Y_MAX : 0.0f;
	_torque_sp(2) = PX4_ISFINITE(_torque_sp(2)) ? _torque_sp(2) / _TORQUE_Z_MAX : 0.0f;

	for(int i=0; i<3; i++)
		_torque_sp(i) = math::constrain(_torque_sp(i), -1.0f, 1.0f);

	//_torque_sp(2) = 0.0f;

	if(!_counter){
		// PX4_INFO("------------------------------");
		PX4_INFO("thrust: %f, %f, %f",
			(double)_thrust_sp(0), (double)_thrust_sp(1), (double)_thrust_sp(2));
		PX4_INFO("torque: %f, %f, %f",
			(double)_torque_sp(0), (double)_torque_sp(1), (double)_torque_sp(2));
	}
}

void AttitudeControlTilt::setInputSetpoint(const AttitudeControlInput &setpoint) {
	AttitudeControlBase::setInputSetpoint(setpoint);
	
	prisma_tilt_pos_out_s pos_out;
	if(_tilt_pos_out_sub.updated()) {
		_tilt_pos_out_sub.update(&pos_out);

		_f_w(0) = pos_out.f_des[0];
		_f_w(1) = pos_out.f_des[1];
		_f_w(2) = pos_out.f_des[2];

	}
	if(!_counter){
		// PX4_INFO("f_des: %f, %f, %f",
		// 	(double)_f_w(0), (double)_f_w(1), (double)_f_w(2));
	}
}

void AttitudeControlTilt::setState(const AttitudeControlState &state) {
	AttitudeControlBase::setState(state);

	_R = Dcmf(_state.attitude);
}

void AttitudeControlTilt::setKr(Vector3f K) {
	_Kr(0) = K(0);
	_Kr(1) = K(1);
	_Kr(2) = K(2);
}

void AttitudeControlTilt::setKq(Vector3f K) {
	_Kq(0) = K(0);
	_Kq(1) = K(1);
	_Kq(2) = K(2);
}

void AttitudeControlTilt::setMass(float m) {
	_mass = m;
}

void AttitudeControlTilt::setIb(const float Ibx, const float Iby, const float Ibz){
	_Ib(0,0) = Ibx;
	_Ib(1,1) = Iby;
	_Ib(2,2) = Ibz;
}

void AttitudeControlTilt::setAngleInputMode(unsigned int mode)
{
	if(mode<3)
		_angleInputMode = mode;
}

void AttitudeControlTilt::resetBuffers()
{
	Eulerf att(_state.attitude);
	_rpy_sp_buffer[0] = att.phi();
	_rpy_sp_buffer[1] = att.theta();
	_rpy_sp_buffer[2] = att.psi();
}
