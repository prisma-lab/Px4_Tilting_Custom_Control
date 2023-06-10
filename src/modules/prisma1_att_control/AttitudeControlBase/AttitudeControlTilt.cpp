#include "AttitudeControlTilt.hpp"
#include <modules/prisma1control/PositionControlBase/ControlMath.hpp>

using namespace matrix;

AttitudeControlTilt::AttitudeControlTilt() {

	_mass = 1.5f;

	_Ib.setZero();
	_Ib(0,0) = 0.029125;
	_Ib(1,1) = 0.029125;
	_Ib(2,2) = 0.055225;

	_Kr(0) = 15.0f;
	_Kr(1) = 15.0f;
	_Kr(2) = 8.0f;

	_Kq(0) = 3.5f;
	_Kq(1) = 3.5f;
	_Kq(2) = 1.5f;

	_counter = 0;

	_f_w.setZero();
}

bool AttitudeControlTilt::update(const float dt){
	AttitudeControlBase::update(dt);

	_counter = (_counter+1)%200;

	_attitudeController();

	_normalization();

	return true;
}

void AttitudeControlTilt::_attitudeController() {

	_thrust_sp = _R.transpose() * _f_w;

	Quaternionf q_des(Eulerf(_roll_sp, _pitch_sp, _input.yaw_sp));
	// Quaternionf q_des(1.0f, 0.0f, 0.0f, 0.0f);
	Quaternionf q_err = q_des * _state.attitude.inversed();

	// q_err = Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);

	_w_sp = Vector3f(q_err(1), q_err(2), q_err(3)).emult(_Kq) * sign(q_err(0));

	_torque_sp = (_w_sp - _state.angular_velocity).emult(_Kr) + _state.angular_velocity.cross(_Ib*_state.angular_velocity);
	
	if(!_counter){
		PX4_INFO("--------------------");
		// PX4_INFO("thrust: %f, %f, %f",
		// 	(double)_thrust_sp(0), (double)_thrust_sp(1), (double)_thrust_sp(2));
		PX4_INFO("Yaw error: %f",
			(double)_input.yaw_sp - (double)Eulerf(_state.attitude).psi());
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

	if(!_counter){
		// PX4_INFO("thrust: %f, %f, %f",
		// 	(double)_thrust_sp(0), (double)_thrust_sp(1), (double)_thrust_sp(2));
		PX4_INFO("torque: %f, %f, %f",
			(double)_torque_sp(0), (double)_torque_sp(1), (double)_torque_sp(2));
	}
}

void AttitudeControlTilt::setInputSetpoint(const AttitudeControlInput &setpoint) {
	AttitudeControlBase::setInputSetpoint(setpoint);

	_roll_sp = _pitch_sp = 0.0f;
	
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
