#include "AttitudeControlPass.hpp"

using namespace matrix;

AttitudeControlPass::AttitudeControlPass():
	_roll_filt(250.0f, 10.0f),
	_pitch_filt(250.0f, 10.0f),
	_roll_deriv(nullptr, "VELD"),
	_roll_dot_deriv(nullptr, "VELD"),
	_pitch_deriv(nullptr, "VELD"),
	_pitch_dot_deriv(nullptr, "VELD") {

	_mass = 1.5f;

	_hover_thrust = 0.71f;

	_Ib.setZero();
	_Ib(0,0) = 0.029125;
	_Ib(1,1) = 0.029125;
	_Ib(2,2) = 0.055225;

	_gain_Ko(0) = 1;
	_gain_Ko(1) = 1;
	_gain_Ko(2) = 1;

	_gain_Do(0) = 0.1;
	_gain_Do(1) = 0.1;
	_gain_Do(2) = 0.8;

	_gain_nu = 1;

	_counter = 0;
}

bool AttitudeControlPass::update(const float dt){
	AttitudeControlBase::update(dt);

	_counter = (_counter+1)%200;

	_thrustAndAttitude();

	_updateDynamicMatrices();

	_attitudeController();

	_normalization();

	return true;
}

void AttitudeControlPass::_thrustAndAttitude(){
	float roll_sp, pitch_sp, yaw_sp;

	yaw_sp = _input.yaw_sp;

	_thrust_sp(0) = _thrust_sp(1) = 0.0f;
	_thrust_sp(2) = _mass * sqrtf(pow(_mu(0), 2)+pow(_mu(1), 2)+pow(_mu(2)-9.81f, 2));

	roll_sp = asin(_mass * (_mu(1)*float(cos(yaw_sp))-_mu(0)*float(sin(yaw_sp)))/_thrust_sp(2));
	pitch_sp = atan((_mu(0)*float(cos(yaw_sp))+_mu(1)*float(sin(yaw_sp)))/(_mu(2)-9.81f));

	_roll_sp = _roll_filt.apply(roll_sp);
	_pitch_sp = _pitch_filt.apply(pitch_sp);

	_pitch_deriv.setDt(_dt);
	_pitch_dot_deriv.setDt(_dt);
	_roll_deriv.setDt(_dt);
	_roll_dot_deriv.setDt(_dt);

	_pitchspeed_sp = _pitch_deriv.update(_pitch_sp);
	_pitchaccel_sp = _pitch_dot_deriv.update(_pitchspeed_sp);

	_rollspeed_sp = _roll_deriv.update(_roll_sp);
	_rollaccel_sp = _roll_dot_deriv.update(_rollspeed_sp);

	_att_sp(0) = _roll_sp;
	_att_sp(1) = _pitch_sp;
	_att_sp(2) = yaw_sp;

	_att_dot_sp(0) = _rollspeed_sp;
	_att_dot_sp(1) = _pitchspeed_sp;
	_att_dot_sp(2) = _input.yaw_dot_sp;

	_att_ddot_sp(0) = _rollaccel_sp;
	_att_ddot_sp(1) = _pitchaccel_sp;
	_att_ddot_sp(2) = _input.yaw_ddot_sp;

	if(!_counter){
		//PX4_INFO("------------------------------");
		//PX4_INFO("Attitude setpoint: %f, %f, %f",
		//	(double)_att_sp(0), (double)_att_sp(1), (double)_att_sp(2));
		//PX4_INFO("Current Attitude: %f, %f, %f",
		//	(double)_att_eul(0), (double)_att_eul(1), (double)_att_eul(2));
	}

}

void AttitudeControlPass::_updateDynamicMatrices() {
	float c_roll, c_pitch;
	float s_roll, s_pitch;

	c_roll   = cos(_att_eul(0));
	c_pitch = cos(_att_eul(1));

	s_roll   = sin(_att_eul(0));
	s_pitch = sin(_att_eul(1));

	_Q.setZero();
	_Q(0,0) = 1;
	_Q(0,2) = -s_pitch;
	_Q(1,1) = c_roll;
	_Q(1,2) = c_pitch*s_roll;
	_Q(2,1) = -s_roll;
	_Q(2,2) = c_pitch*c_roll;


	AxisAnglef rollAngle(Vector3f(1.0f,0.0f,0.0f), _att_eul(0));
	AxisAnglef pitchAngle(Vector3f(0.0f,1.0f,0.0f), _att_eul(1));
	AxisAnglef yawAngle(Vector3f(0.0f,0.0f,1.0f), _att_eul(2));

	Dcmf rollM(rollAngle), pitchM(pitchAngle), yawM(yawAngle);

	_R = yawM * pitchM * rollM;

	Matrix3f Q_dot,  S;

	float roll_dot, pitch_dot;

	roll_dot   = _att_eul(0);
	pitch_dot = _att_eul(1);

	Q_dot.setZero();
	Q_dot(0,2) = -pitch_dot*c_pitch;
	Q_dot(1,1) = -roll_dot*s_roll;
	Q_dot(1,2) = c_pitch*roll_dot*c_roll-pitch_dot*s_pitch*s_roll;
	Q_dot(2,1) = -roll_dot*c_roll;
	Q_dot(2,2) = -pitch_dot*s_pitch*c_roll-roll_dot*c_pitch*s_roll;

	_M = _Q.transpose() * _Ib * _Q;

	S.setZero();
	S(0,1) = -_state.angular_velocity(2);
	S(0,2) = _state.angular_velocity(1);
	S(1,0) = _state.angular_velocity(2);
	S(1,2) = -_state.angular_velocity(0);
	S(2,0) = -_state.angular_velocity(1);
	S(2,1) = _state.angular_velocity(0);

	_att_eul_dot = inv(_Q) * _state.angular_velocity;

	_C = _Q.transpose() * S * _Ib * _Q + _Q.transpose() * _Ib * Q_dot;
}

void AttitudeControlPass::_attitudeController() {

	Vector3f e_eta = _att_eul - _att_sp;
	Vector3f e_dot_eta = _att_eul_dot - _att_dot_sp;

	Vector3f eta_dot_r = _att_dot_sp - _gain_nu * e_eta;
	Vector3f eta_ddot_r = _att_ddot_sp - _gain_nu * e_dot_eta;

	Vector3f nu_eta = e_dot_eta + _gain_nu * e_eta;

	SquareMatrix3f Q_mt, Q_t;
	Q_t = _Q.transpose();
	Q_mt = inv(Q_t);
	_torque_sp = Q_mt * (_M*eta_ddot_r+_C*eta_dot_r-nu_eta.emult(_gain_Do)-e_eta.emult(_gain_Ko));

}

void AttitudeControlPass::_normalization() {
	_thrust_sp(2) = PX4_ISFINITE(_thrust_sp(2)) ? -_thrust_sp(2) / _THRUST_MAX : 0.0f;
	_thrust_sp(2) = math::constrain(_thrust_sp(2), -1.0f, 0.0f);

	_torque_sp(0) = PX4_ISFINITE(_torque_sp(0)) ? _torque_sp(0) / _TORQUE_X_MAX : 0.0f;
	_torque_sp(1) = PX4_ISFINITE(_torque_sp(1)) ? _torque_sp(1) / _TORQUE_Y_MAX : 0.0f;
	_torque_sp(2) = PX4_ISFINITE(_torque_sp(2)) ? _torque_sp(2) / _TORQUE_Z_MAX : 0.0f;

	for(int i=0; i<3; i++)
		_torque_sp(i) = math::constrain(_torque_sp(i), -1.0f, 1.0f);
}

void AttitudeControlPass::setInputSetpoint(const AttitudeControlInput &setpoint) {
	AttitudeControlBase::setInputSetpoint(setpoint);
	
	prisma_virtual_acc_setpoint_s mu_sp;
	_virtual_acc_sp_sub.update(&mu_sp);

	_mu(0) = PX4_ISFINITE(mu_sp.x) ? mu_sp.x : 0.0f;
	_mu(1) = PX4_ISFINITE(mu_sp.y) ? mu_sp.y : 0.0f;
	_mu(2) = PX4_ISFINITE(mu_sp.z) ? mu_sp.z : 0.0f;
}

void AttitudeControlPass::setState(const AttitudeControlState &state) {
	AttitudeControlBase::setState(state);

	_att_eul = Eulerf(_state.attitude);
}
