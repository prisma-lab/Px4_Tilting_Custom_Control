#include "AttitudeControlGeom.hpp"
#include <modules/prisma1control/PositionControlBase/ControlMath.hpp>

using namespace matrix;

AttitudeControlGeom::AttitudeControlGeom() {

	_mass = 1.5f;

	_Ib.setZero();
	_Ib(0,0) = 0.029125;
	_Ib(1,1) = 0.029125;
	_Ib(2,2) = 0.055225;

	_Kr(0) = 15.0f;
	_Kr(1) = 15.0f;
	_Kr(2) = 8.0f;

	_Kom(0) = 3.5f;
	_Kom(1) = 3.5f;
	_Kom(2) = 1.5f;

	_Ki(0) = 0.01f;
	_Ki(1) = 0.01f;
	_Ki(2) = 0.01f;

	_c2 = 1.0f;

	_counter = 0;

	_integral.setZero();

	_Wc.setZero();
	_Wc_dot.setZero();
	_f_w.setZero();
	_Rc.setZero();
}

bool AttitudeControlGeom::update(const float dt){
	AttitudeControlBase::update(dt);

	_counter = (_counter+1)%200;

	_attitudeController();

	_normalization();

	return true;
}

void AttitudeControlGeom::_attitudeController() {

	_thrust_sp(0) = _thrust_sp(1) = 0.0f;
	_thrust_sp(2) = _f_w.dot(_R.col(2));
	
	Vector3f e_R = 0.5f * Dcmf(_Rc.transpose()*_R - _R.transpose()*_Rc).vee();
	Vector3f e_om = _state.angular_velocity - _R.transpose()*_Rc*_Wc;

	_integral += (e_om + _c2 * e_R) * _dt;

	_torque_sp = -e_R.emult(_Kr) - e_om.emult(_Kom) - _integral.emult(_Ki) \
								+ Vector3f(_R.transpose()*_Rc*_Wc).hat()*_Ib*_R.transpose()*_Rc*_Wc \
								+ _Ib*_R.transpose()*_Rc*_Wc_dot;
	
	if(!_counter){
		//PX4_INFO("------------------------------");
		//PX4_INFO("Thrust[2]: %f", (double)_thrust_sp(2));
		//PX4_INFO("b3: %f, %f, %f",
		//	(double)_R(0, 2), (double)_R(1, 2), (double)_R(2, 2));
	}
}

void AttitudeControlGeom::_normalization() {
	_thrust_sp(2) = PX4_ISFINITE(_thrust_sp(2)) ? -_thrust_sp(2) / _THRUST_MAX : 0.0f;
	_thrust_sp(2) = math::constrain(_thrust_sp(2), -1.0f, 0.0f);

	_torque_sp(0) = PX4_ISFINITE(_torque_sp(0)) ? _torque_sp(0) / _TORQUE_X_MAX : 0.0f;
	_torque_sp(1) = PX4_ISFINITE(_torque_sp(1)) ? _torque_sp(1) / _TORQUE_Y_MAX : 0.0f;
	_torque_sp(2) = PX4_ISFINITE(_torque_sp(2)) ? _torque_sp(2) / _TORQUE_Z_MAX : 0.0f;

	for(int i=0; i<3; i++)
		_torque_sp(i) = math::constrain(_torque_sp(i), -1.0f, 1.0f);
}

void AttitudeControlGeom::setInputSetpoint(const AttitudeControlInput &setpoint) {
	AttitudeControlBase::setInputSetpoint(setpoint);
	
	prisma_geom_pos_out_s pos_out;
	if(_geom_pos_out_sub.updated()) {
		_geom_pos_out_sub.update(&pos_out);

		_f_w(0) = pos_out.f_w[0];
		_f_w(1) = pos_out.f_w[1];
		_f_w(2) = pos_out.f_w[2];

		for (int i=0; i<3; i++){
			for (int j=0; j<3; j++){
				_Rc(i, j) = pos_out.rc[3*i+j];
			}
		}

		_Wc(0) = pos_out.wc[0];
		_Wc(1) = pos_out.wc[1];
		_Wc(2) = pos_out.wc[2];

		_Wc_dot(0) = pos_out.wc_dot[0];
		_Wc_dot(1) = pos_out.wc_dot[1];
		_Wc_dot(2) = pos_out.wc_dot[2];
	}
}

void AttitudeControlGeom::setState(const AttitudeControlState &state) {
	AttitudeControlBase::setState(state);

	_R = Dcmf(_state.attitude);
}

void AttitudeControlGeom::setKr(Vector3f K) {
	_Kr(0) = K(0);
	_Kr(1) = K(1);
	_Kr(2) = K(2);
}

void AttitudeControlGeom::setKom(Vector3f K) {
	_Kom(0) = K(0);
	_Kom(1) = K(1);
	_Kom(2) = K(2);
}

void AttitudeControlGeom::setKi(Vector3f K) {
	_Ki(0) = K(0);
	_Ki(1) = K(1);
	_Ki(2) = K(2);
}

void AttitudeControlGeom::setC2(float c) {
	_c2 = c;
}

void AttitudeControlGeom::setMass(float m) {
	_mass = m;
}

void AttitudeControlGeom::setIb(const float Ibx, const float Iby, const float Ibz){
	_Ib(0,0) = Ibx;
	_Ib(1,1) = Iby;
	_Ib(2,2) = Ibz;
}
