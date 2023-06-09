#include "PositionControlGeom.hpp"
#include "ControlMath.hpp"

using namespace matrix;

PositionControlGeom::PositionControlGeom() {

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

	_c1 = 1.0f;
	_sigma = 10.0f;

	_counter = 0;

}

bool PositionControlGeom::update(const float dt){
	PositionControlBase::update(dt);
	
	_counter = (_counter+1)%100;

	_positionController();

	return true;
}

void PositionControlGeom::_positionController(){

	Vector3f e = _state.position - _input.position_sp;
	Vector3f e_dot = _state.velocity - _input.velocity_sp;

	PrismaControlMath::setZeroIfNanVector3f(e);
	PrismaControlMath::setZeroIfNanVector3f(e_dot);
	PrismaControlMath::setZeroIfNanVector3f(_input.acceleration_sp);
	PrismaControlMath::setZeroIfNan(_input.yaw_dot_sp);
	PrismaControlMath::setZeroIfNan(_input.yaw_ddot_sp);

	Vector3f A(0.0f, 0.0f, 0.0f);

	for(int i=0; i<3; i++){

		float deltaI = (e_dot(i) + _c1 * e(i)) * _dt;

		if(PX4_ISFINITE(deltaI)){
			if( (_integral(i) <= -_sigma && deltaI < 0.0f) ||
					(_integral(i) >= _sigma && deltaI > 0.0f) ){
				deltaI = 0.0f;
			}

			_integral(i) += deltaI;
		}
	}
	
	A = -e.emult(_Kx) - e_dot.emult(_Kv) \
				- constrain(_integral, -_sigma, _sigma).emult(_Ki) \
				- Vector3f(0.0f, 0.0f, _mass*G) + _mass * _input.acceleration_sp;

	_f_w = -A;

	vehicle_attitude_s att;
	_attitude_sub.update(&att);

	vehicle_angular_velocity_s w_msg;
	_angular_vel_sub.update(&w_msg);

	Quaternionf att_q(att.q[0], att.q[1], att.q[2], att.q[3]);
	Vector3f w(w_msg.xyz[0], w_msg.xyz[1], w_msg.xyz[2]);
	Vector3f b3 = Dcmf(att_q).col(2);
	Vector3f b3_dot = (Dcmf(att_q)*w.hat()).col(2);

	_thrust_sp(0) = _thrust_sp(1) = 0.0f;
	_thrust_sp(2) = _f_w.dot(b3);
	_thrust_sp(2) = PX4_ISFINITE(_thrust_sp(2)) ? -_thrust_sp(2) / 28.2656f : 0.0f;
	_thrust_sp(2) = math::constrain(_thrust_sp(2), -1.0f, 0.0f);

	float f = -A.dot(b3);
	Vector3f ea = Vector3f(0.0f, 0.0f, G) - f / _mass * b3 - _input.acceleration_sp;

	Vector3f A_dot = -e_dot.emult(_Kx) - ea.emult(_Kv); // +m*xd_3dot

	float f_dot = -A_dot*b3 -A*b3_dot;
	Vector3f eb = -f_dot / _mass * b3 - f / _mass * b3_dot; // -xd_3dot;
	Vector3f A_ddot = - ea.emult(_Kx) - eb.emult(_Kv); // + _mass*xd_4dot;

	Vector3f b3c, b3c_dot, b3c_ddot;
	deriv_unit_vector(-A, -A_dot, -A_ddot, b3c, b3c_dot, b3c_ddot);

	Vector3f b1d;
	float yaw = Eulerf(att_q).psi();
	if (isnan(_input.yaw_sp))
		_input.yaw_sp = yaw + _input.yaw_dot_sp*_dt;
	
	b1d = Vector3f(cos(_input.yaw_sp), sin(_input.yaw_sp), 0.0f);
	
	Vector3f b1d_dot(-sin(_input.yaw_sp), cos(_input.yaw_sp), 0.0f);
	b1d_dot *= _input.yaw_dot_sp;
	Vector3f b1d_ddot(0.0f, 0.0f, _input.yaw_ddot_sp);

	Vector3f A2 = -b1d.hat() * b3c;
	Vector3f A2_dot = -b1d_dot.hat() * b3c - b1d.hat() * b3c_dot;
	Vector3f A2_ddot = -b1d_ddot.hat() * b3c \
						- 2.0f * b1d_dot.hat() * b3c_dot \
						- b1d.hat() * b3c_ddot;

	Vector3f b2c, b2c_dot, b2c_ddot;
	deriv_unit_vector(A2, A2_dot, A2_ddot, b2c, b2c_dot, b2c_ddot);

	Vector3f b1c = b2c.hat()*b3c;
	Vector3f b1c_dot = b2c_dot.hat()*b3c + b2c.hat()*b3c_dot;
	Vector3f b1c_ddot = b2c_ddot.hat() * b3c \
        		+ 2.0f * b2c_dot.hat() * b3c_dot \
        		+ b2c.hat() * b3c_ddot;

	Matrix3f Rc_dot, Rc_ddot;

	_Rc.setCol(0, b1c);
	_Rc.setCol(1, b2c);
	_Rc.setCol(2, b3c);

	Rc_dot.setCol(0, b1c_dot);
	Rc_dot.setCol(1, b2c_dot);
	Rc_dot.setCol(2, b3c_dot);

	Rc_ddot.setCol(0, b1c_ddot);
	Rc_ddot.setCol(1, b2c_ddot);
	Rc_ddot.setCol(2, b3c_ddot);

	_Wc = Dcmf(_Rc.transpose() * Rc_dot).vee();
	_Wc_dot = Dcmf(_Rc.transpose() * Rc_ddot \
        		- _Wc.hat() * _Wc.hat()).vee();

	if(!_counter){
		//PX4_INFO("------------------------------");
		//PX4_INFO("Input acceleration: %f, %f, %f",
		//	(double)_input.acceleration_sp(0), (double)_input.acceleration_sp(1), (double)_input.acceleration_sp(2));
	}

}

void PositionControlGeom::getControlOutput(void *out) {
	
	prisma_geom_pos_out_s *msg = static_cast<prisma_geom_pos_out_s*>(out);

	for (int i=0; i<3; i++){
		for (int j=0; j<3; j++){
			msg->rc[3*i+j] = _Rc(i, j);
		}
	}

	msg->wc[0] = _Wc(0);
	msg->wc[1] = _Wc(1);
	msg->wc[2] = _Wc(2);

	msg->wc_dot[0] = _Wc_dot(0);
	msg->wc_dot[1] = _Wc_dot(1);
	msg->wc_dot[2] = _Wc_dot(2);

	msg->f_w[0] = _f_w(0);
	msg->f_w[1] = _f_w(1);
	msg->f_w[2] = _f_w(2);

	msg->timestamp = hrt_absolute_time();
}

void PositionControlGeom::setPosGains(Vector3f K) {
	_Kx(0) = K(0);
	_Kx(1) = K(1);
	_Kx(2) = K(2);
}

void PositionControlGeom::setVelGains(Vector3f D) {
	_Kv(0) = D(0);
	_Kv(1) = D(1);
	_Kv(2) = D(2);
}

void PositionControlGeom::setIntGains(Vector3f I) {
	_Ki(0) = I(0);
	_Ki(1) = I(1);
	_Ki(2) = I(2);
}

void PositionControlGeom::setSigma(const float sigma) {
	_sigma = sigma;
}

void PositionControlGeom::setC1(const float c1) {
	_c1 = c1;
}

void PositionControlGeom::setMass(const float m) {
	_mass = m;
}

void PositionControlGeom::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &setpoint) {
	setpoint.x = PX4_ISFINITE(_input.position_sp(0)) ? _input.position_sp(0) : _state.position(0);
	setpoint.y = PX4_ISFINITE(_input.position_sp(1)) ? _input.position_sp(1) : _state.position(1);
	setpoint.z = PX4_ISFINITE(_input.position_sp(2)) ? _input.position_sp(2) : _state.position(2);
	setpoint.yaw = _input.yaw_sp;
	setpoint.yawspeed = _input.yaw_dot_sp;
	setpoint.vx = _input.velocity_sp(0);
	setpoint.vy = _input.velocity_sp(1);
	setpoint.vz = _input.velocity_sp(2);
	_input.acceleration_sp.copyTo(setpoint.acceleration);
	_thrust_sp.copyTo(setpoint.thrust);
}

void PositionControlGeom::resetIntegral()
{
	_integral.setZero();
	// PX4_WARN("Resetting integral");
}

void deriv_unit_vector( const Vector3f &A, const Vector3f &A_dot, const Vector3f &A_ddot, \
    Vector3f &q, Vector3f &q_dot, Vector3f &q_ddot )
{
    double nA = A.norm();
    double nA3 = nA * nA * nA; //pow(nA, 3.0);
    double nA5 = nA3 * nA * nA; //pow(nA, 5.0);

    q = A / nA;
    q_dot = A_dot / nA \
        - A * A.dot(A_dot) / nA3;

    q_ddot = A_ddot / nA \
        - A_dot / nA3 * (2 * A.dot(A_dot)) \
        - A / nA3 * (A_dot.dot(A_dot) + A.dot(A_ddot)) \
        + 3.0f * A / nA5 * A.dot(A_dot) * A.dot(A_dot); //pow(A.dot(A_dot), 2.0);
}
