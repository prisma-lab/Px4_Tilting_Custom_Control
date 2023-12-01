#include "AttitudeControlTilt.hpp"
#include <modules/prisma1control/PositionControlBase/ControlMath.hpp>

using namespace matrix;

AttitudeControlTilt::AttitudeControlTilt()
{

	_mass = 3.68f;

	_Ib.setZero();
	_Ib(0, 0) = 0.15;
	_Ib(1, 1) = 0.15;
	_Ib(2, 2) = 0.15;

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

	_att_sp.q_d[0] = NAN;
	_att_sp.q_d[1] = NAN;
	_att_sp.q_d[2] = NAN;
	_att_sp.q_d[3] = NAN;
}

bool AttitudeControlTilt::update(const float dt)
{
	AttitudeControlBase::update(dt);

	_counter = (_counter + 1) % 50;

	_attitudeController();

	_normalization();

	return true;
}

void AttitudeControlTilt::_attitudeController()
{

	PrismaControlMath::setZeroIfNan(_input.yaw_dot_sp);
	PrismaControlMath::setZeroIfNan(_input.yaw_ddot_sp);

	Quaternionf q_rot(1.0f, 0.0f, 0.0f, 0.0f);
	Vector3f rot_axis = Dcmf(_q_buf).col(_angleInputMode);
	if (!_offboard) {
		if (isnan(_input.yaw_sp))
		{
			Eulerf eul_q(_q_buf);
			if( (_angleInputMode == 0 && (abs(eul_q.phi()) < 0.3f   || eul_q.phi()*_input.yaw_dot_sp < 0.0f)) ||
					(_angleInputMode == 1 && (abs(eul_q.theta()) < 0.3f || eul_q.theta()*_input.yaw_dot_sp < 0.0f)) ||
					(_angleInputMode == 2) ){

				_q_buf.rotate(AxisAnglef(rot_axis, _input.yaw_dot_sp*_dt));
			}
		}
	}
	else {
		if (!isnanf(_input.yaw_sp)){
			_q_buf = Quaternionf(Eulerf(0.0f, 0.0f, _input.yaw_sp));
		}

		if(_tilt_att_sp_sub.updated()) {
			_tilt_att_sp_sub.update(&_att_sp);
		}
		
		if(!isnanf(_att_sp.q_d[0]) &&
				!isnanf(_att_sp.q_d[1]) &&
				!isnanf(_att_sp.q_d[2]) &&
				!isnanf(_att_sp.q_d[3])) {

			_q_buf = Quaternionf(_att_sp.q_d[0], _att_sp.q_d[1], _att_sp.q_d[2], _att_sp.q_d[3]);
		}
	}

	_thrust_sp = _R.transpose() * _f_w;

	Quaternionf q1 = (_q_buf).normalized();
	Quaternionf q2 = (_state.attitude).inversed().normalized();
	Quaternionf q_err = q2 * q1;
	q_err.normalize();

	_w_sp = Vector3f(q_err(1), q_err(2), q_err(3)).emult(_Kq);// * sign(q_err(0));
	_w_sp *= q_err(0) > 0.0f ? 1.0f : -1.0f;

	Vector3f w_err = _w_sp - _state.angular_velocity;

	_integral += w_err * _dt;

	_torque_sp = w_err.emult(_Kr) + _state.angular_velocity.cross(_Ib * _state.angular_velocity);
	_torque_sp += _integral.emult(_Ki_w);

	Eulerf att(_state.attitude);
	Eulerf err_eul(q_err);
	Eulerf eul_des(_q_buf);
	if (!_counter)
	{
//		PX4_INFO("--------------------");
//		PX4_INFO("Integral:   %f, %f, %f",
//			(double)_integral(0), (double)_integral(1), (double)_integral(2));
	}
}

void AttitudeControlTilt::_normalization()
{

	for (int i = 0; i < 2; i++)
	{
		_thrust_sp(i) = PX4_ISFINITE(_thrust_sp(i)) ? _thrust_sp(i) / _THRUST_MAX : 0.0f;
		_thrust_sp(i) = math::constrain(_thrust_sp(i), -1.0f, 1.0f);
	}
	_thrust_sp(2) = PX4_ISFINITE(_thrust_sp(2)) ? _thrust_sp(2) / _THRUST_MAX : 0.0f;
	_thrust_sp(2) = math::constrain(_thrust_sp(2), -1.0f, 0.0f);

	_torque_sp(0) = PX4_ISFINITE(_torque_sp(0)) ? _torque_sp(0) / _TORQUE_X_MAX : 0.0f;
	_torque_sp(1) = PX4_ISFINITE(_torque_sp(1)) ? _torque_sp(1) / _TORQUE_Y_MAX : 0.0f;
	_torque_sp(2) = PX4_ISFINITE(_torque_sp(2)) ? _torque_sp(2) / _TORQUE_Z_MAX : 0.0f;
	// _torque_sp(0) = _torque_sp(1) = 0.0f;

	for (int i = 0; i < 3; i++)
		_torque_sp(i) = math::constrain(_torque_sp(i), -1.0f, 1.0f);

	//_torque_sp(2) = 0.0f;

	if (!_counter)
	{
		// PX4_INFO("------------------------------");
		// PX4_INFO("---");
		// PX4_INFO("thrust: %f, %f, %f",
		// 	(double)_thrust_sp(0), (double)_thrust_sp(1), (double)_thrust_sp(2));
		// PX4_INFO("torque: %f, %f, %f",
		// 	(double)_torque_sp(0), (double)_torque_sp(1), (double)_torque_sp(2));
	}
}


void AttitudeControlTilt::setInputSetpoint(const AttitudeControlInput &setpoint)
{
	AttitudeControlBase::setInputSetpoint(setpoint);

	prisma_tilt_pos_out_s pos_out;
	if (_tilt_pos_out_sub.updated())
	{
		_tilt_pos_out_sub.update(&pos_out);

		_f_w(0) = pos_out.f_des[0];
		_f_w(1) = pos_out.f_des[1];
		_f_w(2) = pos_out.f_des[2];
	}
	if (!_counter)
	{
		// PX4_INFO("------------------------------");
		// PX4_INFO("f_des: %f, %f, %f",
		// 	(double)_f_w(0), (double)_f_w(1), (double)_f_w(2));
		// PX4_INFO("Yaw sp - yaw_dot sp:    %f, %f",
		// (double)setpoint.yaw_sp, (double)setpoint.yaw_dot_sp);
	}
}

void AttitudeControlTilt::setState(const AttitudeControlState &state)
{
	AttitudeControlBase::setState(state);

	_R = Dcmf(_state.attitude);
}

void AttitudeControlTilt::setKr(Vector3f K)
{
	_Kr(0) = K(0);
	_Kr(1) = K(1);
	_Kr(2) = K(2);
}

void AttitudeControlTilt::setKq(Vector3f K)
{
	_Kq(0) = K(0);
	_Kq(1) = K(1);
	_Kq(2) = K(2);
}

void AttitudeControlTilt::setKiw(Vector3f K)
{
	_Ki_w(0) = K(0);
	_Ki_w(1) = K(1);
	_Ki_w(2) = K(2);
}

void AttitudeControlTilt::setMass(float m)
{
	_mass = m;
}

void AttitudeControlTilt::setIb(const float Ibx, const float Iby, const float Ibz)
{
	_Ib(0, 0) = Ibx;
	_Ib(1, 1) = Iby;
	_Ib(2, 2) = Ibz;
}

void AttitudeControlTilt::setAngleInputMode(unsigned int mode)
{
	if (mode < 3)
		_angleInputMode = mode;
}

void AttitudeControlTilt::resetBuffers()
{
	_q_buf = _state.attitude;
}
