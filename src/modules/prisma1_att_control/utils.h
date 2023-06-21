#ifndef _utilities_hpp
#define _utilities_hpp

#include <Eigen/Dense>
#ifndef M_PI
#define M_PI 3.14159265358979
#endif

using namespace std;  //calling the standard directory
using namespace Eigen;
 

namespace utilities{ 

	inline Matrix3d rotx(float alpha){
		Matrix3d Rx;
		Rx << 	1,0,0,
			0,cos(alpha),-sin(alpha),
			0,sin(alpha), cos(alpha);
		return Rx;
	}

	inline Matrix3d roty(float beta){
		Matrix3d Ry;
		Ry << 	cos(beta),0,sin(beta),
			0,1,0,
			-sin(beta),0, cos(beta);
		return Ry;
	}

	inline Matrix3d rotz(float gamma){
		Matrix3d Rz;
		Rz << 	cos(gamma),-sin(gamma),0,
			sin(gamma),cos(gamma),0,
			0,0, 1;
		return Rz;
	}


	inline Matrix4d rotx_T(float alpha){
		Matrix4d Tx = Matrix4d::Identity();
		Tx.block(0,0,3,3) = rotx(alpha);
		return Tx;
	}

	inline Matrix4d roty_T(float beta){
		Matrix4d Ty = Matrix4d::Identity();
		Ty.block(0,0,3,3) = roty(beta);
		return Ty;
	}

	inline Matrix4d rotz_T(float gamma){
		Matrix4d Tz = Matrix4d::Identity();
		Tz.block(0,0,3,3) = rotz(gamma);
		return Tz;
	}

	inline Matrix3d skew(Vector3d v)
	{
		Matrix3d S;
		S << 0,	-v[2],	 v[1],		//Skew-symmetric matrix
			v[2],	    0,	-v[0],
			-v[1],	 v[0], 	   0;
		return S;
	}


	inline Matrix3d L_matrix(Matrix3d R_d, Matrix3d R_e)
	{
		Matrix3d L = -0.5 * (skew(R_d.col(0))*skew(R_e.col(0)) + skew(R_d.col(1))*skew(R_e.col(1)) + skew(R_d.col(2))*skew(R_e.col(2)));
		return L;
	}



	inline Vector3d rotationMatrixError(Matrix4d Td, Matrix4d Te)
	{
		
		Matrix3d R_e = Te.block(0,0,3,3);		//Matrix.slice<RowStart, ColStart, NumRows, NumCols>();	
		Matrix3d R_d = Td.block(0,0,3,3);
		
		Vector3d eo = 0.5 * (skew(R_e.col(0))*R_d.col(0) + skew(R_e.col(1))*R_d.col(1) + skew(R_e.col(2))*R_d.col(2)) ;
		return eo;
	}


	inline Vector3d r2quat(Matrix3d R_iniz, float &eta)
	{
		Vector3d epsilon;
		int iu, iv, iw;

		if ( (R_iniz(0,0) >= R_iniz(1,1)) && (R_iniz(0,0) >= R_iniz(2,2)) )
		{
			iu = 0; iv = 1; iw = 2;
		}
		else if ( (R_iniz(1,1) >= R_iniz(0,0)) && (R_iniz(1,1) >= R_iniz(2,2)) )
		{
			iu = 1; iv = 2; iw = 0;
		}
		else
		{
			iu = 2; iv = 0; iw = 1;
		}

		float r = sqrt(1 + R_iniz(iu,iu) - R_iniz(iv,iv) - R_iniz(iw,iw));
		Vector3d q;
		q <<  0,0,0;
		if (r>0)
		{
		float rr = 2*r;
		eta = (R_iniz(iw,iv)-R_iniz(iv,iw)/rr);
		epsilon[iu] = r/2;
		epsilon[iv] = (R_iniz(iu,iv)+R_iniz(iv,iu))/rr;
		epsilon[iw] = (R_iniz(iw,iu)+R_iniz(iu,iw))/rr;
		}
		else
		{
		eta = 1;
		epsilon << 0,0,0;
		}
		return epsilon;
	}


	inline Vector4d rot2quat(Matrix3d R){

		float m00, m01, m02, m10, m11, m12, m20, m21, m22;

		m00 = R(0,0);
		m01 = R(0,1);
		m02 = R(0,2);
		m10 = R(1,0);
		m11 = R(1,1);
		m12 = R(1,2);
		m20 = R(2,0);
		m21 = R(2,1);
		m22 = R(2,2);

		float tr = m00 + m11 + m22;
		float qw, qx, qy, qz, S;
		Vector4d quat;

		if (tr > 0) { 
		  S = sqrt(tr+1.0) * 2; // S=4*qw 
		  qw = 0.25 * S;
		  qx = (m21 - m12) / S;
		  qy = (m02 - m20) / S; 
		  qz = (m10 - m01) / S; 
		} else if ((m00 > m11)&(m00 > m22)) { 
		  S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx 
		  qw = (m21 - m12) / S;
		  qx = 0.25 * S;
		  qy = (m01 + m10) / S; 
		  qz = (m02 + m20) / S; 
		} else if (m11 > m22) { 
		  S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
		  qw = (m02 - m20) / S;
		  qx = (m01 + m10) / S; 
		  qy = 0.25 * S;
		  qz = (m12 + m21) / S; 
		} else { 
		  S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
		  qw = (m10 - m01) / S;
		  qx = (m02 + m20) / S;
		  qy = (m12 + m21) / S;
		  qz = 0.25 * S;
		}

		quat << qw, qx, qy, qz;
		return quat;

	}
	
	
		 //Matrix ortonormalization
	inline Matrix3d matrixOrthonormalization(Matrix3d R){

		SelfAdjointEigenSolver<Matrix3d> es(R.transpose()*R);
		Vector3d D = es.eigenvalues();
		Matrix3d V = es.eigenvectors();
		R = R*((1/sqrt(D(0)))*V.col(0)*V.col(0).transpose() + (1/sqrt(D(1)))*V.col(1)*V.col(1).transpose() + (1/sqrt(D(2)))*V.col(2)*V.col(2).transpose());

		return R;
	}




	//******************************************************************************
	inline Vector3d quaternionError(Matrix4d Tbe, Matrix4d Tbe_d)
	{
		float eta, eta_d;
		Matrix3d R = Tbe.block(0,0,3,3);		//Matrix.slice<RowStart, ColStart, NumRows, NumCols>();	
		Vector3d epsilon = r2quat(R, eta);
		Matrix3d R_d = Tbe_d.block(0,0,3,3);
		Vector3d epsilon_d = r2quat(R_d, eta_d);
		Matrix3d S = skew(epsilon_d);
		Vector3d eo = eta*epsilon_d-eta_d*epsilon-S*epsilon;
		return eo;
	}


	inline Vector3d versorError(Vector3d P_d, Matrix3d Tbe_e, float &theta)
	{
		Vector3d P_e = Tbe_e.block(0,3,3,1);
		Vector3d u_e = Tbe_e.block(0,0,3,1);

		Vector3d u_d = P_d - P_e;
		u_d = u_d/ u_d.norm();
		
		Vector3d r = skew(u_e)*u_d;
		float nr = r.norm();
			
	  if (nr >0){
			r = r/r.norm();
			

			//be carefult to acos( > 1 )
			float u_e_d = u_e.transpose()*u_d;		
			if( fabs(u_e_d) <= 1.0 ) {
				theta = acos(u_e.transpose()*u_d);
			}
			else {
				theta = 0.0;
			}
			
			Vector3d error = r*sin(theta);
			return error;
		}else{
			theta = 0.0;
			return Vector3d::Zero();
		}
	}


	/*Matrix3d utilities::rotation ( float theta, Vector3d r ) {
	     
	    Matrix3d R = Zeros;
	 
	    R[0][0] = r[0]*r[0]*(1-cos(theta)) + cos(theta);
	    R[1][1] = r[1]*r[1]*(1-cos(theta)) + cos(theta);
	    R[2][2] = r[2]*r[2]*(1-cos(theta)) + cos(theta);
	 
	    R[0][1] = r[0]*r[1]*(1-cos(theta)) - r[2]*sin(theta);
	    R[1][0] = r[0]*r[1]*(1-cos(theta)) + r[2]*sin(theta);
	 
	    R[0][2] = r[0]*r[2]*(1-cos(theta)) + r[1]*sin(theta);
	    R[2][0] = r[0]*r[2]*(1-cos(theta)) - r[1]*sin(theta);
	     
	    R[1][2] = r[1]*r[2]*(1-cos(theta)) - r[0]*sin(theta);
	    R[2][1] = r[1]*r[2]*(1-cos(theta)) + r[0]*sin(theta);
	     
	 
	    return R;
	}*/


	inline Matrix3d XYZ2R(Vector3d angles) {
	  	
	  	Matrix3d R = Matrix3d::Zero(); 
	  	Matrix3d R1 = Matrix3d::Zero(); 
	  	Matrix3d R2 = Matrix3d::Zero(); 
	  	Matrix3d R3 = Matrix3d::Zero();

		float cos_phi = cos(angles[0]);
		float sin_phi = sin(angles[0]);
		float cos_theta = cos(angles[1]);
		float sin_theta = sin(angles[1]);
		float cos_psi = cos(angles[2]);
		float sin_psi = sin(angles[2]);

		R1  << 1, 0      , 0, 
			        0, cos_phi, -sin_phi, 
			        0, sin_phi, cos_phi;

		R2  << cos_theta , 0, sin_theta, 
			        0        , 1, 0       , 
			        -sin_theta, 0, cos_theta;

		R3  << cos_psi, -sin_psi, 0, 
			        sin_psi, cos_psi , 0,
			        0      , 0       , 1;

		R = R1*R2*R3;

		return R;
	}

	inline Vector3d MatToRpy( Matrix3d R ) {

		Vector3d rpy;


		rpy(0) = atan2(    R(2,1), R(2,2) );  // roll
		rpy(1) = atan2( -R(2,0), sqrt( R(2,1)*R(2,1) + R(2,2)*R(2,2))); 	//pitch
		rpy(2) = atan2(R(1,0),R(0,0)); 	//yaw
		
		return rpy;
	}


	
	inline Matrix3d RpyToMat( Vector3d rpy) {
		double r, p, y;
		r = rpy(0);
		p = rpy(1);
		y = rpy(2);

		double cf = cos(y);
		double sf = sin(y);

		double ct = cos(p);
		double st = sin(p);

		double cp = cos(r);
		double sp = sin(r);

    	Matrix3d R;
    	R << cf*ct, cf*st*sp-sf*cp, cf*st*cp + sf*sp,
            sf*ct, sf*st*sp+cf*cp, sf*st*cp - cf*sp,
            -st, ct*sp, ct*cp;

	    return R;
  }

	// This method computes the XYZ Euler angles from the Rotational matrix R.
	inline Vector3d R2XYZ(Matrix3d R) {
		double phi=0.0, theta=0.0, psi=0.0;
		Vector3d XYZ = Vector3d::Zero();
		
		theta = asin(R(0,2));
		
		if(fabsf(cos(theta))>pow(10.0,-10.0))
		{
			phi=atan2(-R(1,2)/cos(theta), R(2,2)/cos(theta));
			psi=atan2(-R(0,1)/cos(theta), R(0,0)/cos(theta));
		}
		else
		{
			if(fabsf(theta-M_PI/2.0)<pow(10.0,-5.0))
			{
				psi = 0.0;
				phi = atan2(R(1,0), R(2,0));
				theta = M_PI/2.0;
			}
			else
			{
				psi = 0.0;
				phi = atan2(-R(1,0), R(2,0));
				theta = -M_PI/2.0;
			}
		}
		
		XYZ << phi,theta,psi;
		return XYZ;
	}

	inline Matrix3d angleAxis2Rot(Vector3d ri, float theta){
	Matrix3d R;
	R << ri[0]*ri[0] * (1 - cos(theta)) + cos(theta)           , ri[0] * ri[1] * (1 - cos(theta)) - ri[2] * sin(theta) , ri[0] * ri[2] * (1 - cos(theta)) + ri[1] * sin(theta),
	         ri[0] * ri[1] * (1 - cos(theta)) + ri[2] * sin(theta) , ri[1]*ri[1] * (1 - cos(theta)) + cos(theta)           , ri[1] * ri[2] * (1 - cos(theta)) - ri[0] * sin(theta),
	         ri[0] * ri[2] * (1 - cos(theta)) - ri[1] * sin(theta) , ri[1] * ri[2] * (1 - cos(theta)) + ri[0] * sin(theta) , ri[2]*ri[2] * (1 - cos(theta)) + cos(theta);

	return R;

	}



	inline Vector3d butt_filter(Vector3d x, Vector3d x1, Vector3d x2, float omega_n, float zita, float ctrl_T){
		//applico un filtro di Butterworth del secondo ordine (sfrutto Eulero all'indietro)
		return x1*(2.0 + 2.0*omega_n*zita*ctrl_T)/(omega_n*omega_n*ctrl_T*ctrl_T + 2.0*omega_n*zita*ctrl_T + 1.0) - x2/(omega_n*omega_n*ctrl_T*ctrl_T + 2.0*omega_n*zita*ctrl_T + 1.0) + x*(omega_n*omega_n*ctrl_T*ctrl_T)/(omega_n*omega_n*ctrl_T*ctrl_T + 2.0*omega_n*zita*ctrl_T + 1.0);
	}



	/// converts a rate in Hz to an integer period in ms.
	inline uint16_t rateToPeriod(const float & rate) {
		if (rate > 0)
			return static_cast<uint16_t> (1000.0 / rate);
		else
			return 0;
	}




	    //Quaternion to rotration Matrix
	inline Matrix3d QuatToMat(Vector4d Quat){
		Matrix3d Rot;
		float s = Quat[0];
		float x = Quat[1];
		float y = Quat[2];
		float z = Quat[3];
		Rot << 1-2*(y*y+z*z),2*(x*y-s*z),2*(x*z+s*y),
		2*(x*y+s*z),1-2*(x*x+z*z),2*(y*z-s*x),
		2*(x*z-s*y),2*(y*z+s*x),1-2*(x*x+y*y);
		return Rot;
	}

	inline Vector3d quatToRpy( Vector4d q ) {
		return utilities::MatToRpy( utilities::QuatToMat( q ) );	
	}

	inline Vector4d RpyToQuat( Vector3d rpy ) {
		return utilities::rot2quat( utilities::RpyToMat( rpy ) );
	}
	
	
	inline float rad2deg(float rad){
		float deg;
		deg = 180.0*rad/M_PI;
		return deg;
	}
	
	inline float deg2rad(float deg){
		float rad;
		rad = M_PI*deg/180.0;
		return rad;
	}

	//--------------------SCREW THEORY FUNCTION----------------------------
	inline Eigen::MatrixXd Ad_f(Eigen::Matrix4d T){
		Eigen::MatrixXd Ad_T;
    	Ad_T.resize(6,6);
		Ad_T.setZero();
    	Ad_T.block<3,3>(0,0) = T.block<3,3>(0,0);
    	// Ad_T.block<3,3>(0,3).setZero();
        Ad_T.block<3,3>(3,0) = skew(T.block<3,1>(0,3))*T.block<3,3>(0,0);
        Ad_T.block<3,3>(3,3) = T.block<3,3>(0,0);
		return Ad_T;
	}

	inline Eigen::Matrix4d M_f(Eigen::VectorXd rot_vec, Eigen::VectorXd theta_rest, Eigen::MatrixXd _d_base, int index){
        Eigen::Matrix4d M;
		Eigen::Matrix3d R, R_temp;
		R.setIdentity();
		M.setIdentity();
		Eigen::Vector3d dist;
		dist<<0,0,0;
		
		for (int i=0;i<2;i++){
			if (rot_vec[i]==0){
				R_temp.setIdentity();
			}
			else if (rot_vec[i]==1){
				R_temp = utilities::rotx(theta_rest[i]);
			}
			else if (rot_vec[i]==2){
				R_temp = utilities::roty(theta_rest[i]);			
			}
			else if (rot_vec[i]==3){
				R_temp = utilities::rotz(theta_rest[i]);
			}
			R = R*R_temp;
		}
		for(int i=0;i<=index;i++){
			dist = dist + _d_base.block<1,3>(i,0).transpose(); 
		}
		M.block<3,3>(0,0) = R;
		M.block<3,1>(0,3) = dist;

		return M;
	}

	inline Eigen::MatrixXd G_f(float m, Eigen::Matrix3d I, Eigen::Vector3d inertial_disp){
		Eigen::MatrixXd G, Gc;
		Eigen::Matrix4d T;
		Eigen::Matrix3d identity;
		identity.setIdentity();
		G.resize(6,6);
		Gc.resize(6,6);
		Gc.setIdentity();
		Gc.block<3,3>(0,0) = I;
		Gc.block<3,3>(3,3) = m*identity;
		T.setIdentity();
		T.block<3,1>(0,3) = inertial_disp;
		G = utilities::Ad_f(T.inverse()).transpose()*Gc*utilities::Ad_f(T.inverse());
		return G;
	}

	inline Eigen::MatrixXd ad_f_(Eigen::VectorXd V){
		Eigen::MatrixXd ad;
		ad.resize(6,6);
		ad.setZero();
		ad.block<3,3>(0,0) = utilities::skew(V.head(3));
		ad.block<3,3>(3,0) = utilities::skew(V.tail(3));
		ad.block<3,3>(3,3) = utilities::skew(V.head(3));
		return ad;
	}
}

#endif