#ifndef SO3_UTILS_HPP
#define SO3_UTILS_HPP

#include <Eigen/Dense>

class so3
{
public:
	static Eigen::Quaternion<double> R2q( Eigen::Matrix<double,3,3> R )
	{
		double R11 = R(0,0);
		double R12 = R(0,1);
		double R13 = R(0,2);
		double R21 = R(1,0);
		double R22 = R(1,1);
		double R23 = R(1,2);
		double R31 = R(2,0);
		double R32 = R(2,1);
		double R33 = R(2,2);
		
		Eigen::Matrix<double,4,4> T = Eigen::Matrix<double,4,4>::Ones();
		T(1,1) = -1.0;
		T(1,2) = -1.0;
		T(2,0) = -1.0;
		T(2,2) = -1.0;
		T(3,0) = -1.0;
		T(3,1) = -1.0;
		
		Eigen::Matrix<double,4,1> diag;
		diag(0,0) = R11;
		diag(1,0) = R22;
		diag(2,0) = R33;
		diag(3,0) = 1.0;
		
		Eigen::Matrix<double,4,1> sq = 0.25*(T*diag);
		double q0 = sqrt(sq(0,0));
		double q1 = sqrt(sq(1,0));
		double q2 = sqrt(sq(2,0));
		double q3 = sqrt(sq(3,0));
		
		if( (q0 >= q1) && (q0 >= q2) && (q0 >= q3) )
		{
		    q1 = copysign(q1, R32-R23);
		    q2 = copysign(q2, R13-R31);
		    q3 = copysign(q3, R21-R12);
		}
		else if( (q1 >= q0) && (q1 >= q2) && (q1 >= q3) )
		{
		    q0 = copysign(q0, R32-R23);
		    q2 = copysign(q2, R21+R12);
		    q3 = copysign(q3, R13+R31);
		}
		else if( (q2 >= q0) && (q2 >= q1) && (q2 >= q3) )
		{
		    q0 = copysign(q0, R13-R31);
		    q1 = copysign(q1, R21+R12);
		    q3 = copysign(q3, R32+R23);
		}
		else if( (q3 >= q0) && (q3 >= q1) && (q3 >= q2) )
		{
		    q0 = copysign(q0, R21-R12);
		    q1 = copysign(q1, R31+R13);
		    q2 = copysign(q2, R32+R23);
		}
		
		Eigen::Quaternion<double> q;
		q.w() = q0;
		q.x() = q1;
		q.y() = q2;
		q.z() = q3;
		
		return q;
	}

	static Eigen::Matrix<double,3,1> R2rpy( Eigen::Matrix<double,3,3> R )
	{
		double r = atan2(R(2,1), R(2,2));
		double p = atan2(-R(2,0), sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2)));
		double y = atan2(R(1,0), R(0,0));
		
		Eigen::Matrix<double,3,1> euler;
		euler(0,0) = r;
		euler(1,0) = p;
		euler(2,0) = y;
		
		return euler;
	}
	static Eigen::Matrix<double,3,3> rpy2R( double phi, double theta, double psi )
	{
		Eigen::Matrix<double,3,3> R;
		R(0,0) = cos(psi)*cos(theta);
		R(0,1) = cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi);
		R(0,2) = sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta);
		R(1,0) = cos(theta)*sin(psi);
		R(1,1) = cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta);
		R(1,2) = cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi);
		R(2,0) = -sin(theta);
		R(2,1) = cos(theta)*sin(phi);
		R(2,2) = cos(phi)*cos(theta);
		return R;
	}
	static Eigen::Matrix<double,3,3> Q( double phi, double theta)
	{
		Eigen::Matrix<double, 3, 3> Q;
		Q(0,0) = 1;
		Q(1,0) = 0;
		Q(2,0) = 0;
		Q(0,1) = 0;
		Q(1,1) = cos(phi);
		Q(2,1) = -sin(phi);
		Q(0,2) = -sin(theta);
		Q(1,2) = sin(phi)*cos(theta);
		Q(2,2) = cos(phi)*cos(theta);

		return Q;
	}
	static Eigen::Matrix<double, 3, 3> Qdot( double phi, double theta, double phidot, double thetadot)
	{
		Eigen::Matrix<double, 3, 3> Qdot;
		Qdot(0,0) = 0;
		Qdot(1,0) = 0;
		Qdot(2,0) = 0;
		Qdot(0,1) = 0;
		Qdot(1,1) = -sin(phi)*phidot;
		Qdot(2,1) = -cos(phi)*phidot;
		Qdot(0,2) = -cos(theta)*thetadot;
		Qdot(1,2) = cos(phi)*cos(theta)*phidot - sin(phi)*sin(theta)*thetadot;
		Qdot(2,2) = -sin(phi)*cos(theta)*phidot-cos(phi)*sin(theta)*thetadot;

		return Qdot;
	}
	static Eigen::Matrix<double, 3, 3> hat(Eigen::Matrix<double, 3, 1> p)
	{
		Eigen::Matrix<double, 3, 3> phat;
		phat(0,0) = 0;
		phat(1,0) = p(2,0);
		phat(2,0) = -p(1,0);
		phat(0,1) = -p(2,0);
		phat(1,1) = 0;
		phat(2,1) = p(0,0);
		phat(0,2) = p(1,0);
		phat(1,2) = -p(0,0);
		phat(2,2) = 0;

		return phat;
	}

    static Eigen::Matrix<double, 3, 1> vee(Eigen::Matrix<double, 3, 3> Mtx)
	{
		Eigen::Matrix<double, 3, 1> result_vec;
		result_vec(2) = Mtx(1,0);
		result_vec(0) = Mtx(2,1);
		result_vec(1) = Mtx(0,2);
		
		return result_vec;
	}
};

#endif