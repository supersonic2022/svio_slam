#pragma once

#include "global.h"

class Preintegration
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Eigen::Vector3d _delta_P;    // P_k+1 = P_k + V_k*dt + R_k*a_k*dt*dt/2
	Eigen::Vector3d _delta_V;    // V_k+1 = V_k + R_k*a_k*dt
	Eigen::Matrix3d _delta_R;    // R_k+1 = R_k*exp(w_k*dt).     note: Rwc, Rwc'=Rwc*[w_body]x

	// jacobian of delta measurements w.r.t bias of gyro/acc
	Eigen::Matrix3d _J_P_Biasg;     // position / gyro
	Eigen::Matrix3d _J_P_Biasa;     // position / acc
	Eigen::Matrix3d _J_V_Biasg;     // velocity / gyro
	Eigen::Matrix3d _J_V_Biasa;     // velocity / acc
	Eigen::Matrix3d _J_R_Biasg;   // rotation / gyro

    // noise covariance propagation of delta measurements
	Eigen::Matrix<double, 9, 9> _cov_P_V_Phi;

	double _delta_time;

public:
	Preintegration();
	Preintegration(const Preintegration& pre);

	void reset();

	void update(const Eigen::Vector3d& omega_true, const Eigen::Vector3d& acc_true, const double& dt);

	static Eigen::Matrix3d skew(const Eigen::Vector3d& a)
	{
		return Sophus::SO3d::hat(a);
	}

	static Eigen::Matrix3d Expmap(const Eigen::Vector3d& a)
	{
		return Sophus::SO3d::exp(a).matrix();
	}

	static Eigen::Matrix3d JacobianR(const Eigen::Vector3d& a)
	{
		Eigen::Matrix3d Jr = Eigen::Matrix3d::Identity();
		double theta = a.norm();

		if (theta<0.00001)
		{
			return Jr;// = Matrix3d::Identity();
		}
		else
		{
			Eigen::Vector3d k = a.normalized();  // k - unit direction vector of w
			Eigen::Matrix3d K = skew(k);
			Jr = Eigen::Matrix3d::Identity()
				- (1 - cos(theta)) / theta*K
				+ (1 - sin(theta) / theta)*K*K;
		}
		return Jr;
	}

	static Eigen::Matrix3d JacobianRInv(const Eigen::Vector3d& w)
	{
		Eigen::Matrix3d Jrinv = Eigen::Matrix3d::Identity();
		double theta = w.norm();

		// very small angle
		if (theta < 0.00001)
		{
			return Jrinv;
		}
		else
		{
			Eigen::Vector3d k = w.normalized();  // k - unit direction vector of w
			Eigen::Matrix3d K = Sophus::SO3d::hat(k);
			Jrinv = Eigen::Matrix3d::Identity()
				+ 0.5*Sophus::SO3d::hat(w)
				+ (1.0 - (1.0 + cos(theta))*theta / (2.0*sin(theta))) *K*K;
		}

		return Jrinv;
	}

	inline Eigen::Quaterniond normalizeRotationQ(const Eigen::Quaterniond& r)
	{
		Eigen::Quaterniond _r(r);
		if (_r.w()<0)
		{
			_r.coeffs() *= -1;
		}
		return _r.normalized();
	}

	inline Eigen::Matrix3d normalizeRotationM(const Eigen::Matrix3d& R)
	{
		Eigen::Quaterniond qr(R);
		return normalizeRotationQ(qr).toRotationMatrix();
	}
};