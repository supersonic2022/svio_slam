#pragma once

#include "global.h"

class IMUData
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// covariance of measurement
	static Eigen::Matrix3d _gyrMeasCov;
	static Eigen::Matrix3d _accMeasCov;
	static Eigen::Matrix3d getGyrMeasCov(void) { return _gyrMeasCov; }
	static Eigen::Matrix3d getAccMeasCov(void) { return _accMeasCov; }

	// covariance of bias random walk
	static Eigen::Matrix3d _gyrBiasRWCov;
	static Eigen::Matrix3d _accBiasRWCov;
	static Eigen::Matrix3d getGyrBiasRWCov(void) { return _gyrBiasRWCov; }
	static Eigen::Matrix3d getAccBiasRWCov(void) { return _accBiasRWCov; }

	static double _gyrBiasRw2;
	static double _accBiasRw2;
	static double getGyrBiasRW2(void) { return _gyrBiasRw2; }
	static double getAccBiasRW2(void) { return _accBiasRw2; }

	//custom set function
	static void setGyrBiasRW2_Cov(const double& grw);
	static void setAccBiasRW2_Cov(const double& arw);
	static void setGyrMeasCov(const double& gnoise);
	static void setAccMeasCov(const double& anoise);

	IMUData(const double& gx, const double& gy, const double& gz,
		const double& ax, const double& ay, const double& az,
		const double& t);
	//IMUData(const IMUData& imu);

	IMUData(const Eigen::Matrix<double, 6, 1>& g_a,const double&& t);

	// Raw data of imu's
	Eigen::Vector3d _g;    //gyr data
	Eigen::Vector3d _a;    //acc data
	double _t;      //time duration
};