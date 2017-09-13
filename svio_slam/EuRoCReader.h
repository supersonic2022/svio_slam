#pragma once

#include <iostream>
#include <vector>
#include <string>
#include "global.h"

using namespace std;

struct IMUParam
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Eigen::Matrix4d T_BS;
	int rate_hz;
	double gyroscope_noise_density;
	double gyroscope_random_walk;
	double accelerometer_noise_density;
	double accelerometer_random_walk;
};

struct CameraParam
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Eigen::Matrix4d T_BS;
	int rate_hz;
	Eigen::Vector2i resolution;
	string camera_model;
	Eigen::Vector4d intrinsics;
	string distortion_model;
	Eigen::Vector4d distortion_coefficients;
};


class EuRoCData
{	
public:
	EuRoCData(string dir, int cams = 2, int imus = 1);

	void readImgsAndParams();

	void readImusAndParams();


private:
	const string body_yaml = "body.yaml";
	const string sensor_yaml = "sensor.yaml";
	const string data_csv = "data.csv";
	
	string mav_filedir;
	int cam_num;
	int imu_num;

public:
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	vector<string> cam_data_files;
	vector<vector<string>> img_timestamps;

	EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix<double, 6, 1>)
	vector<vector<pair<string, Eigen::Matrix<double, 6, 1>>>> imu_timestamps;

	vector<CameraParam> cam_params;
	vector<IMUParam> imu_params;

	Eigen::Vector3d gravity;
};