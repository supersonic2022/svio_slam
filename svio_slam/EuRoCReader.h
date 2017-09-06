#pragma once

#include <iostream>
#include <vector>
#include "global.h"

using namespace std;

class EuRoCData
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
	EuRoCData(string dir, int cams = 2, int imus = 1);

	void readImgs();

	void readImus();

private:
	const string body_yaml = "body.yaml";
	const string sensor_yaml = "sensor.yaml";
	const string data_csv = "data.csv";
	
	string mav_filedir;
	int cam_num;
	int imu_num;

public:
	vector<string> cam_data_files;
	vector<vector<string>> img_timestamps;

	EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix<double, 6, 1>)
	vector<vector<pair<string, Eigen::Matrix<double, 6, 1>>>> imu_timestamps;
};