#include "EuRoCReader.h"
#include <fstream>
#include <string>

EuRoCData::EuRoCData(string dir, int cams, int imus):
	mav_filedir(dir), cam_num(cams), imu_num(imus)
{
	readImgsAndParams();
	readImusAndParams();
}

void EuRoCData::readImgsAndParams()
{
	for (int i = 0; i < cam_num; i++)
	{
		vector<string> cam_vec;
		string cam_ = mav_filedir + "/" + "cam" + to_string(i) + "/" ;
		ifstream cam_file(cam_ + data_csv);
		
		string cam_data_file = cam_ + "data/";
		cam_data_files.push_back(cam_data_file);
		
		if (!cam_file.good())
			cerr << "cam" << i << " csv file not found !" << endl;
		int num_of_imgs = 0;
		string cur_line;
		getline(cam_file, cur_line); // first line delete
		while (getline(cam_file, cur_line, ','))
		{
			if (cur_line == "") break;
			num_of_imgs++;
			cam_vec.push_back(cur_line);
			getline(cam_file, cur_line);
		}
		//sort(cam_vec.begin(), cam_vec.end());
		img_timestamps.push_back(cam_vec);
		cout << num_of_imgs << endl;

		cv::FileStorage param_file(cam_ + sensor_yaml, cv::FileStorage::READ);
		CameraParam cp;
		param_file["rate_hz"] >> cp.rate_hz;
		param_file["camera_model"] >> cp.camera_model;
		param_file["distortion_model"] >> cp.distortion_model;

		cv::FileNode T_BS_node = param_file["T_BS"]["data"];
		cv::FileNode resolution_node = param_file["resolution"];
		cv::FileNode intrinsics_node = param_file["intrinsics"];
		cv::FileNode distortion_coefficients_node = param_file["distortion_coefficients"];
		
		cp.T_BS << T_BS_node[0], T_BS_node[1], T_BS_node[2], T_BS_node[3],
			T_BS_node[4], T_BS_node[5], T_BS_node[6], T_BS_node[7],
			T_BS_node[8], T_BS_node[9], T_BS_node[10], T_BS_node[11],
			T_BS_node[12], T_BS_node[13], T_BS_node[14], T_BS_node[15];

		cp.resolution << resolution_node[0], resolution_node[1];
		cp.intrinsics << intrinsics_node[0], intrinsics_node[1], intrinsics_node[2], intrinsics_node[3];
		cp.distortion_coefficients << distortion_coefficients_node[0], distortion_coefficients_node[1],
			distortion_coefficients_node[2], distortion_coefficients_node[3];

		cam_params.push_back(cp);
	}
}


void EuRoCData::readImusAndParams()
{
	for (int i = 0; i < imu_num; i++)
	{
		vector < pair<string, Eigen::Matrix<double, 6, 1>>> imu_pair_vec;
		pair<string, Eigen::Matrix<double, 6, 1>> imu_pair;
		string imu_ = mav_filedir + "/" + "imu" + to_string(i) + "/";
		ifstream imu_file(imu_ + data_csv);

		if (!imu_file.good())
			cerr << "imu" << i << " csv file not found !" << endl;
		int num_of_imus = 0;
		string cur_line;
		getline(imu_file, cur_line); // first line delete
		while (getline(imu_file, cur_line, ','))
		{
			if (cur_line == "") break;
			num_of_imus++;
			imu_pair.first = cur_line;
			for (int i = 0; i < 5; i++)
			{
				getline(imu_file, cur_line, ',');
				imu_pair.second(i) = stod(cur_line);
			}
			getline(imu_file, cur_line);
			imu_pair.second(5) = stod(cur_line);
			imu_pair_vec.push_back(imu_pair);
		}
		imu_timestamps.push_back(imu_pair_vec);
		cout << num_of_imus << endl;

		cv::FileStorage param_file(imu_ + sensor_yaml, cv::FileStorage::READ);
		IMUParam ip;
		param_file["rate_hz"] >> ip.rate_hz;
		param_file["gyroscope_noise_density"] >> ip.gyroscope_noise_density;
		param_file["gyroscope_random_walk"] >> ip.gyroscope_random_walk;
		param_file["accelerometer_noise_density"] >> ip.accelerometer_noise_density;
		param_file["accelerometer_random_walk"] >> ip.accelerometer_random_walk;

		cv::FileNode T_BS_node = param_file["T_BS"]["data"];

		ip.T_BS << T_BS_node[0], T_BS_node[1], T_BS_node[2], T_BS_node[3],
			T_BS_node[4], T_BS_node[5], T_BS_node[6], T_BS_node[7],
			T_BS_node[8], T_BS_node[9], T_BS_node[10], T_BS_node[11],
			T_BS_node[12], T_BS_node[13], T_BS_node[14], T_BS_node[15];

		imu_params.push_back(ip);
	}
}
