#include "EuRoCReader.h"
#include <fstream>
#include <string>

EuRoCData::EuRoCData(string dir, int cams, int imus):
	mav_filedir(dir), cam_num(cams), imu_num(imus)
{
	readImgs();
	readImus();
}

void EuRoCData::readImgs()
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
		sort(cam_vec.begin(), cam_vec.end());
		img_timestamps.push_back(cam_vec);
		cout << num_of_imgs << endl;
	}
}


void EuRoCData::readImus()
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
	}
}