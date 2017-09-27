#include "config.h"
#include "frame_handler_mono.h"
#include "map.h"
#include "frame.h"
#include <vector>
#include <string>
#include "vikit/math_utils.h"
#include "vikit/vision.h"
#include "vikit/abstract_camera.h"
#include "vikit/atan_camera.h"
#include "vikit/pinhole_camera.h"
#include "opencv2/opencv.hpp"
#include <sophus/se3.hpp>
#include <iostream>
#include <iomanip>

#include "viewer.h"
#include <thread>
#include "EuRoCReader.h"

class BenchmarkNode
{	
	EuRoCData* dataset; 
	vk::AbstractCamera* cam_;
	svo::FrameHandlerMono* vo_;
	Viewer* viewer;
	std::thread view;

public:
	BenchmarkNode();
	~BenchmarkNode();
	void runFromFolder();
};

BenchmarkNode::BenchmarkNode()
{
	dataset = new EuRoCData("W:/vio/datasets/MH_01_easy/mav0");
	cam_ = new vk::PinholeCamera(
		dataset->cam_params[0].resolution[0], 
		dataset->cam_params[0].resolution[1], 
		dataset->cam_params[0].intrinsics[0], 
		dataset->cam_params[0].intrinsics[1], 
		dataset->cam_params[0].intrinsics[2], 
		dataset->cam_params[0].intrinsics[3],
		dataset->cam_params[0].distortion_coefficients[0],
		dataset->cam_params[0].distortion_coefficients[1], 
		dataset->cam_params[0].distortion_coefficients[2], 
		dataset->cam_params[0].distortion_coefficients[3]);
	vo_ = new svo::FrameHandlerMono(cam_, dataset);
	viewer = new Viewer(cam_, vo_);
	vo_->start();
	view = std::thread(&Viewer::run, viewer);
	IMUData::setGyrBiasRW2_Cov(dataset->imu_params[0].gyroscope_random_walk);
	IMUData::setGyrMeasCov(dataset->imu_params[0].gyroscope_noise_density);
	IMUData::setAccBiasRW2_Cov(dataset->imu_params[0].accelerometer_random_walk);
	IMUData::setAccMeasCov(dataset->imu_params[0].accelerometer_noise_density);
}

BenchmarkNode::~BenchmarkNode()
{
	if(view.joinable())
		view.join();
	delete vo_;
	delete cam_;
}

void BenchmarkNode::runFromFolder()
{
	for (int img_id = 0; img_id < dataset->img_timestamps[0].size(); ++img_id)
	{
		// load image
		std::stringstream ss;
		ss << dataset->cam_data_files[0] << dataset->img_timestamps[0][img_id] << ".png";
		cv::Mat img(cv::imread(ss.str().c_str(), 0));
		assert(!img.empty());
		
		//cv::imshow("img", img);
		// process frame
		std::cout << "image : " << dataset->img_timestamps[0][img_id] << std::endl;
		//if (img_id == 22)
		//	cout << endl;
		vo_->addImage(img, stod(dataset->img_timestamps[0][img_id]));
		

		while (1)
		{
			static int cur_imu_id = 1;
			//if (img_id == 0) break;
			static double last_img_timestampe = stod(dataset->img_timestamps[0][img_id]);
			if (img_id == (dataset->img_timestamps[0].size() - 1))
			{
				break;
			}
			double next_img_timestamp = stod(dataset->img_timestamps[0][img_id + 1]);
			
			static double last_imu_timestamp = stod(dataset->imu_timestamps[0][cur_imu_id-1].first);
			static double cur_imu_timestamp = stod(dataset->imu_timestamps[0][cur_imu_id].first);
			if (cur_imu_id == (dataset->imu_timestamps[0].size() - 1))
			{
				break;
			}
			double next_imu_timestamp = stod(dataset->imu_timestamps[0][cur_imu_id + 1].first);
			//check head(only in the initial situation)
			if (cur_imu_timestamp < last_img_timestampe)
			{
				cur_imu_id++; 
				last_img_timestampe = cur_imu_timestamp;
				cur_imu_timestamp = next_imu_timestamp;
				continue;
			}

			if (last_imu_timestamp < last_img_timestampe && cur_imu_timestamp >= last_img_timestampe)
			{
				IMUData imudata(dataset->imu_timestamps[0][cur_imu_id-1].second, cur_imu_timestamp - last_img_timestampe);
				vo_->addImu(imudata);
			}

			//check tail
			if (next_imu_timestamp >= next_img_timestamp)
			{
				IMUData imudata(dataset->imu_timestamps[0][cur_imu_id].second, next_img_timestamp - cur_imu_timestamp);
				vo_->addImu(imudata);
				std::cout << "imu : " << dataset->imu_timestamps[0][cur_imu_id].first << std::endl;
				last_img_timestampe = cur_imu_timestamp;
				cur_imu_timestamp = next_imu_timestamp;
				last_img_timestampe = next_img_timestamp; 
				cur_imu_id++;
				break;
			}

			IMUData imudata(dataset->imu_timestamps[0][cur_imu_id].second, next_imu_timestamp - cur_imu_timestamp);
			vo_->addImu(imudata);
			std::cout << "imu : " << dataset->imu_timestamps[0][cur_imu_id].first << std::endl;
			last_img_timestampe = cur_imu_timestamp;
			cur_imu_timestamp = next_imu_timestamp;	
			cur_imu_id++;
		}

		// display tracking quality
		if (vo_->lastFrame() != NULL)
		{
			std::cout << "Frame-Id: " << vo_->lastFrame()->id_ << " \t"
				<< "#Features: " << vo_->lastNumObservations() << " \t"
				<< "Proc. Time: " << vo_->lastProcessingTime() * 1000 << "ms \n";

			// access the pose of the camera via vo_->lastFrame()->T_f_w_.
		}
	}
}


int main()
{
	BenchmarkNode benchmark;
	benchmark.runFromFolder();
	return 0;
}

