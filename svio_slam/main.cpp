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
	vo_ = new svo::FrameHandlerMono(cam_);
	viewer = new Viewer(cam_, vo_);
	vo_->start();
	view = std::thread(&Viewer::run, viewer);
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
		vo_->addImage(img, stod(dataset->img_timestamps[0][img_id]));

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

