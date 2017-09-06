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
	cam_ = new vk::PinholeCamera(752, 480, 315.5, 315.5, 376.0, 240.0);
	vo_ = new svo::FrameHandlerMono(cam_);
	viewer = new Viewer(cam_, vo_);
	vo_->start();
	//view = std::thread(&Viewer::run, viewer);
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
	for (int img_id = 2; img_id < 188; ++img_id)
	{
		// load image
		std::stringstream ss;
		ss << "W:/codecraft/svio_slam/svio_slam/dataset/sin2_tex2_h1_v8_d/img/frame_"
			<< std::setw(6) << std::setfill('0') << img_id << "_0.png";
		if (img_id == 2)
			std::cout << "reading image " << ss.str() << std::endl;
		cv::Mat img(cv::imread(ss.str().c_str(), 0));
		assert(!img.empty());

		// process frame
		vo_->addImage(img, 0.01*img_id);

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


int main_test(int argc, char** argv)
{
	{
		BenchmarkNode benchmark;
		benchmark.runFromFolder();
	}
	printf("BenchmarkNode finished.\n");
	return 0;
}

int main()
{
	EuRoCData dataset("W:/vio/datasets/MH_01_easy/mav0");
	return 0;
}

