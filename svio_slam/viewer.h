#ifndef VIEWER_H
#define VIEWER_H

//reference from orbslam viewer

#include "vikit\abstract_camera.h"
#include "frame_handler_mono.h"

class Viewer
{
public:
	Viewer(vk::AbstractCamera* cam_, svo::FrameHandlerMono* vo_);

	void run();

private:
	float width; 
	float height;

	vk::AbstractCamera* cam;
	svo::FrameHandlerMono* vo;

	double mT;

	//not thread safe yet
	void drawKFs();

	//void drawPoints();

};

#endif


