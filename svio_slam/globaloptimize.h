#pragma once

#include "g2otypes.h"
#include "global.h"
#include "map.h"
#include "frame.h"
#include <thread>
#include "EuRoCReader.h"

class GlobalOptimize
{
public:
	GlobalOptimize(svo::Map* pMap_, EuRoCData* param);

	void run();

private:
	svo::Map* pMap;

	std::list< svo::FramePtr > tmp_kfs; //tempearary kf storage for optimization 
	Eigen::Vector3d gw;
	int nIterations;
	bool bRobust;
	std::thread* thread_;
	EuRoCData* param_;

	void get_kfs();

	void GlobalBundleAdjustmentNavState();
};
