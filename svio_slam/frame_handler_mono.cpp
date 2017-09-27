// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "config.h"
#include "frame_handler_mono.h"
#include "map.h"
#include "frame.h"
#include "feature.h"
#include "point.h"
#include "pose_optimizer.h"
#include "sparse_img_align.h"
#include "vikit/performance_monitor.h""
#include "depth_filter.h"
#ifdef USE_BUNDLE_ADJUSTMENT
#include "bundle_adjustment.h"
//std::bind
#include <functional>
#endif

namespace svo {

FrameHandlerMono::FrameHandlerMono(vk::AbstractCamera* cam, EuRoCData* param) :
	FrameHandlerBase(),
	cam_(cam),
	param_(param),
	reprojector_(cam_, map_),
	depth_filter_(NULL),
	global(NULL),
	imu_state_(NO_FRAME)
{
	initialize();
}

void FrameHandlerMono::initialize()
{
	feature_detection::DetectorPtr feature_detector(
		  new feature_detection::FastDetector(
			  cam_->width(), cam_->height(), Config::gridSize(), Config::nPyrLevels()));
	DepthFilter::callback_t depth_filter_cb = std::bind(
		  &MapPointCandidates::newCandidatePoint, &map_.point_candidates_, std::placeholders::_1, std::placeholders::_2);
	depth_filter_ = new DepthFilter(feature_detector, depth_filter_cb);
	depth_filter_->startThread();
  
	//global = new GlobalOptimize(&map_, param_);
	//globalopt = new std::thread(&GlobalOptimize::run, global);
}

FrameHandlerMono::~FrameHandlerMono()
{
	delete depth_filter_;
}

void FrameHandlerMono::addImage(const cv::Mat& img, const double timestamp)
{
	if(!startFrameProcessingCommon(timestamp))
		return;

	// some cleanup from last iteration, can't do before because of visualization
	core_kfs_.clear();
	overlap_kfs_.clear();

	// create new frame
	SVO_START_TIMER("pyramid_creation");
	new_frame_.reset(new Frame(cam_, img.clone(), timestamp));
	SVO_STOP_TIMER("pyramid_creation");

	imu_state_ = NEW_FRAME;

	// process frame
	UpdateResult res = RESULT_FAILURE;
	if(stage_ == STAGE_DEFAULT_FRAME)
		res = processFrame();
	else if(stage_ == STAGE_SECOND_FRAME)
		res = processSecondFrame();
	else if(stage_ == STAGE_FIRST_FRAME)
		res = processFirstFrame();
	else if(stage_ == STAGE_RELOCALIZING)
		res = relocalizeFrame(SE3d(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()),
                          map_.getClosestKeyframe(last_frame_));


	if (stage_ != STAGE_FIRST_FRAME)
	{
		new_frame_->SetInitialNavStateAndBias(new_frame_->imuState);
		new_frame_->imuPreint = imuPreint;
		new_frame_->UpdateNavState(imuPreint, param_->gravity);
		if (new_frame_->imuState.Get_dBias_Acc().norm() > 1e-6) cerr << "PredictNavStateByIMU1 current Frame dBias acc not zero" << endl;
		if (new_frame_->imuState.Get_dBias_Gyr().norm() > 1e-6) cerr << "PredictNavStateByIMU1 current Frame dBias gyr not zero" << endl;
	}

	
	Eigen::Matrix3d svo_so3 = (param_->cam_params[0].T_BS.block<3, 3>(0, 0)) * (new_frame_->T_f_w_.so3().matrix());
	Eigen::Quaterniond q(svo_so3);
	cout << "svo result so3 : " << endl << q.coeffs() << endl;
	//cout << "imu result so3 : " << endl << new_frame_->imuState.Get_R().log() << endl;
	//cout << "position : " << new_frame_->imuState.Get_P() << endl;
	cout << endl;

	//cv::Mat img_show;  
	//cv::cvtColor(img, img_show, cv::COLOR_GRAY2RGB);
	//for (auto feat_it : new_frame_->fts_)
	//{
	//	const float r = 5;
	//	cv::Point2f pt1, pt2;
	//	cv::Point2f pt;
	//	pt.x = feat_it->px[0];
	//	pt.y = feat_it->px[1];
	//	pt1.x = feat_it->px[0] - r;
	//	pt1.y = feat_it->px[1] - r;
	//	pt2.x = feat_it->px[0] + r;
	//	pt2.y = feat_it->px[1] + r;

	//	cv::rectangle(img_show, pt1, pt2, cv::Scalar(0, 255, 0));
	//	cv::circle(img_show, pt, r, cv::Scalar(0, 255, 0), -1);
	//}
	//cv::imshow("ORB-SLAM2: Current Frame", img_show);
	//cv::waitKey(0);

	// set last frame
	last_frame_ = new_frame_;
	new_frame_.reset();
	// finish processing
	finishFrameProcessingCommon(last_frame_->id_, res, last_frame_->nObs());
}

void FrameHandlerMono::addImu(const IMUData& imu) 
{
	if (imu_state_ == NO_FRAME)
		return;
	if (imu_state_ == NEW_FRAME)
	{
		imuPreint.reset();
		imu_state_ = LAST_FRAME;
	}
	
	Eigen::Vector3d bg;
	Eigen::Vector3d ba;
	if (!last_frame_)
	{
		//prior bias form okvis(maybe should add this prior in first frame? yes)		
		bg.setConstant(0.03);
		ba.setConstant(0.1);
		last_frame_->imuState.Set_BiasGyr(bg);
		last_frame_->imuState.Set_BiasAcc(ba);
	}
	else
	{
		bg = last_frame_->imuState.Get_BiasGyr();
		ba = last_frame_->imuState.Get_BiasAcc();
	}

	imuPreint.update(imu._g - bg, imu._a - ba, imu._t);
}

FrameHandlerMono::UpdateResult FrameHandlerMono::processFirstFrame()
{
	new_frame_->T_f_w_ = SE3d(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
	if(klt_homography_init_.addFirstFrame(new_frame_) == initialization::FAILURE)
		return RESULT_NO_KEYFRAME;
	new_frame_->setKeyframe();
	map_.addKeyframe(new_frame_);
	stage_ = STAGE_SECOND_FRAME;
	SVO_INFO_STREAM("Init: Selected first frame.");
	return RESULT_IS_KEYFRAME;
}

FrameHandlerBase::UpdateResult FrameHandlerMono::processSecondFrame()
{
	initialization::InitResult res = klt_homography_init_.addSecondFrame(new_frame_);
	if(res == initialization::FAILURE)
		return RESULT_FAILURE;
	else if(res == initialization::NO_KEYFRAME)
		return RESULT_NO_KEYFRAME;

	// two-frame bundle adjustment
#ifdef USE_BUNDLE_ADJUSTMENT
	ba::twoViewBA(new_frame_.get(), map_.lastKeyframe().get(), Config::lobaThresh(), &map_);
#endif

	new_frame_->setKeyframe();
	double depth_mean, depth_min;
	frame_utils::getSceneDepth(*new_frame_, depth_mean, depth_min);
	depth_filter_->addKeyframe(new_frame_, depth_mean, 0.5*depth_min);

	// add frame to map
	map_.addKeyframe(new_frame_);
	stage_ = STAGE_DEFAULT_FRAME;
	klt_homography_init_.reset();
	SVO_INFO_STREAM("Init: Selected second frame, triangulated initial map.");
	return RESULT_IS_KEYFRAME;
}

FrameHandlerBase::UpdateResult FrameHandlerMono::processFrame()
{
	// Set initial pose TODO use prior(prior can be replaced by imu preintegration)
	new_frame_->T_f_w_ = last_frame_->T_f_w_;

	// step1 : sparse image align
	SVO_START_TIMER("sparse_img_align");
	SparseImgAlign img_align(Config::kltMaxLevel(), Config::kltMinLevel(),
                           30, SparseImgAlign::GaussNewton, false, false);
	size_t img_align_n_tracked = img_align.run(last_frame_, new_frame_);
	SVO_STOP_TIMER("sparse_img_align");
	SVO_LOG(img_align_n_tracked);
	SVO_DEBUG_STREAM("Img Align:\t Tracked = " << img_align_n_tracked);

	// step2 : map reprojection & feature alignment
	SVO_START_TIMER("reproject");
	reprojector_.reprojectMap(new_frame_, overlap_kfs_);
	SVO_STOP_TIMER("reproject");
	const size_t repr_n_new_references = reprojector_.n_matches_;
	const size_t repr_n_mps = reprojector_.n_trials_;
	SVO_LOG2(repr_n_mps, repr_n_new_references);
	SVO_DEBUG_STREAM("Reprojection:\t nPoints = "<<repr_n_mps<<"\t \t nMatches = "<<repr_n_new_references);
	if(repr_n_new_references < Config::qualityMinFts())
	{
		SVO_WARN_STREAM_THROTTLE(1.0, "Not enough matched features.");
		new_frame_->T_f_w_ = last_frame_->T_f_w_; // reset to avoid crazy pose jumps
		tracking_quality_ = TRACKING_INSUFFICIENT;
		return RESULT_FAILURE;
	}

	// step3 : pose optimization
	SVO_START_TIMER("pose_optimizer");
	size_t sfba_n_edges_final;
	double sfba_thresh, sfba_error_init, sfba_error_final;
	pose_optimizer::optimizeGaussNewton(
		Config::poseOptimThresh(), Config::poseOptimNumIter(), false,
		new_frame_, sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);
	SVO_STOP_TIMER("pose_optimizer");
	SVO_LOG4(sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);
	SVO_DEBUG_STREAM("PoseOptimizer:\t ErrInit = "<<sfba_error_init<<"px\t thresh = "<<sfba_thresh);
	SVO_DEBUG_STREAM("PoseOptimizer:\t ErrFin. = "<<sfba_error_final<<"px\t nObsFin. = "<<sfba_n_edges_final);
	if(sfba_n_edges_final < 20)
		return RESULT_FAILURE;

	// step4 : structure optimization
	SVO_START_TIMER("point_optimizer");
	optimizeStructure(new_frame_, Config::structureOptimMaxPts(), Config::structureOptimNumIter());
	SVO_STOP_TIMER("point_optimizer");

	// select keyframe
	core_kfs_.insert(new_frame_);
	setTrackingQuality(sfba_n_edges_final);
	if(tracking_quality_ == TRACKING_INSUFFICIENT)
	{
		new_frame_->T_f_w_ = last_frame_->T_f_w_; // reset to avoid crazy pose jumps
		return RESULT_FAILURE;
	}
  double depth_mean, depth_min;
  frame_utils::getSceneDepth(*new_frame_, depth_mean, depth_min);
  if(!needNewKf(depth_mean) || tracking_quality_ == TRACKING_BAD)
  {
    depth_filter_->addFrame(new_frame_);
    return RESULT_NO_KEYFRAME;
  }
  new_frame_->setKeyframe();
  SVO_DEBUG_STREAM("New keyframe selected.");

  // new keyframe selected
  for(Features::iterator it=new_frame_->fts_.begin(); it!=new_frame_->fts_.end(); ++it)
    if((*it)->point != NULL)
      (*it)->point->addFrameRef(*it);
  map_.point_candidates_.addCandidatePointToFrame(new_frame_);

  // optional bundle adjustment
#ifdef USE_BUNDLE_ADJUSTMENT
  if(Config::lobaNumIter() > 0)
  {
    SVO_START_TIMER("local_ba");
    setCoreKfs(Config::coreNKfs());
    size_t loba_n_erredges_init, loba_n_erredges_fin;
    double loba_err_init, loba_err_fin;
    ba::localBA(new_frame_.get(), &core_kfs_, &map_,
                loba_n_erredges_init, loba_n_erredges_fin,
                loba_err_init, loba_err_fin);
    SVO_STOP_TIMER("local_ba");
    SVO_LOG4(loba_n_erredges_init, loba_n_erredges_fin, loba_err_init, loba_err_fin);
    SVO_DEBUG_STREAM("Local BA:\t RemovedEdges {"<<loba_n_erredges_init<<", "<<loba_n_erredges_fin<<"} \t "
                     "Error {"<<loba_err_init<<", "<<loba_err_fin<<"}");
  }
#endif

  // init new depth-filters
  depth_filter_->addKeyframe(new_frame_, depth_mean, 0.5*depth_min);

  // if limited number of keyframes, remove the one furthest apart
  if(Config::maxNKfs() > 2 && map_.size() >= Config::maxNKfs())
  {
    FramePtr furthest_frame = map_.getFurthestKeyframe(new_frame_->pos());
    depth_filter_->removeKeyframe(furthest_frame); // TODO this interrupts the mapper thread, maybe we can solve this better
    map_.safeDeleteFrame(furthest_frame);
  }

  // add keyframe to map
  map_.addKeyframe(new_frame_);

  return RESULT_IS_KEYFRAME;
}

FrameHandlerMono::UpdateResult FrameHandlerMono::relocalizeFrame(
    const SE3d& T_cur_ref,
    FramePtr ref_keyframe)
{
  SVO_WARN_STREAM_THROTTLE(1.0, "Relocalizing frame");
  if(ref_keyframe == nullptr)
  {
    SVO_INFO_STREAM("No reference keyframe.");
    return RESULT_FAILURE;
  }
  SparseImgAlign img_align(Config::kltMaxLevel(), Config::kltMinLevel(),
                           30, SparseImgAlign::GaussNewton, false, false);
  size_t img_align_n_tracked = img_align.run(ref_keyframe, new_frame_);
  if(img_align_n_tracked > 30)
  {
    SE3d T_f_w_last = last_frame_->T_f_w_;
    last_frame_ = ref_keyframe;
    FrameHandlerMono::UpdateResult res = processFrame();
    if(res != RESULT_FAILURE)
    {
      stage_ = STAGE_DEFAULT_FRAME;
      SVO_INFO_STREAM("Relocalization successful.");
    }
    else
      new_frame_->T_f_w_ = T_f_w_last; // reset to last well localized pose
    return res;
  }
  return RESULT_FAILURE;
}

bool FrameHandlerMono::relocalizeFrameAtPose(
    const int keyframe_id,
    const SE3d& T_f_kf,
    const cv::Mat& img,
    const double timestamp)
{
  FramePtr ref_keyframe;
  if(!map_.getKeyframeById(keyframe_id, ref_keyframe))
    return false;
  new_frame_.reset(new Frame(cam_, img.clone(), timestamp));
  UpdateResult res = relocalizeFrame(T_f_kf, ref_keyframe);
  if(res != RESULT_FAILURE) {
    last_frame_ = new_frame_;
    return true;
  }
  return false;
}

void FrameHandlerMono::resetAll()
{
  resetCommon();
  last_frame_.reset();
  new_frame_.reset();
  core_kfs_.clear();
  overlap_kfs_.clear();
  depth_filter_->reset();
}

void FrameHandlerMono::setFirstFrame(const FramePtr& first_frame)
{
  resetAll();
  last_frame_ = first_frame;
  last_frame_->setKeyframe();
  map_.addKeyframe(last_frame_);
  stage_ = STAGE_DEFAULT_FRAME;
}

bool FrameHandlerMono::needNewKf(double scene_depth_mean)
{
  for(auto it=overlap_kfs_.begin(), ite=overlap_kfs_.end(); it!=ite; ++it)
  {
    Eigen::Vector3d relpos = new_frame_->w2f(it->first->pos());
    if(fabs(relpos.x())/scene_depth_mean < Config::kfSelectMinDist() &&
       fabs(relpos.y())/scene_depth_mean < Config::kfSelectMinDist()*0.8 &&
       fabs(relpos.z())/scene_depth_mean < Config::kfSelectMinDist()*1.3)
      return false;
  }
  return true;
}

void FrameHandlerMono::setCoreKfs(size_t n_closest)
{
  size_t n = std::min(n_closest, overlap_kfs_.size()-1);
  //convert to c++ 11 and do some modification(may not work !!!)
  std::partial_sort(overlap_kfs_.begin(), overlap_kfs_.begin() + n, overlap_kfs_.end(),
					std::bind(std::greater<size_t>(),
                    std::bind(&pair<FramePtr, size_t>::second, std::placeholders::_1),
                    std::bind(&pair<FramePtr, size_t>::second, std::placeholders::_2)));
  for_each(overlap_kfs_.begin(), overlap_kfs_.end(), [&](pair<FramePtr,size_t>& i){ core_kfs_.insert(i.first); });
}

} // namespace svo
