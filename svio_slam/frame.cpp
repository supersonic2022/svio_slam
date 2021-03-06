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

#define NOMINMAX

#include <stdexcept>
#include "frame.h"
#include "feature.h"
#include "point.h"
#include "config.h"
//#include <boost/bind.hpp>
#include "vikit/math_utils.h"
#include "vikit/vision.h"
#include "vikit/performance_monitor.h"
#include "fast/fast.h"

namespace svo {

int Frame::frame_counter_ = 0;
int Frame::kf_counter_ = 0;

Frame::Frame(vk::AbstractCamera* cam, const cv::Mat& img, double timestamp) :
	id_(frame_counter_++),
	kf_id_(-1),
    timestamp_(timestamp),
    cam_(cam),
    key_pts_(5),
    is_keyframe_(false),
    v_kf_(NULL)
{
  initFrame(img);
}

Frame::~Frame()
{
  std::for_each(fts_.begin(), fts_.end(), [&](Feature* i){delete i;});
}

void Frame::initFrame(const cv::Mat& img)
{
	// check image
	if(img.empty() || img.type() != CV_8UC1 || img.cols != cam_->width() || img.rows != cam_->height())
	throw std::runtime_error("Frame: provided image has not the same size as the camera model or image is not grayscale");

	// Set keypoints to NULL
	std::for_each(key_pts_.begin(), key_pts_.end(), [&](Feature* ftr){ ftr=NULL; });

	// Build Image Pyramid
	frame_utils::createImgPyramid(img, std::max(Config::nPyrLevels(), Config::kltMaxLevel()+1), img_pyr_);
}

void Frame::SetInitialNavStateAndBias(const NavState& ns)
{
	imuState = ns;

	imuState.Set_BiasGyr(ns.Get_BiasGyr() + ns.Get_dBias_Gyr());
	imuState.Set_BiasAcc(ns.Get_BiasAcc() + ns.Get_dBias_Acc());
	imuState.Set_DeltaBiasGyr(Eigen::Vector3d::Zero());
	imuState.Set_DeltaBiasAcc(Eigen::Vector3d::Zero());
}

void Frame::UpdateNavState(const Preintegration& imupreint, const Eigen::Vector3d& gw)
{
	Eigen::Matrix3d dR = imupreint._delta_R;
	Eigen::Vector3d dP = imupreint._delta_P;
	Eigen::Vector3d dV = imupreint._delta_V;
	double dt = imupreint._delta_time;

	Eigen::Vector3d Pwbpre = imuState.Get_P();
	Eigen::Matrix3d Rwbpre = imuState.Get_RotMatrix();
	Eigen::Vector3d Vwbpre = imuState.Get_V();

	Eigen::Matrix3d Rwb = Rwbpre * dR;
	Eigen::Vector3d Pwb = Pwbpre + Vwbpre*dt + 0.5 * gw * dt * dt + Rwbpre*dP;
	Eigen::Vector3d Vwb = Vwbpre + gw * dt + Rwbpre * dV;

	// Here assume that the pre-integration is re-computed after bias updated, so the bias term is ignored
	imuState.Set_Pos(Pwb);
	imuState.Set_Vel(Vwb);
	imuState.Set_Rot(Rwb);

	// Test log
	if (imuState.Get_dBias_Gyr().norm()>1e-6 || imuState.Get_dBias_Acc().norm()>1e-6) 
		std::cerr << "delta bias in updateNS is not zero" << imuState.Get_dBias_Gyr().transpose() 
				  << ", " << imuState.Get_dBias_Acc().transpose() << std::endl;
}

void Frame::UpdatePoseFromNS(const Eigen::Matrix4d &Tbc)
{
	Eigen::Matrix3d Rbc_ = Tbc.block<3, 3>(0, 0);
	Eigen::Vector3d Pbc_ = Tbc.block<3, 1>(0, 3); 
	Eigen::Matrix3d Rwb_ = imuState.Get_RotMatrix();
	Eigen::Vector3d Pwb_ = imuState.Get_P();

	Eigen::Matrix3d Rcw_ = (Rwb_ * Rbc_).transpose();
	Eigen::Vector3d Pwc_ = Rwb_*Pbc_ + Pwb_;
	Eigen::Vector3d Pcw_ = -Rcw_*Pwc_;

	Eigen::Matrix4d Tcw_ = Eigen::Matrix4d::Identity();
	Tcw_.block<3, 3>(0, 0) = Rcw_;
	Tcw_.block<3, 1>(0, 3) = Pcw_;

	T_f_w_ = Sophus::SE3d(Tcw_);
}

void Frame::setKeyframe()
{
  is_keyframe_ = true;
  kf_counter_++;
  kf_id_ = kf_counter_;
  setKeyPoints();
}

void Frame::addFeature(Feature* ftr)
{
  fts_.push_back(ftr);
}

void Frame::setKeyPoints()
{
  for(size_t i = 0; i < 5; ++i)
    if(key_pts_[i] != NULL)
      if(key_pts_[i]->point == NULL)
        key_pts_[i] = NULL;

  std::for_each(fts_.begin(), fts_.end(), [&](Feature* ftr){ if(ftr->point != NULL) checkKeyPoints(ftr); });
}

void Frame::checkKeyPoints(Feature* ftr)
{
  const int cu = cam_->width()/2;
  const int cv = cam_->height()/2;

  // center pixel
  if(key_pts_[0] == NULL)
    key_pts_[0] = ftr;
  else if(std::max(std::fabs(ftr->px[0]-cu), std::fabs(ftr->px[1]-cv))
        < std::max(std::fabs(key_pts_[0]->px[0]-cu), std::fabs(key_pts_[0]->px[1]-cv)))
    key_pts_[0] = ftr;

  if(ftr->px[0] >= cu && ftr->px[1] >= cv)
  {
    if(key_pts_[1] == NULL)
      key_pts_[1] = ftr;
    else if((ftr->px[0]-cu) * (ftr->px[1]-cv)
          > (key_pts_[1]->px[0]-cu) * (key_pts_[1]->px[1]-cv))
      key_pts_[1] = ftr;
  }
  if(ftr->px[0] >= cu && ftr->px[1] < cv)
  {
    if(key_pts_[2] == NULL)
      key_pts_[2] = ftr;
    else if((ftr->px[0]-cu) * (ftr->px[1]-cv)
          > (key_pts_[2]->px[0]-cu) * (key_pts_[2]->px[1]-cv))
      key_pts_[2] = ftr;
  }
  if(ftr->px[0] < cv && ftr->px[1] < cv)
  {
    if(key_pts_[3] == NULL)
      key_pts_[3] = ftr;
    else if((ftr->px[0]-cu) * (ftr->px[1]-cv)
          > (key_pts_[3]->px[0]-cu) * (key_pts_[3]->px[1]-cv))
      key_pts_[3] = ftr;
  }
  if(ftr->px[0] < cv && ftr->px[1] >= cv)
  {
    if(key_pts_[4] == NULL)
      key_pts_[4] = ftr;
    else if((ftr->px[0]-cu) * (ftr->px[1]-cv)
          > (key_pts_[4]->px[0]-cu) * (key_pts_[4]->px[1]-cv))
      key_pts_[4] = ftr;
  }
}

void Frame::removeKeyPoint(Feature* ftr)
{
  bool found = false;
  std::for_each(key_pts_.begin(), key_pts_.end(), [&](Feature*& i){
    if(i == ftr) {
      i = NULL;
      found = true;
    }
  });
  if(found)
    setKeyPoints();
}

bool Frame::isVisible(const Eigen::Vector3d& xyz_w) const
{
  Eigen::Vector3d xyz_f = T_f_w_*xyz_w;
  if(xyz_f.z() < 0.0)
    return false; // point is behind the camera
  Eigen::Vector2d px = f2c(xyz_f);
  if(px[0] >= 0.0 && px[1] >= 0.0 && px[0] < cam_->width() && px[1] < cam_->height())
    return true;
  return false;
}


/// Utility functions for the Frame class
namespace frame_utils {

void createImgPyramid(const cv::Mat& img_level_0, int n_levels, ImgPyr& pyr)
{
  pyr.resize(n_levels);
  pyr[0] = img_level_0;
  for(int i=1; i<n_levels; ++i)
  {
    pyr[i] = cv::Mat(pyr[i-1].rows/2, pyr[i-1].cols/2, CV_8U);
    vk::halfSample(pyr[i-1], pyr[i]);
  }
}

bool getSceneDepth(const Frame& frame, double& depth_mean, double& depth_min)
{
  std::vector<double> depth_vec;
  depth_vec.reserve(frame.fts_.size());
  depth_min = std::numeric_limits<double>::max();
  for(auto it=frame.fts_.begin(), ite=frame.fts_.end(); it!=ite; ++it)
  {
    if((*it)->point != NULL)
    {
      const double z = frame.w2f((*it)->point->pos_).z();
      depth_vec.push_back(z);
      depth_min = fmin(z, depth_min);
    }
  }
  if(depth_vec.empty())
  {
    SVO_WARN_STREAM("Cannot set scene depth. Frame has no point-observations!");
    return false;
  }
  depth_mean = vk::getMedian(depth_vec);
  return true;
}

} // namespace frame_utils
} // namespace svo
