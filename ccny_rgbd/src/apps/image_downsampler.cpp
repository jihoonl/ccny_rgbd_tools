/**
 *  @file keyframe_mapper.cpp
 *  @author Ivan Dryanovski <ivan.dryanovski@gmail.com>
 * 
 *  @section LICENSE
 * 
 *  Copyright (C) 2013, City University of New York
 *  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ccny_rgbd/apps/image_downsampler.h"

namespace ccny_rgbd {

ImageDownsampler::ImageDownsampler(
  const ros::NodeHandle& nh, 
  const ros::NodeHandle& nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  rgb_it(nh_),
  depth_it(nh_)
{
  ROS_INFO("Starting RGBD Keyframe Sender");
   
  // **** params
  
  initParams();
  
  // **** publishers

  pub_rgb_ = rgb_it.advertise(
      "/rgb/image_downsampled", queue_size_);
  pub_depth_ = depth_it.advertise(
      "/depth/image_downsampled", queue_size_);
  pub_info_  = nh_.advertise<CameraInfoMsg>(
      "/rgb/camera_info_downsampled", queue_size_);

  
  // **** subscribers

  sub_rgb_.subscribe(rgb_it,     "/rgb/image_raw",   queue_size_);
  sub_depth_.subscribe(depth_it, "/depth/image_raw", queue_size_);
  sub_info_.subscribe(nh_,       "/rgb/camera_info",  queue_size_);

  // Synchronize inputs.
  sync_.reset(new RGBDSynchronizer3(
                RGBDSyncPolicy3(queue_size_), sub_rgb_, sub_depth_, sub_info_));
   
  sync_->registerCallback(boost::bind(&ImageDownsampler::RGBDCallback, this, _1, _2, _3));
}

ImageDownsampler::~ImageDownsampler()
{

}

void ImageDownsampler::initParams()
{
  bool verbose;
  
  if (!nh_private_.getParam ("verbose", verbose))
    verbose = false;
  if (!nh_private_.getParam ("queue_size", queue_size_))
    queue_size_ = 5;
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("kf_dist_eps", kf_dist_eps_))
    kf_dist_eps_  = 0.10;
  if (!nh_private_.getParam ("kf_angle_eps", kf_angle_eps_))
    kf_angle_eps_  = 10.0 * M_PI / 180.0;
}
  
void ImageDownsampler::RGBDCallback(
  const ImageMsg::ConstPtr& rgb_msg,
  const ImageMsg::ConstPtr& depth_msg,
  const CameraInfoMsg::ConstPtr& info_msg)
{

	cv_bridge::CvImageConstPtr gray_image = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::MONO8);
	cv_bridge::CvImageConstPtr depth_image = cv_bridge::toCvShare(depth_msg);

	cv_bridge::CvImagePtr gray_image_downsampled(new cv_bridge::CvImage);
	cv_bridge::CvImagePtr depth_image_downsampled(new cv_bridge::CvImage);

	cv::resize(gray_image->image, gray_image_downsampled->image, cv::Size(0,0), 0.5, 0.5, CV_INTER_AREA);
	cv::resize(depth_image->image, depth_image_downsampled->image, cv::Size(0,0), 0.5, 0.5, CV_INTER_NN);

	gray_image_downsampled->header = gray_image->header;
	depth_image_downsampled->header = depth_image->header;

	gray_image_downsampled->encoding = gray_image->encoding;
	depth_image_downsampled->encoding = depth_image->encoding;

	CameraInfoMsg::Ptr info_downscaled(new CameraInfoMsg);
	*info_downscaled = *info_msg;

	for(int i=0; i<6; i++){
		info_downscaled->K[i] /= 2;
	}

	for(int i=0; i<8; i++){
		info_downscaled->P[i] /= 2;
	}

	info_downscaled->height /= 2;
	info_downscaled->width /= 2;

	pub_rgb_.publish(gray_image_downsampled->toImageMsg());
	pub_depth_.publish(depth_image_downsampled->toImageMsg());
	pub_info_.publish(info_downscaled);


}


} // namespace ccny_rgbd
