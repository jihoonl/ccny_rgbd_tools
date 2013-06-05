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

#include "ccny_rgbd/apps/keyframe_sender.h"

namespace ccny_rgbd {

KeyframeSender::KeyframeSender(
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
      "keyframes/rgb", queue_size_);
  pub_depth_ = depth_it.advertise(
      "keyframes/depth", queue_size_);
  pub_info_  = nh_.advertise<CameraInfoMsg>(
      "keyframes/info", queue_size_);

  
  // **** subscribers

  sub_rgb_.subscribe(rgb_it,     "/rgbd/rgb",   queue_size_);
  sub_depth_.subscribe(depth_it, "/rgbd/depth", queue_size_);
  sub_info_.subscribe(nh_,       "/rgbd/info",  queue_size_);

  // Synchronize inputs.
  sync_.reset(new RGBDSynchronizer3(
                RGBDSyncPolicy3(queue_size_), sub_rgb_, sub_depth_, sub_info_));
   
  sync_->registerCallback(boost::bind(&KeyframeSender::RGBDCallback, this, _1, _2, _3));
}

KeyframeSender::~KeyframeSender()
{

}

void KeyframeSender::initParams()
{
  bool verbose;
  
  if (!nh_private_.getParam ("verbose", verbose))
    verbose = false;
  if (!nh_private_.getParam ("queue_size", queue_size_))
    queue_size_ = 5;
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "odom";
  if (!nh_private_.getParam ("kf_dist_eps", kf_dist_eps_))
    kf_dist_eps_  = 0.10;
  if (!nh_private_.getParam ("kf_angle_eps", kf_angle_eps_))
    kf_angle_eps_  = 10.0 * M_PI / 180.0;

  std::string tf_prefix_ = tf::getPrefixParam(nh_private_);
  fixed_frame_ = tf::resolve(tf_prefix_, fixed_frame_);

}
  
void KeyframeSender::RGBDCallback(
  const ImageMsg::ConstPtr& rgb_msg,
  const ImageMsg::ConstPtr& depth_msg,
  const CameraInfoMsg::ConstPtr& info_msg)
{
  tf::StampedTransform transform;

  const ros::Time& time = rgb_msg->header.stamp;
  double dist, angle;

  try{
    tf_listener_.waitForTransform(
     fixed_frame_, rgb_msg->header.frame_id, time, ros::Duration(0.1));
    tf_listener_.lookupTransform(
      fixed_frame_, rgb_msg->header.frame_id, time, transform);  
  }
  catch(...)
  {
    return;
  }
  

  getTfDifference(transform, previous_transform, dist, angle);

  if (dist > kf_dist_eps_ || angle > kf_angle_eps_) {
	  previous_transform = transform;

	  cv_bridge::CvImagePtr bgr_image = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);

	  pub_rgb_.publish(bgr_image->toImageMsg());
	  pub_depth_.publish(depth_msg);
	  pub_info_.publish(info_msg);


  }


}


} // namespace ccny_rgbd
