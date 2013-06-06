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

#include "ccny_rgbd/apps/keyframe_multi_mapper.h"
#include <boost/lexical_cast.hpp>

namespace ccny_rgbd {

KeyframeMultiMapper::KeyframeMultiMapper(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private)
{
  ROS_INFO("Starting RGBD Keyframe Multi Mapper");

  initParams();

  keyframes_pub_ = nh_.advertise<PointCloudT>(
     "keyframes", queue_size_);

  robot_mappers.resize(num_robots);
  map_transforms.resize(num_robots);

  for(int i=0; i<num_robots; i++){
	  map_transforms[i].setIdentity();
	  map_transforms[i].setOrigin(tf::Vector3(20.0*i, 0,0));
	  robot_mappers[i].reset(new KeyframeOnlineMapper(nh, nh_private, "cloudbot" + boost::lexical_cast<std::string>(i+1)));
  }

  boost::thread t1(boost::bind(&KeyframeMultiMapper::publishMapTransforms, this));
  boost::thread t2(boost::bind(&KeyframeMultiMapper::publishPointcloud, this));


}

KeyframeMultiMapper::~KeyframeMultiMapper()
{

}

void KeyframeMultiMapper::initParams()
{

  if (!nh_private_.getParam ("verbose", verbose))
    verbose = false;
  if (!nh_private_.getParam ("queue_size", queue_size_))
	  queue_size_ = 5;
  if (!nh_private_.getParam ("num_robots", num_robots))
	  num_robots = 1;


}

void KeyframeMultiMapper::publishMapTransforms() {

	printf("Initialized map to odom transform sender\n");

	while(true){
		for (int i = 0; i < num_robots; i++) {
			tf::StampedTransform transform_msg(map_transforms[i], ros::Time::now(),
					"/world",
					"/cloudbot" + boost::lexical_cast<std::string>(i + 1)
							+ "/map");
			tf_broadcaster_.sendTransform(transform_msg);
			usleep(33333/num_robots);
		}
	}

}

void KeyframeMultiMapper::publishPointcloud() {

	while (true) {

		PointCloudT::Ptr map(new PointCloudT), filtered_map(new PointCloudT);

		float voxel_size = 0.02f;

		for (int j = 0; j < num_robots; j++)
			for (int i = 0; i < robot_mappers[j]->keyframes_.size(); i++) {
				rgbdtools::RGBDKeyframe& keyframe = robot_mappers[j]->keyframes_[i];

				// construct a cloud from the images
				PointCloudT cloud;
				keyframe.constructDensePointCloud(cloud, 5.5,
						0.03);

				// cloud transformed to the fixed frame

				PointCloudT::Ptr cloud_ff(new PointCloudT), cloud_ff_sub(
						new PointCloudT);

				pcl::transformPointCloud(cloud, *cloud_ff, eigenAffineFromTf(map_transforms[j]) * keyframe.pose);

				pcl::VoxelGrid<PointT> sor1;
				sor1.setInputCloud(cloud_ff);
				sor1.setLeafSize(voxel_size, voxel_size, voxel_size);
				sor1.filter(*cloud_ff_sub);

				cloud_ff_sub->header.frame_id = "/world";

				*map += *cloud_ff;

			}

		// Create the filtering object
		pcl::VoxelGrid<PointT> sor;
		sor.setInputCloud(map);
		sor.setLeafSize(voxel_size, voxel_size, voxel_size);
		sor.filter(*filtered_map);
		filtered_map->header.frame_id = "/world";

		keyframes_pub_.publish(filtered_map);

		printf("Initialized map to odom transform sender\n");
	}

}


} // namespace ccny_rgbd
