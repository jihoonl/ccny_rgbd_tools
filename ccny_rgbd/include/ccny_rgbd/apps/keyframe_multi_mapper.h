/**
 *  @file keyframe_mapper.h
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

#ifndef CCNY_RGBD_KEYFRAME_MULTI_MAPPER_H
#define CCNY_RGBD_KEYFRAME_MULTI_MAPPER_H

#include "ccny_rgbd/apps/keyframe_online_mapper.h"

namespace ccny_rgbd {


class KeyframeMultiMapper
{
  public:

    /** @brief Constructor from ROS nodehandles
     * @param nh the public nodehandle
     * @param nh_private the private nodehandle
     */  
	KeyframeMultiMapper(const ros::NodeHandle& nh,
                   const ros::NodeHandle& nh_private);
    
    /** @brief Default destructor
     */
    virtual ~KeyframeMultiMapper();

    /** @brief Initializes all the parameters from the ROS param server
     */
    void initParams();

    void publishMapTransforms();
    void publishPointcloud();
    
  protected:

    ros::NodeHandle nh_;          ///< public nodehandle
    ros::NodeHandle nh_private_;  ///< private nodepcdhandle
    
    ros::Publisher keyframes_pub_;
    tf::TransformBroadcaster tf_broadcaster_;

  private:

    int queue_size_;
    bool verbose;
    int num_robots;
    
    tbb::concurrent_vector<KeyframeOnlineMapper::Ptr> robot_mappers;
    tbb::concurrent_vector<tf::Transform> map_transforms;


};

} // namespace ccny_rgbd

#endif // CCNY_RGBD_KEYFRAME_MAPPER_H
