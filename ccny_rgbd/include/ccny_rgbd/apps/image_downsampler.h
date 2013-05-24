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

#ifndef CCNY_RGBD_IMAGE_DOWNSAMPLER_H
#define CCNY_RGBD_IMAGE_DOWNSAMPLER_H

#include <ros/ros.h>
#include <ros/publisher.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
#include <rgbdtools/rgbdtools.h>

#include "ccny_rgbd/types.h"
#include "ccny_rgbd/util.h"


namespace ccny_rgbd {

/** @brief Sends a series of RGBD keyframes.
 * 
 */    
class ImageDownsampler
{
  public:

    /** @brief Constructor from ROS nodehandles
     * @param nh the public nodehandle
     * @param nh_private the private nodehandle
     */  
	ImageDownsampler(const ros::NodeHandle& nh,
                   const ros::NodeHandle& nh_private);
    
    /** @brief Default destructor
     */
    virtual ~ImageDownsampler();

    /** @brief Initializes all the parameters from the ROS param server
     */
    void initParams();
    
  protected:

    ros::NodeHandle nh_;          ///< public nodehandle
    ros::NodeHandle nh_private_;  ///< private nodepcdhandle
    
    std::string fixed_frame_;     ///< the fixed frame (usually "odom")
    
    int queue_size_;  ///< Subscription queue size
    
    tf::StampedTransform previous_transform; ///< transform from previous RGBD Keyframe to base frame
    
    /** @brief Main callback for RGB, Depth, and CameraInfo messages
     * 
     * @param depth_msg Depth message (16UC1, in mm)
     * @param rgb_msg RGB message (8UC2, YUV422)
     * @param info_msg CameraInfo message, applies to both RGB and depth images
     */
    virtual void RGBDCallback(const ImageMsg::ConstPtr& rgb_msg,
                              const ImageMsg::ConstPtr& depth_msg,
                              const CameraInfoMsg::ConstPtr& info_msg);

  private:

    ImageTransport rgb_it;
    ImageTransport depth_it;


    ImagePublisher pub_rgb_;
    ImagePublisher pub_depth_;
    ros::Publisher pub_info_;
    


    tf::TransformListener tf_listener_; ///< ROS transform listener

    /** @brief Image transport for RGB message subscription */
    boost::shared_ptr<ImageTransport> rgb_it_;
    
    /** @brief Image transport for depth message subscription */
    boost::shared_ptr<ImageTransport> depth_it_;
    
    /** @brief Callback syncronizer */
    boost::shared_ptr<RGBDSynchronizer3> sync_;
          
    /** @brief RGB message subscriber */
    ImageSubFilter      sub_rgb_;
    
    /** @brief Depth message subscriber */
    ImageSubFilter      sub_depth_;  
   
    /** @brief Camera info message subscriber */
    CameraInfoSubFilter sub_info_;
    
    // params
    double kf_dist_eps_;  ///< linear distance threshold between keyframes
    double kf_angle_eps_; ///< angular distance threshold between keyframes

          
    
};

} // namespace ccny_rgbd

#endif // CCNY_RGBD_IMAGE_DOWNSAMPLER_H
