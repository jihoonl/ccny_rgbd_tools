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

#ifndef CCNY_RGBD_KEYFRAME_ONLINE_MAPPER_H
#define CCNY_RGBD_KEYFRAME_ONLINE_MAPPER_H

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <boost/regex.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <rgbdtools/rgbdtools.h>
#include <tbb/concurrent_vector.h>

#include "ccny_rgbd/types.h"
#include "ccny_rgbd/util.h"
#include "ccny_rgbd/GenerateGraph.h"
#include "ccny_rgbd/SolveGraph.h"
#include "ccny_rgbd/AddManualKeyframe.h"
#include "ccny_rgbd/PublishKeyframe.h"
#include "ccny_rgbd/PublishKeyframes.h"
#include "ccny_rgbd/Save.h"
#include "ccny_rgbd/Load.h"

namespace ccny_rgbd {


class KeyframeOnlineMapper
{
  public:

	typedef boost::shared_ptr<KeyframeOnlineMapper> Ptr;

    /** @brief Constructor from ROS nodehandles
     * @param nh the public nodehandle
     * @param nh_private the private nodehandle
     */  
	KeyframeOnlineMapper(const ros::NodeHandle& nh,
                   const ros::NodeHandle& nh_private, std::string prefix = std::string());
    
    /** @brief Default destructor
     */
    virtual ~KeyframeOnlineMapper();

    /** @brief Initializes all the parameters from the ROS param server
     */
    void initParams();


    //rgbdtools::KeyframeVector keyframes_;    ///< vector of RGBD Keyframes
    tbb::concurrent_vector<rgbdtools::RGBDKeyframe> keyframes_;
    tbb::concurrent_vector<rgbdtools::KeyframeAssociation> associations_;

  protected:

    ros::NodeHandle nh_;          ///< public nodehandle
    ros::NodeHandle nh_private_;  ///< private nodepcdhandle
    
    std::string prefix_;

    std::string fixed_frame_;     ///< the fixed frame (usually "map")
    std::string odom_frame_;     ///< the fixed frame (usually "odom")
    
    int queue_size_;  ///< Subscription queue size
    
    double max_range_;  ///< Maximum threshold for  range (in the z-coordinate of the camera frame)
    double max_stdev_;  ///< Maximum threshold for range (z-coordinate) standard deviation


    

    /** @brief Main callback for RGB, Depth, and CameraInfo messages
     * 
     * @param depth_msg Depth message (16UC1, in mm)
     * @param rgb_msg RGB message (8UC3)
     * @param info_msg CameraInfo message, applies to both RGB and depth images
     */
    virtual void RGBDCallback(const ImageMsg::ConstPtr& rgb_msg,
                              const ImageMsg::ConstPtr& depth_msg,
                              const CameraInfoMsg::ConstPtr& info_msg);

  private:

    ros::Publisher keyframes_pub_;    ///< ROS publisher for the keyframe point clouds
    

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
    double pcd_map_res_; ///< downsampling resolution of pcd map (in meters)
    double octomap_res_;  ///< tree resolution for octomap (in meters)
    double kf_dist_eps_;  ///< linear distance threshold between keyframes
    double kf_angle_eps_; ///< angular distance threshold between keyframes
    bool octomap_with_color_; ///< whetehr to save Octomaps with color info      
    double max_map_z_;   ///< maximum z (in fixed frame) when exporting maps.

    bool verbose;
    int graph_n_keypoints;
    int graph_n_candidates;
    int graph_k_nearest_neighbors;


    int ransac_max_iterations_;
	int sac_min_inliers_;
	bool matcher_use_desc_ratio_test_;
	double matcher_max_desc_ratio_;
	double matcher_max_desc_dist_;
	double sac_max_eucl_dist_sq_;
	bool sac_reestimate_tf_;
	double ransac_sufficient_inlier_ratio_;
	double ransac_confidence_;
	double log_one_minus_ransac_confidence_;
	bool sac_save_results_;
	std::string output_path_;


          
    // state vars
    bool manual_add_;   ///< flag indicating whetehr a manual add has been requested

    int rgbd_frame_index_;

    rgbdtools::KeyframeGraphDetector graph_detector_;  ///< builds graph from the keyframes
    rgbdtools::KeyframeGraphSolverG2O graph_solver_;    ///< optimizes the graph for global alignement


    //rgbdtools::KeyframeAssociationVector associations_; ///< keyframe associations that form the graph
    
    PathMsg path_msg_;    /// < contains a vector of positions of the camera (not base) pose
    

    void getCandidateMatches(
      const rgbdtools::RGBDKeyframe& frame_q, const rgbdtools::RGBDKeyframe& frame_t,
      rgbdtools::DMatchVector& candidate_matches);

    int pairwiseMatchingRANSAC(
      const rgbdtools::RGBDKeyframe& frame_t,
      const rgbdtools::RGBDKeyframe& frame_q,
      rgbdtools::DMatchVector& best_inlier_matches,
      Eigen::Matrix4f& best_transformation);


    void optimizationLoop();


    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;
    g2o::BlockSolverX * solver_ptr;


    void addVertex(
      const AffineTransform& vertex_pose,
      int vertex_idx);

    void addEdge(
      int from_idx,
      int to_idx,
      const AffineTransform& relative_pose,
      const Eigen::Matrix<double,6,6>& information_matrix);

    void optimizeGraph();

    void getOptimizedPoses(AffineTransformVector& poses);
    void publishMapTransform();

    tf::Transform map_to_odom;
    tf::TransformBroadcaster tf_broadcaster_;



    /** @brief processes an incoming RGBD frame with a given pose,
     * and determines whether a keyframe should be inserted
     * @param frame the incoming RGBD frame (image)
     * @param pose the pose of the base frame when RGBD image was taken
     * @retval true a keyframe was inserted
     * @retval false no keyframe was inserted
     */
    bool processFrame(const rgbdtools::RGBDFrame& frame, const tf::StampedTransform& pose);

};

} // namespace ccny_rgbd

#endif // CCNY_RGBD_KEYFRAME_ONLINE_MAPPER_H
