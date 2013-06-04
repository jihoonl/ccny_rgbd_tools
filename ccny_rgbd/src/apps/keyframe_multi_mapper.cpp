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

namespace ccny_rgbd {

KeyframeMultiMapper::KeyframeMultiMapper(
  const ros::NodeHandle& nh, 
  const ros::NodeHandle& nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  rgbd_frame_index_(0)
{
  ROS_INFO("Starting RGBD Keyframe Multi Mapper");
   
  // **** params
  
  initParams();
  
  map_to_odom.setIdentity();

  optimizer.setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
  optimizer.setVerbose(true);

  linearSolver = new g2o::LinearSolverCholmod<
			g2o::BlockSolverX::PoseMatrixType>();

  solver_ptr = new g2o::BlockSolverX(&optimizer, linearSolver);
  optimizer.setSolver(solver_ptr);


  // **** publishers
  
  keyframes_pub_ = nh_.advertise<PointCloudT>(
    "keyframes", queue_size_);
  poses_pub_ = nh_.advertise<visualization_msgs::Marker>( 
    "keyframe_poses", queue_size_);
  kf_assoc_pub_ = nh_.advertise<visualization_msgs::Marker>( 
    "keyframe_associations", queue_size_);
  path_pub_ = nh_.advertise<PathMsg>( 
    "mapper_path", queue_size_);
  
  // **** services
  
  pub_keyframe_service_ = nh_.advertiseService(
    "publish_keyframe", &KeyframeMultiMapper::publishKeyframeSrvCallback, this);
  pub_keyframes_service_ = nh_.advertiseService(
    "publish_keyframes", &KeyframeMultiMapper::publishKeyframesSrvCallback, this);
  save_pcd_map_service_ = nh_.advertiseService(
    "save_pcd_map", &KeyframeMultiMapper::savePcdMapSrvCallback, this);
  save_octomap_service_ = nh_.advertiseService(
    "save_octomap", &KeyframeMultiMapper::saveOctomapSrvCallback, this);
  add_manual_keyframe_service_ = nh_.advertiseService(
    "add_manual_keyframe", &KeyframeMultiMapper::addManualKeyframeSrvCallback, this);
   solve_graph_service_ = nh_.advertiseService(
    "solve_graph", &KeyframeMultiMapper::solveGraphSrvCallback, this);
 
  // **** subscribers

  ImageTransport rgb_it(nh_);
  ImageTransport depth_it(nh_);

  image_transport::TransportHints rgb_th("compressed");
  image_transport::TransportHints depth_th("compressedDepth");


  sub_rgb_.subscribe(rgb_it,     "/keyframes/rgb",   queue_size_, rgb_th);
  sub_depth_.subscribe(depth_it, "/keyframes/depth", queue_size_, depth_th);
  sub_info_.subscribe(nh_,       "/keyframes/info",  queue_size_);

  // Synchronize inputs.
  sync_.reset(new RGBDSynchronizer3(
                RGBDSyncPolicy3(queue_size_), sub_rgb_, sub_depth_, sub_info_));
   
  sync_->registerCallback(boost::bind(&KeyframeMultiMapper::RGBDCallback, this, _1, _2, _3));

  boost::thread t(boost::bind(&KeyframeMultiMapper::optimizationLoop, this));
  boost::thread t1(boost::bind(&KeyframeMultiMapper::publishMapTransform, this));

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
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/map";
  if (!nh_private_.getParam ("pcd_map_res", pcd_map_res_))
    pcd_map_res_ = 0.01;
  if (!nh_private_.getParam ("octomap_res", octomap_res_))
    octomap_res_ = 0.05;
  if (!nh_private_.getParam ("octomap_with_color", octomap_with_color_))
   octomap_with_color_ = true;
  if (!nh_private_.getParam ("kf_dist_eps", kf_dist_eps_))
    kf_dist_eps_  = 0.10;
  if (!nh_private_.getParam ("kf_angle_eps", kf_angle_eps_))
    kf_angle_eps_  = 10.0 * M_PI / 180.0;
  if (!nh_private_.getParam ("max_range", max_range_))
    max_range_  = 5.5;
  if (!nh_private_.getParam ("max_stdev", max_stdev_))
    max_stdev_  = 0.03;
  if (!nh_private_.getParam ("max_map_z", max_map_z_))
    max_map_z_ = std::numeric_limits<double>::infinity();
   
  // configure graph detection 
  bool graph_matcher_use_desc_ratio_test = true;
    
  if (!nh_private_.getParam ("graph/n_keypoints", graph_n_keypoints))
    graph_n_keypoints = 500;
  if (!nh_private_.getParam ("graph/n_candidates", graph_n_candidates))
    graph_n_candidates = 15;
  if (!nh_private_.getParam ("graph/k_nearest_neighbors", graph_k_nearest_neighbors))
    graph_k_nearest_neighbors = 4;
  
  graph_detector_.setNKeypoints(graph_n_keypoints);
  graph_detector_.setNCandidates(graph_n_candidates);   
  graph_detector_.setKNearestNeighbors(graph_k_nearest_neighbors);    
  graph_detector_.setMatcherUseDescRatioTest(graph_matcher_use_desc_ratio_test);
  
  graph_detector_.setSACReestimateTf(false);
  graph_detector_.setSACSaveResults(false);
  graph_detector_.setVerbose(verbose);





   matcher_use_desc_ratio_test_ = true;
   matcher_max_desc_ratio_ = 0.75;  // when ratio_test = true
   matcher_max_desc_dist_ = 0.5;    // when ratio_test = false

   // common SAC params
   sac_max_eucl_dist_sq_ = 0.03 * 0.03;
   sac_min_inliers_ = 20;  // this or more are needed
   sac_reestimate_tf_ = false;

   // RANSAC params
   ransac_confidence_ = 0.99;
   ransac_max_iterations_ = 1000;
   ransac_sufficient_inlier_ratio_ = 0.75;

   // derived parameters
   log_one_minus_ransac_confidence_ = log(1.0 - ransac_confidence_);

   sac_save_results_ = true;
   output_path_ = "/home/vsu/mapping_debug";

}
  
void KeyframeMultiMapper::RGBDCallback(
  const ImageMsg::ConstPtr& rgb_msg,
  const ImageMsg::ConstPtr& depth_msg,
  const CameraInfoMsg::ConstPtr& info_msg)
{
  tf::StampedTransform transform;

  const ros::Time& time = rgb_msg->header.stamp;

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
  
  // create a new frame and increment the counter
  rgbdtools::RGBDFrame frame;
  createRGBDFrameFromROSMessages(rgb_msg, depth_msg, info_msg, frame); 
  frame.index = rgbd_frame_index_;
  rgbd_frame_index_++;
  

  bool result = processFrame(frame, transform);
  //if (result) publishKeyframeData(keyframes_.size() - 1);

  //publishPath();
}



bool KeyframeMultiMapper::processFrame(
  const rgbdtools::RGBDFrame& frame,
  const tf::StampedTransform & pose)
{
  // add the frame pose to the path vector
  geometry_msgs::PoseStamped frame_pose; 
  tf::Transform frame_tf = pose;
  tf::poseTFToMsg(frame_tf, frame_pose.pose);
 
  // update the header of the pose for the path
  frame_pose.header.frame_id = fixed_frame_;
  frame_pose.header.seq = frame.header.seq;
  frame_pose.header.stamp.sec = frame.header.stamp.sec;
  frame_pose.header.stamp.nsec = frame.header.stamp.nsec;
    
  path_msg_.poses.push_back(frame_pose);

  rgbdtools::RGBDKeyframe keyframe(frame);
  keyframe.pose = eigenAffineFromTf(pose);

  cv::SurfDescriptorExtractor extractor;

  double surf_threshold = 400;
  double min_surf_threshold = 25;
  bool upright = true;

  while (surf_threshold >= min_surf_threshold) {
		cv::SurfFeatureDetector detector(surf_threshold, 4, 2, true, upright);
		keyframe.keypoints.clear();
		detector.detect(keyframe.rgb_img, keyframe.keypoints);



		if ((int) keyframe.keypoints.size() < graph_n_keypoints) {
			if (verbose)
				printf(
						"[KF %d] %d SURF keypoints detected (threshold: %.1f)\n",
						(int) keyframes_.size() + 1,
						(int) keyframe.keypoints.size(), surf_threshold);

			surf_threshold /= 2.0;
		} else {
			keyframe.keypoints.resize(graph_n_keypoints);

			if (verbose)
				printf(
						"[KF %d] %d SURF keypoints detected (threshold: %.1f)\n",
						(int) keyframes_.size() + 1,
						(int) keyframe.keypoints.size(), surf_threshold);

			break;
		}

	}


  if (sac_save_results_)
  {
	cv::Mat kp_img;
	cv::drawKeypoints(keyframe.rgb_img, keyframe.keypoints, kp_img);
	std::stringstream ss1;
	ss1 << "kp_" << keyframe.header.seq;
	cv::imwrite(output_path_ + "/" + ss1.str() + ".png", kp_img);
  }


  extractor.compute(keyframe.rgb_img, keyframe.keypoints, keyframe.descriptors);
  keyframe.computeDistributions();


  // build matcher
  cv::Ptr<cv::flann::IndexParams> indexParams;
  indexParams = new cv::flann::KDTreeIndexParams();

  cv::Ptr<cv::flann::SearchParams> searchParams = new cv::flann::SearchParams(32);
  keyframe.matcher.reset(new cv::FlannBasedMatcher(indexParams, searchParams));


  // train
  std::vector<cv::Mat> descriptors_vector;
  descriptors_vector.push_back(keyframe.descriptors);
  keyframe.matcher->add(descriptors_vector);
  keyframe.matcher->train();

  tbb::concurrent_vector<rgbdtools::RGBDKeyframe>::iterator current_keyframe_it, it;

  current_keyframe_it = keyframes_.push_back(keyframe);

  for(it = keyframes_.begin(); it != current_keyframe_it; it++){

	  std::vector<cv::DMatch> inlier_matches;

	  // perform ransac matching, b onto a
	  Eigen::Matrix4f transformation;

	  int iterations = pairwiseMatchingRANSAC(
	        *it, *current_keyframe_it, inlier_matches, transformation);


	  if (inlier_matches.size() >= sac_min_inliers_) {

			// add an association
			rgbdtools::KeyframeAssociation association;
			association.type = rgbdtools::KeyframeAssociation::RANSAC;
			association.it_a = current_keyframe_it;
			association.it_b = it;
			association.matches = inlier_matches;
			association.a2b = transformation;
			associations_.push_back(association);


			ROS_INFO("Inliers size %d, K1 %d, K2 %d", inlier_matches.size(), it->keypoints.size(), current_keyframe_it->keypoints.size());

			// save the results to file
			if (sac_save_results_) {
				cv::Mat img_matches;
				cv::drawMatches(it->rgb_img, it->keypoints,
						current_keyframe_it->rgb_img, current_keyframe_it->keypoints,
						inlier_matches, img_matches);

				std::stringstream ss1;
				ss1 << it->header.seq << "_to_" << current_keyframe_it->header.seq;
				cv::imwrite(output_path_ + "/" + ss1.str() + ".png",
						img_matches);
			}

		}

  }


  std::cerr << "Finnished processing frame " << keyframes_.size() << " " << associations_.size() << std::endl;

  return true;
}



void KeyframeMultiMapper::optimizationLoop() {

	printf("Initialized Optimization loop\n");

	int last_processed_keyframe = 0;
	int last_processed_association = 0;

	int update_on_n_new_keyframes = 5;

	while (true) {

		int current_keyframes_size = keyframes_.size();
		int current_associations_size = associations_.size();

		if ((current_keyframes_size - last_processed_keyframe) < update_on_n_new_keyframes) {
		//`if(current_keyframes_size < 5 || current_associations_size < 5) {
			sleep(1);
			continue;
		} else {
			last_processed_keyframe = current_keyframes_size;
			last_processed_association = current_associations_size;
		}


		for (int i=0; i < current_keyframes_size; i++) {
			rgbdtools::RGBDKeyframe& keyframe =
					keyframes_[i];
			keyframe.index = i;
			addVertex(keyframe.pose, i);
		}

		rgbdtools::InformationMatrix ransac_inf =
				rgbdtools::InformationMatrix::Identity();

		for (int i=0; i < current_associations_size; i++) {

			const rgbdtools::KeyframeAssociation& association =
					associations_[i];

			// skip non-ransac associations
			if (association.type != rgbdtools::KeyframeAssociation::RANSAC)
				continue;

			// calculate the information matrix
			int n_matches = association.matches.size();
			rgbdtools::InformationMatrix inf = ransac_inf;

			// add the edge
			addEdge(association.it_a->index, association.it_b->index,
					association.a2b, inf);
		}


		// run the optimization
		printf("Optimizing...\n");
		optimizeGraph();

		// update the keyframe poses
		printf("Updating keyframe poses...\n");

		rgbdtools::Pose pose_before_optimization = keyframes_[current_keyframes_size-1].pose;

		AffineTransformVector optimized_poses;
		optimized_poses.resize(current_keyframes_size);
		getOptimizedPoses(optimized_poses);

		for (int kf_idx = 0; kf_idx < current_keyframes_size; ++kf_idx) {
			rgbdtools::RGBDKeyframe& keyframe = keyframes_[kf_idx];
			keyframe.pose = optimized_poses[kf_idx];
		}

		rgbdtools::Pose pose_after_optimization = keyframes_[current_keyframes_size-1].pose;

		//map_to_odom = tfFromEigenAffine(pose_after_optimization.inverse() * pose_before_optimization);

		PointCloudT::Ptr map(new PointCloudT), filtered_map(new PointCloudT);

		float voxel_size = 0.025f;

		for (int i = 0; i < current_keyframes_size; i++) {
			rgbdtools::RGBDKeyframe& keyframe = keyframes_[i];

			// construct a cloud from the images
			PointCloudT cloud;
			keyframe.constructDensePointCloud(cloud, max_range_, max_stdev_);

			// cloud transformed to the fixed frame

			PointCloudT::Ptr cloud_ff(new PointCloudT), cloud_ff_sub(new PointCloudT);

			pcl::transformPointCloud(cloud, *cloud_ff, keyframe.pose);

			pcl::VoxelGrid<PointT> sor1;
			sor1.setInputCloud(cloud_ff);
			sor1.setLeafSize(voxel_size, voxel_size, voxel_size);
			sor1.filter(*cloud_ff_sub);


			cloud_ff_sub->header.frame_id = fixed_frame_;

			*map += *cloud_ff;

		}

		// Create the filtering object
		pcl::VoxelGrid<PointT> sor;
		sor.setInputCloud(map);
		sor.setLeafSize(voxel_size, voxel_size, voxel_size);
		sor.filter(*filtered_map);
		filtered_map->header.frame_id = fixed_frame_;

		keyframes_pub_.publish(filtered_map);

	}
}

void KeyframeMultiMapper::publishMapTransform() {

	printf("Initialized map to odom transform sender\n");

	while(true){
		tf::StampedTransform transform_msg(
		  map_to_odom, ros::Time::now(), fixed_frame_, "/odom");
		  tf_broadcaster_.sendTransform (transform_msg);

		  usleep(33333);
	}

}

bool KeyframeMultiMapper::publishKeyframeSrvCallback(
  PublishKeyframe::Request& request,
  PublishKeyframe::Response& response)
{
  int kf_idx = request.id;
  
  if (kf_idx >= 0 && kf_idx < (int)keyframes_.size())
  {
    ROS_INFO("Publishing keyframe %d", kf_idx);
    publishKeyframeData(kf_idx);
    publishKeyframePose(kf_idx);
    return true;
  }
  else
  {
    ROS_ERROR("Index out of range");
    return false;  
  }
}

bool KeyframeMultiMapper::publishKeyframesSrvCallback(
  PublishKeyframes::Request& request,
  PublishKeyframes::Response& response)
{ 
  bool found_match = false;

  // regex matching - try match the request string against each
  // keyframe index
  boost::regex expression(request.re);
  
  for (unsigned int kf_idx = 0; kf_idx < keyframes_.size(); ++kf_idx)
  {
    std::stringstream ss;
    ss << kf_idx;
    std::string kf_idx_string = ss.str();
      
    boost::smatch match;
    
    if(boost::regex_match(kf_idx_string, match, expression))
    {
      found_match = true;
      ROS_INFO("Publishing keyframe %d", kf_idx);
      publishKeyframeData(kf_idx);
      publishKeyframePose(kf_idx);
      usleep(25000);
    }
  }

  publishPath();

  return found_match;
}

void KeyframeMultiMapper::publishKeyframeData(int i)
{
  rgbdtools::RGBDKeyframe& keyframe = keyframes_[i];

  // construct a cloud from the images
  PointCloudT cloud;
  keyframe.constructDensePointCloud(cloud, max_range_, max_stdev_);
  
  // cloud transformed to the fixed frame
  PointCloudT cloud_ff; 
  pcl::transformPointCloud(cloud, cloud_ff, keyframe.pose);

  cloud_ff.header.frame_id = fixed_frame_;

  keyframes_pub_.publish(cloud_ff);
}


void KeyframeMultiMapper::publishKeyframePoses()
{
  for(unsigned int kf_idx = 0; kf_idx < keyframes_.size(); ++kf_idx)
  {
    publishKeyframePose(kf_idx);
  }
}

void KeyframeMultiMapper::publishKeyframePose(int i)
{
  rgbdtools::RGBDKeyframe& keyframe = keyframes_[i];

  // **** publish camera pose

  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = fixed_frame_;
  marker.ns = "keyframe_poses";
  marker.id = i;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.points.resize(2);

  // start point for the arrow
  tf::Transform keyframe_pose = tfFromEigenAffine(keyframe.pose);
  marker.points[0].x = keyframe_pose.getOrigin().getX();
  marker.points[0].y = keyframe_pose.getOrigin().getY();
  marker.points[0].z = keyframe_pose.getOrigin().getZ();

  // end point for the arrow
  tf::Transform ep; 
  ep.setIdentity();
  ep.setOrigin(tf::Vector3(0.00, 0.00, 0.12)); // z = arrow length
  ep = keyframe_pose * ep;

  marker.points[1].x = ep.getOrigin().getX();
  marker.points[1].y = ep.getOrigin().getY();
  marker.points[1].z = ep.getOrigin().getZ(); 
  
  marker.scale.x = 0.02; // shaft radius
  marker.scale.y = 0.05; // head radius

  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  poses_pub_.publish(marker);

  // **** publish frame index text

  visualization_msgs::Marker marker_text;
  marker_text.header.stamp = ros::Time::now();
  marker_text.header.frame_id = fixed_frame_;
  marker_text.ns = "keyframe_indexes";
  marker_text.id = i;
  marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker_text.action = visualization_msgs::Marker::ADD;

  tf::poseTFToMsg(keyframe_pose, marker_text.pose);

  marker_text.pose.position.z -= 0.05;

  char label[6];
  sprintf(label, "%d", i);
  marker_text.text = label;

  marker_text.color.a = 1.0;
  marker_text.color.r = 1.0;
  marker_text.color.g = 1.0;
  marker_text.color.b = 0.0;

  marker_text.scale.z = 0.05; // shaft radius

  poses_pub_.publish(marker_text);
}


bool KeyframeMultiMapper::savePcdMapSrvCallback(
  Save::Request& request,
  Save::Response& response)
{
  ROS_INFO("Saving map as pcd...");
  const std::string& path = request.filename; 
  bool result = savePcdMap(path);
  
  if (result) ROS_INFO("Pcd map saved to %s", path.c_str());
  else ROS_ERROR("Pcd map saving failed");
  
  return result;
}

bool KeyframeMultiMapper::saveOctomapSrvCallback(
  Save::Request& request,
  Save::Response& response)
{
  ROS_INFO("Saving map as Octomap...");
  const std::string& path = request.filename;
  bool result = saveOctomap(path);
    
  if (result) ROS_INFO("Octomap saved to %s", path.c_str());
  else ROS_ERROR("Octomap saving failed");
    
  return result;
}

bool KeyframeMultiMapper::addManualKeyframeSrvCallback(
  AddManualKeyframe::Request& request,
  AddManualKeyframe::Response& response)
{
  manual_add_ = true;

  return true;
}



bool KeyframeMultiMapper::solveGraphSrvCallback(
  SolveGraph::Request& request,
  SolveGraph::Response& response)
{
  ros::WallTime start = ros::WallTime::now();
  
  // Graph solving: keyframe positions only, path is interpolated
  //graph_solver_.solve(keyframes_, associations_);
  updatePathFromKeyframePoses();
    
  // Graph solving: keyframe positions and VO path
  /*
  AffineTransformVector path;
  pathROSToEigenAffine(path_msg_, path);
  graph_solver_.solve(keyframes_, associations_, path);
  pathEigenAffineToROS(path, path_msg_);
  */
  
  double dur = getMsDuration(start);
  
  ROS_INFO("Solving took %.1f ms", dur);
    
  publishPath();
  publishKeyframePoses();
  //publishKeyframeAssociations();

  return true;
}


/** In the event that the keyframe poses change (from pose-graph solving)
 * this function will propagete teh changes in the path message
 */
void KeyframeMultiMapper::updatePathFromKeyframePoses()
{   
  int kf_size = keyframes_.size();
  int f_size = path_msg_.poses.size();
  
  // temporary store the new path
  AffineTransformVector path_new;
  path_new.resize(f_size);
  
  if (kf_size < 2) return; 
  
  for (int kf_idx = 0; kf_idx < kf_size - 1; ++kf_idx)
  {
    // the indices of the current and next keyframes (a and b)   
    const rgbdtools::RGBDKeyframe& keyframe_a = keyframes_[kf_idx];
    const rgbdtools::RGBDKeyframe& keyframe_b = keyframes_[kf_idx + 1];
    
    // the corresponding frame indices
    int f_idx_a = keyframe_a.index;
    int f_idx_b = keyframe_b.index;
         
    // the new poses of keyframes a and b (after graph solving)
    tf::Transform kf_pose_a = tfFromEigenAffine(keyframe_a.pose);
    tf::Transform kf_pose_b = tfFromEigenAffine(keyframe_b.pose);
    
    // the previous pose of keyframe a and b (before graph solving)
    tf::Transform kf_pose_a_prev, kf_pose_b_prev;
    tf::poseMsgToTF(path_msg_.poses[f_idx_a].pose, kf_pose_a_prev);
    tf::poseMsgToTF(path_msg_.poses[f_idx_b].pose, kf_pose_b_prev);
    
    // the motion, in the camera frame (after and before graph solving)
    tf::Transform kf_motion      = kf_pose_a.inverse() * kf_pose_b;
    tf::Transform kf_motion_prev = kf_pose_a_prev.inverse() * kf_pose_b_prev;
    
    // the correction from the graph solving
    tf::Transform correction = kf_motion_prev.inverse() * kf_motion;
    
    // update the poses in-between keyframes
    for (int f_idx = f_idx_a; f_idx < f_idx_b; ++f_idx)
    {
      // calculate interpolation scale
      double interp_scale = (double)(f_idx - f_idx_a) / (double)(f_idx_b - f_idx_a);
      
      // create interpolated correction translation and rotation
      tf::Vector3 v_interp = correction.getOrigin() * interp_scale;
      tf::Quaternion q_interp = tf::Quaternion::getIdentity();
      q_interp.slerp(correction.getRotation(), interp_scale);
      
      // create interpolated correction
      tf::Transform interpolated_correction;
      interpolated_correction.setOrigin(v_interp);
      interpolated_correction.setRotation(q_interp);
      
      // the previous frame pose
      tf::Transform frame_pose_prev;
      tf::poseMsgToTF(path_msg_.poses[f_idx].pose, frame_pose_prev);
      
      // the pevious frame motion
      tf::Transform frame_motion_prev = kf_pose_a_prev.inverse() * frame_pose_prev;
      
      // the interpolated motion
      tf::Transform interpolated_motion = frame_motion_prev * interpolated_correction;
      
      // calculate the interpolated pose
      path_new[f_idx] = keyframe_a.pose * eigenAffineFromTf(interpolated_motion);
    }  
  }

  // update the last pose
  const rgbdtools::RGBDKeyframe& last_kf = keyframes_[kf_size - 1];

  tf::Transform last_kf_pose_prev;
  tf::poseMsgToTF(path_msg_.poses[last_kf.index].pose, last_kf_pose_prev);
  
  // update the poses in-between last keyframe and end of vo path
  for (int f_idx = last_kf.index; f_idx < f_size; ++f_idx)
  {
    // the previous frame pose
    tf::Transform frame_pose_prev;
    tf::poseMsgToTF(path_msg_.poses[f_idx].pose, frame_pose_prev);
    
    // the pevious frame motion
    tf::Transform frame_motion_prev = last_kf_pose_prev.inverse() * frame_pose_prev;
    
    // calculate the new pose
    path_new[f_idx] = last_kf.pose * eigenAffineFromTf(frame_motion_prev);
  } 
  
  // copy over the interpolated path
  pathEigenAffineToROS(path_new, path_msg_);
}


bool KeyframeMultiMapper::savePcdMap(const std::string& path)
{
  PointCloudT pcd_map;
  buildPcdMap(pcd_map);
  
  // write out
  pcl::PCDWriter writer;
  int result_pcd = writer.writeBinary<PointT>(path, pcd_map);  

  if (result_pcd < 0) return false;
  else return true;
}

void KeyframeMultiMapper::buildPcdMap(PointCloudT& map_cloud)
{
  PointCloudT::Ptr aggregate_cloud(new PointCloudT());
  aggregate_cloud->header.frame_id = fixed_frame_;

  // aggregate all frames into single cloud
  for (unsigned int kf_idx = 0; kf_idx < keyframes_.size(); ++kf_idx)
  {
    const rgbdtools::RGBDKeyframe& keyframe = keyframes_[kf_idx];
    
    PointCloudT cloud;   
    keyframe.constructDensePointCloud(cloud, max_range_, max_stdev_);

    PointCloudT cloud_tf;
    pcl::transformPointCloud(cloud, cloud_tf, keyframe.pose);
    cloud_tf.header.frame_id = fixed_frame_;

    *aggregate_cloud += cloud_tf;
  }

  // filter cloud using voxel grid, and for max z
  pcl::VoxelGrid<PointT> vgf;
  vgf.setInputCloud(aggregate_cloud);
  vgf.setLeafSize(pcd_map_res_, pcd_map_res_, pcd_map_res_);
  vgf.setFilterFieldName("z");
  vgf.setFilterLimits (-std::numeric_limits<double>::infinity(), max_map_z_);

  vgf.filter(map_cloud);
}

bool KeyframeMultiMapper::saveOctomap(const std::string& path)
{
  bool result;

  if (octomap_with_color_)
  {
    octomap::ColorOcTree tree(octomap_res_);   
    buildColorOctomap(tree);
    result = tree.write(path);
  }
  else
  {
    octomap::OcTree tree(octomap_res_);   
    buildOctomap(tree);
    result = tree.write(path);
  }
  
  return result;
}

void KeyframeMultiMapper::buildOctomap(octomap::OcTree& tree)
{
  ROS_INFO("Building Octomap...");
  
  octomap::point3d sensor_origin(0.0, 0.0, 0.0);  

  for (unsigned int kf_idx = 0; kf_idx < keyframes_.size(); ++kf_idx)
  {
    ROS_INFO("Processing keyframe %u", kf_idx);
    const rgbdtools::RGBDKeyframe& keyframe = keyframes_[kf_idx];
    
    PointCloudT cloud;
    keyframe.constructDensePointCloud(cloud, max_range_, max_stdev_);
           
    octomap::pose6d frame_origin = poseTfToOctomap(tfFromEigenAffine(keyframe.pose));

    // build octomap cloud from pcl cloud
    octomap::Pointcloud octomap_cloud;
    for (unsigned int pt_idx = 0; pt_idx < cloud.points.size(); ++pt_idx)
    {
      const PointT& p = cloud.points[pt_idx];
      if (!std::isnan(p.z))
        octomap_cloud.push_back(p.x, p.y, p.z);
    }
    
    tree.insertScan(octomap_cloud, sensor_origin, frame_origin);
  }
}

void KeyframeMultiMapper::buildColorOctomap(octomap::ColorOcTree& tree)
{
  ROS_INFO("Building Octomap with color...");

  octomap::point3d sensor_origin(0.0, 0.0, 0.0);  

  for (unsigned int kf_idx = 0; kf_idx < keyframes_.size(); ++kf_idx)
  {
    ROS_INFO("Processing keyframe %u", kf_idx);
    const rgbdtools::RGBDKeyframe& keyframe = keyframes_[kf_idx];
       
    // construct the cloud
    PointCloudT::Ptr cloud_unf(new PointCloudT());
    keyframe.constructDensePointCloud(*cloud_unf, max_range_, max_stdev_);
  
    // perform filtering for max z
    pcl::transformPointCloud(*cloud_unf, *cloud_unf, keyframe.pose);
    PointCloudT cloud;
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud (cloud_unf);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-std::numeric_limits<double>::infinity(), max_map_z_);
    pass.filter(cloud);
    pcl::transformPointCloud(cloud, cloud, keyframe.pose.inverse());
    
    octomap::pose6d frame_origin = poseTfToOctomap(tfFromEigenAffine(keyframe.pose));
    
    // build octomap cloud from pcl cloud
    octomap::Pointcloud octomap_cloud;
    for (unsigned int pt_idx = 0; pt_idx < cloud.points.size(); ++pt_idx)
    {
      const PointT& p = cloud.points[pt_idx];
      if (!std::isnan(p.z))
        octomap_cloud.push_back(p.x, p.y, p.z);
    }
    
    // insert scan (only xyz considered, no colors)
    tree.insertScan(octomap_cloud, sensor_origin, frame_origin);
    
    // insert colors
    PointCloudT cloud_tf;
    pcl::transformPointCloud(cloud, cloud_tf, keyframe.pose);
    for (unsigned int pt_idx = 0; pt_idx < cloud_tf.points.size(); ++pt_idx)
    {
      const PointT& p = cloud_tf.points[pt_idx];
      if (!std::isnan(p.z))
      {
        octomap::point3d endpoint(p.x, p.y, p.z);
        octomap::ColorOcTreeNode* n = tree.search(endpoint);
        if (n) n->setColor(p.r, p.g, p.b); 
      }
    }
    
    tree.updateInnerOccupancy();
  }
}

void KeyframeMultiMapper::publishPath()
{
  path_msg_.header.frame_id = fixed_frame_; 
  path_pub_.publish(path_msg_);
}

bool KeyframeMultiMapper::savePath(const std::string& filepath)
{
  // open file
  std::string filename = filepath + "/path.txt";
  std::ofstream file(filename.c_str());
  if (!file.is_open()) return false;

  file << "# index seq stamp.sec stamp.nsec x y z qx qy qz qw" << std::endl;

  for (unsigned int idx = 0; idx < path_msg_.poses.size(); ++idx)
  {
    const geometry_msgs::PoseStamped& pose = path_msg_.poses[idx];
    
    file << idx << " "
         << pose.header.seq << " "
         << pose.header.stamp.sec << " "
         << pose.header.stamp.nsec << " "
         << pose.pose.position.x << " "
         << pose.pose.position.y << " "
         << pose.pose.position.z << " "
         << pose.pose.orientation.x << " "
         << pose.pose.orientation.y << " "
         << pose.pose.orientation.z << " " 
         << pose.pose.orientation.w << std::endl;
  }

  file.close();
  
  return true;
}

bool KeyframeMultiMapper::savePathTUMFormat(const std::string& filepath)
{
  // open file
  std::string filename = filepath + "/path.tum.txt";
  std::ofstream file(filename.c_str());
  if (!file.is_open()) return false;

  file << "# stamp x y z qx qy qz qw" << std::endl;

  for (unsigned int idx = 0; idx < path_msg_.poses.size(); ++idx)
  {
    const geometry_msgs::PoseStamped& pose = path_msg_.poses[idx];
    
    file << pose.header.stamp.sec << "."
         << pose.header.stamp.nsec << " "
         << pose.pose.position.x << " "
         << pose.pose.position.y << " "
         << pose.pose.position.z << " "
         << pose.pose.orientation.x << " "
         << pose.pose.orientation.y << " "
         << pose.pose.orientation.z << " " 
         << pose.pose.orientation.w << std::endl;
  }

  file.close();
    
  return true;
}

bool KeyframeMultiMapper::loadPath(const std::string& filepath)
{
  path_msg_.poses.clear();

  // open file
  std::string filename = filepath + "/path.txt";
  std::ifstream file(filename.c_str());
  if (!file.is_open()) return false;
  
  std::string line;

  // get header
  getline(file, line);

  // read each line
  while(std::getline(file, line))
  {
    std::istringstream is(line);
    
    // fill out pose information  
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = fixed_frame_;
    int idx;
       
    is >> idx
       >> pose.header.seq 
       >> pose.header.stamp.sec 
       >> pose.header.stamp.nsec 
       >> pose.pose.position.x 
       >> pose.pose.position.y 
       >> pose.pose.position.z 
       >> pose.pose.orientation.x 
       >> pose.pose.orientation.y 
       >> pose.pose.orientation.z 
       >> pose.pose.orientation.w;
                 
    // add to poses vector  
    path_msg_.poses.push_back(pose);
  }
    
  file.close();
  return true;
}


// frame_a = train, frame_b = query
void KeyframeMultiMapper::getCandidateMatches(
		const rgbdtools::RGBDKeyframe& frame_q, const rgbdtools::RGBDKeyframe& frame_t,
  rgbdtools::DMatchVector& candidate_matches)
{
  // **** build candidate matches ***********************************
  // assumes detectors and distributions are computed
  // establish all matches from b to a

  if (matcher_use_desc_ratio_test_)
  {
    std::vector<rgbdtools::DMatchVector> all_matches2;

    frame_t.matcher->knnMatch(
      frame_q.descriptors, all_matches2, 2);

    for (unsigned int m_idx = 0; m_idx < all_matches2.size(); ++m_idx)
    {
      const cv::DMatch& match1 = all_matches2[m_idx][0];
      const cv::DMatch& match2 = all_matches2[m_idx][1];

      double ratio =  match1.distance / match2.distance;

      // remove bad matches - ratio test, valid keypoints
      if (ratio < matcher_max_desc_ratio_)
      {
        int idx_q = match1.queryIdx;
        int idx_t = match1.trainIdx;

        if (frame_t.kp_valid[idx_t] && frame_q.kp_valid[idx_q])
          candidate_matches.push_back(match1);
      }
    }
  }
  else
  {
	rgbdtools::DMatchVector all_matches;

	frame_t.matcher->match(
      frame_q.descriptors, all_matches);

    for (unsigned int m_idx = 0; m_idx < all_matches.size(); ++m_idx)
    {
      const cv::DMatch& match = all_matches[m_idx];

      // remove bad matches - descriptor distance, valid keypoints
      if (match.distance < matcher_max_desc_dist_)
      {
        int idx_q = match.queryIdx;
        int idx_t = match.trainIdx;

        if (frame_t.kp_valid[idx_t] && frame_q.kp_valid[idx_q])
          candidate_matches.push_back(match);
      }
    }
  }

}

// frame_a = train, frame_b = query
int KeyframeMultiMapper::pairwiseMatchingRANSAC(
  const rgbdtools::RGBDKeyframe& frame_q,
  const rgbdtools::RGBDKeyframe& frame_t,
  rgbdtools::DMatchVector& best_inlier_matches,
  Eigen::Matrix4f& best_transformation)
{
  // constants
  int min_sample_size = 3;

  rgbdtools::DMatchVector candidate_matches;
  getCandidateMatches(frame_q, frame_t, candidate_matches);

  // check if enough matches are present
  if (candidate_matches.size() < min_sample_size)  return 0;
  if (candidate_matches.size() < sac_min_inliers_) return 0;

  ROS_INFO("Candidate matches %d", candidate_matches.size());

  // **** build 3D features for SVD ********************************

  PointCloudFeature features_t, features_q;

  features_t.resize(candidate_matches.size());
  features_q.resize(candidate_matches.size());

  for (int m_idx = 0; m_idx < candidate_matches.size(); ++m_idx)
  {
    const cv::DMatch& match = candidate_matches[m_idx];
    int idx_q = match.queryIdx;
    int idx_t = match.trainIdx;

    PointFeature& p_t = features_t[m_idx];
    p_t.x = frame_t.kp_means[idx_t](0,0);
    p_t.y = frame_t.kp_means[idx_t](1,0);
    p_t.z = frame_t.kp_means[idx_t](2,0);

    PointFeature& p_q = features_q[m_idx];
    p_q.x = frame_q.kp_means[idx_q](0,0);
    p_q.y = frame_q.kp_means[idx_q](1,0);
    p_q.z = frame_q.kp_means[idx_q](2,0);
  }

  // **** main RANSAC loop ****************************************

  TransformationEstimationSVD svd;
  Eigen::Matrix4f transformation; // transformation used inside loop
  best_inlier_matches.clear();
  int iteration = 0;

  std::set<int> mask;

  while(true)
  //for (iteration = 0; iteration < ransac_max_iterations_; ++iteration)
  {
    // generate random indices
    IntVector sample_idx;
    rgbdtools::get3RandomIndices(candidate_matches.size(), mask, sample_idx);

    // build initial inliers from random indices
    IntVector inlier_idx;
    std::vector<cv::DMatch> inlier_matches;

    for (unsigned int s_idx = 0; s_idx < sample_idx.size(); ++s_idx)
    {
      int m_idx = sample_idx[s_idx];
      inlier_idx.push_back(m_idx);
      inlier_matches.push_back(candidate_matches[m_idx]);
    }

    // estimate transformation from minimum set of random samples
    svd.estimateRigidTransformation(
      features_q, inlier_idx,
      features_t, inlier_idx,
      transformation);

    // evaluate transformation fitness by checking distance to all points
    PointCloudFeature features_q_tf;
    pcl::transformPointCloud(features_q, features_q_tf, transformation);

    for (int m_idx = 0; m_idx < candidate_matches.size(); ++m_idx)
    {
      // euclidedan distance test
      const PointFeature& p_t = features_t[m_idx];
      const PointFeature& p_q = features_q_tf[m_idx];
      float eucl_dist_sq = rgbdtools::distEuclideanSq(p_t, p_q);

      if (eucl_dist_sq < sac_max_eucl_dist_sq_)
      {
        inlier_idx.push_back(m_idx);
        inlier_matches.push_back(candidate_matches[m_idx]);

        // reestimate transformation from all inliers
        if (sac_reestimate_tf_)
        {
          svd.estimateRigidTransformation(
            features_q, inlier_idx,
            features_t, inlier_idx,
            transformation);
          pcl::transformPointCloud(features_q, features_q_tf, transformation);
        }
      }
    }

    // check if inliers are better than the best model so far
    if (inlier_matches.size() > best_inlier_matches.size())
    {
      svd.estimateRigidTransformation(
        features_q, inlier_idx,
        features_t, inlier_idx,
        transformation);

      best_transformation = transformation;
      best_inlier_matches = inlier_matches;
    }

    double best_inlier_ratio = (double) best_inlier_matches.size() /
                               (double) candidate_matches.size();

    // **** termination: iterations + inlier ratio
    if(best_inlier_matches.size() < sac_min_inliers_)
    {
      if (iteration >= ransac_max_iterations_) break;
    }
    // **** termination: confidence ratio test
    else
    {
      double h = log_one_minus_ransac_confidence_ /
                log(1.0 - pow(best_inlier_ratio, min_sample_size));

      if (iteration > (int)(h+1)) break;
    }

    iteration++;
  }

  return iteration;
}




void KeyframeMultiMapper::addVertex(
  const AffineTransform& vertex_pose,
  int vertex_idx)
{
  // TODO: use eigen quaternion, not manual conversion
  //Transform Eigen::Matrix4f into 3D traslation and rotation for g2o
  double yaw,pitch,roll;
  yaw   = atan2f(vertex_pose(1,0),vertex_pose(0,0));
  pitch = asinf(-vertex_pose(2,0));
  roll  = atan2f(vertex_pose(2,1),vertex_pose(2,2));

  g2o::Vector3d t(vertex_pose(0,3),vertex_pose(1,3),vertex_pose(2,3));
  g2o::Quaterniond q;
  q.x()=sin(roll/2)*cos(pitch/2)*cos(yaw/2)-cos(roll/2)*sin(pitch/2)*sin(yaw/2);
  q.y()=cos(roll/2)*sin(pitch/2)*cos(yaw/2)+sin(roll/2)*cos(pitch/2)*sin(yaw/2);
  q.z()=cos(roll/2)*cos(pitch/2)*sin(yaw/2)-sin(roll/2)*sin(pitch/2)*cos(yaw/2);
  q.w()=cos(roll/2)*cos(pitch/2)*cos(yaw/2)+sin(roll/2)*sin(pitch/2)*sin(yaw/2);

  g2o::SE3Quat pose(q,t); // vertex pose

  // TODO: smart pointers

  // set up node
  g2o::VertexSE3 *vc = new g2o::VertexSE3();
  vc->estimate() = pose;
  vc->setId(vertex_idx);

  // set first pose fixed
  if (vertex_idx == 0)
    vc->setFixed(true);

  // add to optimizer
  optimizer.addVertex(vc);
}

void KeyframeMultiMapper::addEdge(
  int from_idx,
  int to_idx,
  const AffineTransform& relative_pose,
  const Eigen::Matrix<double,6,6>& information_matrix)
{
  // TODO: use eigen quaternion, not manual conversion
  //Transform Eigen::Matrix4f into 3D traslation and rotation for g2o
  double yaw,pitch,roll;
  yaw   = atan2f(relative_pose(1,0),relative_pose(0,0));
  pitch = asinf(-relative_pose(2,0));
  roll  = atan2f(relative_pose(2,1),relative_pose(2,2));

  g2o::Vector3d t(relative_pose(0,3),relative_pose(1,3),relative_pose(2,3));
  g2o::Quaterniond q;
  q.x()=sin(roll/2)*cos(pitch/2)*cos(yaw/2)-cos(roll/2)*sin(pitch/2)*sin(yaw/2);
  q.y()=cos(roll/2)*sin(pitch/2)*cos(yaw/2)+sin(roll/2)*cos(pitch/2)*sin(yaw/2);
  q.z()=cos(roll/2)*cos(pitch/2)*sin(yaw/2)-sin(roll/2)*sin(pitch/2)*cos(yaw/2);
  q.w()=cos(roll/2)*cos(pitch/2)*cos(yaw/2)+sin(roll/2)*sin(pitch/2)*sin(yaw/2);

  // relative transformation
  g2o::SE3Quat transf(q,t);

  // TODO: smart pointers

  g2o::EdgeSE3* edge = new g2o::EdgeSE3;
  edge->vertices()[0] = optimizer.vertex(from_idx);
  edge->vertices()[1] = optimizer.vertex(to_idx);
  edge->setMeasurement(transf);

  //Set the information matrix
  edge->setInformation(information_matrix);

  optimizer.addEdge(edge);
}

void KeyframeMultiMapper::optimizeGraph()
{
  //Prepare and run the optimization
  optimizer.initializeOptimization();

  //Set the initial Levenberg-Marquardt lambda
  optimizer.setUserLambdaInit(0.01);

  //Run optimization
  optimizer.optimize(20);
}

void KeyframeMultiMapper::getOptimizedPoses(AffineTransformVector& poses)
{
  for (unsigned int idx = 0; idx < poses.size(); ++idx)
  {
    //Transform the vertex pose from G2O quaternion to Eigen::Matrix4f
    g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(idx));
    double optimized_pose_quat[7];
    vertex->getEstimateData(optimized_pose_quat);

    AffineTransform optimized_pose;
    double qx,qy,qz,qr,qx2,qy2,qz2,qr2;

    qx=optimized_pose_quat[3];
    qy=optimized_pose_quat[4];
    qz=optimized_pose_quat[5];
    qr=optimized_pose_quat[6];
    qx2=qx*qx;
    qy2=qy*qy;
    qz2=qz*qz;
    qr2=qr*qr;

    optimized_pose(0,0)=qr2+qx2-qy2-qz2;
    optimized_pose(0,1)=2*(qx*qy-qr*qz);
    optimized_pose(0,2)=2*(qz*qx+qr*qy);
    optimized_pose(0,3)=optimized_pose_quat[0];
    optimized_pose(1,0)=2*(qx*qy+qr*qz);
    optimized_pose(1,1)=qr2-qx2+qy2-qz2;
    optimized_pose(1,2)=2*(qy*qz-qr*qx);
    optimized_pose(1,3)=optimized_pose_quat[1];
    optimized_pose(2,0)=2*(qz*qx-qr*qy);
    optimized_pose(2,1)=2*(qy*qz+qr*qx);
    optimized_pose(2,2)=qr2-qx2-qy2+qz2;
    optimized_pose(2,3)=optimized_pose_quat[2];

    //Set the optimized pose to the vector of poses
    poses[idx] = optimized_pose;
  }
}













} // namespace ccny_rgbd
