#include <iostream>
#include <sstream>

#include "candidate_locator.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
// #include <pcl/visualization/cloud_viewer.h>

CandidateLocator::CandidateLocator() : it_(nh_), tf_listener_(ros::Duration(60))
{
                                           //     /depth/image_raw
  sub_depth_cam_info_ = it_.subscribeCamera("/kinect2/sd/image_depth_rect", 1000, &CandidateLocator::cameraInfoCallback, this);

  // pub_point_clouds_ = nh_.advertise<candidate_locator::ArrayPointClouds>("/candidate_point_clouds", 1);
  
    bool cheat_mode; 
    nh_.param<bool>("/cheat_mode", cheat_mode, true);
    
    camera_frame_ = "/camera_depth_frame";
    if (cheat_mode)    
      camera_frame_ = "/real_camera_pose";

  ROS_INFO("Constructed CandidateLocator.");
}

void CandidateLocator::candidatesCallback(const object_candidates::SnapshotMsg& msg)
{
  // pub_point_clouds_.publish(locateCandidates(
  //   msg.depth_image,
  //   msg.rgb_image,
  //   msg.rgb_info,
  //   msg.candidates));
}

pcl::PointCloud<pcl::PointXYZRGB> CandidateLocator::getMinCluster(pcl::PointCloud<pcl::PointXYZRGB> point_cloud){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud_filtered = point_cloud;
    
    //writer.write<pcl::PointXYZRGB> ("original.pcd", point_cloud, false); // !*

    /*
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(cloud_filtered);
    while  (! viewer.wasStopped ())
    { //no -op  until  viewer  stopped
    }*/

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    int j = 0;
    double min_dist = 9999999;
    int min_cluster = -1;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_clusters;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
        pcl::PointXYZRGB point = cloud_filtered->points[*pit];
        cloud_cluster->points.push_back (point);
        Eigen::Vector3f vector(point.x,point.y,point.z);
        double dist = vector.norm();
        if (dist < min_dist){
          min_dist = dist;
          min_cluster = j;
        }        
      }
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      cloud_clusters.push_back(cloud_cluster);

      ROS_INFO_STREAM("PointCloud representing the Cluster " << j << " has " << cloud_cluster->points.size () << " data points.");
      j++;
    }
    
    if (min_cluster == -1)
      return point_cloud;
      
    return *cloud_clusters[min_cluster];
}

candidate_locator::ArrayPointClouds CandidateLocator::locateCandidates(
  const sensor_msgs::Image& depth_image,
  const sensor_msgs::Image& rgb_image,
  const sensor_msgs::CameraInfo& rgb_info,
  const object_candidates::ArrayImages& candidates)
{
  ROS_INFO_STREAM("");
  ROS_INFO_STREAM("----------");

  // If cam_model_ has not been assigned (i.e. the cameraInfoCallback method has not
  // run at least once) then the localisation won't work; hence, return straight away.
  // Commented out when method signature changed to have a return value
  // if (!cam_model_assigned) { return; }

  candidate_locator::ArrayPointClouds array_pc_msg;

  // Assign depth image to member variable
  try
  {
    depth_image_ = cv_bridge::toCvCopy(depth_image, "32FC1")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("%s. Encoding of image is %s", e.what(), depth_image.encoding.c_str());
  }

  // Get transforms
  this->getTransforms(depth_image.header.stamp);

  // Variables for candidate localisation
  cv::Mat candidate;

  ROS_INFO_STREAM("Number of candidates: " << candidates.data.size());

  // Iterate through candidates and localise
  for(uint i = 0; i < candidates.data.size(); i++)
  {
    try
    {
      candidate = cv_bridge::toCvCopy(candidates.data[i], "mono8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("%s", e.what());
    }

    pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
    sensor_msgs::PointCloud2 pc_msg;

    ROS_INFO_STREAM("Candidate " << candidate_id_ << ": ");
    this->calculateObjectPoints(candidate, point_cloud);

    // pcl::toROSMsg(this->getMinCluster(point_cloud), pc_msg);
    pcl::toROSMsg(point_cloud, pc_msg);
    ROS_INFO_STREAM("Point cloud size: " << point_cloud.size());
          
    pc_msg.header.frame_id = camera_frame_;
    
    // pcl_ros::transformPointCloud(
    //   "/map",
    //   map_transform_,
    //   pc_msg,
    //   pc_msg);

    array_pc_msg.data.push_back(pc_msg);
  }

  return array_pc_msg;

}

void CandidateLocator::getTransforms(const ros::Time& stamp)
{
  ROS_INFO("Requesting transforms for timestamp %d.%d", stamp.sec, stamp.nsec);
  
  try
  {
    tf_listener_.waitForTransform(
      "/map",
      camera_frame_,
      stamp,
      ros::Duration(2.0));

    tf_listener_.lookupTransform(
      "/map",
      camera_frame_,
      stamp,
      map_transform_);

    // tf_listener_.lookupTransform(
    //   "/camera_optical_frame",
    //   "/camera_depth_frame",
    //   stamp,
    //   camera_transform_);

    ROS_INFO("Transforms received.");

  }
  catch (tf::TransformException &e)
  {
    ROS_ERROR("%s", e.what());
  }
}

void CandidateLocator::calculateObjectPoints(cv::Mat& candidate, pcl::PointCloud<pcl::PointXYZRGB>& point_cloud)
{
  // Calculate desired "colour" from candidate_id_
  uint red, green, blue = 0;
  red = candidate_id_ / 256;
  green = candidate_id_ % 256;
  ROS_DEBUG_STREAM("Candidate " << candidate_id_ << ": red = " << red << " green = " << green);
  candidate_id_++;

  // Code adapted from function GazeboRosOpenniKinect::FillPointCloudHelper

  uint candidate_size = 0;
  uint channels = candidate.channels();
  uint nRows = candidate.rows;
  uint nCols = candidate.cols * channels;

  double point_cloud_cutoff = 0.4;
  double point_cloud_cutoff_max = 5.0;

  // double hfov = this->parentSensor->DepthCamera()->HFOV().Radian();
  // TODO: set hfov dynamically from camera info (if possible)
  double hfov = 1.021018; // 58.5 degrees
  double fl = (double)nCols / (2.0 * tan(hfov/2.0));
  
  // convert depth to point cloud
  for (uint32_t j=0; j<nCols; j++)
  {
    double pAngle;
    if (nCols>1) pAngle = atan2( (double)j - 0.5*(double)(nCols-1), fl);
    else            pAngle = 0.0;

    for (uint32_t i=0; i<nRows; i++)
    {
      if (candidate.ptr<uchar>(i)[nCols-1-j] != 0)
      {
        double yAngle;
        if (nRows>1) yAngle = atan2( (double)i - 0.5*(double)(nRows-1), fl);
        else            yAngle = 0.0;

        double depth = depth_image_.at<float>(i, nCols-j) / 1000;
        ROS_INFO_STREAM("Depth: " << depth);
        if(depth > point_cloud_cutoff &&
           depth < point_cloud_cutoff_max)
        {
          candidate_size++;

          pcl::PointXYZRGB pclPoint;
          pclPoint.x = depth * tan(yAngle);
          pclPoint.y = depth * tan(pAngle);
          pclPoint.z = depth;
          pclPoint.r = red;
          pclPoint.g = green;
          pclPoint.b = blue;
          point_cloud.push_back(pclPoint);
        }
        // else //point in the unseeable range
        // {
        //   *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN ();
        //   point_cloud_msg.is_dense = false;
        // }
      }
    }
  }

  ROS_INFO_STREAM("Point cloud created for candidate. Candidate size: " << candidate_size << ", " << point_cloud.size());
}

void CandidateLocator::cameraInfoCallback(const sensor_msgs::ImageConstPtr& depth_img_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  cam_model_.fromCameraInfo(info_msg);

  if(cam_model_assigned == false)
  {
    cam_model_assigned = true;
    ROS_INFO("Camera model assigned.");
    ROS_INFO_STREAM("cx: " << cam_model_.cx() << " cy: " << cam_model_.cy());
    ROS_INFO_STREAM("fx: " << cam_model_.fx() << " fy: " << cam_model_.fy());
    ROS_INFO_STREAM("Tx: " << cam_model_.Tx() << " Ty: " << cam_model_.Ty());
  }
}
