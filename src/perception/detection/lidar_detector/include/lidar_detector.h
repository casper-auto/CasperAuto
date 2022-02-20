#ifndef LIDAR_DETECTOR_H
#define LIDAR_DETECTOR_H

#include <chrono>
#include <ctime>
#include <iostream>
#include <string>
#include <vector>

// pcl
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

// ros
#include <ros/ros.h>

#include <derived_object_msgs/Object.h>
#include <derived_object_msgs/ObjectArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "box.h"

class LidarDetector {
public:
  //! @brief Constructor
  //!
  LidarDetector();

  //! @brief Destructor
  //!
  ~LidarDetector();

private:
  // ros
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // publishers
  ros::Publisher objects_cloud_pub_;
  ros::Publisher plane_cloud_pub_;
  ros::Publisher detected_objects_pub_;

  // subscribers
  ros::Subscriber point_cloud_sub_;

  //! @brief timer calling periodicUpdate
  //!
  ros::Timer periodic_update_timer_;

  // var
  pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr objects_cloud_ptr_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud_ptr_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_clusters_;
  derived_object_msgs::ObjectArray detected_objects_;

  //! @brief callback function which is called for periodic updates
  //!
  void periodicUpdate(const ros::TimerEvent &event);

  //! @brief
  //! @param[in]
  //!
  void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

  //! @brief
  //! @param[in]
  //!
  visualization_msgs::MarkerArray convertToVisualizationMakers(
      const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_clusters);

  //! @brief
  //! @param[in]
  //!
  void numPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  //! @brief
  //! @param[in]
  //!
  pcl::PointCloud<pcl::PointXYZ>::Ptr
  filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float filter_res,
              Eigen::Vector4f min_point, Eigen::Vector4f max_point);

  //! @brief
  //! @param[in]
  //!
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,
            pcl::PointCloud<pcl::PointXYZ>::Ptr>
  separateClouds(pcl::PointIndices::Ptr inliers,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  //! @brief
  //! @param[in]
  //!
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,
            pcl::PointCloud<pcl::PointXYZ>::Ptr>
  segmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int max_iterations,
               float distance_thresh);

  //! @brief Perform euclidean clustering to group detected obstacles.
  //! @param[in]
  //!
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
  clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float tolerance,
             int min_size, int max_size);

  Box boundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster);
};

#endif /* LIDAR_DETECTOR_H */
