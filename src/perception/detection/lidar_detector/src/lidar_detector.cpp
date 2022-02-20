#include "lidar_detector.h"

// constructor:
LidarDetector::LidarDetector() : nh_(), nh_private_("~") {
  in_cloud_ptr_ =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  filtered_cloud_ptr_ =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  objects_cloud_ptr_ =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  plane_cloud_ptr_ =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

  objects_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/objects_cloud", 10);
  plane_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/plane_cloud", 10);
  detected_objects_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/detected_objects", 10);

  point_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
      "/carla/ego_vehicle/lidar/point_cloud2", 1,
      &LidarDetector::lidarCallback, this);

  periodic_update_timer_ = nh_.createTimer(
      ros::Duration(1. / 10), &LidarDetector::periodicUpdate, this);
}

// de-constructor:
LidarDetector::~LidarDetector() {}

void LidarDetector::periodicUpdate(const ros::TimerEvent &event) {

  filtered_cloud_ptr_ =
      filterCloud(in_cloud_ptr_, 0.5f, Eigen::Vector4f(-30, -6.5, -2.5, 1),
                  Eigen::Vector4f(30, 6.5, 2.5, 1));

  // segment the point cloud to objectCloud and planeCloud
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,
            pcl::PointCloud<pcl::PointXYZ>::Ptr>
      segmented_cloud = segmentPlane(filtered_cloud_ptr_, 100, 0.2);

  objects_cloud_ptr_ = segmented_cloud.first;
  plane_cloud_ptr_ = segmented_cloud.second;

  cloud_clusters_ = clustering(objects_cloud_ptr_, 1.0, 3, 30);

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*objects_cloud_ptr_, cloud_msg);
  cloud_msg.header.frame_id = "ego_vehicle/lidar";
  objects_cloud_pub_.publish(cloud_msg);

  sensor_msgs::PointCloud2 cloud_msg2;
  pcl::toROSMsg(*plane_cloud_ptr_, cloud_msg2);
  cloud_msg2.header.frame_id = "ego_vehicle/lidar";
  plane_cloud_pub_.publish(cloud_msg2);

  std::cout << "+++++++++++++++++++++" << std::endl;
  std::cout << cloud_clusters_.size() << std::endl;
  std::cout << "+++++++++++++++++++++" << std::endl;

  visualization_msgs::MarkerArray marker_array =
      convertToVisualizationMakers(cloud_clusters_);

  detected_objects_pub_.publish(marker_array);
}

void LidarDetector::lidarCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
  pcl::fromROSMsg(*msg, *in_cloud_ptr_);
  // pcl::PCLPointCloud2 pcl_pc2;
  // pcl_conversions::toPCL(*msg, pcl_pc2);
  // pcl::fromPCLPointCloud2(pcl_pc2, *in_cloud_ptr_);
}

visualization_msgs::MarkerArray LidarDetector::convertToVisualizationMakers(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_clusters) {
  visualization_msgs::MarkerArray object_marker_array;

  int cluster_id = 0;

  for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloud_clusters) {
    // Find bounding box for one of the clusters
    pcl::PointXYZ min_point, max_point;
    pcl::getMinMax3D(*cluster, min_point, max_point);

    double center_x = (min_point.x + max_point.x) / 2.0;
    double center_y = (min_point.y + max_point.y) / 2.0;
    double center_z = (min_point.z + max_point.z) / 2.0;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "ego_vehicle/lidar";
    marker.header.stamp = ros::Time();
    marker.ns = "lidar/detection";
    marker.id = cluster_id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = center_x;
    marker.pose.position.y = center_y;
    marker.pose.position.z = center_z;
    marker.scale.x = 5;
    marker.scale.y = 2;
    marker.scale.z = 2;
    marker.frame_locked = false;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;

    object_marker_array.markers.push_back(marker);

    ++cluster_id;
  }

  std::cout << "=====================" << std::endl;
  std::cout << cluster_id << std::endl;
  std::cout << "=====================" << std::endl;

  return object_marker_array;
}

void LidarDetector::numPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  std::cout << cloud->points.size() << std::endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
LidarDetector::filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                           float filter_res, Eigen::Vector4f min_point,
                           Eigen::Vector4f max_point) {

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // Done:: Fill in the function to do voxel grid point reduction and region
  // based filtering
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped(
      new pcl::PointCloud<pcl::PointXYZ>);

  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(filter_res, filter_res, filter_res);
  sor.filter(*cloud_filtered);

  // CropBox
  pcl::CropBox<pcl::PointXYZ> cropBox(true);
  cropBox.setInputCloud(cloud_filtered);
  cropBox.setMin(min_point);
  cropBox.setMax(max_point);
  cropBox.filter(*cloud_cropped);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds"
            << std::endl;

  return cloud_cropped;
}

std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,
          pcl::PointCloud<pcl::PointXYZ>::Ptr>
LidarDetector::separateClouds(pcl::PointIndices::Ptr inliers,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  // Create two new point clouds, one cloud with obstacles and other with
  // segmented plane
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr road(
      new pcl::PointCloud<pcl::PointXYZ>());

  for (auto &idx : inliers->indices) {
    road->points.push_back(cloud->points[idx]);
  }

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  // Extract the inliers
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstacles);
  std::cerr << "PointCloud representing the obstacles: "
            << obstacles->width * obstacles->height << " data points."
            << std::endl;

  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,
            pcl::PointCloud<pcl::PointXYZ>::Ptr>
      seg_result(obstacles, road);

  return seg_result;
}

std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,
          pcl::PointCloud<pcl::PointXYZ>::Ptr>
LidarDetector::segmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                            int max_iterations, float distance_thresh) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // Fill in this function to find inliers for the cloud.
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(max_iterations);
  seg.setDistanceThreshold(distance_thresh);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0) {
    std::cerr << "Could not estimate a planar model for the given dataset."
              << std::endl;
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
            << coefficients->values[1] << " " << coefficients->values[2] << " "
            << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count()
            << " milliseconds" << std::endl;

  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,
            pcl::PointCloud<pcl::PointXYZ>::Ptr>
      seg_result = separateClouds(inliers, cloud);

  return seg_result;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
LidarDetector::clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                          float tolerance, int min_size, int max_size) {

  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(tolerance);
  ec.setMinClusterSize(min_size);
  ec.setMaxClusterSize(max_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  for (std::vector<pcl::PointIndices>::const_iterator it =
           cluster_indices.begin();
       it != cluster_indices.end(); ++it) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
        new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin();
         pit != it->indices.end(); ++pit) {
      cloud_cluster->push_back((*cloud)[*pit]);
    }
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    clusters.push_back(cloud_cluster);

    std::cout << "PointCloud representing the Cluster: "
              << cloud_cluster->size() << " data points." << std::endl;
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count()
            << " milliseconds and found " << clusters.size()
            << " clusters" << std::endl;

  return clusters;
}

Box LidarDetector::boundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster) {

  // Find bounding box for one of the clusters
  pcl::PointXYZ min_point, max_point;
  pcl::getMinMax3D(*cluster, min_point, max_point);

  Box box;
  box.x_min = min_point.x;
  box.y_min = min_point.y;
  box.z_min = min_point.z;
  box.x_max = max_point.x;
  box.y_max = max_point.y;
  box.z_max = max_point.z;

  return box;
}
