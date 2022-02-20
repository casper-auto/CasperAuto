#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/don.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/common/common.h>

#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>

#include <pcl/segmentation/extract_clusters.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>

#include <visualization_msgs/Marker.h>

#include "autoware_msgs/Centroids.h"
#include "autoware_msgs/CloudCluster.h"
#include "autoware_msgs/CloudClusterArray.h"
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

#include <tf/tf.h>

#include <yaml-cpp/yaml.h>

#include "opencv2/core/core.hpp"
#include <opencv2/core/version.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#if (CV_MAJOR_VERSION == 3)

#include "gencolors.cpp"

#endif

#include "cluster.h"

#ifdef GPU_CLUSTERING

#include "gpu_euclidean_clustering.h"

#endif

#define APP_NAME "euclidean_clustering"

using namespace cv;

/**
  class EuclideanClusterDetector
*/
class EuclideanClusterDetector {
public:
  EuclideanClusterDetector();

  ~EuclideanClusterDetector() {}

private:
  // ros nodehandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // ros publishers
  ros::Publisher cluster_cloud_pub_;
  ros::Publisher ground_cloud_pub_;
  ros::Publisher centroid_pub_;
  ros::Publisher clusters_message_pub_;
  ros::Publisher points_lanes_cloud_pub_;
  ros::Publisher detected_objects_pub_;

  // ros subsribers
  ros::Subscriber raw_point_cloud_sub_;

  // vars
  std::string points_topic_;
  std::string gridmap_topic_;
  std::string str_distances_;
  std::string str_ranges_;
  std_msgs::Header velodyne_header_;
  std::string output_frame_;
  bool velodyne_transform_available_;
  bool downsample_cloud_;
  bool pose_estimation_;
  double leaf_size_;
  int cluster_size_min_;
  int cluster_size_max_;
  double initial_quat_w_; // originally const
  bool remove_ground_;    // only ground
  bool using_sensor_cloud_;
  bool use_diffnormals_;
  double clip_min_height_;
  double clip_max_height_;
  bool keep_lanes_;
  double keep_lane_left_distance_;
  double keep_lane_right_distance_;
  double remove_points_upto_;
  double cluster_merge_threshold_;
  double clustering_distance_;
  bool use_gpu_;
  std::chrono::system_clock::time_point start_, end_;
  std::vector<std::vector<geometry_msgs::Point>> way_area_points_;
  std::vector<cv::Scalar> colors_;
  pcl::PointCloud<pcl::PointXYZ> sensor_cloud_;
  visualization_msgs::Marker visualization_marker_;
  bool use_multiple_thres_;
  std::vector<double> clustering_distances_;
  std::vector<double> clustering_ranges_;
  tf::StampedTransform *transform_;
  tf::StampedTransform *velodyne_output_transform_;
  tf::TransformListener *transform_listener_;
  tf::TransformListener *vectormap_transform_listener_;

  void rawPointCloudCallback(
      const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud);

  tf::StampedTransform findTransform(const std::string &in_target_frame,
                                     const std::string &in_source_frame);

  geometry_msgs::Point transformPoint(const geometry_msgs::Point &point,
                                      const tf::Transform &tf);

  void
  transformBoundingBox(const jsk_recognition_msgs::BoundingBox &in_boundingbox,
                       jsk_recognition_msgs::BoundingBox &out_boundingbox,
                       const std::string &in_target_frame,
                       const std_msgs::Header &in_header);

  void
  publishDetectedObjects(const autoware_msgs::CloudClusterArray &in_clusters);

  void publishCloudClusters(const ros::Publisher *in_publisher,
                            const autoware_msgs::CloudClusterArray &in_clusters,
                            const std::string &in_target_frame,
                            const std_msgs::Header &in_header);

  void publishCentroids(const ros::Publisher *in_publisher,
                        const autoware_msgs::Centroids &in_centroids,
                        const std::string &in_target_frame,
                        const std_msgs::Header &in_header);

  void publishCloud(
      const ros::Publisher *in_publisher,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr);

  void publishColorCloud(
      const ros::Publisher *in_publisher,
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_to_publish_ptr);

  void keepLanePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
                      float in_left_lane_threshold = 1.5,
                      float in_right_lane_threshold = 1.5);

#ifdef GPU_CLUSTERING

  std::vector<ClusterPtr>
  clusterAndColorGpu(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
                     autoware_msgs::Centroids &in_out_centroids,
                     double in_max_cluster_distance = 0.5);
#endif

  std::vector<ClusterPtr>
  clusterAndColor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
                  autoware_msgs::Centroids &in_out_centroids,
                  double in_max_cluster_distance = 0.5);

  void checkClusterMerge(size_t in_cluster_id,
                         std::vector<ClusterPtr> &in_clusters,
                         std::vector<bool> &in_out_visited_clusters,
                         std::vector<size_t> &out_merge_indices,
                         double in_merge_threshold);

  void mergeClusters(const std::vector<ClusterPtr> &in_clusters,
                     std::vector<ClusterPtr> &out_clusters,
                     std::vector<size_t> in_merge_indices,
                     const size_t &current_index,
                     std::vector<bool> &in_out_merged_clusters);

  void checkAllForMerge(std::vector<ClusterPtr> &in_clusters,
                        std::vector<ClusterPtr> &out_clusters,
                        float in_merge_threshold);

  void segmentByDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
                         autoware_msgs::Centroids &in_out_centroids,
                         autoware_msgs::CloudClusterArray &in_out_clusters);

  void removeFloor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr out_nofloor_cloud_ptr,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr out_onlyfloor_cloud_ptr,
                   float in_max_height = 0.2, float in_floor_max_angle = 0.1);

  void downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
                       float in_leaf_size = 0.2);

  void clipCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
                 float in_min_height = -1.3, float in_max_height = 0.5);

  void differenceNormalsSegmentation(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
      pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr);

  void removePointsUpTo(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
                        const double in_distance);

  void downsamplePoints(const Mat &src, Mat &dst, size_t count);

  void generateColors(std::vector<Scalar> &colors, size_t count,
                      size_t factor = 100);
};

////////////////////////////////////////////////////////////////////////////////

EuclideanClusterDetector::EuclideanClusterDetector()
    : nh_(""), nh_private_("~") {
  // ros param server
  if (nh_private_.getParam("points_node", points_topic_)) {
    ROS_INFO("euclidean_cluster > Setting points node to %s",
             points_topic_.c_str());
  } else {
    ROS_INFO("euclidean_cluster > No points node received, defaulting to "
             "points_raw, you can use "
             "_points_node:=YOUR_TOPIC");
    points_topic_ = "/points_raw";
  }

  use_diffnormals_ = false;
  if (nh_private_.getParam("use_diffnormals", use_diffnormals_)) {
    if (use_diffnormals_)
      ROS_INFO("Euclidean Clustering: Applying difference of normals on "
               "clustering pipeline");
    else
      ROS_INFO("Euclidean Clustering: Difference of Normals will not be used.");
  }

  /* Initialize tuning parameter */
  nh_private_.param("downsample_cloud", downsample_cloud_, false);
  ROS_INFO("[%s] downsample_cloud: %d", APP_NAME, downsample_cloud_);
  nh_private_.param("remove_ground", remove_ground_, true);
  ROS_INFO("[%s] remove_ground: %d", APP_NAME, remove_ground_);
  nh_private_.param("leaf_size", leaf_size_, 0.1);
  ROS_INFO("[%s] leaf_size: %f", APP_NAME, leaf_size_);
  nh_private_.param("cluster_size_min", cluster_size_min_, 20);
  ROS_INFO("[%s] cluster_size_min %d", APP_NAME, cluster_size_min_);
  nh_private_.param("cluster_size_max", cluster_size_max_, 100000);
  ROS_INFO("[%s] cluster_size_max: %d", APP_NAME, cluster_size_max_);
  nh_private_.param("pose_estimation", pose_estimation_, false);
  ROS_INFO("[%s] pose_estimation: %d", APP_NAME, pose_estimation_);
  nh_private_.param("clip_min_height", clip_min_height_, -1.3);
  ROS_INFO("[%s] clip_min_height: %f", APP_NAME, clip_min_height_);
  nh_private_.param("clip_max_height", clip_max_height_, 0.5);
  ROS_INFO("[%s] clip_max_height: %f", APP_NAME, clip_max_height_);
  nh_private_.param("keep_lanes", keep_lanes_, false);
  ROS_INFO("[%s] keep_lanes: %d", APP_NAME, keep_lanes_);
  nh_private_.param("keep_lane_left_distance", keep_lane_left_distance_, 5.0);
  ROS_INFO("[%s] keep_lane_left_distance: %f", APP_NAME,
           keep_lane_left_distance_);
  nh_private_.param("keep_lane_right_distance", keep_lane_right_distance_, 5.0);
  ROS_INFO("[%s] keep_lane_right_distance: %f", APP_NAME,
           keep_lane_right_distance_);
  nh_private_.param("cluster_merge_threshold", cluster_merge_threshold_, 1.5);
  ROS_INFO("[%s] cluster_merge_threshold: %f", APP_NAME,
           cluster_merge_threshold_);
  nh_private_.param<std::string>("output_frame", output_frame_, "velodyne");
  ROS_INFO("[%s] output_frame: %s", APP_NAME, output_frame_.c_str());

  nh_private_.param("remove_points_upto", remove_points_upto_, 0.0);
  ROS_INFO("[%s] remove_points_upto: %f", APP_NAME, remove_points_upto_);

  nh_private_.param("clustering_distance", clustering_distance_, 0.75);
  ROS_INFO("[%s] clustering_distance: %f", APP_NAME, clustering_distance_);

  nh_private_.param("use_gpu", use_gpu_, false);
  ROS_INFO("[%s] use_gpu: %d", APP_NAME, use_gpu_);

  nh_private_.param("use_multiple_thres", use_multiple_thres_, false);
  ROS_INFO("[%s] use_multiple_thres: %d", APP_NAME, use_multiple_thres_);

  nh_private_.param("clustering_distances", str_distances_,
                    std::string("[0.5,1.1,1.6,2.1,2.6]"));
  ROS_INFO("[%s] clustering_distances: %s", APP_NAME, str_distances_.c_str());
  nh_private_.param("clustering_ranges", str_ranges_,
                    std::string("[15,30,45,60]"));
  ROS_INFO("[%s] clustering_ranges: %s", APP_NAME, str_ranges_.c_str());

  if (use_multiple_thres_) {
    YAML::Node distances = YAML::Load(str_distances_);
    YAML::Node ranges = YAML::Load(str_ranges_);
    size_t distances_size = distances.size();
    size_t ranges_size = ranges.size();
    if (distances_size == 0 || ranges_size == 0) {
      ROS_ERROR("Invalid size of clustering_ranges or/and clustering_distance. \
    The size of clustering distance and clustering_ranges should not be 0");
      ros::shutdown();
    }
    if ((distances_size - ranges_size) != 1) {
      ROS_ERROR("Invalid size of clustering_ranges or/and clustering_distance. \
    Expecting that (distances_size - ranges_size) == 1 ");
      ros::shutdown();
    }
    for (size_t i_distance = 0; i_distance < distances_size; i_distance++) {
      clustering_distances_.push_back(distances[i_distance].as<double>());
    }
    for (size_t i_range = 0; i_range < ranges_size; i_range++) {
      clustering_ranges_.push_back(ranges[i_range].as<double>());
    }
  }

  velodyne_transform_available_ = false;

  tf::StampedTransform transform;
  tf::TransformListener listener;
  tf::TransformListener vectormap_tf_listener;

  vectormap_transform_listener_ = &vectormap_tf_listener;
  transform_ = &transform;
  transform_listener_ = &listener;

#if (CV_MAJOR_VERSION == 3)
  generateColors(colors_, 255);
#else
  generateColors(colors_, 255);
#endif

  cluster_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/points_cluster", 1);
  ground_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/points_ground", 1);
  centroid_pub_ =
      nh_.advertise<autoware_msgs::Centroids>("/cluster_centroids", 1);

  points_lanes_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/points_lanes", 1);
  clusters_message_pub_ = nh_.advertise<autoware_msgs::CloudClusterArray>(
      "/detection/lidar_detector/cloud_clusters", 1);
  detected_objects_pub_ = nh_.advertise<autoware_msgs::DetectedObjectArray>(
      "/detection/lidar_detector/objects", 1);

  // Create a ROS subscriber for the input point cloud
  raw_point_cloud_sub_ = nh_.subscribe(
      points_topic_, 1, &EuclideanClusterDetector::rawPointCloudCallback, this);

  initial_quat_w_ = 1.0;
  using_sensor_cloud_ = false;

  // Spin
  ros::spin();
}

void EuclideanClusterDetector::rawPointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud) {
  // start_ = std::chrono::system_clock::now();

  if (!using_sensor_cloud_) {
    using_sensor_cloud_ = true;

    pcl::PointCloud<pcl::PointXYZ>::Ptr current_sensor_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr removed_points_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlanes_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr nofloor_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr onlyfloor_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr diffnormals_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clipped_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_clustered_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZRGB>);

    autoware_msgs::Centroids centroids;
    autoware_msgs::CloudClusterArray cloud_clusters;

    pcl::fromROSMsg(*in_sensor_cloud, *current_sensor_cloud_ptr);

    velodyne_header_ = in_sensor_cloud->header;

    if (remove_points_upto_ > 0.0) {
      removePointsUpTo(current_sensor_cloud_ptr, removed_points_cloud_ptr,
                       remove_points_upto_);
    } else {
      removed_points_cloud_ptr = current_sensor_cloud_ptr;
    }

    if (downsample_cloud_)
      downsampleCloud(removed_points_cloud_ptr, downsampled_cloud_ptr,
                      leaf_size_);
    else
      downsampled_cloud_ptr = removed_points_cloud_ptr;

    clipCloud(downsampled_cloud_ptr, clipped_cloud_ptr, clip_min_height_,
              clip_max_height_);

    if (keep_lanes_)
      keepLanePoints(clipped_cloud_ptr, inlanes_cloud_ptr,
                     keep_lane_left_distance_, keep_lane_right_distance_);
    else
      inlanes_cloud_ptr = clipped_cloud_ptr;

    if (remove_ground_) {
      removeFloor(inlanes_cloud_ptr, nofloor_cloud_ptr, onlyfloor_cloud_ptr);
      publishCloud(&ground_cloud_pub_, onlyfloor_cloud_ptr);
    } else {
      nofloor_cloud_ptr = inlanes_cloud_ptr;
    }

    publishCloud(&points_lanes_cloud_pub_, nofloor_cloud_ptr);

    if (use_diffnormals_)
      differenceNormalsSegmentation(nofloor_cloud_ptr, diffnormals_cloud_ptr);
    else
      diffnormals_cloud_ptr = nofloor_cloud_ptr;

    segmentByDistance(diffnormals_cloud_ptr, colored_clustered_cloud_ptr,
                      centroids, cloud_clusters);

    publishColorCloud(&cluster_cloud_pub_, colored_clustered_cloud_ptr);

    centroids.header = velodyne_header_;

    publishCentroids(&centroid_pub_, centroids, output_frame_,
                     velodyne_header_);

    cloud_clusters.header = velodyne_header_;

    publishCloudClusters(&clusters_message_pub_, cloud_clusters, output_frame_,
                         velodyne_header_);

    using_sensor_cloud_ = false;
  }
}

tf::StampedTransform
EuclideanClusterDetector::findTransform(const std::string &in_target_frame,
                                        const std::string &in_source_frame) {
  tf::StampedTransform transform;

  try {
    // What time should we use?
    vectormap_transform_listener_->lookupTransform(
        in_target_frame, in_source_frame, ros::Time(0), transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return transform;
  }

  return transform;
}

geometry_msgs::Point
EuclideanClusterDetector::transformPoint(const geometry_msgs::Point &point,
                                         const tf::Transform &tf) {
  tf::Point tf_point;
  tf::pointMsgToTF(point, tf_point);

  tf_point = tf * tf_point;

  geometry_msgs::Point ros_point;
  tf::pointTFToMsg(tf_point, ros_point);

  return ros_point;
}

void EuclideanClusterDetector::transformBoundingBox(
    const jsk_recognition_msgs::BoundingBox &in_boundingbox,
    jsk_recognition_msgs::BoundingBox &out_boundingbox,
    const std::string &in_target_frame, const std_msgs::Header &in_header) {
  geometry_msgs::PoseStamped pose_in, pose_out;
  pose_in.header = in_header;
  pose_in.pose = in_boundingbox.pose;
  try {
    transform_listener_->transformPose(in_target_frame, ros::Time(), pose_in,
                                       in_header.frame_id, pose_out);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("transformBoundingBox: %s", ex.what());
  }
  out_boundingbox.pose = pose_out.pose;
  out_boundingbox.header = in_header;
  out_boundingbox.header.frame_id = in_target_frame;
  out_boundingbox.dimensions = in_boundingbox.dimensions;
  out_boundingbox.value = in_boundingbox.value;
  out_boundingbox.label = in_boundingbox.label;
}

void EuclideanClusterDetector::publishDetectedObjects(
    const autoware_msgs::CloudClusterArray &in_clusters) {
  autoware_msgs::DetectedObjectArray detected_objects;
  detected_objects.header = in_clusters.header;

  for (size_t i = 0; i < in_clusters.clusters.size(); i++) {
    autoware_msgs::DetectedObject detected_object;
    detected_object.header = in_clusters.header;
    detected_object.label = "unknown";
    detected_object.score = 1.;
    detected_object.space_frame = in_clusters.header.frame_id;
    detected_object.pose = in_clusters.clusters[i].bounding_box.pose;
    detected_object.dimensions = in_clusters.clusters[i].dimensions;
    detected_object.pointcloud = in_clusters.clusters[i].cloud;
    detected_object.convex_hull = in_clusters.clusters[i].convex_hull;
    detected_object.valid = true;

    detected_objects.objects.push_back(detected_object);
  }
  detected_objects_pub_.publish(detected_objects);
}

void EuclideanClusterDetector::publishCloudClusters(
    const ros::Publisher *in_publisher,
    const autoware_msgs::CloudClusterArray &in_clusters,
    const std::string &in_target_frame, const std_msgs::Header &in_header) {
  if (in_target_frame != in_header.frame_id) {
    autoware_msgs::CloudClusterArray clusters_transformed;
    clusters_transformed.header = in_header;
    clusters_transformed.header.frame_id = in_target_frame;
    for (auto i = in_clusters.clusters.begin(); i != in_clusters.clusters.end();
         i++) {
      autoware_msgs::CloudCluster cluster_transformed;
      cluster_transformed.header = in_header;
      try {
        transform_listener_->lookupTransform(in_target_frame,
                                             velodyne_header_.frame_id,
                                             ros::Time(), *transform_);
        pcl_ros::transformPointCloud(in_target_frame, *transform_, i->cloud,
                                     cluster_transformed.cloud);
        transform_listener_->transformPoint(in_target_frame, ros::Time(),
                                            i->min_point, in_header.frame_id,
                                            cluster_transformed.min_point);
        transform_listener_->transformPoint(in_target_frame, ros::Time(),
                                            i->max_point, in_header.frame_id,
                                            cluster_transformed.max_point);
        transform_listener_->transformPoint(in_target_frame, ros::Time(),
                                            i->avg_point, in_header.frame_id,
                                            cluster_transformed.avg_point);
        transform_listener_->transformPoint(
            in_target_frame, ros::Time(), i->centroid_point, in_header.frame_id,
            cluster_transformed.centroid_point);

        cluster_transformed.dimensions = i->dimensions;
        cluster_transformed.eigen_values = i->eigen_values;
        cluster_transformed.eigen_vectors = i->eigen_vectors;

        cluster_transformed.convex_hull = i->convex_hull;
        cluster_transformed.bounding_box.pose.position =
            i->bounding_box.pose.position;
        if (pose_estimation_) {
          cluster_transformed.bounding_box.pose.orientation =
              i->bounding_box.pose.orientation;
        } else {
          cluster_transformed.bounding_box.pose.orientation.w = initial_quat_w_;
        }
        clusters_transformed.clusters.push_back(cluster_transformed);
      } catch (tf::TransformException &ex) {
        ROS_ERROR("publishCloudClusters: %s", ex.what());
      }
    }
    in_publisher->publish(clusters_transformed);
    publishDetectedObjects(clusters_transformed);
  } else {
    in_publisher->publish(in_clusters);
    publishDetectedObjects(in_clusters);
  }
}

void EuclideanClusterDetector::publishCentroids(
    const ros::Publisher *in_publisher,
    const autoware_msgs::Centroids &in_centroids,
    const std::string &in_target_frame, const std_msgs::Header &in_header) {
  if (in_target_frame != in_header.frame_id) {
    autoware_msgs::Centroids centroids_transformed;
    centroids_transformed.header = in_header;
    centroids_transformed.header.frame_id = in_target_frame;
    for (auto i = centroids_transformed.points.begin();
         i != centroids_transformed.points.end(); i++) {
      geometry_msgs::PointStamped centroid_in, centroid_out;
      centroid_in.header = in_header;
      centroid_in.point = *i;
      try {
        transform_listener_->transformPoint(in_target_frame, ros::Time(),
                                            centroid_in, in_header.frame_id,
                                            centroid_out);

        centroids_transformed.points.push_back(centroid_out.point);
      } catch (tf::TransformException &ex) {
        ROS_ERROR("publishCentroids: %s", ex.what());
      }
    }
    in_publisher->publish(centroids_transformed);
  } else {
    in_publisher->publish(in_centroids);
  }
}

void EuclideanClusterDetector::publishCloud(
    const ros::Publisher *in_publisher,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr) {
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
  cloud_msg.header = velodyne_header_;
  in_publisher->publish(cloud_msg);
}

void EuclideanClusterDetector::publishColorCloud(
    const ros::Publisher *in_publisher,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_to_publish_ptr) {
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
  cloud_msg.header = velodyne_header_;
  in_publisher->publish(cloud_msg);
}

void EuclideanClusterDetector::keepLanePoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
    float in_left_lane_threshold, float in_right_lane_threshold) {
  pcl::PointIndices::Ptr far_indices(new pcl::PointIndices);
  for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++) {
    pcl::PointXYZ current_point;
    current_point.x = in_cloud_ptr->points[i].x;
    current_point.y = in_cloud_ptr->points[i].y;
    current_point.z = in_cloud_ptr->points[i].z;

    if (current_point.y > (in_left_lane_threshold) ||
        current_point.y < -1.0 * in_right_lane_threshold) {
      far_indices->indices.push_back(i);
    }
  }
  out_cloud_ptr->points.clear();
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(in_cloud_ptr);
  extract.setIndices(far_indices);
  extract.setNegative(
      true); // true removes the indices, false leaves only the indices
  extract.filter(*out_cloud_ptr);
}

#ifdef GPU_CLUSTERING

std::vector<ClusterPtr> EuclideanClusterDetector::clusterAndColorGpu(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
    autoware_msgs::Centroids &in_out_centroids,
    double in_max_cluster_distance) {
  std::vector<ClusterPtr> clusters;

  // Convert input point cloud to vectors of x, y, and z

  int size = in_cloud_ptr->points.size();

  if (size == 0)
    return clusters;

  float *tmp_x, *tmp_y, *tmp_z;

  tmp_x = (float *)malloc(sizeof(float) * size);
  tmp_y = (float *)malloc(sizeof(float) * size);
  tmp_z = (float *)malloc(sizeof(float) * size);

  for (int i = 0; i < size; i++) {
    pcl::PointXYZ tmp_point = in_cloud_ptr->at(i);

    tmp_x[i] = tmp_point.x;
    tmp_y[i] = tmp_point.y;
    tmp_z[i] = tmp_point.z;
  }

  GpuEuclideanCluster gecl_cluster;

  gecl_cluster.setInputPoints(tmp_x, tmp_y, tmp_z, size);
  gecl_cluster.setThreshold(in_max_cluster_distance);
  gecl_cluster.setMinClusterPts(cluster_size_min_);
  gecl_cluster.setMaxClusterPts(cluster_size_max_);
  gecl_cluster.extractClusters();
  std::vector<GpuEuclideanCluster::GClusterIndex> cluster_indices =
      gecl_cluster.getOutput();

  unsigned int k = 0;

  for (auto it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
    ClusterPtr cluster(new Cluster());
    cluster->SetCloud(in_cloud_ptr, it->points_in_cluster, velodyne_header_, k,
                      (int)colors_[k].val[0], (int)colors_[k].val[1],
                      (int)colors_[k].val[2], "", pose_estimation_);
    clusters.push_back(cluster);

    k++;
  }

  free(tmp_x);
  free(tmp_y);
  free(tmp_z);

  return clusters;
}

#endif

std::vector<ClusterPtr> EuclideanClusterDetector::clusterAndColor(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
    autoware_msgs::Centroids &in_out_centroids,
    double in_max_cluster_distance) {
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);

  // create 2d pc
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*in_cloud_ptr, *cloud_2d);
  // make it flat
  for (size_t i = 0; i < cloud_2d->points.size(); i++) {
    cloud_2d->points[i].z = 0;
  }

  if (cloud_2d->points.size() > 0)
    tree->setInputCloud(cloud_2d);

  std::vector<pcl::PointIndices> cluster_indices;

  // perform clustering on 2d cloud
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(in_max_cluster_distance); //
  ec.setMinClusterSize(cluster_size_min_);
  ec.setMaxClusterSize(cluster_size_max_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_2d);
  ec.extract(cluster_indices);
  // use indices on 3d cloud

  /////////////////////////////////
  //---  3. Color clustered points
  /////////////////////////////////
  unsigned int k = 0;
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cluster (new
  // pcl::PointCloud<pcl::PointXYZRGB>);

  std::vector<ClusterPtr> clusters;
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new
  // pcl::PointCloud<pcl::PointXYZRGB>);//coord + color cluster
  for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
    ClusterPtr cluster(new Cluster());
    cluster->SetCloud(in_cloud_ptr, it->indices, velodyne_header_, k,
                      (int)colors_[k].val[0], (int)colors_[k].val[1],
                      (int)colors_[k].val[2], "", pose_estimation_);
    clusters.push_back(cluster);

    k++;
  }
  // std::cout << "Clusters: " << k << std::endl;
  return clusters;
}

void EuclideanClusterDetector::checkClusterMerge(
    size_t in_cluster_id, std::vector<ClusterPtr> &in_clusters,
    std::vector<bool> &in_out_visited_clusters,
    std::vector<size_t> &out_merge_indices, double in_merge_threshold) {
  // std::cout << "checkClusterMerge" << std::endl;
  pcl::PointXYZ point_a = in_clusters[in_cluster_id]->GetCentroid();
  for (size_t i = 0; i < in_clusters.size(); i++) {
    if (i != in_cluster_id && !in_out_visited_clusters[i]) {
      pcl::PointXYZ point_b = in_clusters[i]->GetCentroid();
      double distance =
          sqrt(pow(point_b.x - point_a.x, 2) + pow(point_b.y - point_a.y, 2));
      if (distance <= in_merge_threshold) {
        in_out_visited_clusters[i] = true;
        out_merge_indices.push_back(i);
        // std::cout << "Merging " << in_cluster_id << " with " << i << " dist:"
        // << distance << std::endl;
        checkClusterMerge(i, in_clusters, in_out_visited_clusters,
                          out_merge_indices, in_merge_threshold);
      }
    }
  }
}

void EuclideanClusterDetector::mergeClusters(
    const std::vector<ClusterPtr> &in_clusters,
    std::vector<ClusterPtr> &out_clusters, std::vector<size_t> in_merge_indices,
    const size_t &current_index, std::vector<bool> &in_out_merged_clusters) {
  // std::cout << "mergeClusters:" << in_merge_indices.size() << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB> sum_cloud;
  pcl::PointCloud<pcl::PointXYZ> mono_cloud;
  ClusterPtr merged_cluster(new Cluster());
  for (size_t i = 0; i < in_merge_indices.size(); i++) {
    sum_cloud += *(in_clusters[in_merge_indices[i]]->GetCloud());
    in_out_merged_clusters[in_merge_indices[i]] = true;
  }
  std::vector<int> indices(sum_cloud.points.size(), 0);
  for (size_t i = 0; i < sum_cloud.points.size(); i++) {
    indices[i] = i;
  }

  if (sum_cloud.points.size() > 0) {
    pcl::copyPointCloud(sum_cloud, mono_cloud);
    merged_cluster->SetCloud(
        mono_cloud.makeShared(), indices, velodyne_header_, current_index,
        (int)colors_[current_index].val[0], (int)colors_[current_index].val[1],
        (int)colors_[current_index].val[2], "", pose_estimation_);
    out_clusters.push_back(merged_cluster);
  }
}

void EuclideanClusterDetector::checkAllForMerge(
    std::vector<ClusterPtr> &in_clusters, std::vector<ClusterPtr> &out_clusters,
    float in_merge_threshold) {
  // std::cout << "checkAllForMerge" << std::endl;
  std::vector<bool> visited_clusters(in_clusters.size(), false);
  std::vector<bool> merged_clusters(in_clusters.size(), false);
  size_t current_index = 0;
  for (size_t i = 0; i < in_clusters.size(); i++) {
    if (!visited_clusters[i]) {
      visited_clusters[i] = true;
      std::vector<size_t> merge_indices;
      checkClusterMerge(i, in_clusters, visited_clusters, merge_indices,
                        in_merge_threshold);
      mergeClusters(in_clusters, out_clusters, merge_indices, current_index++,
                    merged_clusters);
    }
  }
  for (size_t i = 0; i < in_clusters.size(); i++) {
    // check for clusters not merged, add them to the output
    if (!merged_clusters[i]) {
      out_clusters.push_back(in_clusters[i]);
    }
  }

  // ClusterPtr cluster(new Cluster());
}

void EuclideanClusterDetector::segmentByDistance(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
    autoware_msgs::Centroids &in_out_centroids,
    autoware_msgs::CloudClusterArray &in_out_clusters) {
  // cluster the pointcloud according to the distance of the points using
  // different thresholds (not only one for the entire pc) in this way, the
  // points farther in the pc will also be clustered

  // 0 => 0-15m d=0.5
  // 1 => 15-30 d=1
  // 2 => 30-45 d=1.6
  // 3 => 45-60 d=2.1
  // 4 => >60   d=2.6

  std::vector<ClusterPtr> all_clusters;

  if (!use_multiple_thres_) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZ>);

    for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++) {
      pcl::PointXYZ current_point;
      current_point.x = in_cloud_ptr->points[i].x;
      current_point.y = in_cloud_ptr->points[i].y;
      current_point.z = in_cloud_ptr->points[i].z;

      cloud_ptr->points.push_back(current_point);
    }
#ifdef GPU_CLUSTERING
    if (use_gpu_) {
      all_clusters = clusterAndColorGpu(cloud_ptr, out_cloud_ptr,
                                        in_out_centroids, clustering_distance_);
    } else {
      all_clusters = clusterAndColor(cloud_ptr, out_cloud_ptr, in_out_centroids,
                                     clustering_distance_);
    }
#else
    all_clusters = clusterAndColor(cloud_ptr, out_cloud_ptr, in_out_centroids,
                                   clustering_distance_);
#endif
  } else {
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_segments_array(5);
    for (unsigned int i = 0; i < cloud_segments_array.size(); i++) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(
          new pcl::PointCloud<pcl::PointXYZ>);
      cloud_segments_array[i] = tmp_cloud;
    }

    for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++) {
      pcl::PointXYZ current_point;
      current_point.x = in_cloud_ptr->points[i].x;
      current_point.y = in_cloud_ptr->points[i].y;
      current_point.z = in_cloud_ptr->points[i].z;

      float origin_distance =
          sqrt(pow(current_point.x, 2) + pow(current_point.y, 2));

      if (origin_distance < clustering_ranges_[0]) {
        cloud_segments_array[0]->points.push_back(current_point);
      } else if (origin_distance < clustering_ranges_[1]) {
        cloud_segments_array[1]->points.push_back(current_point);

      } else if (origin_distance < clustering_ranges_[2]) {
        cloud_segments_array[2]->points.push_back(current_point);

      } else if (origin_distance < clustering_ranges_[3]) {
        cloud_segments_array[3]->points.push_back(current_point);

      } else {
        cloud_segments_array[4]->points.push_back(current_point);
      }
    }

    std::vector<ClusterPtr> local_clusters;
    for (unsigned int i = 0; i < cloud_segments_array.size(); i++) {
#ifdef GPU_CLUSTERING
      if (use_gpu_) {
        local_clusters =
            clusterAndColorGpu(cloud_segments_array[i], out_cloud_ptr,
                               in_out_centroids, clustering_distances_[i]);
      } else {
        local_clusters =
            clusterAndColor(cloud_segments_array[i], out_cloud_ptr,
                            in_out_centroids, clustering_distances_[i]);
      }
#else
      local_clusters =
          clusterAndColor(cloud_segments_array[i], out_cloud_ptr,
                          in_out_centroids, clustering_distances_[i]);
#endif
      all_clusters.insert(all_clusters.end(), local_clusters.begin(),
                          local_clusters.end());
    }
  }

  // Clusters can be merged or checked in here
  //....
  // check for mergable clusters
  std::vector<ClusterPtr> mid_clusters;
  std::vector<ClusterPtr> final_clusters;

  if (all_clusters.size() > 0)
    checkAllForMerge(all_clusters, mid_clusters, cluster_merge_threshold_);
  else
    mid_clusters = all_clusters;

  if (mid_clusters.size() > 0)
    checkAllForMerge(mid_clusters, final_clusters, cluster_merge_threshold_);
  else
    final_clusters = mid_clusters;

  // Get final PointCloud to be published
  for (unsigned int i = 0; i < final_clusters.size(); i++) {
    *out_cloud_ptr = *out_cloud_ptr + *(final_clusters[i]->GetCloud());

    jsk_recognition_msgs::BoundingBox bounding_box =
        final_clusters[i]->GetBoundingBox();
    geometry_msgs::PolygonStamped polygon = final_clusters[i]->GetPolygon();
    jsk_rviz_plugins::Pictogram pictogram_cluster;
    pictogram_cluster.header = velodyne_header_;

    // PICTO
    pictogram_cluster.mode = pictogram_cluster.STRING_MODE;
    pictogram_cluster.pose.position.x = final_clusters[i]->GetMaxPoint().x;
    pictogram_cluster.pose.position.y = final_clusters[i]->GetMaxPoint().y;
    pictogram_cluster.pose.position.z = final_clusters[i]->GetMaxPoint().z;
    tf::Quaternion quat(0.0, -0.7, 0.0, 0.7);
    tf::quaternionTFToMsg(quat, pictogram_cluster.pose.orientation);
    pictogram_cluster.size = 4;
    std_msgs::ColorRGBA color;
    color.a = 1;
    color.r = 1;
    color.g = 1;
    color.b = 1;
    pictogram_cluster.color = color;
    pictogram_cluster.character = std::to_string(i);
    // PICTO

    // pcl::PointXYZ min_point = final_clusters[i]->GetMinPoint();
    // pcl::PointXYZ max_point = final_clusters[i]->GetMaxPoint();
    pcl::PointXYZ center_point = final_clusters[i]->GetCentroid();
    geometry_msgs::Point centroid;
    centroid.x = center_point.x;
    centroid.y = center_point.y;
    centroid.z = center_point.z;
    bounding_box.header = velodyne_header_;
    polygon.header = velodyne_header_;

    if (final_clusters[i]->IsValid()) {

      in_out_centroids.points.push_back(centroid);

      autoware_msgs::CloudCluster cloud_cluster;
      final_clusters[i]->ToROSMessage(velodyne_header_, cloud_cluster);
      in_out_clusters.clusters.push_back(cloud_cluster);
    }
  }
}

void EuclideanClusterDetector::removeFloor(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_nofloor_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_onlyfloor_cloud_ptr,
    float in_max_height, float in_floor_max_angle) {

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setAxis(Eigen::Vector3f(0, 0, 1));
  seg.setEpsAngle(in_floor_max_angle);

  seg.setDistanceThreshold(in_max_height); // floor distance
  seg.setOptimizeCoefficients(true);
  seg.setInputCloud(in_cloud_ptr);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0) {
    std::cout << "Could not estimate a planar model for the given dataset."
              << std::endl;
  }

  // REMOVE THE FLOOR FROM THE CLOUD
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(in_cloud_ptr);
  extract.setIndices(inliers);
  extract.setNegative(
      true); // true removes the indices, false leaves only the indices
  extract.filter(*out_nofloor_cloud_ptr);

  // EXTRACT THE FLOOR FROM THE CLOUD
  extract.setNegative(
      false); // true removes the indices, false leaves only the indices
  extract.filter(*out_onlyfloor_cloud_ptr);
}

void EuclideanClusterDetector::downsampleCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
    float in_leaf_size) {
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(in_cloud_ptr);
  sor.setLeafSize((float)in_leaf_size, (float)in_leaf_size,
                  (float)in_leaf_size);
  sor.filter(*out_cloud_ptr);
}

void EuclideanClusterDetector::clipCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
    float in_min_height, float in_max_height) {
  out_cloud_ptr->points.clear();
  for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++) {
    if (in_cloud_ptr->points[i].z >= in_min_height &&
        in_cloud_ptr->points[i].z <= in_max_height) {
      out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
    }
  }
}

void EuclideanClusterDetector::differenceNormalsSegmentation(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr) {
  float small_scale = 0.5;
  float large_scale = 2.0;
  float angle_threshold = 0.5;
  pcl::search::Search<pcl::PointXYZ>::Ptr tree;
  if (in_cloud_ptr->isOrganized()) {
    tree.reset(new pcl::search::OrganizedNeighbor<pcl::PointXYZ>());
  } else {
    tree.reset(new pcl::search::KdTree<pcl::PointXYZ>(false));
  }

  // Set the input pointcloud for the search tree
  tree->setInputCloud(in_cloud_ptr);

  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> normal_estimation;
  // pcl::gpu::NormalEstimation<pcl::PointXYZ, pcl::PointNormal>
  // normal_estimation;
  normal_estimation.setInputCloud(in_cloud_ptr);
  normal_estimation.setSearchMethod(tree);

  normal_estimation.setViewPoint(std::numeric_limits<float>::max(),
                                 std::numeric_limits<float>::max(),
                                 std::numeric_limits<float>::max());

  pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale(
      new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale(
      new pcl::PointCloud<pcl::PointNormal>);

  normal_estimation.setRadiusSearch(small_scale);
  normal_estimation.compute(*normals_small_scale);

  normal_estimation.setRadiusSearch(large_scale);
  normal_estimation.compute(*normals_large_scale);

  pcl::PointCloud<pcl::PointNormal>::Ptr diffnormals_cloud(
      new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal>(*in_cloud_ptr,
                                                       *diffnormals_cloud);

  // Create DoN operator
  pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal,
                                     pcl::PointNormal>
      diffnormals_estimator;
  diffnormals_estimator.setInputCloud(in_cloud_ptr);
  diffnormals_estimator.setNormalScaleLarge(normals_large_scale);
  diffnormals_estimator.setNormalScaleSmall(normals_small_scale);

  diffnormals_estimator.initCompute();

  diffnormals_estimator.computeFeature(*diffnormals_cloud);

  pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond(
      new pcl::ConditionOr<pcl::PointNormal>());
  range_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(
      new pcl::FieldComparison<pcl::PointNormal>(
          "curvature", pcl::ComparisonOps::GT, angle_threshold)));
  // Build the filter
  pcl::ConditionalRemoval<pcl::PointNormal> cond_removal;
  cond_removal.setCondition(range_cond);
  cond_removal.setInputCloud(diffnormals_cloud);

  pcl::PointCloud<pcl::PointNormal>::Ptr diffnormals_cloud_filtered(
      new pcl::PointCloud<pcl::PointNormal>);

  // Apply filter
  cond_removal.filter(*diffnormals_cloud_filtered);

  pcl::copyPointCloud<pcl::PointNormal, pcl::PointXYZ>(*diffnormals_cloud,
                                                       *out_cloud_ptr);
}

void EuclideanClusterDetector::removePointsUpTo(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
    const double in_distance) {
  out_cloud_ptr->points.clear();
  for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++) {
    float origin_distance = sqrt(pow(in_cloud_ptr->points[i].x, 2) +
                                 pow(in_cloud_ptr->points[i].y, 2));
    if (origin_distance > in_distance) {
      out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
    }
  }
}

void EuclideanClusterDetector::downsamplePoints(const Mat &src, Mat &dst,
                                                size_t count) {
  CV_Assert(count >= 2);
  CV_Assert(src.cols == 1 || src.rows == 1);
  CV_Assert(src.total() >= count);
  CV_Assert(src.type() == CV_8UC3);

  dst.create(1, (int)count, CV_8UC3);
  // TODO: optimize by exploiting symmetry in the distance matrix
  Mat dists((int)src.total(), (int)src.total(), CV_32FC1, Scalar(0));
  if (dists.empty())
    std::cerr << "Such big matrix cann't be created." << std::endl;

  for (int i = 0; i < dists.rows; i++) {
    for (int j = i; j < dists.cols; j++) {
      float dist =
          (float)norm(src.at<Point3_<uchar>>(i) - src.at<Point3_<uchar>>(j));
      dists.at<float>(j, i) = dists.at<float>(i, j) = dist;
    }
  }

  double maxVal;
  Point maxLoc;
  minMaxLoc(dists, 0, &maxVal, 0, &maxLoc);

  dst.at<Point3_<uchar>>(0) = src.at<Point3_<uchar>>(maxLoc.x);
  dst.at<Point3_<uchar>>(1) = src.at<Point3_<uchar>>(maxLoc.y);

  Mat activedDists(0, dists.cols, dists.type());
  Mat candidatePointsMask(1, dists.cols, CV_8UC1, Scalar(255));
  activedDists.push_back(dists.row(maxLoc.y));
  candidatePointsMask.at<uchar>(0, maxLoc.y) = 0;

  for (size_t i = 2; i < count; i++) {
    activedDists.push_back(dists.row(maxLoc.x));
    candidatePointsMask.at<uchar>(0, maxLoc.x) = 0;

    Mat minDists;
    reduce(activedDists, minDists, 0, 3);
    minMaxLoc(minDists, 0, &maxVal, 0, &maxLoc, candidatePointsMask);
    dst.at<Point3_<uchar>>((int)i) = src.at<Point3_<uchar>>(maxLoc.x);
  }
}

void EuclideanClusterDetector::generateColors(std::vector<Scalar> &colors,
                                              size_t count, size_t factor) {
  if (count < 1)
    return;

  colors.resize(count);

  if (count == 1) {
    colors[0] = Scalar(0, 0, 255); // red
    return;
  }
  if (count == 2) {
    colors[0] = Scalar(0, 0, 255); // red
    colors[1] = Scalar(0, 255, 0); // green
    return;
  }

  // Generate a set of colors in RGB space. A size of the set is severel times
  // (=factor) larger then the needed count of colors.
  Mat bgr(1, (int)(count * factor), CV_8UC3);
  randu(bgr, 0, 256);

  // Convert the colors set to Lab space.
  // Distances between colors in this space correspond a human perception.
  Mat lab;
  cvtColor(bgr, lab, cv::COLOR_BGR2Lab);

  // Subsample colors from the generated set so that
  // to maximize the minimum distances between each other.
  // Douglas-Peucker algorithm is used for this.
  Mat lab_subset;
  downsamplePoints(lab, lab_subset, count);

  // Convert subsampled colors back to RGB
  Mat bgr_subset;
  cvtColor(lab_subset, bgr_subset, cv::COLOR_Lab2BGR);

  CV_Assert(bgr_subset.total() == count);
  for (size_t i = 0; i < count; i++) {
    Point3_<uchar> c = bgr_subset.at<Point3_<uchar>>((int)i);
    colors[i] = Scalar(c.x, c.y, c.z);
  }
}

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "lidar_euclidean_cluster_detect");

  EuclideanClusterDetector detector;

  return 0;
}
