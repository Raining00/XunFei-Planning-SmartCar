#ifndef LASER_DETECTION_H
#define LASER_DETECTION_H
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/GetMap.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/visualization/cloud_viewer.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <signal.h>
#include <visualization_msgs/Marker.h>

#include "laser_geometry/laser_geometry.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "pcl_ros/point_cloud.h"
#include "roborts_msgs/ArmorPos.h"
#include "roborts_msgs/ArmorsPos.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "zmq.hpp"

struct Point {
  float x;
  float y;
};
struct Pose {
  float x;
  float y;
  float vx;
  float vy;
};

float getPointDistance(Point p1, Point p2) { return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y)); }

void nearstPoint(Point p_in, std::vector<Point> p_vec, Point &p_out, float &min_dist) {
  min_dist = getPointDistance(p_in, p_vec[0]);
  int min_idx = 0;
  for (int i = 1; i < p_vec.size(); i++) {
    float current_d = getPointDistance(p_in, p_vec[i]);
    if (current_d < min_dist) {
      min_dist = current_d;
      min_idx = i;
    }
  }
  p_out = p_vec[min_idx];
}

class LaserDetection {
 public:
  LaserDetection() : _context(1), _clientSocket(_context, ZMQ_REQ) {
    _clientSocket.connect("tcp://192.168.1.127:5555");
    static_map_topic_ = "/static_map";
    map_frame_ = "/map";
    laser_frame_ = "base_laser_link";
    self_pose_.x = 0.0;
    self_pose_.y = 0.0;
    map_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    odom_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    LoadMap();
    std::string tf_prefix = tf::getPrefixParam(nh);
    std::string scan_frame = "scan";

    if (!tf_prefix.empty()) {
      scan_frame = "base_scan";
      laser_frame_ = tf::resolve(tf_prefix, laser_frame_);
    }
    laser_sub_ = nh.subscribe<sensor_msgs::LaserScan>(scan_frame, 1, &LaserDetection::UpdateLaser, this); // automatically add /scale
    //change_sub_ = nh.subscribe<geometry_msgs::Twist>("change_mode", 1, &LaserDetection::ChangeMode, this);
    seg_pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("dynamic_obstacle", 10);
    in_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("in_map_pc", 10);
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("/possible_obstacle_marker", 1);

    // preset marker property
    markers.header.frame_id = "/map";
    markers.header.stamp = ros::Time::now();
    markers.ns = "point";
    markers.action = visualization_msgs::Marker::ADD;
    markers.pose.orientation.w = 1.0;
    markers.type = visualization_msgs::Marker::POINTS;
    markers.scale.x = 0.1;
    markers.scale.y = 0.1;

    // node params
    nh.param<double>("segment_distance_threshold", segment_distance_threshold, 0.03);
    nh.param<double>("cluster_tolerance", cluster_tolerance, 0.1);
    nh.param<int>("min_cluster_size", min_cluster_size, 8);
    nh.param<double>("max_detection_threshold", max_detection_threshold, 1.4);
    nh.param<double>("min_detection_threshold", min_detection_threshold, 0.4);
    nh.param<bool>("show_inlier_pc", show_inlier_pc, true);
  }
  void LoadMap();
  void UpdateLaser(const sensor_msgs::LaserScanConstPtr &msg);
  void Detection(const pcl::PointCloud<pcl::PointXYZ>::Ptr &laser);
  void PublishSegmentPointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &seg_laser);
  void PublishInMapPointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &seg_laser);


 private:
  tf::TransformListener listener_;
  ros::NodeHandle nh;
  ros::Subscriber laser_sub_;
  ros::Subscriber change_sub_;
  ros::Publisher seg_pc_pub_, in_map_pub_, marker_pub_;
  Point self_pose_;

  std::string static_map_topic_, map_frame_, laser_frame_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud, odom_cloud;
  visualization_msgs::Marker markers;

  double segment_distance_threshold, change_segment_distance_threshold, cluster_tolerance, max_detection_threshold, min_detection_threshold;
  int min_cluster_size;
  bool show_inlier_pc;
  // zmq
  zmq::context_t _context;
  zmq::socket_t _clientSocket;
};
#endif // LASER_DETECTION_H
