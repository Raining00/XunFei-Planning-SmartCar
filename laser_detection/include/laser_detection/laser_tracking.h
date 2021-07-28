#ifndef LASER_TRACKING_H
#define LASER_TRACKING_H
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

#include <list>

#include "nav_msgs/Path.h"
#include "roborts_msgs/Obstacles.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"
struct Point {
  double x;
  double y;
};
std::vector<Point> dead_area{{1.88, 0.2}, {2.49, -0.85}, {2.94, -2.42}, {2.21, -3.54}, {1.04, -2.87}, {0.99, -2.43}};
double findNearDistanceToPointVec(Point p, std::vector<Point> p_vec) {
  if (p_vec.empty()) {
    return 100.0;
  } else {
    double near_distance = sqrt((p.x - p_vec[0].x) * (p.x - p_vec[0].x) + (p.y - p_vec[0].y) * (p.y - p_vec[0].y));
    for (int i = 1; i < p_vec.size(); i++) {
      double current_distance = sqrt((p.x - p_vec[i].x) * (p.x - p_vec[i].x) + (p.y - p_vec[i].y) * (p.y - p_vec[i].y));
      if (current_distance < near_distance) near_distance = current_distance;
    }
    return near_distance;
  }
}
Point findNearToPointVec(Point p, std::vector<Point> p_vec) {
  if (p_vec.empty()) {
    return p;
  } else {
    int near_index = 0;
    double near_distance = sqrt((p.x - p_vec[0].x) * (p.x - p_vec[0].x) + (p.y - p_vec[0].y) * (p.y - p_vec[0].y));
    for (int i = 1; i < p_vec.size(); i++) {
      double current_distance = sqrt((p.x - p_vec[i].x) * (p.x - p_vec[i].x) + (p.y - p_vec[i].y) * (p.y - p_vec[i].y));
      if (current_distance < near_distance) {
        near_distance = current_distance;
        near_index = i;
      }
    }
    return p_vec[near_index];
  }
}
double getDistance(Point p, Point tp) { return sqrt((p.x - tp.x) * (p.x - tp.x) + (p.y - tp.y) * (p.y - tp.y)); }

void printObstacles(std::vector<Point> points) {
  std::cout << "===============================" << std::endl;
  for (auto &point : points) { std::cout << "x: " << point.x << " y: " << point.y << std::endl; }
  std::cout << "===============================" << std::endl;
}
class LaserTracking {
 public:
  LaserTracking() {
    std::vector<float> invalid_armor;
    invalid_armor.push_back(-10);
    invalid_armor.push_back(-10);
    robo_x_ = 0.0;
    robo_y_ = 0.0;
    robo_yaw_ = 0.0;
    updated_robo_state_ = false;
    obstacles_sub_ = nh_.subscribe("obstacles", 1, &LaserTracking::ReceiveObstacles, this);

    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/passed_obstacle_marker", 1);

    // preset marker property
    markers.header.frame_id = "/map";
    markers.header.stamp = ros::Time::now();
    markers.ns = "point";
    markers.action = visualization_msgs::Marker::ADD;
    markers.pose.orientation.w = 1.0;
    markers.type = visualization_msgs::Marker::POINTS;
    markers.scale.x = 0.1;
    markers.scale.y = 0.1;
    std_msgs::ColorRGBA C;
    C.r = 0.0f;
    C.g = 0.0f;
    C.b = 0.0f;
    C.a = 1.0;
    markers.color = C;

    nh_.param<double>("distance_threshold", distance_threshold_, 0.3);
    nh_.param<double>("query_threshold", query_threshold_, 0.5);
    // ss_ = nh_.advertiseService("query_obstacle", &LaserTracking::QueryObstacle, this);
    client_ = nh_.serviceClient<roborts_msgs::Obstacles>("obstacles");
  }
  void ReceiveObstacles(const geometry_msgs::PoseArray &msg);
  void PublishPassedObstacles(std::vector<Point> points);
  // robo pose
  void UpdateRoboState();

  void UpdateObstacles(std::vector<Point> laser_obstacles);
  void UpdatePassedObstacle(Point p);

  // bool QueryObstacle(roborts_msgs::Obstacle::Request &req, roborts_msgs::Obstacle::Response &res);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber obstacles_sub_;
  ros::Publisher marker_pub_;
  // ros::ServiceServer ss_;
  ros::ServiceClient client_;

  double distance_threshold_, query_threshold_;

  // robo state
  tf::TransformListener tf_listener_;
  double robo_x_, robo_y_, robo_yaw_;
  bool updated_robo_state_;

  std::vector<Point> passed_obstacles;

  visualization_msgs::Marker markers;
};
#endif // LASER_TRACKING_H
