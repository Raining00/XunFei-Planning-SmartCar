#ifndef LASER_TRACKING_H
#define LASER_TRACKING_H
#include <geometry_msgs/PoseArray.h>
#include <signal.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Core>
#include <list>

#include "nav_msgs/Path.h"
#include "roborts_msgs/Obstacles.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"
struct Point {
  double x;
  double y;
  int type;
};
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
inline double getDistance(Point p, Point tp) { return sqrt((p.x - tp.x) * (p.x - tp.x) + (p.y - tp.y) * (p.y - tp.y)); }

void printObstacles(std::vector<Point> points) {
  std::cout << "===============================" << std::endl;
  for (auto &point : points) { std::cout << "x: " << point.x << " y: " << point.y << std::endl; }
  std::cout << "===============================" << std::endl;
}
class LaserTracking {
 public:
  LaserTracking() {
    robo_x_ = 0.0;
    robo_y_ = 0.0;
    robo_yaw_ = 0.0;
    updated_robo_state_ = false;
    obstacles_sub_ = nh_.subscribe("/possible_obstacle_marker", 1, &LaserTracking::ReceiveObstacles, this);
    vision_sub_ = nh_.subscribe("/visualization_globalobj_marker", 1, &LaserTracking::ReceiveVision, this);
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

    nh_.param<double>("distance_threshold", tracking_threshold, 0.3);
    nh_.param<double>("query_threshold", query_threshold_, 0.5);
    nh_.param<bool>("only_blind_area", only_blind_area, true);
    nh_.param<bool>("publish_obstacles", publish_obstacles, true);

    nh_.getParam("/blind_area", blind_areas_xml);
    for (int i = 0; i < blind_areas_xml.size(); i += 1) {
      std::vector<Point> blind_area;
      for (int j = 0; j < blind_areas_xml[i].size(); j += 2) {
        Point point{};
        point.x = blind_areas_xml[i][j];
        point.y = blind_areas_xml[i][j + 1];
        blind_area.push_back(point);
      }
      blind_areas.push_back(blind_area);
    }

    nh_.getParam("/dead_area", dead_areas_xml);
    for (int i = 0; i < dead_areas_xml.size(); i += 1) {
      std::vector<Point> dead_area;
      for (int j = 0; j < dead_areas_xml[i].size(); j += 2) {
        Point point{};
        point.x = dead_areas_xml[i][j];
        point.y = dead_areas_xml[i][j + 1];
        dead_area.push_back(point);
      }
      dead_areas.push_back(dead_area);
    }
    nh_.getParam("/forbidden_area", forbidden_areas_xml);
    for (int i = 0; i < forbidden_areas_xml.size(); i += 1) {
      std::vector<Point> forbidden_area;
      for (int j = 0; j < forbidden_areas_xml[i].size(); j += 2) {
        Point point{};
        point.x = forbidden_areas_xml[i][j];
        point.y = forbidden_areas_xml[i][j + 1];
        forbidden_area.push_back(point);
      }
      forbidden_areas.push_back(forbidden_area);
    }
    nh_.getParam("/block_area", block_areas_xml);
    for (int i = 0; i < block_areas_xml.size(); i += 1) {
      std::vector<Point> block_area;
      for (int j = 0; j < block_areas_xml[i].size(); j += 2) {
        Point point{};
        point.x = block_areas_xml[i][j];
        point.y = block_areas_xml[i][j + 1];
        block_area.push_back(point);
      }
      block_areas.push_back(block_area);
    }
    ss_ = nh_.advertiseService("query_obstacle", &LaserTracking::QueryObstacle, this);
    client_ = nh_.serviceClient<roborts_msgs::Obstacles>("obstacles");
  }
  void ReceiveObstacles(const visualization_msgs::Marker &msg);
  void ReceiveVision(const visualization_msgs::Marker &msg);
  void PublishPassedObstacles(std::vector<Point> points);
  // robo pose
  void UpdateRoboState();

  void UpdateObstacles(std::vector<Point> laser_obstacles);
  void CallObstacle(Point p);
  void UpdatePassedObstacle(Point p);

  bool QueryObstacle(roborts_msgs::Obstacles::Request &req, roborts_msgs::Obstacles::Response &res);
  int GetArea(Point point, std::vector<std::vector<Point>> polygons);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber obstacles_sub_, vision_sub_;
  ros::Publisher marker_pub_;
  ros::ServiceServer ss_;
  ros::ServiceClient client_;

  double tracking_threshold, query_threshold_;

  // robo state
  tf::TransformListener tf_listener_;
  double robo_x_, robo_y_, robo_yaw_;
  bool updated_robo_state_;

  std::vector<std::vector<Point>> blind_areas;
  XmlRpc::XmlRpcValue blind_areas_xml;
  std::vector<std::vector<Point>> dead_areas;
  XmlRpc::XmlRpcValue dead_areas_xml;
  std::vector<std::vector<Point>> forbidden_areas;
  XmlRpc::XmlRpcValue forbidden_areas_xml;
  std::vector<std::vector<Point>> block_areas;
  XmlRpc::XmlRpcValue block_areas_xml;
  std::vector<int> passed_blind_areas;
  std::vector<Point> passed_obstacles;
  bool only_blind_area;
  bool publish_obstacles;
  visualization_msgs::Marker markers;
};
#endif // LASER_TRACKING_H
