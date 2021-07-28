#include "laser_detection/laser_tracking.h"

void LaserTracking::UpdateRoboState() {
  tf::StampedTransform transform_pose_;
  try {
    tf_listener_.lookupTransform("map", "base_link", ros::Time(0), transform_pose_);
    robo_x_ = transform_pose_.getOrigin().x();
    robo_y_ = transform_pose_.getOrigin().y();
    double roll, pitch;
    transform_pose_.getBasis().getEulerYPR(robo_yaw_, pitch, roll);
    updated_robo_state_ = true;
  } catch (tf::TransformException ex) { ROS_WARN("Cannot find the robo pose in map, localization node maybe die"); }
}
void LaserTracking::UpdatePassedObstacle(Point p) {
  if (passed_obstacles.empty()) {
    printf("Detecting dead area");
    passed_obstacles.push_back(p);
    double distance = findNearDistanceToPointVec(p, dead_area);
    if (distance < query_threshold_) {
      roborts_msgs::Obstacles srv;
      srv.request.x = p.x;
      srv.request.y = p.y;
      if (client_.call(srv)) { ROS_INFO("Send obstacle in dead area."); }
    }
  } else {
    Point p_t{};
    p_t.x = p.x;
    p_t.y = p.y;
    double near_distance = getDistance(passed_obstacles[0], p_t);
    int idx = 0;
    for (int i = 1; i < passed_obstacles.size(); i++) {
      double current_distance = getDistance(passed_obstacles[i], p_t);
      if (current_distance < near_distance) {
        near_distance = current_distance;
        idx = i;
      }
    }
    if (near_distance <= distance_threshold_) {
      passed_obstacles[idx].x = (passed_obstacles[idx].x + p.x) / 2;
      passed_obstacles[idx].y = (passed_obstacles[idx].y + p.y) / 2;
    } else {
      passed_obstacles.push_back(p);
      printf("Detecting dead area");
      double distance = findNearDistanceToPointVec(p, dead_area);
      if (distance < query_threshold_) {
        roborts_msgs::Obstacles srv;
        srv.request.x = p.x;
        srv.request.y = p.y;
        if (client_.call(srv)) { ROS_INFO("Send obstacle in dead area."); }
      }
    }
  }
}

void LaserTracking::UpdateObstacles(std::vector<Point> laser_obstacles) {
  std::vector<Point> active_obstacles;
  for (auto &laser_obstacle : laser_obstacles) {
    Point point{};
    point.x = laser_obstacle.x;
    point.y = laser_obstacle.y;
    if (updated_robo_state_) {
      UpdatePassedObstacle(point);
      active_obstacles.push_back(point);
    }
  }

  std::vector<Point> update_active_obstacles;
  for (auto &active_obstacle : active_obstacles) {
    double distance = findNearDistanceToPointVec(active_obstacle, passed_obstacles);
    if (distance < distance_threshold_) { update_active_obstacles.push_back(active_obstacle); }
  }
  std::cout << "Passed obstacle number: " << passed_obstacles.size() << std::endl;
  printObstacles(passed_obstacles);
  PublishPassedObstacles(passed_obstacles);

  std::cout << "Active obstacle number: " << update_active_obstacles.size() << std::endl;
  printObstacles(update_active_obstacles);
  // PublishActivePoint(update_active_obstacles);
}

void LaserTracking::ReceiveObstacles(const geometry_msgs::PoseArray &msg) {
  UpdateRoboState();
  std::vector<Point> received_obstacles;
  std::vector<Point> obstacles_to_update;
  if (!msg.poses.empty()) {
    received_obstacles.clear();
    for (const auto &pose : msg.poses) {
      Point obstacle{};
      obstacle.x = pose.position.x;
      obstacle.y = pose.position.y;
      obstacles_to_update.push_back(obstacle);
    }
  }
  printf("===============================\n");
  UpdateObstacles(obstacles_to_update);
}

void LaserTracking::PublishPassedObstacles(std::vector<Point> points) {
  markers.points.clear();
  markers.action = visualization_msgs::Marker::ADD;
  for (auto &point : points) {
    geometry_msgs::Point pt;
    pt.x = point.x;
    pt.y = point.y;
    pt.z = 0.5;
    markers.points.push_back(pt);
  }
  marker_pub_.publish(markers);
  markers.action = visualization_msgs::Marker::DELETEALL;
}
// bool LaserTracking::QueryObstacle(roborts_msgs::Obstacle::Request &req, roborts_msgs::Obstacle::Response &res) {
//   try {
//     Point pt{};
//     pt.x = req.x;
//     pt.y = req.y;
//     double distance = findNearDistanceToPointVec(pt, passed_obstacles);
//     if (distance < query_threshold_) {
//       res.is_exist = true;
//       Point target = findNearToPointVec(pt, passed_obstacles);
//       res.pose.position.x = target.x;
//       res.pose.position.y = target.y;
//       res.pose.position.z = 0.2;
//     } else {
//       res.is_exist = false;
//     }
//     return true;
//   } catch (ros::Exception e) { return false; }
// }

int main(int argc, char **argv) {
  ros::init(argc, argv, "laser_tracking");
  ros::NodeHandle nh;
  LaserTracking laser;
  ros::spin();
}
