#include "laser_detection/laser_tracking.h"

int LaserTracking::GetArea(Point point, std::vector<std::vector<Point>> polygons) {
  int N = polygons.size();
  for (int M = 0; M < N; ++M) {
    int n = polygons[M].size();
    int i, j, c = 0;
    for (i = 0, j = n - 1; i < n; j = i++) {
      if (((polygons[M][i].y > point.y) != (polygons[M][j].y > point.y))
          && (point.x
              < (polygons[M][j].x - polygons[M][i].x) * (point.y - polygons[M][i].y) / (polygons[M][j].y - polygons[M][i].y) + polygons[M][i].x)) {
        c = !c;
      }
    }
    if (c == 1) return M;
    else
      continue;
  }
  return -1;
}

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
void LaserTracking::CallObstacle(Point p) {
  UpdateRoboState();
  Point pos{robo_x_, robo_y_, 0};
  if (GetArea(pos, block_areas) == -1) {
    ROS_WARN("Can't call service in this area, %f, %f.", robo_x_, robo_y_);
    return;
  }
  if (GetArea(p, dead_areas) != -1) {
    ROS_WARN("Obstacle in dead area.");
    return;
  }
  if (publish_obstacles) {
    if (only_blind_area) {
      ROS_WARN("Detected dead area, %f, %f, %f, %f.", p.x, p.y, robo_x_, robo_y_);
      if (GetArea(p, blind_areas) != -1) {
        roborts_msgs::Obstacles srv;
        srv.request.x = p.x;
        srv.request.y = p.y;
        srv.request.type = 1;
        if (client_.call(srv)) {
          ROS_WARN("Send obstacle in dead area.x: %f,y: %f.", p.x, p.y);
          if (srv.response.errorInfo) {
            passed_obstacles.push_back(p);
            ROS_WARN("Accepted");
          } else {
            ROS_WARN("Rejected");
          }
        } else {
          ROS_WARN("Error in calling service.x: %f,y: %f.", p.x, p.y);
        }
      }
    } else {
      ROS_WARN("Detected dead area, %f, %f, %f, %f.", p.x, p.y, robo_x_, robo_y_);
      roborts_msgs::Obstacles srv;
      srv.request.x = p.x;
      srv.request.y = p.y;
      srv.request.type = 1;
      if (client_.call(srv)) {
        ROS_WARN("Send obstacle in dead area.x: %f,y: %f.", p.x, p.y);
        if (srv.response.errorInfo) {
          passed_obstacles.push_back(p);
          ROS_WARN("Accepted");
        } else {
          ROS_WARN("Rejected");
        }
      } else {
        ROS_WARN("Error in calling service.x: %f,y: %f.", p.x, p.y);
      }
    }
  } else {
    passed_obstacles.push_back(p);
  }
}
void LaserTracking::UpdatePassedObstacle(Point p) {
  if (passed_obstacles.empty()) {
    CallObstacle(p);
  } else {
    double near_distance = getDistance(passed_obstacles[0], p);
    int idx = 0;
    for (int i = 1; i < passed_obstacles.size(); i++) {
      double current_distance = getDistance(passed_obstacles[i], p);
      if (current_distance < near_distance) {
        near_distance = current_distance;
        idx = i;
      }
    }
    if (near_distance <= tracking_threshold) {
      passed_obstacles[idx].x = (passed_obstacles[idx].x + p.x) / 2;
      passed_obstacles[idx].y = (passed_obstacles[idx].y + p.y) / 2;
    } else {
      CallObstacle(p);
    }
  }
}

void LaserTracking::UpdateObstacles(std::vector<Point> laser_obstacles) {
  std::vector<Point> active_obstacles;
  for (auto &laser_obstacle : laser_obstacles) {
    Point point{};
    point.x = laser_obstacle.x;
    point.y = laser_obstacle.y;
    point.type = 1;
    if (updated_robo_state_) {
      UpdatePassedObstacle(point);
      active_obstacles.push_back(point);
    }
  }
  PublishPassedObstacles(passed_obstacles);
}

void LaserTracking::ReceiveObstacles(const visualization_msgs::Marker &msg) {
  UpdateRoboState();
  std::vector<Point> received_obstacles;
  std::vector<Point> obstacles_to_update;
  if (!msg.points.empty()) {
    received_obstacles.clear();
    for (const auto &pt : msg.points) {
      // if (pt.z == 0.8 || pt.z == 0.6) continue;
      Point obstacle{};
      obstacle.x = pt.x;
      obstacle.y = pt.y;
      obstacles_to_update.push_back(obstacle);
    }
  }
  // printf("===============================\n");
  UpdateObstacles(obstacles_to_update);
}
void LaserTracking::ReceiveVision(const visualization_msgs::Marker &msg) {
  for (auto pt : msg.points) {
    Point p{};
    p.x = pt.x;
    p.y = pt.y;
    p.type = 2;
    if (passed_obstacles.empty()) {
      passed_obstacles.push_back(p);
    } else {
      double near_distance = getDistance(passed_obstacles[0], p);
      int idx = 0;
      for (int i = 1; i < passed_obstacles.size(); i++) {
        double current_distance = getDistance(passed_obstacles[i], p);
        if (current_distance < near_distance) {
          near_distance = current_distance;
          idx = i;
        }
      }
      if (near_distance <= tracking_threshold) {
        passed_obstacles[idx].x = (passed_obstacles[idx].x + p.x) / 2;
        passed_obstacles[idx].y = (passed_obstacles[idx].y + p.y) / 2;
      } else {
        passed_obstacles.push_back(p);
      }
    }
  }
}

void LaserTracking::PublishPassedObstacles(std::vector<Point> points) {
  markers.points.clear();
  markers.action = visualization_msgs::Marker::ADD;
  for (auto &point : points) {
    if (point.type == 1) {
      geometry_msgs::Point pt;
      pt.x = point.x;
      pt.y = point.y;
      pt.z = 0.7;
      markers.points.push_back(pt);
    }
  }
  marker_pub_.publish(markers);
  markers.action = visualization_msgs::Marker::DELETEALL;
}

bool LaserTracking::QueryObstacle(roborts_msgs::Obstacles::Request &req, roborts_msgs::Obstacles::Response &res) {
  try {
    Point pt{};
    pt.x = req.x;
    pt.y = req.y;
    if (GetArea(pt, blind_areas) != -1) return true;
  } catch (ros::Exception e) { return false; }
}
void SignalHandler(int sig) { ros::shutdown(); }
int main(int argc, char **argv) {
  ros::init(argc, argv, "laser_tracking");
  ros::NodeHandle nh;
  LaserTracking laser;
  signal(SIGTERM, SignalHandler);

  ros::spin();
}
