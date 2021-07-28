/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#ifndef ROBORTS_DECISION_BLACKBOARD_H
#define ROBORTS_DECISION_BLACKBOARD_H

#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// roborts_msgs
#include "roborts_msgs/RobotStatus.h"
#include "roborts_msgs/TwistAccel.h"

#include "io/io.h"
#include "../proto/decision.pb.h"
#include "costmap/costmap_interface.h"

// communication.h
#include "communication.h"

// wifi part
#include <unistd.h>
#include <thread>

namespace roborts_decision
{

  //阈值
  struct Threshold
  {
    float near_dist;
    float near_angle;
    float heat_upper_bound;
    float detect_dist;
  };
  struct DecisionInfoPool
  {
    std::string strategy;                    
    geometry_msgs::PoseStamped my_goal;
  };

  class Blackboard
  {
  public:
    typedef std::shared_ptr<Blackboard> Ptr;
    typedef roborts_costmap::CostmapInterface CostMap;
    typedef roborts_costmap::Costmap2D CostMap2D;
    explicit Blackboard(const std::string &proto_file_path)
    {

      //make_shread 返回一个制定类型的shared_ptr指针
      tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));
      //读取行为决策代价地图
      std::string map_path = ros::package::getPath("roborts_costmap") +
                             "/config/costmap_parameter_config_for_decision.prototxt";
      costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_,
                                               map_path);
      charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();

      costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();

      mark_id = 0;
      mark_able = false;

      // Enemy fake pose
      ros::NodeHandle rviz_nh("/move_base_simple");
      goal_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &Blackboard::GoalCallback, this);
      // node handle
      ros::NodeHandle nh;
      //更新运动i信息
      vel_acc_sub_ = nh.subscribe<roborts_msgs::TwistAccel>("cmd_vel_acc", 10, &Blackboard::VelAccCallback, this);
      //领取任务ID
      mark_sub = nh.subscribe<geometry_msgs::Twist>("markerid", 2, &Blackboard::MarkCallback, this);

      // pub
      cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10, this);
      std::string tf_prefix = tf::getPrefixParam(nh);
      pose_frameID = "base_link";
      if (!tf_prefix.empty())
      {
        pose_frameID = tf::resolve(tf_prefix, pose_frameID);
      }

      //加载决策配置文件
      roborts_decision::DecisionConfig decision_config;
      roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config);

      info.strategy = decision_config.strategy();

      // goal passport
      info.my_goal = InitMapPose();
    }

    ~Blackboard() = default;

    // vel call back
    void VelAccCallback(const roborts_msgs::TwistAccel::ConstPtr &msg)
    {
      my_vel_x = msg->twist.linear.x;
      my_vel_y = msg->twist.linear.y;
    }

    // Goal
    void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal)
    {
      new_goal_ = true;
      goal_ = *goal;
    }

    void MarkCallback(const geometry_msgs::Twist::ConstPtr &mark)
    {
      mark_id = mark->linear.x;
      if (mark_id > 2 || mark_id < 0)
        mark_able = false;
      else if (mark_id > 0 && mark_id < 3)
        mark_able = true;
    }

    geometry_msgs::PoseStamped GetGoal() const
    {
      return goal_;
    }

    geometry_msgs::PoseStamped GetMyGoal() const
    {
      return info.my_goal;
    }

    void SetMyGoal(geometry_msgs::PoseStamped goal)
    {
      info.my_goal = goal;
    }

    bool IsNewGoal()
    {
      if (new_goal_)
      {
        new_goal_ = false;
        return true;
      }
      else
      {
        return false;
      }
    }

    unsigned int GetMarkID() const
    {
      if (mark_able == true)
        return mark_id;
      else
        return 99;
    }

    bool GetMarkAble()const
    {
      return mark_able;
    }

    bool IsInStuckArea()
    {
      ROS_INFO("In Stuck Area, replanning.....");
      geometry_msgs::PoseStamped curPose = GetRobotMapPose();
      double yaw = tf::getYaw(curPose.pose.orientation);
      float cur_x = curPose.pose.position.x, cur_y = curPose.pose.position.y;
      float x[4], y[4];
      float deg_30 = 30 / 180 * 3.1415926;
      float d = 0.3;
      x[0] = std::max(0.0, std::min(7.99, cur_x + d * std::cos(yaw + deg_30)));
      y[0] = std::max(0.0, std::min(4.99, cur_y + d * std::sin(yaw + deg_30)));
      x[1] = std::max(0.0, std::min(7.99, cur_x + d * std::cos(yaw - deg_30)));
      y[1] = std::max(0.0, std::min(4.99, cur_y + d * std::sin(yaw - deg_30)));
      x[2] = std::max(0.0, std::min(7.99, cur_x - d * std::cos(yaw + deg_30)));
      y[2] = std::max(0.0, std::min(4.99, cur_y - d * std::sin(yaw + deg_30)));
      x[3] = std::max(0.0, std::min(7.99, cur_x - d * std::cos(yaw - deg_30)));
      y[3] = std::max(0.0, std::min(4.99, cur_y - d * std::sin(yaw - deg_30)));
      unsigned int mx, my;
      for (int i = 0; i < 4; i++)
      {
        GetCostMap2D()->World2Map(x[i], y[i], mx, my);
        if (GetCostMap2D()->GetCost(mx, my) < 253)
          return false;
      }

      // In stuck area

      const double sqrt2 = 1.414;
      double cost;
      double c_x, c_y;
      double int_x[] = {-d, -d / sqrt2, 0, d / sqrt2, d, d / sqrt2, 0, -d / sqrt2};
      double int_y[] = {0, -d / sqrt2, -d, d / sqrt2, 0, d / sqrt2, d, -d / sqrt2};
      int u_x, u_y;
      double acc_angle = 0.0;
      double cur_angle = 0.0;
      double count = 0;
      for (int i = 0; i <= 7; i++)
      {
        c_x = cur_x + int_x[i];
        c_y = cur_y + int_y[i];
        GetCostMap2D()->World2MapWithBoundary(c_x, c_y, u_x, u_y);
        cost = GetCostMap2D()->GetCost(u_x, u_y);
        if (cost >= 253)
        {
          cur_angle = std::atan2(int_y[i], int_x[i] + 0.00001);
          cur_angle = cur_angle > 0 ? cur_angle : cur_angle + 2 * 3.1415926;
          acc_angle += cur_angle;
          count++;
        }
      }
      acc_angle = acc_angle / count;
      acc_angle = acc_angle - 3.1415926;
      acc_angle = acc_angle < -3.1415926 ? acc_angle + 2 * 3.1415926 : acc_angle;

      // in case of rotate in a roll.
      c_x = cur_x + d * std::cos(acc_angle);
      c_y = cur_y + d * std::sin(acc_angle);
      GetCostMap2D()->World2MapWithBoundary(c_x, c_y, u_x, u_y);
      cost = GetCostMap2D()->GetCost(u_x, u_y);
      if (cost >= 253)
      {
        acc_angle = acc_angle > 0 ? acc_angle - 3.1415926 : acc_angle + 3.1415926;
      }
      //   printf("acc_angle: %f\n", acc_angle * 180 / 3.14);

      // get yaw
      acc_angle = acc_angle - yaw;
      double vx, vy;
      vx = cos(acc_angle);
      vy = sin(acc_angle);
      geometry_msgs::Twist tw;
      tw.linear.x = vx;
      tw.linear.y = vy;
      tw.linear.z = 0;
      // has sent cmd_vel
      cmd_vel_pub_.publish(tw);

      return true;
    }

    /*---------------------------------- Tools ------------------------------------------*/

    geometry_msgs::Quaternion GetRelativeQuaternion(const geometry_msgs::PoseStamped to, const geometry_msgs::PoseStamped from)
    {
      double dy, dx;
      dy = to.pose.position.y - from.pose.position.y;
      dx = to.pose.position.x - from.pose.position.x;
      double yaw = std::atan2(dy, dx);
      return tf::createQuaternionMsgFromYaw(yaw);
    }

    geometry_msgs::Quaternion GetRelativeQuaternion(const double to_x, const double to_y, const geometry_msgs::PoseStamped from)
    {
      double dx, dy;
      dy = to_y - from.pose.position.y;
      dx = to_x - from.pose.position.x;
      double yaw = std::atan2(dy, dx);
      return tf::createQuaternionMsgFromYaw(yaw);
    }

    double GetDistance(const geometry_msgs::PoseStamped &pose1,
                       const geometry_msgs::PoseStamped &pose2)
    {
      const geometry_msgs::Point point1 = pose1.pose.position;
      const geometry_msgs::Point point2 = pose2.pose.position;
      const double dx = point1.x - point2.x;
      const double dy = point1.y - point2.y;
      return std::sqrt(dx * dx + dy * dy);
    }

    double GetEulerDistance(const float x1, const float y1, const float x2, const float y2)
    {
      return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

    double GetAngle(const geometry_msgs::PoseStamped &pose1,
                    const geometry_msgs::PoseStamped &pose2)
    {
      const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation;
      const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
      tf::Quaternion rot1, rot2;
      tf::quaternionMsgToTF(quaternion1, rot1);
      tf::quaternionMsgToTF(quaternion2, rot2);
      return rot1.angleShortestPath(rot2);
    }

    geometry_msgs::PoseStamped Point2PoseStamped(Point point)
    {
      geometry_msgs::PoseStamped ps;
      ps.header.frame_id = "map";
      ps.pose.position.x = point.x();
      ps.pose.position.y = point.y();
      ps.pose.position.z = point.z();
      tf::Quaternion q = tf::createQuaternionFromRPY(point.roll(),
                                                     point.pitch(),
                                                     point.yaw());
      ps.pose.orientation.x = q.x();
      ps.pose.orientation.y = q.y();
      ps.pose.orientation.z = q.z();
      ps.pose.orientation.w = q.w();
      return ps;
    }

    const geometry_msgs::PoseStamped GetRobotMapPose()
    {
      UpdateRobotPose();
      return self_pose_;
    }

    const std::shared_ptr<CostMap> GetCostMap()
    {
      return costmap_ptr_;
    }

    const CostMap2D *GetCostMap2D()
    {
      return costmap_2d_;
    }

    const unsigned char *GetCharMap()
    {
      return charmap_;
    }

    DecisionInfoPool info;
    Threshold threshold;

  private:
    geometry_msgs::PoseStamped InitMapPose()
    {
      geometry_msgs::PoseStamped p;
      p.header.frame_id = "map";
      p.pose.position.x = -99;
      p.pose.position.y = -99;
      return p;
    }

    void UpdateRobotPose()
    {
      tf::Stamped<tf::Pose> robot_tf_pose;
      robot_tf_pose.setIdentity();

      robot_tf_pose.frame_id_ = pose_frameID;
      robot_tf_pose.stamp_ = ros::Time();
      try
      {
        geometry_msgs::PoseStamped robot_pose;
        tf::poseStampedTFToMsg(robot_tf_pose, robot_pose);
        tf_ptr_->transformPose("map", robot_pose, self_pose_);
      }
      catch (tf::LookupException &ex)
      {
        ROS_ERROR("Transform Error looking up robot pose: %s", ex.what());
      }
    }
    //! tf
    std::shared_ptr<tf::TransformListener> tf_ptr_;

    //! Enenmy detection
    ros::Subscriber vel_acc_sub_, goal_sub_, mark_sub;
    // publisher
    ros::Publisher cmd_vel_pub_;

    //code detect
    unsigned int mark_id;
    bool mark_able;

    //! Goal info
    geometry_msgs::PoseStamped goal_;
    bool new_goal_;

    //! cost map
    std::shared_ptr<CostMap> costmap_ptr_;
    CostMap2D *costmap_2d_;
    unsigned char *charmap_;

    //! robot map pose
    geometry_msgs::PoseStamped self_pose_, enemy_;
    // frame id
    std::string pose_frameID;

    // fusion
    double my_vel_x, my_vel_y;
  };
} //namespace roborts_decision
#endif //ROBORTS_DECISION_BLACKBOARD_H
