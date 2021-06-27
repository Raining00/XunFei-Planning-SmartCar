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

#include "roborts_msgs/ArmorDetectionAction.h"
// roborts_msgs 
#include "roborts_msgs/ArmorPos.h"
#include "roborts_msgs/ArmorsPos.h"
#include "roborts_msgs/ShooterCmd.h"
#include "roborts_msgs/RobotStatus.h"
#include "roborts_msgs/RobotHeat.h"
#include "roborts_msgs/GameStatus.h"
#include "roborts_msgs/RobotShoot.h"
#include "roborts_msgs/ProjectileSupply.h"
#include "roborts_msgs/SupplierStatus.h"
#include "roborts_msgs/BonusStatus.h"

// #include "roborts_msgs/AllyPose.h"
#include "roborts_msgs/FusionTarget.h"
#include "roborts_msgs/Target.h"
#include "roborts_msgs/DodgeMode.h"
#include "roborts_msgs/GimbalAngle.h"




#include "io/io.h"
#include "../proto/decision.pb.h"
#include "costmap/costmap_interface.h"

// communication.h
#include "communication.h"


// wifi part
#include <zmq.hpp>
#include <unistd.h>
#include <thread>



namespace roborts_decision{

//阈值
struct Threshold{
    float near_dist;
    float near_angle;
    float heat_upper_bound;
    float detect_dist;

};
//机器人行为信息池
struct DecisionInfoPool{
  int remaining_time;               //剩余时间
  int times_to_supply;      //
  int times_to_buff;            //增益次数？
  int game_status;          //当前比赛阶段
  int shoot_hz;                     //射击频率
  bool is_begin;                //是否开始  
  bool is_master;               //是否是master机器人（还有从机器人）
  bool team_blue;           //蓝队
  bool has_buff;                //是否有增益效果   
  bool has_ally;                //盟友是否存活（是否有盟友）
  bool has_my_enemy;    //自身是否检测到敌军
  bool has_ally_enemy;  //盟军是否检测到敌军    
  bool has_first_enemy;//是否检测到第一个敌人
  bool has_second_enemy;    //是否检测到第二个敌人     （Robomaster 为 2 vs  2 的对抗赛，敌军一共有两台装甲）
  bool can_shoot;                       //当前是否能射击
  bool can_dodge;               //当前能否躲开攻击
  bool is_supplying;            //是否提供支援
  bool is_shielding;            //是否处于提供掩护的状态
  bool got_last_enemy;  //是否找到最后一个敌军
  bool use_refree;              //裁判模式？？？

  // ally enemy part
  bool has_ally_first_enemy;    //盟军是否找到第一个敌人
  bool has_ally_second_enemy;   //盟军是否找到第二个敌人

  bool is_hitted;                       //是否正在遭受攻击
  bool is_chase;                //是否追击
  bool valid_camera_armor;      //相机装甲是否有效
  bool valid_front_camera_armor;        //前摄像头装甲是否有效
  bool valid_back_camera_armor;         //后置摄像头装甲是否有效
  int remain_bullet;                                    //剩余的子弹数量
  int remain_hp;                                            //剩余的血量
  int frequency;                                        //频率？？
  float speed;                                                     //行动速度
  int ally_remain_bullet;                               //盟军剩余子弹数目
  int ally_remain_hp;                                       //盟军剩余血量
  int heat;                                                                 //生命值
  double ally_dist;                                                 //盟军和自己之间的距离 
  double first_enemy_dist;                          //第一个敌人和自己距离  
  double second_enemy_dist;                     //第二个敌人和自己的距离
  float ally_yaw;                                                   //盟军当前航向角
  std::string strategy;                                         //策略，战略
  geometry_msgs::PoseStamped ally;          //盟军位置
  geometry_msgs::PoseStamped first_enemy;       //敌军一号位置
  geometry_msgs::PoseStamped second_enemy;      //敌军二号位置
  // ally enemy part
  geometry_msgs::PoseStamped ally_first_enemy; 
  geometry_msgs::PoseStamped ally_second_enemy;     
  // all goal
  geometry_msgs::PoseStamped my_goal;   //装甲自身的导航目标点
  geometry_msgs::PoseStamped ally_goal; //盟军的导航目标点

  geometry_msgs::PoseStamped last_enemy;//上一次发现敌人所在的目标点

  geometry_msgs::PoseStamped my_reload;//
  geometry_msgs::PoseStamped my_shield;
  geometry_msgs::PoseStamped opp_reload;
  geometry_msgs::PoseStamped opp_shield;
  geometry_msgs::PoseStamped start_position;//起始位置坐标
};


class Blackboard {
 public:
  typedef std::shared_ptr<Blackboard> Ptr;
  typedef roborts_costmap::CostmapInterface CostMap;
  typedef roborts_costmap::Costmap2D CostMap2D;
  explicit Blackboard(const std::string &proto_file_path):
        armor_detection_actionlib_client_("armor_detection_node_action", true),//armor装甲，护甲      装甲检测
        context_(1), masterSocket_(context_, ZMQ_REP), clientSocket_(context_, ZMQ_REQ){

        //make_shread 返回一个制定类型的shared_ptr指针
    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));
    //读取行为决策代价地图
    std::string map_path = ros::package::getPath("roborts_costmap") + \
      "/config/costmap_parameter_config_for_decision.prototxt";
    costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_,
                                             map_path);
    charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();
   
    costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();

    // Enemy fake pose
    ros::NodeHandle rviz_nh("/move_base_simple");
    enemy_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &Blackboard::GoalCallback, this);

    // node handle
    ros::NodeHandle nh;
    //当前状态（决策）
    robot_status_sub_ = nh.subscribe<roborts_msgs::RobotStatus>("robot_status", 10, &Blackboard::RobotStatusCallback, this);
    //加速度接受，更新运动i信息 
    vel_acc_sub_ = nh.subscribe<roborts_msgs::TwistAccel>("cmd_vel_acc", 10, &Blackboard::VelAccCallback, this);
    // pub
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10, this);

    // all frame ID update
    std::string tf_prefix = tf::getPrefixParam(nh);
    pose_frameID = "base_link";
    gimbal_frameID = "gimbal";
    if (!tf_prefix.empty()) {
        pose_frameID = tf::resolve(tf_prefix, pose_frameID);
        gimbal_frameID = tf::resolve(tf_prefix, gimbal_frameID);
    }
    
    //加载决策配置文件
    roborts_decision::DecisionConfig decision_config;
    roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config);
    //=====================================是否使用摄像头=======================================================//
    use_camera = decision_config.use_camera();
    if (use_camera){

      armor_detection_actionlib_client_.waitForServer();

      ROS_INFO("Armor detection module has been connected!");

    }
    //=========================================================================================================//

    // initialize setting----------------------------------------------------------
    info.remaining_time = 299;
    info.times_to_supply = 2;
    info.times_to_buff = 1;
    info.use_refree = decision_config.use_refree();
    info.is_begin = false;

    info.frequency = 0;
    info.speed = 0.0;
    info.strategy = decision_config.strategy();

    // goal passport
    info.my_goal = InitMapPose();

  }

  ~Blackboard() = default;

  // vel call back
  void VelAccCallback(const roborts_msgs::TwistAccel::ConstPtr &msg){
      my_vel_x = msg->twist.linear.x;
      my_vel_y = msg->twist.linear.y;
  }

  // Robot status call back
  void RobotStatusCallback(const roborts_msgs::RobotStatus::ConstPtr &rb_status){
      static unsigned int count=0;
      info.remain_hp = rb_status->remain_hp;
    //   info.team_blue = bool(rb_status->id == 13 || rb_status->id==14);
      if (info.remain_hp < remain_hp - 50){
          info.is_hitted = true;
          count = 0;
          remain_hp = info.remain_hp;
      }
      else{
          if (count < 300) count++;
          else  info.is_hitted = false;
      }

         
  }

  // Goal
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal){
    new_goal_ = true;
    goal_ = *goal;
  }

  geometry_msgs::PoseStamped GetGoal() const {
    return goal_;
  }

   geometry_msgs::PoseStamped GetMyGoal() const {
    return info.my_goal;
  }
  
  void SetMyGoal(geometry_msgs::PoseStamped goal){
      info.my_goal = goal;
  }
  
  bool IsNewGoal(){
    if(new_goal_){
      new_goal_ =  false;
      return true;
    } else{
      return false;
    }
  }

  bool hasOtherUnitsInThisArea(const geometry_msgs::PoseStamped target){
      std::vector<geometry_msgs::PoseStamped>  others;
      double dist;
      others.push_back(info.ally);
      if (info.has_first_enemy)
          others.push_back(info.first_enemy);
      if (info.has_second_enemy)
          others.push_back(info.second_enemy);
      if (info.has_ally_first_enemy)
          others.push_back(info.ally_first_enemy);
      if (info.has_ally_second_enemy)
          others.push_back(info.ally_second_enemy);
      for (int i=0; i<others.size(); i++){
         dist = GetDistance(target, others[i]);
         if (dist <= threshold.near_dist)
            return true;
      }
      return false;
      
  }


  bool IsInStuckArea(){
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
      for (int i=0; i<4; i++){
          GetCostMap2D()->World2Map(x[i], y[i], mx, my);
          if (GetCostMap2D()->GetCost(mx, my) <253)
            return false;

      }

      // In stuck area
      
      const double sqrt2 = 1.414;
      double cost;
      double c_x, c_y;
      double int_x[] = {-d, -d/sqrt2, 0, d/sqrt2, d, d/sqrt2, 0, -d/sqrt2};
      double int_y[] = {0, -d/sqrt2, -d, d/sqrt2, 0, d/sqrt2, d, -d/sqrt2};
      int u_x, u_y;
      double acc_angle = 0.0;
      double cur_angle = 0.0;
      double count = 0;
      for (int i=0; i<=7; i++){
            c_x = cur_x + int_x[i];
            c_y = cur_y + int_y[i];
            GetCostMap2D()->World2MapWithBoundary(c_x, c_y, u_x, u_y);
            cost = GetCostMap2D()->GetCost(u_x, u_y);
            if (cost >= 253){
                cur_angle = std::atan2(int_y[i], int_x[i] + 0.00001);
                cur_angle = cur_angle >0? cur_angle : cur_angle + 2 * 3.1415926;
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
      if (cost >=253){
          acc_angle = acc_angle >0 ? acc_angle - 3.1415926 : acc_angle + 3.1415926;
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

  geometry_msgs::Quaternion GetRelativeQuaternion(const geometry_msgs::PoseStamped to,const geometry_msgs::PoseStamped from){
      double dy, dx;
      dy = to.pose.position.y - from.pose.position.y;
      dx = to.pose.position.x - from.pose.position.x;
      double yaw = std::atan2(dy, dx);    
      return tf::createQuaternionMsgFromYaw(yaw);
  }

  geometry_msgs::Quaternion GetRelativeQuaternion(const double to_x, const double to_y, const geometry_msgs::PoseStamped from){
      double dx, dy;
      dy = to_y - from.pose.position.y;
      dx = to_x - from.pose.position.x;
      double yaw = std::atan2(dy, dx);
      return tf::createQuaternionMsgFromYaw(yaw);
  }

  double GetDistance(const geometry_msgs::PoseStamped &pose1,
                     const geometry_msgs::PoseStamped &pose2) {
    const geometry_msgs::Point point1 = pose1.pose.position;
    const geometry_msgs::Point point2 = pose2.pose.position;
    const double dx = point1.x - point2.x;
    const double dy = point1.y - point2.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  double GetEulerDistance(const float x1, const float y1, const float x2, const float y2){
     return std::sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
  }

  double GetAngle(const geometry_msgs::PoseStamped &pose1,
                  const geometry_msgs::PoseStamped &pose2) {
    const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation;
    const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
    tf::Quaternion rot1, rot2;
    tf::quaternionMsgToTF(quaternion1, rot1);
    tf::quaternionMsgToTF(quaternion2, rot2);
    return rot1.angleShortestPath(rot2);
  }

  geometry_msgs::PoseStamped Point2PoseStamped(Point point){
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

  const geometry_msgs::PoseStamped GetRobotMapPose() {
    UpdateRobotPose();
    return self_pose_;
  }

  const geometry_msgs::PoseStamped GetRobotGimbalMapPose(){
    UpdateRobotGimbalPose();
    return gimbal_pose_;
  }

  const std::shared_ptr<CostMap> GetCostMap(){
    return costmap_ptr_;
  }

  const CostMap2D* GetCostMap2D() {
    return costmap_2d_;
  }

  const unsigned char* GetCharMap() {
    return charmap_;
  }


  DecisionInfoPool info;
  Threshold threshold;

 private:
  ComInfo setComInfo(){
      ComInfo ci;
      ci.times_to_supply = info.times_to_supply;
      ci.times_to_buff = info.times_to_buff;
      ci.hp = info.remain_hp;
      ci.shoot_1 = my_shoot_1_cnt;
      ci.shoot_2 = my_shoot_2_cnt;
      ci.has_enemy = info.has_my_enemy;
      ci.bullet = info.remain_bullet;
      ci.pose_x = self_pose_.pose.position.x;
      ci.pose_y = self_pose_.pose.position.y;
      ci.yaw = tf::getYaw(self_pose_.pose.orientation);
      ci.first_valid = info.has_first_enemy;
      ci.first_enemy_x = info.first_enemy.pose.position.x;
      ci.first_enemy_y = info.first_enemy.pose.position.y;
      
      ci.second_valid = info.has_second_enemy;
      ci.second_enemy_x = info.second_enemy.pose.position.x;
      ci.second_enemy_y = info.second_enemy.pose.position.y;
      
      ci.goal_x = info.my_goal.pose.position.x;
      ci.goal_y = info.my_goal.pose.position.y;

      // fusion info
      ci.fusion.num_id_target = num_id_target;
      ci.fusion.num_no_id_target = num_no_id_target;
      unsigned int bound = 2 <num_id_target?2:num_id_target;
      for (int i=0; i<bound; i++) ci.fusion.id_targets[i] = id_targets[i];
      bound = 4 < num_no_id_target? 4:num_no_id_target;
      for (int i=0; i<bound; i++) ci.fusion.no_id_targets[i] = no_id_targets[i];
      return ci;
  }

  void getComInfo(ComInfo ci){
 
      if (info.remaining_time % 60 <= 1){
          info.times_to_buff = 1;
          info.times_to_supply = 2;
      }
      else{
          info.times_to_supply = std::min(info.times_to_supply, ci.times_to_supply);
          info.times_to_buff = std::min(info.times_to_buff, ci.times_to_buff);
      }
      
      info.ally_remain_hp = ci.hp;
      info.ally_remain_bullet = ci.bullet;
      info.ally.header.stamp = ros::Time::now();
      info.ally.pose.position.x = ci.pose_x;
      info.ally.pose.position.y = ci.pose_y;
      info.ally.pose.position.z = 0;
      info.ally.pose.orientation = tf::createQuaternionMsgFromYaw(ci.yaw);
      info.has_ally_first_enemy = ci.first_valid;
      if (ci.first_valid){ 
          info.ally_first_enemy.pose.position.x = ci.first_enemy_x;
          info.ally_first_enemy.pose.position.y = ci.first_enemy_y;
      }
      info.has_ally_second_enemy = ci.second_valid;
      if (ci.second_valid) {
          info.ally_second_enemy.pose.position.x = ci.second_enemy_x;
          info.ally_second_enemy.pose.position.y = ci.second_enemy_y;
      }
      info.ally_goal.pose.position.x = ci.goal_x;
      info.ally_goal.pose.position.y = ci.goal_y;
      info.has_ally_enemy = ci.has_enemy;

    // fusion info
      ally_num_id_target = ci.fusion.num_id_target;
      ally_num_no_id_target = ci.fusion.num_no_id_target;
      unsigned int bound = 2 <ally_num_id_target? 2:ally_num_id_target;
      for (int i=0; i<bound; i++)  ally_id_targets[i] = ci.fusion.id_targets[i];
      bound = 4 < ally_num_no_id_target? 4 :ally_num_no_id_target;
      for (int i=0; i<bound; i++)  ally_no_id_targets[i] = ci.fusion.no_id_targets[i];

      
    // prepare to publish fusion data
      FS.id_targets.clear();
      FS.no_id_targets.clear();
      FS.num_id_target = ally_num_id_target;
      FS.num_no_id_target = ally_num_no_id_target;
      for (int i=0; i< ally_num_id_target; i++) FS.id_targets.push_back(ally_id_targets[i]);
      for (int i=0; i< ally_num_no_id_target; i++) FS.no_id_targets.push_back(ally_no_id_targets[i]);

      // ally shoot cnt
      ally_shoot_1_cnt = ci.shoot_1;
      ally_shoot_2_cnt = ci.shoot_2;
  }

  // timeit shoot bullet control
  void _ShootThread(int hz){
      ros::Rate loop(hz);
      while (ros::ok()){
          roborts_msgs::ShooterCmd cmd;
          cmd.is_shoot = true;
          cmd.shoot_cmd = 1;
          cmd.c_shoot_cmd = 0;
          cmd.shoot_freq = 2500;
          shoot_pub_.publish(cmd);
          info.remain_bullet = info.remain_bullet - 1;
          if (! is_in_shoot_state)
             break;
          loop.sleep();
          
      }
  }


  // master and client communication function
  void CommunicateMaster(){
    ComInfo ci;
    ros::Rate loop(50);
    while (ros::ok()){
        zmq::message_t rec_message;
        masterSocket_.recv(&rec_message);
        memcpy(&ci, rec_message.data(), sizeof(ci));
        getComInfo(ci);   
        //---------------------------------------------------------------------------------------------------------------------------------------
        ci = setComInfo();
        zmq::message_t send_message(sizeof(ci));
        memcpy(send_message.data(), &ci, sizeof(ci));
        
        masterSocket_.send(send_message);

        ally_pub_.publish(info.ally);
        fusion_pub_.publish(FS);
        info.has_ally = true;
       
        loop.sleep();
    }
  }

  void CommunicateClient(){
    
    ComInfo ci;
    ros::Rate loop(50);

    while (ros::ok()){

        ci = setComInfo();
        zmq::message_t send_message(sizeof(ci));
        memcpy(send_message.data(), &ci, sizeof(ci));
        
        clientSocket_.send(send_message);
        //----------------------------------------------------------------------------------------------------------------------------------------
        
        zmq::message_t rec_message;
        clientSocket_.recv(&rec_message);
        memcpy(&ci, rec_message.data(), sizeof(ci));
        getComInfo(ci);
        
        geometry_msgs::PoseStamped ap;
       
        ally_pub_.publish(info.ally);
        fusion_pub_.publish(FS);
        info.has_ally = true;
       
        loop.sleep();       
    }
  }




  void UpdateRobotPose() {
    tf::Stamped<tf::Pose> robot_tf_pose;
    robot_tf_pose.setIdentity();
     
    robot_tf_pose.frame_id_ = pose_frameID;
    robot_tf_pose.stamp_ = ros::Time();
    try {
      geometry_msgs::PoseStamped robot_pose;
      tf::poseStampedTFToMsg(robot_tf_pose, robot_pose);
      tf_ptr_->transformPose("map", robot_pose, self_pose_);
    }
    catch (tf::LookupException &ex) {
      ROS_ERROR("Transform Error looking up robot pose: %s", ex.what());
    }
  }


  void UpdateRobotGimbalPose(){
    if (use_camera){
        tf::Stamped<tf::Pose> gimbal_tf_pose;
        gimbal_tf_pose.setIdentity();

        gimbal_tf_pose.frame_id_ = gimbal_frameID;
        gimbal_tf_pose.stamp_ = ros::Time();
        try{
            geometry_msgs::PoseStamped gimbal_pose;
            tf::poseStampedTFToMsg(gimbal_tf_pose, gimbal_pose);
            tf_ptr_->transformPose("map", gimbal_pose, gimbal_pose_);

        }
        catch (tf::LookupException &ex){
            ROS_ERROR("Transform Error looking up gimbal pose: %s", ex.what());
        }
    }
    
  }

  geometry_msgs::PoseStamped InitMapPose(){
      geometry_msgs::PoseStamped p;
      p.header.frame_id = "map";
      p.pose.position.x = -99;
      p.pose.position.y = -99;
      return p;
  }


  //! list<death points>
  std::list<geometry_msgs::Point> death_Points;


  //! tf
  std::shared_ptr<tf::TransformListener> tf_ptr_;

  //! Enenmy detection
  ros::Subscriber enemy_sub_;
  ros::Subscriber armor_sub_, camera_armor_sub_, back_camera_sub_, info_sub_, fusion_target_sub_, cmd_gimbal_sub_;
  ros::Subscriber robot_status_sub_, robot_shoot_sub_, robot_heat_sub_, game_status_sub_, supply_sub_, buff_sub_, vel_acc_sub_;
  // publisher
  ros::Publisher shoot_pub_, ally_pub_, fusion_pub_, dodge_pub_, supply_pub_, cmd_vel_pub_;

  //! Goal info
  geometry_msgs::PoseStamped goal_;
  bool new_goal_;

  //! Enemy info
  actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> armor_detection_actionlib_client_;
  roborts_msgs::ArmorDetectionGoal armor_detection_goal_;
  geometry_msgs::PoseStamped enemy_pose_;
  bool enemy_detected_;

  //! cost map
  std::shared_ptr<CostMap> costmap_ptr_;
  CostMap2D* costmap_2d_;
  unsigned char* charmap_;

  //! robot map pose
  geometry_msgs::PoseStamped self_pose_, enemy_;
  //! robot gimbal pose
  geometry_msgs::PoseStamped gimbal_pose_;

  // point
  geometry_msgs::Point camera_enemy_;

  //remain hp
  int remain_hp;


  // zmq
  zmq::context_t context_;
  zmq::socket_t masterSocket_, clientSocket_;
  std::thread masterThread, clientThread;
  std::thread shoot_thread;
  bool is_in_shoot_state;
 
  


  // frame id
  std::string pose_frameID, gimbal_frameID;
  bool use_camera;

  // fusion
  unsigned int num_id_target, ally_num_id_target;
  unsigned int num_no_id_target, ally_num_no_id_target;
  unsigned int my_shoot_1_cnt, my_shoot_2_cnt, ally_shoot_1_cnt, ally_shoot_2_cnt;
  roborts_msgs::Target id_targets[2], ally_id_targets[2];
  roborts_msgs::Target no_id_targets[4], ally_no_id_targets[4];
  roborts_msgs::FusionTarget FS;
  double my_vel_x, my_vel_y;

  bool _gimbal_can, in_dodge;
  
  bool _front_first_enemy, _front_second_enemy;
  bool _back_first_enemy, _back_second_enemy;
  bool _dodge_in_reload;
};
} //namespace roborts_decision
#endif //ROBORTS_DECISION_BLACKBOARD_H
