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

#include <csignal>

#include "local_planner/local_planner_node.h"

namespace roborts_local_planner
{
using roborts_common::NodeState;
//局部路径规格类
LocalPlannerNode::LocalPlannerNode()
  :  // as_为action的变量
  as_(local_planner_nh_, "local_planner_node_action", boost::bind(&LocalPlannerNode::ExcuteCB, this, _1), false)
  , initialized_(false)
  , node_state_(roborts_common::NodeState::IDLE)
  , node_error_info_(roborts_common::ErrorCode::OK)
  , max_error_(5)
  , local_cost_(nullptr)
  , tf_(nullptr)
{
  if (Init().IsOK())
  {
    ROS_INFO("local planner initialize completed.");
  }
  else
  {
    ROS_WARN("local planner initialize failed.");
    SetNodeState(NodeState::FAILURE);
  }

  as_.start();
}

LocalPlannerNode::~LocalPlannerNode()
{
  StopPlanning();
}

roborts_common::ErrorInfo LocalPlannerNode::Init()
{
  ROS_INFO("local planner start");
  LocalAlgorithms local_algorithms;
  //读取局部路径规划配置文件，该文件包含使用哪一中局部规划算法，路径更新频率的信息
  std::string full_path = ros::package::getPath("roborts_planning") + "/local_planner/config/local_planner.prototxt";
  //配置文件格式为prototxt，使用该函数来将文件中的内容转化为程序中可调用的参数列表
  roborts_common::ReadProtoFromTextFile(full_path.c_str(), &local_algorithms);

  if (&local_algorithms == nullptr)
  {
    //找不到配置文件，没有这个配置文件，抛出找不到文件的异常
    return roborts_common::ErrorInfo(roborts_common::ErrorCode::LP_INITILIZATION_ERROR, "Cannot load local planner "
                                                                                        "protobuf configuration file.");
  }
  selected_algorithm_ = local_algorithms.selected_algorithm();

  frequency_ = local_algorithms.frequency();
  ROS_WARN("Frequency = %f ",frequency_);
  tf_ = std::make_shared<tf::TransformListener>(ros::Duration(10));
  //加载局部代价地图参数
  std::string map_path = ros::package::getPath("roborts_costmap") + "/config/"
                                                                    "costmap_parameter_config_for_local_plan.prototxt";
  //将从代价地图配置文件中的内容从字符串转变为可以提取的参数名和参数值的配对
  local_cost_ = std::make_shared<roborts_costmap::CostmapInterface>("local_costmap", *tf_, map_path.c_str());

  local_planner_ = roborts_common::AlgorithmFactory<LocalPlannerBase>::CreateAlgorithm(selected_algorithm_);
  //缺失局部代价地图的异常处理（局部代价地图文件路径不对，不存在该文件等）
  if (local_planner_ == nullptr)
  {
    ROS_ERROR("global planner algorithm instance can't be loaded");
    return roborts_common::ErrorInfo(roborts_common::ErrorCode::LP_INITILIZATION_ERROR, "local planner algorithm "
                                                                                        "instance can't be loaded");
  }

  std::string name;
  //获取代价地图中全局坐标系的参数值
  visual_frame_ = local_cost_->GetGlobalFrameID();
  visual_ = LocalVisualizationPtr(new LocalVisualization(local_planner_nh_, visual_frame_));
  //加速度信息发布
  vel_pub_ = local_planner_nh_.advertise<roborts_msgs::TwistAccel>("cmd_vel_acc", 5);
  change_sub_ = local_planner_nh_.subscribe("change_mode", 5, &LocalPlannerNode::changeCallback, this);

  return roborts_common::ErrorInfo(roborts_common::ErrorCode::OK);
}

void LocalPlannerNode::ExcuteCB(const roborts_msgs::LocalPlannerGoal::ConstPtr &command)
{
  roborts_common::ErrorInfo error_info = GetErrorInfo();
  NodeState node_state = GetNodeState();

  //节点启动失败
  if (node_state == NodeState::FAILURE)
  {
    roborts_msgs::LocalPlannerFeedback feedback;
    roborts_msgs::LocalPlannerResult result;
    feedback.error_code = error_info.error_code();
    feedback.error_msg = error_info.error_msg();
    result.error_code = feedback.error_code;
    as_.publishFeedback(feedback);
    as_.setAborted(result, feedback.error_msg);
    ROS_ERROR("Initialization Failed, Failed to execute action!");
    return;
  }
  if (plan_mtx_.try_lock())
  {
    // route为nav_msgs::path类型的变量
    //当没有全局规划者给出全局计划时，使用局部目标local_goal_表达机器人终点
    local_planner_->SetPlan(command->route, local_goal_);
    plan_mtx_.unlock();
    plan_condition_.notify_one();
  }

  ROS_INFO("Send Plan!");
  if (node_state == NodeState::IDLE)
  {
    StartPlanning();
  }

  while (ros::ok())
  {
    //等待1us
    //扩展 C++同一方法的函数还有   seconds   以s为单位和   milliseconds   以ms为单位
    std::this_thread::sleep_for(std::chrono::microseconds(1));

    if (as_.isPreemptRequested())
    {
      ROS_INFO("Action Preempted");
      if (as_.isNewGoalAvailable())
      {
        as_.setPreempted();
        break;
      }
      else
      {
        as_.setPreempted();
        StopPlanning();
        break;
      }
    }
    node_state = GetNodeState();
    error_info = GetErrorInfo();

    if (node_state == NodeState::RUNNING || node_state == NodeState::SUCCESS || node_state == NodeState::FAILURE)
    {
      roborts_msgs::LocalPlannerFeedback feedback;
      roborts_msgs::LocalPlannerResult result;
      if (!error_info.IsOK())
      {
        feedback.error_code = error_info.error_code();
        feedback.error_msg = error_info.error_msg();
        SetErrorInfo(roborts_common::ErrorInfo::OK());

        as_.publishFeedback(feedback);
      }
      if (node_state == NodeState::SUCCESS)
      {
        result.error_code = error_info.error_code();
        as_.setSucceeded(result, error_info.error_msg());
        StopPlanning();
        break;
      }
      else if (node_state == NodeState::FAILURE)
      {
        result.error_code = error_info.error_code();
        as_.setAborted(result, error_info.error_msg());
        StopPlanning();
        break;
      }
    }
  }
}

void LocalPlannerNode::Loop()
{
  roborts_common::ErrorInfo error_info = local_planner_->Initialize(local_cost_, tf_, visual_);
  if (error_info.IsOK())
  {
    ROS_INFO("local planner algorithm initialize completed.");
  }
  else
  {
    ROS_WARN("local planner algorithm initialize failed.");
    SetNodeState(NodeState::FAILURE);
    SetErrorInfo(error_info);
  }
  // std::chrono是在C++11中引入的，是一个模板库，用来处理时间和日期的Time library。
  std::chrono::microseconds sleep_time = std::chrono::microseconds(0);
  int error_count = 0;

  while (GetNodeState() == NodeState::RUNNING)
  {
    // unique_lock 占用互斥量，保证进程的正常进行
    std::unique_lock<std::mutex> plan_lock(plan_mutex_);
    plan_condition_.wait_for(plan_lock, sleep_time);
    auto begin = std::chrono::steady_clock::now();
    roborts_common::ErrorInfo error_info = local_planner_->ComputeVelocityCommands(cmd_vel_);
    auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin);
    int need_time = 1000 / frequency_;
    sleep_time = std::chrono::milliseconds(need_time) - cost_time;

    if (sleep_time <= std::chrono::milliseconds(0))
    {
      // LOG_WARNING << "The time planning once is " << cost_time.count() << " beyond the expected time "
      //        << std::chrono::milliseconds(50).count();
      sleep_time = std::chrono::milliseconds(0);
      // SetErrorInfo(ErrorInfo(ErrorCode::GP_TIME_OUT_ERROR, "Planning once time out."));
    }

    if (error_info.IsOK())
    {
      error_count = 0;
      vel_pub_.publish(cmd_vel_);
      if (local_planner_->IsGoalReached())
      {
        SetNodeState(NodeState::SUCCESS);
      }
    }
    else if (error_count > max_error_ && max_error_ > 0)
    {
      ROS_WARN("Can not finish plan with max retries( %d  )", max_error_);
      error_info = roborts_common::ErrorInfo(roborts_common::ErrorCode::LP_MAX_ERROR_FAILURE, "over max error.");
      SetNodeState(NodeState::FAILURE);
    }
    else
    {
      error_count++;
      ROS_ERROR("Can not get cmd_vel for once. %s error count:  %d", error_info.error_msg().c_str(), error_count);
    }

    SetErrorInfo(error_info);
  }

  cmd_vel_.twist.linear.x = 0;
  cmd_vel_.twist.linear.y = 0;
  cmd_vel_.twist.angular.z = 0;

  cmd_vel_.accel.linear.x = 0;
  cmd_vel_.accel.linear.y = 0;
  cmd_vel_.accel.angular.z = 0;
  //  for (int i = 0; i < 10; ++i) {
  vel_pub_.publish(cmd_vel_);
  //    usleep(5000);
  //  }
}

void LocalPlannerNode::SetErrorInfo(const roborts_common::ErrorInfo error_info)
{
  std::lock_guard<std::mutex> guard(node_error_info_mtx_);
  node_error_info_ = error_info;
}

void LocalPlannerNode::SetNodeState(const roborts_common::NodeState &node_state)
{
  std::lock_guard<std::mutex> guard(node_state_mtx_);
  node_state_ = node_state;
}

roborts_common::NodeState LocalPlannerNode::GetNodeState()
{
  std::lock_guard<std::mutex> guard(node_state_mtx_);
  return node_state_;
}

roborts_common::ErrorInfo LocalPlannerNode::GetErrorInfo()
{
  std::lock_guard<std::mutex> guard(node_error_info_mtx_);
  return node_error_info_;
}

void LocalPlannerNode::StartPlanning()
{
  if (local_planner_thread_.joinable())
  {
    local_planner_thread_.join();
  }

  SetNodeState(roborts_common::NodeState::RUNNING);
  local_planner_thread_ = std::thread(std::bind(&LocalPlannerNode::Loop, this));
}

void LocalPlannerNode::StopPlanning()
{
  SetNodeState(roborts_common::IDLE);
  if (local_planner_thread_.joinable())
  {
    local_planner_thread_.join();
  }
}

void LocalPlannerNode::AlgorithmCB(const roborts_common::ErrorInfo &algorithm_error_info)
{
  SetErrorInfo(algorithm_error_info);
}

void LocalPlannerNode::changeCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  if (msg->linear.x != 0)
  {
    std::chrono::microseconds sleep_time = std::chrono::microseconds(0);
    std::unique_lock<std::mutex> plan_lock(plan_mutex_);
    plan_condition_.wait_for(plan_lock, sleep_time);
    local_planner_->ReloadParams(msg->linear.x);
    if(msg->linear.z != 0)
    {
      frequency_ = msg->linear.z;
      ROS_WARN("mode has changed! frequency = %2f",frequency_);
    }
  }
}

}  // namespace roborts_local_planner

void SignalHandler(int signal)
{
  if (ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown())
  {
    ros::shutdown();
  }
}

int main(int argc, char **argv)
{
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);
  ros::init(argc, argv, "local_planner_node", ros::init_options::NoSigintHandler);

  roborts_local_planner::LocalPlannerNode local_planner;

  /*
  ===========================ROS多线程订阅消息(ros::asyncspinner)==========================================
  对于一些只订阅一个话题的简单节点来说，我们使用ros::spin()进入接收循环，每当有订阅的话题发布时，
  进入回调函数接收和处理消息数据。但是更多的时候，一个节点往往要接收和处理不同来源的数据，并且
  这些数据的产生频率也各不相同，当我们在一个回调函数里耗费太多时间时，会导致其他回调函数被阻塞，
  导致数据丢失。这种场合需要给一个节点开辟多个线程，保证数据流的畅通。
======================================================================================================
  */
  ros::AsyncSpinner async_spinner(2);
  async_spinner.start();
  ros::waitForShutdown();
  local_planner.StopPlanning();

  return 0;
}
