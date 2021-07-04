#include <ros/ros.h>

#include "executor/chassis_executor.h"
#include "example_behavior/goal_behavior.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "sel_behavior_node");
  
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/sel_behave.prototxt";

  auto chassis_executor = new roborts_decision::ChassisExecutor;
  auto blackboard = new roborts_decision::Blackboard(full_path);

  // Behaviors
  roborts_decision::GoalBehavior       goal_behavior(chassis_executor, blackboard);

  ros::Rate rate(10);
  while(ros::ok()){
    ros::spinOnce();
    // if (blackboard->CanDodge()){
    //     blackboard->StartDodge();
    // }
    goal_behavior.Run();
    rate.sleep();
  }
  return 0;
}
