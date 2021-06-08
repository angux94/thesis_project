#include <iostream>
#include <tf2/LinearMath/Transform.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <geometry_msgs/Pose.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <rosbag/bag.h>

using namespace std;

geometry_msgs::Pose desired, initial;
std::vector<double> joint_value = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plan_storer");

  ros::NodeHandle node_handle;

  tf::TransformListener listener;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group =
  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  move_group.setPlannerId("RRTConnect");

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("pedestal");

  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.25;
  visual_tools.publishText(text_pose, "UR5 robot", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();
  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO("Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
  std::ostream_iterator<std::string>(std::cout, ", "));

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
  moveit_msgs::RobotTrajectory trajectory;

  std::vector<double> end_joints;


  joint_value[0] = 0.7897;
  joint_value[1] = -2.3015;
  joint_value[2] = 1.8849;
  joint_value[3] = 0.2061;
  joint_value[4] = 1.5631;
  joint_value[5] = -0.00698132;


  move_group.setJointValueTarget(joint_value);

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO( "Completed the plan from home  %s", success ? "" : "FAILED");
  if(success){
    ROS_INFO("Visualizing plan 1 as trajectory line");
    //visual_tools.publishAxisLabeled(initial, "initial");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);

    //std::cout << my_plan.trajectory_ << std::endl;

    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    move_group.execute(my_plan);
  }

  if(!success)
  return 0;


  std::vector<double> new_goal = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  new_goal[0] = -0.23509;
  new_goal[1] = -0.30717;
  new_goal[2] = 0.53372;
  new_goal[3] = 0.4527;
  new_goal[4] = -0.7316;
  new_goal[5] = 1.9249; 

  move_group.setJointValueTarget(new_goal);

  success = (move_group.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO( "Completed the plan from home to new_goal  %s", success ? "" : "FAILED");
  if(success){
    ROS_INFO("Visualizing plan 2 as trajectory line");
    //visual_tools.publishAxisLabeled(initial, "initial");
    visual_tools.publishText(text_pose, "plan2", rvt::WHITE, rvt::XLARGE);

    //std::cout << my_plan.trajectory_ << std::endl;

    visual_tools.publishTrajectoryLine(my_plan2.trajectory_, joint_model_group);
    visual_tools.trigger();
    move_group.execute(my_plan2);
  }

  if(!success)
  return 0;
  end_joints = move_group.getCurrentJointValues();


  std_msgs::Float64 num[2];

  num[0].data = my_plan.planning_time_;
  num[1].data = my_plan2.planning_time_;
  
  moveit_msgs::RobotState r_state[2];
  r_state[0] = my_plan.start_state_;
  r_state[1] = my_plan2.start_state_;

  moveit_msgs::RobotTrajectory traj[2];
  traj[0] = my_plan.trajectory_;
  traj[1] = my_plan2.trajectory_;

  rosbag::Bag bag;
  bag.open("test.bag", rosbag::bagmode::Write);

  //bag.write("plans", ros::Time::now(), my_plan.trajectory_);
  bag.write("time", ros::Time::now(), num[0]);
  bag.write("time", ros::Time::now(), num[1]);
  cout << num[0].data << endl;
  cout << num[1].data << endl;

  bag.write("state", ros::Time::now(), r_state[0]);
  bag.write("state", ros::Time::now(), r_state[1]);
  cout << r_state[0] << endl;
  cout << r_state[1] << endl;

  bag.write("trajectory", ros::Time::now(), traj[0]);
  bag.write("trajectory", ros::Time::now(), traj[1]);
  //cout << traj << endl;

  //cout << my_plan.trajectory_ << endl;
  bag.close();

/*
  rosbag::Bag bag;
  bag.open("test.bag", rosbag::bagmode::Write);


  double i;
  i = 42;

  bag.write("numbers", ros::Time::now(), i);

  bag.close();
*/



  ros::spinOnce();
  visual_tools.deleteAllMarkers();
  ros::shutdown();
  return 0;
};
