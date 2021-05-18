#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <turtlesim/Spawn.h>
#include <math.h>
#include <std_msgs/Float64MultiArray.h>
using namespace std;
int main(int argc, char** argv){
  ros::init(argc, argv, "goal_publisher");

  ros::NodeHandle node;

  ros::Publisher pub = node.advertise<geometry_msgs::PoseStamped>("/des_goal", 1000);
  
  ros::Publisher pub_joint = node.advertise<std_msgs::Float64MultiArray>("/jvalue",1000);
  
  geometry_msgs::PoseStamped init;
  init.header.frame_id = "des_init";
  init.pose.position.x = 0.0;
  init.pose.position.y = 0.5;
  init.pose.position.z = 1.0;

  double step_size = 0.02;  

  geometry_msgs::PoseStamped goal;

  goal.header.frame_id = "des_goal";
  goal.pose.position.x = 0.5;
  goal.pose.position.y = 0.5;
  goal.pose.position.z = 1.0;

  std_msgs::Float64MultiArray joint_value;

  joint_value.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  int count = 0;
  int type_goal = 0;

  ros::Rate rate(10.0);
  
  
  while (node.ok()){
    //cout << "here" << endl;
    // for cartesian goals
    
    if (count >= 10){
      if(type_goal==0){
        cout << "case 0" << endl;
        if (goal.pose.position.x > -0.5){
          goal.pose.position.x -= 0.05;
        }

        else if (goal.pose.position.x <= -0.5){
          goal.pose.position.x = -0.5;
          goal.pose.position.z += 0.05;
          if (goal.pose.position.z >= 1.5){
            goal.pose.position.z = 1.5;
          }
        }

        else{
          cout << "Final goal reached" << endl;
          break;
        }

        cout << "Goal: " << endl;
        cout << "x = " << goal.pose.position.x << endl;
        cout << "y = " << goal.pose.position.y << endl;
        cout << "z = " << goal.pose.position.z << endl;
        pub.publish(goal);
      }

      if(type_goal==1){
        cout << "case 1" << endl;
        if(joint_value.data[0] < 1.57){
          joint_value.data[0] += step_size;
        }
        else if (joint_value.data[0] >= 1.57){
          joint_value.data[0] = 1.57;
          joint_value.data[1] += step_size;
          if(joint_value.data[1] >= 1.57){
            joint_value.data[1] = 1.57;
          }
        }
        else{
          cout << "Final goal reached" << endl;
          break;
        }

        cout << "Joints: " << endl;
        cout << "joint 0: " << joint_value.data[0] << endl;
        cout << "joint 1: " << joint_value.data[1] << endl;
        cout << "joint 2: " << joint_value.data[2] << endl;

        pub_joint.publish(joint_value);
      }
      
      count = 0;
    }
    count += 1;
    //cout << "Goal: " << endl;
    //cout << "x = " << goal.pose.position.x << endl;
    //cout << "y = " << goal.pose.position.y << endl;
    //cout << "z = " << goal.pose.position.z << endl;
    //pub.publish(goal);

    //cout << "Joints: " << endl;
    //cout << "joint 0: " << joint_value.data[0] << endl;
    //cout << "joint 1: " << joint_value.data[1] << endl;
    //cout << "joint 2: " << joint_value.data[2] << endl;

    //pub_joint.publish(joint_value);
    





    //pub.publish(init);
    //pub.publish(goal);
      
    rate.sleep();
  }
  
  ros::shutdown();
  return 0;
};