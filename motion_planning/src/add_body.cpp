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
#include <geometric_shapes/shape_operations.h>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "add_body");

  ros::NodeHandle node_handle;

  tf::TransformListener listener;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface group("manipulator");

  moveit::planning_interface::PlanningSceneInterface current_scene;
  sleep(2.0);

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = group.getPlanningFrame();

  /* The id of the object is used to identify it. */
  collision_object.id = "chair_base";

  /* Define a box to add to the world. */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.60;
  primitive.dimensions[1] = 1.00;
  primitive.dimensions[2] = 0.30;

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x =  0.0;
  box_pose.position.y = 0.78;
  box_pose.position.z =  0.15;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;  
  collision_objects.push_back(collision_object);  

  // Now, let's add the collision object into the world
  ROS_INFO("Add an object into the world");  
  //current_scene.addCollisionObjects(collision_objects);
  sleep(2.0);


  // MESH obstacle  
  moveit_msgs::CollisionObject co;
  co.header.frame_id = group.getPlanningFrame();

  // Id
  co.id = "chair";

  // Obstacle definition
  //Eigen::Vector3d b(0.001, 0.001, 0.001);
  //package://myur5_description/meshes/chair.stl
  shapes::Mesh* m = shapes::createMeshFromResource("package://ur_description/meshes/ur5/collision/chair.stl"); 
  ROS_INFO("Chair mesh loaded");

  shape_msgs::Mesh mesh;
  shapes::ShapeMsg mesh_msg;  
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  co.meshes.resize(1);
  co.mesh_poses.resize(1);
  co.meshes[0] = mesh;
  co.header.frame_id = group.getPlanningFrame();   
  co.mesh_poses[0].position.x = 0.0;
  co.mesh_poses[0].position.y = 0.0;
  co.mesh_poses[0].position.z = 0.0;
  co.mesh_poses[0].orientation.w= 1.0; 
  co.mesh_poses[0].orientation.x= 0.0; 
  co.mesh_poses[0].orientation.y= 0.0;
  co.mesh_poses[0].orientation.z= 0.0;   

  co.meshes.push_back(mesh);
  co.mesh_poses.push_back(co.mesh_poses[0]);
  co.operation = co.ADD;
  
  collision_objects.push_back(co);
  ROS_INFO("Chair added into the world");
  current_scene.addCollisionObjects(collision_objects);
  sleep(2.0);


  ros::shutdown();
  return 0;
}