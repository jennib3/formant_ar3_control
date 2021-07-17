/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"

#include <boost/thread/mutex.hpp>

#include "sensor_msgs/JointState.h"

#include <tf2/LinearMath/Quaternion.h>

#include <math.h>

class Arm_control_class {

private:
  bool new_move{false};
  bool new_plan{false};
  boost::mutex control_mutex;
  double arm_position[3];
  double arm_euler[3];

public:

  void set_new_move(bool value) {
    control_mutex.lock();
    new_move = value;
    control_mutex.unlock();
  }

  bool get_new_move() {
    control_mutex.lock();
    bool ret = new_move;
    control_mutex.unlock();
    return ret;
  }

  void set_new_plan(bool value) {
    control_mutex.lock();
    new_plan = value;
    control_mutex.unlock();
  }

  bool get_new_plan() {
    control_mutex.lock();
    bool ret = new_plan;
    control_mutex.unlock();
    return ret;
  }

  void set_arm_x(double value) {
    control_mutex.lock();
    arm_position[0] = value;
    control_mutex.unlock();
  }

  void set_arm_y(double value) {
    control_mutex.lock();
    arm_position[1] = value;
    control_mutex.unlock();
  }

  void set_arm_z(double value) {
    control_mutex.lock();
    arm_position[2] = value;
    control_mutex.unlock();
  }
  
  double get_arm_x() {
    double ret;
    control_mutex.lock();
    ret = arm_position[0];
    control_mutex.unlock();
    return ret;
  }

  double get_arm_y() {
    double ret;
    control_mutex.lock();
    ret = arm_position[1];
    control_mutex.unlock();
    return ret;
  }

  double get_arm_z() {
    double ret;
    control_mutex.lock();
    ret = arm_position[2];
    control_mutex.unlock();
    return ret;
  }


  void set_arm_roll(double value) {
    control_mutex.lock();
    arm_euler[0] = value;
    control_mutex.unlock();
  }
  
  void set_arm_pitch(double value) {
    control_mutex.lock();
    arm_euler[1] = value;
    control_mutex.unlock();
  }

  void set_arm_yaw(double value) {
    control_mutex.lock();
    arm_euler[2] = value;
    control_mutex.unlock();
  }  

  double get_arm_roll() {
    double ret;
    control_mutex.lock();
    ret = arm_euler[0];
    control_mutex.unlock();
    return ret;
  }

  double get_arm_pitch() {
    double ret;
    control_mutex.lock();
    ret = arm_euler[1];
    control_mutex.unlock();
    return ret;
  }

  double get_arm_yaw() {
    double ret;
    control_mutex.lock();
    ret = arm_euler[2];
    control_mutex.unlock();
    return ret;
  }

};

sensor_msgs::JointState proposed_joint_state;
Arm_control_class arm_control;


bool plan_pose(moveit::planning_interface::MoveGroupInterface *move_group, geometry_msgs::Pose my_pose, ros::Publisher planned_joints_pub){

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  move_group->setPoseTarget(my_pose);

  move_group->setPlanningTime(3.0);
  
  bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("msg", "Planning for Pose %s", success ? "SUCCESS" : "FAILED");

  if (success) {
    robot_state::RobotState start_state(*move_group->getCurrentState());
    robot_state::RobotState state(start_state);

    const std::vector<double> joints = my_plan.trajectory_.joint_trajectory.points.back().positions;

    // for (int i=0; i < joints.size(); ++i) {
    //   ROS_INFO_NAMED("msg", "joint %i at %f", i, joints[i]);
    // }


    proposed_joint_state.position = joints;
    planned_joints_pub.publish(proposed_joint_state);

  }

  return success;
};


bool plan_joints(moveit::planning_interface::MoveGroupInterface *move_group, std::vector<double> joint_group_positions) {

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  move_group->setJointValueTarget(joint_group_positions);

  bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("msg", "Planning for Joints %s", success ? "SUCCESS" : "FAILED");

  return success;
}


void plan_and_move(const std_msgs::Bool::ConstPtr& msg) {
  // msg->data;
  ROS_INFO_NAMED("msg", "plan_and_move");
  arm_control.set_new_move(true);
}

void new_x_value(const std_msgs::Float64::ConstPtr& msg) {

  arm_control.set_arm_x(msg->data);
  arm_control.set_new_plan(true);
  // ROS_INFO_NAMED("msg", "new x value %f", msg->data);

}

void new_y_value(const std_msgs::Float64::ConstPtr& msg) {

  arm_control.set_arm_y(msg->data);
  arm_control.set_new_plan(true);
  // ROS_INFO_NAMED("msg", "new y value %f", msg->data);

}

void new_z_value(const std_msgs::Float64::ConstPtr& msg) {

  arm_control.set_arm_z(msg->data);
  arm_control.set_new_plan(true);
  // ROS_INFO_NAMED("msg", "new z value %f", msg->data);

}

void new_roll_value(const std_msgs::Float64::ConstPtr& msg) {

  arm_control.set_arm_roll(msg->data);
  arm_control.set_new_plan(true);
  // ROS_INFO_NAMED("msg", "new roll value %f", msg->data);

}

void new_pitch_value(const std_msgs::Float64::ConstPtr& msg) {

  arm_control.set_arm_pitch(msg->data);
  arm_control.set_new_plan(true);
  // ROS_INFO_NAMED("msg", "new pitch value %f", msg->data);

}

void new_yaw_value(const std_msgs::Float64::ConstPtr& msg) {

  arm_control.set_arm_yaw(msg->data);
  arm_control.set_new_plan(true);
  // ROS_INFO_NAMED("msg", "new yaw value %f", msg->data);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "formant_ar3_control_demo");
  ros::NodeHandle node_handle;

  arm_control.set_arm_x(-0.007443);
  arm_control.set_arm_y(-0.064925);
  arm_control.set_arm_z(0.717915);

  // Initialize the message;
  proposed_joint_state.name = {"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"};
  proposed_joint_state.velocity = {0.0,0.0,0.0,0.0,0.0,0.0};
  proposed_joint_state.effort = {0.0,0.0,0.0,0.0,0.0,0.0};
  proposed_joint_state.position = {0.0,0.0,0.0,0.0,0.0,0.0};

  ros::Publisher planned_joints_pub = node_handle.advertise<sensor_msgs::JointState>("/joint_states_planned", 1000);

  ros::Subscriber sub = node_handle.subscribe("formant_plan_move", 1000, plan_and_move);

  ros::Subscriber sub_x = node_handle.subscribe("ar3_x", 1000, new_x_value);
  ros::Subscriber sub_y = node_handle.subscribe("ar3_y", 1000, new_y_value);
  ros::Subscriber sub_z = node_handle.subscribe("ar3_z", 1000, new_z_value);

  ros::Subscriber sub_roll  = node_handle.subscribe("ar3_roll",  1000, new_roll_value);
  ros::Subscriber sub_pitch = node_handle.subscribe("ar3_pitch", 1000, new_pitch_value);
  ros::Subscriber sub_yaw   = node_handle.subscribe("ar3_yaw",   1000, new_yaw_value);


  ros::AsyncSpinner spinner(1);
  spinner.start();


  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  ros::Rate rate(10);
  while (ros::ok()) {


move_group.setEndEffectorLink("link_6");
geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
      ROS_INFO_NAMED("msg", "current orientation %f %f %f %f and Position %f %f %f", current_pose.pose.orientation.w, 
                                                                                          current_pose.pose.orientation.x, 
                                                                                          current_pose.pose.orientation.y, 
                                                                                          current_pose.pose.orientation.z,
                                                                                          current_pose.pose.position.x,
                                                                                          current_pose.pose.position.y,
                                                                                          current_pose.pose.position.z);


    geometry_msgs::Pose my_pose;
    double roll, pitch, yaw;
    tf2::Quaternion static_quat; // x,y,z,w
    static_quat[0] = 0.0000f;   // x
    static_quat[1] = 0.0000f;   // y
    static_quat[2] = 1.0000f;   // z
    static_quat[3] = 0.0000f;   // w
    tf2::Quaternion quat;


    if (arm_control.get_new_move() || arm_control.get_new_plan()) {

      my_pose.position.x = arm_control.get_arm_x();
      my_pose.position.y = arm_control.get_arm_y();
      my_pose.position.z = arm_control.get_arm_z();

      // yaw = atan2(my_pose.position.y, my_pose.position.x);
      // double xp = cos(yaw)*my_pose.position.x + sin(yaw)*my_pose.position.y;
      // pitch = atan2(my_pose.position.z, xp);
      // roll = 0.0f;

      quat.setRPY( arm_control.get_arm_roll(), 
                   arm_control.get_arm_pitch(), 
                   arm_control.get_arm_yaw() );
      // quat.setRPY( 0, 0, 0 );
      quat *= static_quat;
      quat.normalize();
    }


    if (arm_control.get_new_move()) {

      my_pose.orientation.w = quat[3];
      my_pose.orientation.x = quat[0];
      my_pose.orientation.y = quat[1];
      my_pose.orientation.z = quat[2];

      ROS_INFO_NAMED("msg", "Planning for orientation %f %f %f %f and Position %f %f %f", my_pose.orientation.w, 
                                                                                          my_pose.orientation.x, 
                                                                                          my_pose.orientation.y, 
                                                                                          my_pose.orientation.z,
                                                                                          my_pose.position.x,
                                                                                          my_pose.position.y,
                                                                                          my_pose.position.z);

      bool success = plan_pose(&move_group, my_pose, planned_joints_pub);

      if (success) {
        move_group.move();
        ROS_INFO_NAMED("msg", "Move complete");
      }        

      arm_control.set_new_move(false);


    } else if (arm_control.get_new_plan()) {
      


    



      my_pose.orientation.w = quat[3];
      my_pose.orientation.x = quat[0];
      my_pose.orientation.y = quat[1];
      my_pose.orientation.z = quat[2];


      ROS_INFO_NAMED("msg", "Planning for orientation %f %f %f %f and Position %f %f %f", my_pose.orientation.w, 
                                                                                          my_pose.orientation.x, 
                                                                                          my_pose.orientation.y, 
                                                                                          my_pose.orientation.z,
                                                                                          my_pose.position.x,
                                                                                          my_pose.position.y,
                                                                                          my_pose.position.z);

      bool success = plan_pose(&move_group, my_pose, planned_joints_pub);

      arm_control.set_new_plan(false);
    }

    rate.sleep();
  }


  ros::shutdown();
  return 0;
}
