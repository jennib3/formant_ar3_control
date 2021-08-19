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
  std::vector<double> joint_position;
  bool should_home{false};
  double vel_scale{0.1f};
  double acc_scale{0.1f};

  uint8_t plan_type{0};

public:

  static const uint8_t POSE_PLAN{0};
  static const uint8_t JOINT_PLAN{1};

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

  void set_new_plan(bool value, uint8_t new_plan_type=POSE_PLAN) {
    control_mutex.lock();
    new_plan = value;
    plan_type = new_plan_type;
    control_mutex.unlock();
  }

  bool get_new_plan() {
    control_mutex.lock();
    bool ret = new_plan;
    control_mutex.unlock();
    return ret;
  }

  uint8_t get_plan_type() {
    uint8_t value;
    control_mutex.lock();
    value = plan_type;
    control_mutex.unlock();
    return value;
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

  void set_arm_rpy(int i, double value) {
    control_mutex.lock();
    arm_euler[i] = value;
    control_mutex.unlock();
  }
  
  void set_arm_pos(int i, double value) {
    control_mutex.lock();
    arm_position[i] = value;
    control_mutex.unlock();
  }

  double get_arm_rpy(int i) {
    double ret;
    control_mutex.lock();
    ret = arm_euler[i];
    control_mutex.unlock();
    return ret;
  }

  double get_arm_pos(int i) {
    double ret;
    control_mutex.lock();
    ret = arm_position[i];
    control_mutex.unlock();
    return ret;
  }


  void set_home(bool value) {
    control_mutex.lock();
    should_home = value;
    control_mutex.unlock();
  }

  bool get_home() {
    control_mutex.lock();
    bool ret = should_home;
    control_mutex.unlock();
    return ret;
  }

  void set_vel_scale(double value) {
    control_mutex.lock();
    vel_scale = value;
    control_mutex.unlock();
  }  

  double get_vel_scale() {
    double ret;
    control_mutex.lock();
    ret = vel_scale;
    control_mutex.unlock();
    return ret;
  }

  void set_acc_scale(double value) {
    control_mutex.lock();
    acc_scale = value;
    control_mutex.unlock();
  }  

  double get_acc_scale() {
    double ret;
    control_mutex.lock();
    ret = acc_scale;
    control_mutex.unlock();
    return ret;
  }

  void set_joint_position(double position, uint8_t index) {
    control_mutex.lock();
    joint_position[index] = position;
    control_mutex.unlock();
  }

  double get_joint_position(uint8_t index) {
    double value;
    control_mutex.lock();
    value = joint_position[index];
    control_mutex.unlock();

    return value;
  }

  std::vector<double> get_joint_positions() {
    std::vector<double> values;
    control_mutex.lock();
    values = joint_position;
    control_mutex.unlock();

    return values;
  }

  Arm_control_class(uint8_t num_joints) {

    control_mutex.lock();
    // For some reason the AR3 starts with joint 1 instead of 0, 
    // so I'll pad this vector with a (0) index so everything lines up elsewhere
    for (auto i=0; i <= num_joints; ++i) {
      joint_position.push_back(0.0f);
    }
    control_mutex.unlock();
  }

};

sensor_msgs::JointState proposed_joint_state;
Arm_control_class arm_control(6);
Arm_control_class arm_planning(6);
Arm_control_class new_arm_planning(6);


bool plan_pose(moveit::planning_interface::MoveGroupInterface *move_group, geometry_msgs::Pose my_pose, ros::Publisher planned_joints_pub){

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  move_group->setPoseTarget(my_pose);
  move_group->setMaxVelocityScalingFactor(arm_control.get_vel_scale());
  move_group->setMaxAccelerationScalingFactor(arm_control.get_acc_scale());

  move_group->setPlanningTime(0.3);
  
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


bool plan_joints(moveit::planning_interface::MoveGroupInterface *move_group, std::vector<double> joint_group_positions, ros::Publisher planned_joints_pub) {

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

joint_group_positions.erase(joint_group_positions.begin());

  move_group->setJointValueTarget(joint_group_positions);
  move_group->setMaxVelocityScalingFactor(arm_control.get_vel_scale());
  move_group->setMaxAccelerationScalingFactor(arm_control.get_acc_scale());

  bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("msg", "Planning for Joints %s", success ? "SUCCESS" : "FAILED");

  for (int i=0; i < joint_group_positions.size(); ++i) {
    ROS_INFO_NAMED("msg", "joint %i plan angle %f", i+1, joint_group_positions[i]);
  }

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
}


void plan_and_move(const std_msgs::Bool::ConstPtr& msg) {
  // msg->data;
  ROS_INFO_NAMED("msg", "plan_and_move");
  arm_control.set_new_move(true);
}

void home(const std_msgs::Bool::ConstPtr& msg) {
  // msg->data;
  ROS_INFO_NAMED("msg", "home");
  arm_control.set_home(true);
  arm_control.set_new_plan(true);
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

void new_vel_value(const std_msgs::Float64::ConstPtr& msg) {

  arm_control.set_vel_scale(msg->data);
  ROS_INFO_NAMED("msg", "new vel scale value %f", msg->data);

}

void new_acc_value(const std_msgs::Float64::ConstPtr& msg) {

  arm_control.set_acc_scale(msg->data);
  ROS_INFO_NAMED("msg", "new acc scale value %f", msg->data);

}

void new_joint_value(const std_msgs::Float64::ConstPtr& msg, uint8_t index) {

  arm_control.set_joint_position(msg->data, index);
  ROS_INFO_NAMED("msg", "new joing %i value %f", index, msg->data);
  arm_control.set_new_plan(true, arm_control.JOINT_PLAN);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "formant_ar3_control_demo");
  ros::NodeHandle node_handle;

  arm_planning.set_arm_x(-0.007443f);
  arm_planning.set_arm_y(-0.064925f);
  arm_planning.set_arm_z(0.717915f);

  arm_planning.set_arm_roll(0.0f);
  arm_planning.set_arm_pitch(0.0f);
  arm_planning.set_arm_yaw(0.0f);

  // Initialize the message;
  proposed_joint_state.name = {"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"};
  proposed_joint_state.velocity = {0.0,0.0,0.0,0.0,0.0,0.0};
  proposed_joint_state.effort = {0.0,0.0,0.0,0.0,0.0,0.0};
  proposed_joint_state.position = {0.0,0.0,0.0,0.0,0.0,0.0};

  ros::Publisher planned_joints_pub = node_handle.advertise<sensor_msgs::JointState>("/joint_states_planned", 1000);
  

  ros::Subscriber sub = node_handle.subscribe("formant_plan_move", 1000, plan_and_move);
  ros::Subscriber home_sub = node_handle.subscribe("home", 1000, home);

  ros::Subscriber sub_x = node_handle.subscribe("ar3_x", 1000, new_x_value);
  ros::Subscriber sub_y = node_handle.subscribe("ar3_y", 1000, new_y_value);
  ros::Subscriber sub_z = node_handle.subscribe("ar3_z", 1000, new_z_value);

  ros::Subscriber sub_roll  = node_handle.subscribe("ar3_roll",  1000, new_roll_value);
  ros::Subscriber sub_pitch = node_handle.subscribe("ar3_pitch", 1000, new_pitch_value);
  ros::Subscriber sub_yaw   = node_handle.subscribe("ar3_yaw",   1000, new_yaw_value);

  ros::Subscriber sub_vel   = node_handle.subscribe("ar3_vel_scale",   1000, new_vel_value);
  ros::Subscriber sub_acc   = node_handle.subscribe("ar3_acc_scale",   1000, new_acc_value);

  ros::Subscriber joint_1_sub = node_handle.subscribe<std_msgs::Float64>("joint_1_angle", 1000, boost::bind(new_joint_value, _1, 1));
  ros::Subscriber joint_2_sub = node_handle.subscribe<std_msgs::Float64>("joint_2_angle", 1000, boost::bind(new_joint_value, _1, 2));
  ros::Subscriber joint_3_sub = node_handle.subscribe<std_msgs::Float64>("joint_3_angle", 1000, boost::bind(new_joint_value, _1, 3));
  ros::Subscriber joint_4_sub = node_handle.subscribe<std_msgs::Float64>("joint_4_angle", 1000, boost::bind(new_joint_value, _1, 4));
  ros::Subscriber joint_5_sub = node_handle.subscribe<std_msgs::Float64>("joint_5_angle", 1000, boost::bind(new_joint_value, _1, 5));
  ros::Subscriber joint_6_sub = node_handle.subscribe<std_msgs::Float64>("joint_6_angle", 1000, boost::bind(new_joint_value, _1, 6));

  ros::AsyncSpinner spinner(1);
  spinner.start();


  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  bool should_home = true;
  bool successful_plan = false;
  geometry_msgs::Pose last_good_pose;

  double pos_thresh = 0.5f;
  double rpy_thresh = 0.5f;
  double pos_scaler = 0.01f;
  double rpy_scaler = 0.03f;

  // Reset values to last known good state
  for (int i=0; i < 3; ++i) {
    new_arm_planning.set_arm_pos(i, arm_planning.get_arm_pos(i));
    new_arm_planning.set_arm_rpy(i, arm_planning.get_arm_rpy(i));

  }


  ros::Rate rate(30);
  while (ros::ok()) {


// move_group.setEndEffectorLink("link_6");
// geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
//       ROS_INFO_NAMED("msg", "current orientation %f %f %f %f and Position %f %f %f", current_pose.pose.orientation.w, 
//                                                                                           current_pose.pose.orientation.x, 
//                                                                                           current_pose.pose.orientation.y, 
//                                                                                           current_pose.pose.orientation.z,
//                                                                                           current_pose.pose.position.x,
//                                                                                           current_pose.pose.position.y,
//                                                                                           current_pose.pose.position.z);


  // Update the planning info with new rates
  double new_xyz_rates[3];
  double new_rpy_rates[3];
  double xyz_pos[3];
  double rpy_angles[3];


  for (int i=0; i < 3; ++i) {
    new_xyz_rates[i] = arm_control.get_arm_pos(i);
    new_rpy_rates[i] = arm_control.get_arm_rpy(i);
    xyz_pos[i] = new_arm_planning.get_arm_pos(i);
    rpy_angles[i] = new_arm_planning.get_arm_rpy(i);


    // ROS_INFO_NAMED("msg", "index %i data %f %f %f %f", i, xyz_pos[i], new_xyz_rates[i], rpy_angles[i], new_rpy_rates[i]);

    if (abs(new_xyz_rates[i]) > pos_thresh) {

      new_arm_planning.set_arm_pos(i, xyz_pos[i] + new_xyz_rates[i]*pos_scaler);
      ROS_INFO_NAMED("msg", "Changing xyz %i to %f",i, xyz_pos[i] + new_xyz_rates[i]*pos_scaler);
      arm_control.set_new_plan(true);

    }

    if (abs(new_rpy_rates[i]) > rpy_thresh) {
      new_arm_planning.set_arm_rpy(i, rpy_angles[i] + new_rpy_rates[i]*rpy_scaler);
      // ROS_INFO_NAMED("msg", "Changing rpy %i",i);
      arm_control.set_new_plan(true);
    }
  }


  // if we got a home command reset our values
  if (arm_control.get_home()) {
    arm_planning.set_arm_x(-0.007443f);
    arm_planning.set_arm_y(-0.064925f);
    arm_planning.set_arm_z(0.717915f);

    arm_planning.set_arm_roll(0.0f);
    arm_planning.set_arm_pitch(0.0f);
    arm_planning.set_arm_yaw(0.0f);

    // Reset values to home
    for (int i=0; i < 3; ++i) {
      new_arm_planning.set_arm_pos(i, arm_planning.get_arm_pos(i));
      new_arm_planning.set_arm_rpy(i, arm_planning.get_arm_rpy(i));

    }

    arm_control.set_home(false);
    should_home = true;

  }




    geometry_msgs::Pose my_pose;
    double roll, pitch, yaw;
    tf2::Quaternion static_quat; // x,y,z,w
    static_quat[0] = 0.0000f;   // x
    static_quat[1] = 0.0000f;   // y
    static_quat[2] = 1.0000f;   // z
    static_quat[3] = 0.0000f;   // w
    tf2::Quaternion quat;


    if (arm_control.get_new_move() || arm_control.get_new_plan() || should_home) {

      my_pose.position.x = new_arm_planning.get_arm_pos(0);
      my_pose.position.y = new_arm_planning.get_arm_y();
      my_pose.position.z = new_arm_planning.get_arm_z();

      quat.setRPY( new_arm_planning.get_arm_roll(), 
                   new_arm_planning.get_arm_pitch(), 
                   new_arm_planning.get_arm_yaw() );

      // ROS_INFO_NAMED("msg", "what %f %f %f %f",my_pose.position.x, new_arm_planning.get_arm_pos(0), new_arm_planning.get_arm_x(), 1.0);
      // quat.setRPY( 0, 0, 0 );
      quat *= static_quat;
      quat.normalize();
    }


    // Plan if we need to
    if (arm_control.get_new_plan() || should_home) {
      

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

      bool success;
      if (arm_control.get_plan_type() == arm_control.POSE_PLAN || should_home) {
        success = plan_pose(&move_group, my_pose, planned_joints_pub);
      } else if (arm_control.get_plan_type() == arm_control.JOINT_PLAN) {
        success = plan_joints(&move_group, arm_control.get_joint_positions(), planned_joints_pub);
      }

      // We only need to capture if the plan was successful since we'll store the last good plan
      if (success) {
        successful_plan = true;
        last_good_pose = my_pose;
        // update the last known good pose
        for (int i=0; i < 3; ++i) {
          arm_planning.set_arm_pos(i, new_arm_planning.get_arm_pos(i));
          arm_planning.set_arm_rpy(i, new_arm_planning.get_arm_rpy(i));

        }
      } else {
        // Reset values to last known good state
        for (int i=0; i < 3; ++i) {
          new_arm_planning.set_arm_pos(i, arm_planning.get_arm_pos(i));
          new_arm_planning.set_arm_rpy(i, arm_planning.get_arm_rpy(i));

        }
      }

      arm_control.set_new_plan(false, arm_control.get_plan_type());
      if (should_home) should_home = false;
    }


    // Move if we need to
    if (arm_control.get_new_move()) {

      // my_pose.orientation.w = quat[3];
      // my_pose.orientation.x = quat[0];
      // my_pose.orientation.y = quat[1];
      // my_pose.orientation.z = quat[2];

      // ROS_INFO_NAMED("msg", "Planning for orientation %f %f %f %f and Position %f %f %f", my_pose.orientation.w, 
      //                                                                                     my_pose.orientation.x, 
      //                                                                                     my_pose.orientation.y, 
      //                                                                                     my_pose.orientation.z,
      //                                                                                     my_pose.position.x,
      //                                                                                     my_pose.position.y,
      //                                                                                     my_pose.position.z);

      // bool success = plan_pose(&move_group, my_pose, planned_joints_pub);

      // if (success) {
      if (successful_plan) {
        bool success;
        if (arm_control.get_plan_type() == arm_control.POSE_PLAN) {
          ROS_INFO_NAMED("msg", "Planning pose for move command");
          success = plan_pose(&move_group, my_pose, planned_joints_pub);
        } else if (arm_control.get_plan_type() == arm_control.JOINT_PLAN) {
          ROS_INFO_NAMED("msg", "Planning pose for joint command");
          success = plan_joints(&move_group, arm_control.get_joint_positions(), planned_joints_pub);
        }
        ROS_INFO_NAMED("msg", "Commanding Move");
        move_group.move();
        ROS_INFO_NAMED("msg", "Move complete");
      }        

      arm_control.set_new_move(false);


    }

    if (successful_plan) planned_joints_pub.publish(proposed_joint_state);

    rate.sleep();
  }


  ros::shutdown();
  return 0;
}
