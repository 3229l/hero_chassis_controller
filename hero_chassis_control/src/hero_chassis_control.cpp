//
// Created by qiayuan on 2/6/21.
//

#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller {

HeroChassisController::~HeroChassisController() {
  sub_cmd_vel.shutdown();
}

bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                  ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
  controller_nh.getParam("Wheel_Track", Wheel_Track);
  controller_nh.getParam("Wheel_Base", Wheel_Base);
  controller_nh.getParam("Wheel_Radius", Wheel_Radius);

  front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
  front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
  back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
  back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

  last_time = ros::Time::now();
  sub_cmd_vel = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &HeroChassisController::get_chassis_pose, this);
  ROS_INFO("Subscribed to cmd_vel");

  return true;
}

void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {
  ROS_INFO("Update called");
  //excepted and actual velocity of wheels
  calculate_wheel_excepeted_velocity();
  vel_actual[1] = front_left_joint_.getVelocity();
  vel_actual[2] = front_right_joint_.getVelocity();
  vel_actual[3] = back_left_joint_.getVelocity();
  vel_actual[4] = back_right_joint_.getVelocity();

  calculate_chassis_actual_velocity();

   // 系统会根据控制器的输出值（如 PID 控制器的计算结果）调用 setCommand()
/*  front_left_joint_.setCommand(cmd_[state_][0]);
  front_right_joint_.setCommand(cmd_[state_][1]);
  back_left_joint_.setCommand(cmd_[state_][2]);
  back_right_joint_.setCommand(cmd_[state_][3]); */
}

void HeroChassisController::get_chassis_pose(const geometry_msgs::Twist::ConstPtr &msg) {
  Vx_expected = msg->linear.x;
  Vy_expected = msg->linear.y;
  Vw_expected = msg->angular.z;
  ROS_INFO("Informations of chassis_pose: "
           "Vx_expected:%f, Vy_expected:%f, Vw_expected:%f",
           msg->linear.x, msg->linear.y, msg->angular.z);
}

void HeroChassisController::calculate_wheel_excepeted_velocity() {
  vel_expected[1] = (Vx_expected - Vy_expected - Vw_expected * (Wheel_Base + Wheel_Track) / 2) / Wheel_Radius;
  vel_expected[2] = (Vx_expected + Vy_expected + Vw_expected * (Wheel_Base + Wheel_Track) / 2) / Wheel_Radius;
  vel_expected[3] = (Vx_expected + Vy_expected - Vw_expected * (Wheel_Base + Wheel_Track) / 2) / Wheel_Radius;
  vel_expected[4] = (Vx_expected - Vy_expected + Vw_expected * (Wheel_Base + Wheel_Track) / 2) / Wheel_Radius;
}

void HeroChassisController::calculate_chassis_actual_velocity() {
  Vx_actual = (vel_actual[1] + vel_actual[2] + vel_actual[3] + vel_actual[4]) * Wheel_Radius / 4;
  Vy_actual = ( - vel_actual[1] + vel_actual[2] + vel_actual[3] - vel_actual[4]) * Wheel_Radius / 4;
  Vw_actual = ( - vel_actual[1] + vel_actual[2] - vel_actual[3] + vel_actual[4]) * Wheel_Radius / (2*Wheel_Base + 2*Wheel_Track);
}

}//namespace

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)