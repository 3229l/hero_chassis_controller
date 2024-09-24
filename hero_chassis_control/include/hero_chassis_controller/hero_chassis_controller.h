//
// Created by qiayuan on 2/6/21.
//

#ifndef HERO_CHASSIS_CONTROLLER_SIMPLE_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_SIMPLE_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_publisher.h>

// #define RADIUS 0.05

namespace hero_chassis_controller {

class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
 public:
  HeroChassisController() = default;
  ~HeroChassisController() override;

  bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

  void update(const ros::Time &time, const ros::Duration &period) override;

  ros::Subscriber sub_cmd_vel;
  hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;
 private:
  //Information of cmd_vel
  double vel_expected[5]{0.0};
  //wheel_velocity_actual
  double vel_actual[5]{0.0};
  //Calculate hero_move_pose
  double Vx_actual{0.0}, Vy_actual{0.0}, Vw_actual{0.0};
  //Information of geometry_msgs::Twist &msg
  double Vx_expected{0.0}, Vy_expected{0.0}, Vw_expected{0.0};
  //information 0f chassis
  double Wheel_Track{};//轮距
  double Wheel_Base{};//轴距
  double Wheel_Radius{};//轮子半径

  ros::Time last_time;
  ros::Time now;
  //CallBack of subscriber cmd_vel
  void get_chassis_pose(const geometry_msgs::Twist::ConstPtr &msg);
  //Calculate velocity of wheels
  void calculate_wheel_excepeted_velocity();
  //Calculate velocity of chassis
  void calculate_chassis_actual_velocity();

};
}// namespace hero_chassis_controller

#endif //HERO_CHASSIS_CONTROLLER_SIMPLE_CHASSIS_CONTROLLER_H
