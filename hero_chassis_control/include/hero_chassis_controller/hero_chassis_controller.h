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
#include <control_toolbox/pid.h>
#include <dynamic_reconfigure/server.h>
#include <hero_chassis_control/PidConfig.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

// #define RADIUS 0.05

namespace hero_chassis_controller {

class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
 public:
  HeroChassisController() = default;
  ~HeroChassisController() override;

  bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

  void update(const ros::Time &time, const ros::Duration &period) override;

  void reconfigureCallback(hero_chassis_control::PidConfig &config, uint32_t level);

  ros::Subscriber sub_cmd_vel;
  hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;
  control_toolbox::Pid pid_front_left_, pid_front_right_, pid_back_left_, pid_back_right_;

  dynamic_reconfigure::Server<hero_chassis_control::PidConfig>::CallbackType f;
  dynamic_reconfigure::Server<hero_chassis_control::PidConfig> server_;

  ros::Publisher odom_pub;
  tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::Quaternion odom_quat;
  geometry_msgs::TransformStamped odom_trans;
  nav_msgs::Odometry odom;

  geometry_msgs::Vector3Stamped vector_in, vector_out, last_vector_out;
  tf::StampedTransform transform;
  tf::TransformListener listener;

private:
  int loop_count_{};
  //wheel_velocity_actual
  double vel_actual[5]{0.0};
  //Information of cmd_vel
  double vel_expected[5]{0.0};
  //Exponential Smoothing about vel_actual
  double Alpha;
  double vel_smoothed[5]{0.0};
  //Calculate hero_move_pose
  double Vx_actual{0.0}, Vy_actual{0.0}, Vw_actual{0.0};
  //Information of geometry_msgs::Twist &msg
  double Vx_expected{0.0}, Vy_expected{0.0}, Vw_expected{0.0};
  //Information 0f chassis
  double Wheel_Track{};//轮距
  double Wheel_Base{};//轴距
  double Wheel_Radius{};//轮子半径
  //Pid error
  double error[5]{0.0};
  //Odom pose
  double dt{};
  double x{0.0}, y{0.0}, th{0.0};
  bool Global_Coordinate_Mode{};
  bool Transform_Available;

  ros::Time last_time;
  ros::Time current_time;

  std::unique_ptr<
      realtime_tools::RealtimePublisher<
          control_msgs::JointControllerState> > controller_state_publisher_;
  //CallBack of subscriber cmd_vel
  void getChassisPose(const geometry_msgs::Twist::ConstPtr &msg);
  //Calculate velocity of wheels
  void calculateWheelExcepetedVelocity();
  //Calculate velocity of chassis
  void calculateChassisActualVelocity();
  void Transform_broadcast();
  void Odometry_publish();
};

}// namespace hero_chassis_controller

#endif //HERO_CHASSIS_CONTROLLER_SIMPLE_CHASSIS_CONTROLLER_H
