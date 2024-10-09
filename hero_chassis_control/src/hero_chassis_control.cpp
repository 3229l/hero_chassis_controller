//
// Created by lzy on 24-9-24.
//

#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller {

HeroChassisController::~HeroChassisController() {
  sub_cmd_vel.shutdown();
  odom_pub.shutdown();
}

bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                  ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
  controller_nh.getParam("Wheel_Track", Wheel_Track);
  controller_nh.getParam("Wheel_Base", Wheel_Base);
  controller_nh.getParam("Wheel_Radius", Wheel_Radius);
  controller_nh.getParam("Alpha", Alpha);
  controller_nh.getParam("Global_Coordinate_Mode", Global_Coordinate_Mode);
  controller_nh.getParam("Transform_Available", Transform_Available);
  controller_nh.getParam("Tf_Publish_Interval", Tf_Publish_Interval);

  front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
  front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
  back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
  back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

  pid_front_left_.init(ros::NodeHandle(controller_nh, "pid_front_left_"));
  pid_front_right_.init(ros::NodeHandle(controller_nh, "pid_front_right_"));
  pid_back_left_.init(ros::NodeHandle(controller_nh, "pid_back_left_"));
  pid_back_right_.init(ros::NodeHandle(controller_nh, "pid_back_right_"));
  // 动态重配置
  f = boost::bind(&HeroChassisController::reconfigureCallback, this, _1, _2);
  server_.setCallback(f);

  last_time = ros::Time::now();
  //start cmd_vel subscriber
  sub_cmd_vel = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &HeroChassisController::getChassisPose, this);
  // ROS_INFO("Subscribed to cmd_vel");
  //Start realtime state publisher
  controller_state_publisher_ = std::make_unique<realtime_tools::RealtimePublisher
      <control_msgs::JointControllerState>>(controller_nh, "state", 1);
  //Start odometry publisher
  odom_pub = root_nh.advertise<nav_msgs::Odometry>("odom", 50);

  return true;
}

void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {
  current_time = time;
  //excepted and actual velocity of wheels
  calculateWheelExcepetedVelocity();
  vel_actual[1] = front_left_joint_.getVelocity();
  vel_actual[2] = front_right_joint_.getVelocity();
  vel_actual[3] = back_left_joint_.getVelocity();
  vel_actual[4] = back_right_joint_.getVelocity();
  // ROS_INFO("Informations of : vel_actual \n"
  //              "vel_actual[1]:%f, vel_actual[2]:%f, vel_actual[3]:%f, vel_actual[4]:%f ",
  //              vel_actual[1], vel_actual[2], vel_actual[3], vel_actual[4]);
  calculateChassisActualVelocity();
  //broadcast Transform from "base_link" to "odom"
  if((current_time - last_tf_publish_time).toSec() >= Tf_Publish_Interval) {
    Transform_broadcast();
    last_tf_publish_time = current_time;
  }
  //publish the odometry message
  Odometry_publish();

  if(Global_Coordinate_Mode) {
    vector_in.header.frame_id = "odom";
    vector_in.header.stamp = ros::Time::now();
    vector_in.vector.x = Vx_expected;
    vector_in.vector.y = Vy_expected;
    try {
      listener.lookupTransform("base_link", "odom", ros::Time(0), transform);
      listener.transformVector("base_link", vector_in, vector_out);
      Vx_expected = vector_out.vector.x;
      Vy_expected = vector_out.vector.y;
      last_vector_out = vector_out;
      Transform_Available = true;
    }catch(tf::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      if (Transform_Available) {
        // 使用上一次成功的变换
        Vx_expected = last_vector_out.vector.x;
        Vy_expected = last_vector_out.vector.y;
      }
    }
  }
  //pid control
  for ( int i = 1; i <= 4; i++ ) {
    // 使用上一次的 vel_smoothed[i] 来进行平滑计算
    vel_smoothed[i] = Alpha * vel_actual[i] + (1 - Alpha) * vel_smoothed[i];
    error[i] = vel_expected[i] - vel_smoothed[i];
  }
  front_left_joint_.setCommand(pid_front_left_.computeCommand(error[1], period));
  front_right_joint_.setCommand(pid_front_right_.computeCommand(error[2],period));
  back_left_joint_.setCommand(pid_back_left_.computeCommand(error[3],period));
  back_right_joint_.setCommand(pid_back_right_.computeCommand(error[4],period));
  // ROS_INFO("Wheel Commands: FL: %f, FR: %f, BL: %f, BR: %f",
  //          pid_front_left_.computeCommand(error[1], period),
  //          pid_front_right_.computeCommand(error[2], period),
  //          pid_back_left_.computeCommand(error[3], period),
  //          pid_back_right_.computeCommand(error[4], period));

  if (loop_count_ % 10 == 0) {
    if (controller_state_publisher_ && controller_state_publisher_->trylock()) {
      controller_state_publisher_->msg_.header.stamp = current_time;
      controller_state_publisher_->msg_.set_point = vel_expected[1];
      controller_state_publisher_->msg_.process_value = vel_actual[1];
      controller_state_publisher_->msg_.error = error[1];
      controller_state_publisher_->msg_.time_step = period.toSec();
      controller_state_publisher_->msg_.command = pid_front_left_.computeCommand(error[1], period);

      double dummy;
      bool antiwindup;
      pid_front_left_.getGains(controller_state_publisher_->msg_.p,
                                controller_state_publisher_->msg_.i,
                                controller_state_publisher_->msg_.d,
                                controller_state_publisher_->msg_.i_clamp,
                                dummy,
                                antiwindup);
      controller_state_publisher_->msg_.antiwindup = static_cast<char>(antiwindup);
      controller_state_publisher_->unlockAndPublish();
    }
  }
  loop_count_++;
  last_time = current_time;
}

void HeroChassisController::getChassisPose(const geometry_msgs::Twist::ConstPtr &msg) {
  Vx_expected = msg->linear.x;
  Vy_expected = msg->linear.y;
  Vw_expected = msg->angular.z;
  // ROS_INFO("Informations of chassis_pose: \n"
  //          "Vx_expected:%f, Vy_expected:%f, Vw_expected:%f ",
  //          msg->linear.x, msg->linear.y, msg->angular.z);
}

void HeroChassisController::calculateWheelExcepetedVelocity() {
  vel_expected[1] = (Vx_expected - Vy_expected - Vw_expected * (Wheel_Base + Wheel_Track) / 2) / Wheel_Radius;
  vel_expected[2] = (Vx_expected + Vy_expected + Vw_expected * (Wheel_Base + Wheel_Track) / 2) / Wheel_Radius;
  vel_expected[3] = (Vx_expected + Vy_expected - Vw_expected * (Wheel_Base + Wheel_Track) / 2) / Wheel_Radius;
  vel_expected[4] = (Vx_expected - Vy_expected + Vw_expected * (Wheel_Base + Wheel_Track) / 2) / Wheel_Radius;
  // ROS_INFO("Informations of : vel_expected \n"
  //            "vel_expected[1]:%f, vel_expected[2]:%f, vel_expected[3]:%f, vel_expected[4]:%f ",
  //            vel_expected[1], vel_expected[2], vel_expected[3], vel_expected[4]);
}

void HeroChassisController::calculateChassisActualVelocity() {
  Vx_actual = (vel_actual[1] + vel_actual[2] + vel_actual[3] + vel_actual[4]) * Wheel_Radius / 4;
  Vy_actual = ( - vel_actual[1] + vel_actual[2] + vel_actual[3] - vel_actual[4]) * Wheel_Radius / 4;
  Vw_actual = ( - vel_actual[1] + vel_actual[2] - vel_actual[3] + vel_actual[4]) * Wheel_Radius / (2*Wheel_Base + 2*Wheel_Track);
}

void HeroChassisController::reconfigureCallback(hero_chassis_control::PidConfig &config, uint32_t level) {
  // 更新 PID 参数
  pid_front_left_.setGains(config.front_left_p, config.front_left_i, config.front_left_d,config.i_max, config.i_min, config.antiwindup);
  pid_front_right_.setGains(config.front_right_p, config.front_right_i, config.front_right_d,config.i_max, config.i_min, config.antiwindup);
  pid_back_left_.setGains(config.back_left_p, config.back_left_i, config.back_left_d,config.i_max, config.i_min, config.antiwindup);
  pid_back_right_.setGains(config.back_right_p, config.back_right_i, config.back_right_d,config.i_max, config.i_min, config.antiwindup);
}

void HeroChassisController::Transform_broadcast() {
  dt = (current_time - last_time).toSec();
  double delta_x = (Vx_actual * cos(th) - Vy_actual * sin(th)) * dt;
  double delta_y = (Vx_actual * sin(th) + Vy_actual * cos(th)) * dt;
  double delta_th = Vw_actual * dt;
  x += delta_x;
  y += delta_y;
  th += delta_th;

  odom_quat = tf::createQuaternionMsgFromYaw(th);
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";
  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;
  //send the transform
  odom_broadcaster.sendTransform(odom_trans);
}

void HeroChassisController::Odometry_publish() {
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";
  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = Vx_actual;
  odom.twist.twist.linear.y = Vy_actual;
  odom.twist.twist.angular.z = Vw_actual;
  //publish the message
  odom_pub.publish(odom);
}

}//namespace

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)