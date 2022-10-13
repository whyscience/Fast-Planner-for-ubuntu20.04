#include <iostream>
#include <string.h>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
//#include <dynamic_reconfigure/server.h>
//#include <so3_disturbance_generator/DisturbanceUIConfig.h>
#include "pose_utils.h"

using namespace arma;
using namespace std;

#define CORRECTION_RATE 1

rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubo;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubc;
rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pubf;
rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pubm;
//so3_disturbance_generator::DisturbanceUIConfig config;
nav_msgs::msg::Odometry noisy_odom;
geometry_msgs::msg::PoseStamped correction;

struct DynamicConfig {
  double fxy;
  double stdfxy;
  double fz;
  double stdfz;
  double mrp;
  double stdmrp;
  double myaw;
  double stdmyaw;
  bool enable_noisy_odom;
  double stdxyz;
  double stdvxyz;
  double stdrp;
  double stdyaw;
  bool enable_drift_odom;
  double stdvdriftxyz;
  double stdvdriftyaw;
  double vdriftx;
  double vdrifty;
  double vdriftz;
  double vdriftyaw;
  bool place_holder;
  bool state;
  std::string name;
};
DynamicConfig config{};

void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg) {
  noisy_odom.header = msg->header;
  correction.header = msg->header;
  // Get odom
  colvec pose(6);
  colvec vel(3);
  pose(0) = msg->pose.pose.position.x;
  pose(1) = msg->pose.pose.position.y;
  pose(2) = msg->pose.pose.position.z;
  colvec q = zeros<colvec>(4);
  q(0) = msg->pose.pose.orientation.w;
  q(1) = msg->pose.pose.orientation.x;
  q(2) = msg->pose.pose.orientation.y;
  q(3) = msg->pose.pose.orientation.z;
  pose.rows(3, 5) = R_to_ypr(quaternion_to_R(q));
  vel(0) = msg->twist.twist.linear.x;
  vel(1) = msg->twist.twist.linear.y;
  vel(2) = msg->twist.twist.linear.z;
  // Drift Odom 
  static colvec drift_pose = pose;
  static colvec drift_vel = vel;
  static colvec correction_pose = zeros<colvec>(6);
  static colvec prev_pose = pose;
  static rclcpp::Time prev_pose_t = msg->header.stamp;
  if (config.enable_drift_odom) {
    double dt = rclcpp::Time(msg->header.stamp).seconds() - prev_pose_t.seconds();
    prev_pose_t = msg->header.stamp;
    colvec d = pose_update(pose_inverse(prev_pose), pose);
    prev_pose = pose;
    d(0) += (config.vdriftx + config.stdvdriftxyz * as_scalar(randn(1))) * dt;
    d(1) += (config.vdrifty + config.stdvdriftxyz * as_scalar(randn(1))) * dt;
    d(2) += (config.vdriftz + config.stdvdriftxyz * as_scalar(randn(1))) * dt;
    d(3) += (config.vdriftyaw + config.stdvdriftyaw * as_scalar(randn(1))) * dt;
    drift_pose = pose_update(drift_pose, d);
    drift_vel = ypr_to_R(drift_pose.rows(3, 5)) * trans(ypr_to_R(pose.rows(3, 5))) * vel;
    correction_pose = pose_update(pose, pose_inverse(drift_pose));
  } else {
    drift_pose = pose;
    drift_vel = vel;
    correction_pose = zeros<colvec>(6);
  }
  // Noisy Odom
  static colvec noisy_pose = drift_pose;
  static colvec noisy_vel = drift_vel;
  if (config.enable_noisy_odom) {
    colvec noise_pose = zeros<colvec>(6);
    colvec noise_vel = zeros<colvec>(3);
    noise_pose(0) = config.stdxyz * as_scalar(randn(1));
    noise_pose(1) = config.stdxyz * as_scalar(randn(1));
    noise_pose(2) = config.stdxyz * as_scalar(randn(1));
    noise_pose(3) = config.stdyaw * as_scalar(randn(1));
    noise_pose(4) = config.stdrp * as_scalar(randn(1));
    noise_pose(5) = config.stdrp * as_scalar(randn(1));
    noise_vel(0) = config.stdvxyz * as_scalar(randn(1));
    noise_vel(1) = config.stdvxyz * as_scalar(randn(1));
    noise_vel(2) = config.stdvxyz * as_scalar(randn(1));
    noisy_pose = drift_pose + noise_pose;
    noisy_vel = drift_vel + noise_vel;
    noisy_odom.pose.covariance[0 + 0 * 6] = config.stdxyz * config.stdxyz;
    noisy_odom.pose.covariance[1 + 1 * 6] = config.stdxyz * config.stdxyz;
    noisy_odom.pose.covariance[2 + 2 * 6] = config.stdxyz * config.stdxyz;
    noisy_odom.pose.covariance[(0 + 3) + (0 + 3) * 6] = config.stdyaw * config.stdyaw;
    noisy_odom.pose.covariance[(1 + 3) + (1 + 3) * 6] = config.stdrp * config.stdrp;
    noisy_odom.pose.covariance[(2 + 3) + (2 + 3) * 6] = config.stdrp * config.stdrp;
    noisy_odom.twist.covariance[0 + 0 * 6] = config.stdvxyz * config.stdvxyz;
    noisy_odom.twist.covariance[1 + 1 * 6] = config.stdvxyz * config.stdvxyz;
    noisy_odom.twist.covariance[2 + 2 * 6] = config.stdvxyz * config.stdvxyz;
  } else {
    noisy_pose = drift_pose;
    noisy_vel = drift_vel;
    noisy_odom.pose.covariance[0 + 0 * 6] = 0;
    noisy_odom.pose.covariance[1 + 1 * 6] = 0;
    noisy_odom.pose.covariance[2 + 2 * 6] = 0;
    noisy_odom.pose.covariance[(0 + 3) + (0 + 3) * 6] = 0;
    noisy_odom.pose.covariance[(1 + 3) + (1 + 3) * 6] = 0;
    noisy_odom.pose.covariance[(2 + 3) + (2 + 3) * 6] = 0;
    noisy_odom.twist.covariance[0 + 0 * 6] = 0;
    noisy_odom.twist.covariance[1 + 1 * 6] = 0;
    noisy_odom.twist.covariance[2 + 2 * 6] = 0;
  }
  // Assemble and publish odom
  noisy_odom.pose.pose.position.x = noisy_pose(0);
  noisy_odom.pose.pose.position.y = noisy_pose(1);
  noisy_odom.pose.pose.position.z = noisy_pose(2);
  noisy_odom.twist.twist.linear.x = noisy_vel(0);
  noisy_odom.twist.twist.linear.y = noisy_vel(1);
  noisy_odom.twist.twist.linear.z = noisy_vel(2);
  colvec noisy_q = R_to_quaternion(ypr_to_R(noisy_pose.rows(3, 5)));
  noisy_odom.pose.pose.orientation.w = noisy_q(0);
  noisy_odom.pose.pose.orientation.x = noisy_q(1);
  noisy_odom.pose.pose.orientation.y = noisy_q(2);
  noisy_odom.pose.pose.orientation.z = noisy_q(3);
  pubo->publish(noisy_odom);
  // Check time interval and publish correction
  static rclcpp::Time prev_correction_t = msg->header.stamp;
  if (rclcpp::Time(msg->header.stamp).seconds() - prev_correction_t.seconds() > 1.0 / CORRECTION_RATE) {
    prev_correction_t = msg->header.stamp;
    correction.pose.position.x = correction_pose(0);
    correction.pose.position.y = correction_pose(1);
    correction.pose.position.z = correction_pose(2);
    colvec correction_q = R_to_quaternion(ypr_to_R(correction_pose.rows(3, 5)));
    correction.pose.orientation.w = correction_q(0);
    correction.pose.orientation.x = correction_q(1);
    correction.pose.orientation.y = correction_q(2);
    correction.pose.orientation.z = correction_q(3);
    pubc->publish(correction);
  }
}

/*void config_callback(so3_disturbance_generator::DisturbanceUIConfig &_config, uint32_t level) {
  config = _config;
}*/

void set_disturbance() {
  geometry_msgs::msg::Vector3 f;
  geometry_msgs::msg::Vector3 m;
  f.x = config.fxy + config.stdfxy * as_scalar(randn(1));
  f.y = config.fxy + config.stdfxy * as_scalar(randn(1));
  f.z = config.fz + config.stdfz * as_scalar(randn(1));
  m.x = config.mrp + config.stdmrp * as_scalar(randn(1));
  m.y = config.mrp + config.stdmrp * as_scalar(randn(1));
  m.z = config.myaw + config.stdmyaw * as_scalar(randn(1));
  pubf->publish(f);
  pubm->publish(m);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr n;

  auto sub1 = n->create_subscription<nav_msgs::msg::Odometry>("odom", 10, odom_callback);
  pubo = n->create_publisher<nav_msgs::msg::Odometry>("noisy_odom", 10);
  pubc = n->create_publisher<geometry_msgs::msg::PoseStamped>("correction", 10);
  pubf = n->create_publisher<geometry_msgs::msg::Vector3>("force_disturbance", 10);
  pubm = n->create_publisher<geometry_msgs::msg::Vector3>("moment_disturbance", 10);

  n->declare_parameter<double>("fxy", 0.0);
  n->declare_parameter<double>("stdfxy", 0.0);
  n->declare_parameter<double>("fz", 0.0);
  n->declare_parameter<double>("stdfz", 0.0);
  n->declare_parameter<double>("mrp", 0.0);
  n->declare_parameter<double>("stdmrp", 0.0);
  n->declare_parameter<double>("myaw", 0.0);
  n->declare_parameter<double>("stdmyaw", 0.0);
  n->declare_parameter<double>("stdxyz", 0.0);
  n->declare_parameter<double>("stdvxyz", 0.0);
  n->declare_parameter<double>("stdrp", 0.0);
  n->declare_parameter<double>("stdyaw", 0.0);
  n->declare_parameter<double>("stdvdriftxyz", 0.0);
  n->declare_parameter<double>("stdvdriftyaw", 0.0);
  n->declare_parameter<double>("vdriftx", 0.0);
  n->declare_parameter<double>("vdrifty", 0.0);
  n->declare_parameter<double>("vdriftz", 0.0);
  n->declare_parameter<double>("vdriftyaw", 0.0);
  n->declare_parameter<bool>("enable_noisy_odom", false);
  n->declare_parameter<bool>("enable_drift_odom", true);
  n->declare_parameter<bool>("place_holder", true);
  n->declare_parameter<bool>("state", true);
  n->declare_parameter<std::string>("name", "");

  n->get_parameter("fxy", config.fxy);
  n->get_parameter("stdfxy", config.stdfxy);
  n->get_parameter("fz", config.fz);
  n->get_parameter("stdfz", config.stdfz);
  n->get_parameter("mrp", config.mrp);
  n->get_parameter("stdmrp", config.stdmrp);
  n->get_parameter("myaw", config.myaw);
  n->get_parameter("stdmyaw", config.stdmyaw);
  n->get_parameter("stdxyz", config.stdxyz);
  n->get_parameter("stdvxyz", config.stdvxyz);
  n->get_parameter("stdrp", config.stdrp);
  n->get_parameter("stdyaw", config.stdyaw);
  n->get_parameter("stdvdriftxyz", config.stdvdriftxyz);
  n->get_parameter("stdvdriftyaw", config.stdvdriftyaw);
  n->get_parameter("vdriftx", config.vdriftx);
  n->get_parameter("vdrifty", config.vdrifty);
  n->get_parameter("vdriftz", config.vdriftz);
  n->get_parameter("vdriftyaw", config.vdriftyaw);
  n->get_parameter("enable_noisy_odom", config.enable_noisy_odom);
  n->get_parameter("enable_drift_odom", config.enable_drift_odom);
  n->get_parameter("place_holder", config.place_holder);
  n->get_parameter("state", config.state);
  n->get_parameter("name", config.name);

  // Dynamic Reconfig
  //  dynamic_reconfigure::Server<so3_disturbance_generator::DisturbanceUIConfig> server;
  //  dynamic_reconfigure::Server<so3_disturbance_generator::DisturbanceUIConfig>::CallbackType ff;
  // ff = std::bind(&config_callback, _1, _2);
  // server.setCallback(ff);

  rclcpp::Rate r(100.0);
  while (rclcpp::ok()) {
    rclcpp::spin_some(n);
    set_disturbance();
    r.sleep();
  }

  return 0;
}
