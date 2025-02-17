/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/



#include "bspline/non_uniform_bspline.h"
#include <nav_msgs/msg/odometry.hpp>
#include "quadrotor_msgs/msg/bspline.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "rclcpp/rclcpp.hpp"

rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cmd_vis_pub;
rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub;

nav_msgs::msg::Odometry odom;

quadrotor_msgs::msg::PositionCommand cmd;
// double pos_gain[3] = {5.7, 5.7, 6.2};
// double vel_gain[3] = {3.4, 3.4, 4.0};
double pos_gain[3] = {5.7, 5.7, 6.2};
double vel_gain[3] = {3.4, 3.4, 4.0};

using fast_planner::NonUniformBspline;

bool receive_traj_ = false;
vector<NonUniformBspline> traj_;
double traj_duration_;
rclcpp::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_;
double time_forward_;

vector<Eigen::Vector3d> traj_cmd_, traj_real_;

void displayTrajWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                          int id) {
  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = rclcpp::Clock().now();
  mk.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::msg::Marker::DELETE;
  mk.id = id;

  traj_pub->publish(mk);

  mk.action = visualization_msgs::msg::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::msg::Point pt;
  for (int i = 0; i < int(path.size()); i++) {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  traj_pub->publish(mk);
  //rclcpp::Duration(0.001).sleep();
  rclcpp::Rate sleepRate(1000);
  sleepRate.sleep();
}

void drawCmd(const Eigen::Vector3d &pos, const Eigen::Vector3d &vec, const int &id,
             const Eigen::Vector4d &color) {
  visualization_msgs::msg::Marker mk_state;
  mk_state.header.frame_id = "world";
  mk_state.header.stamp = rclcpp::Clock().now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::msg::Marker::ARROW;
  mk_state.action = visualization_msgs::msg::Marker::ADD;

  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;

  geometry_msgs::msg::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);

  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);

  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);

  cmd_vis_pub->publish(mk_state);
}

void bsplineCallback(quadrotor_msgs::msg::Bspline::SharedPtr msg) {
  // parse pos traj

  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);

  Eigen::VectorXd knots(msg->knots.size());
  for (int i = 0; i < msg->knots.size(); ++i) {
    knots(i) = msg->knots[i];
  }

  for (int i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(i, 0) = msg->pos_pts[i].x;
    pos_pts(i, 1) = msg->pos_pts[i].y;
    pos_pts(i, 2) = msg->pos_pts[i].z;
  }

  NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // parse yaw traj

  Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  for (int i = 0; i < msg->yaw_pts.size(); ++i) {
    yaw_pts(i, 0) = msg->yaw_pts[i];
  }

  NonUniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());
  traj_.push_back(yaw_traj);
  traj_.push_back(yaw_traj.getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;
}

void replanCallback(std_msgs::msg::Empty::SharedPtr msg) {
  /* reset duration */
  const double time_out = 0.01;
  rclcpp::Time time_now = rclcpp::Clock().now();
  double t_stop = (time_now - start_time_).seconds() + time_out;
  traj_duration_ = min(t_stop, traj_duration_);
}

void newCallback(std_msgs::msg::Empty::SharedPtr msg) {
  traj_cmd_.clear();
  traj_real_.clear();
}

void odomCallbck(nav_msgs::msg::Odometry::SharedPtr msg) {
  if (msg->child_frame_id == "X" || msg->child_frame_id == "O") return;

  odom = *msg;

  traj_real_.push_back(
      Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

  if (traj_real_.size() > 10000) traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 1000);
}

void visCallback() {
  // displayTrajWithColor(traj_real_, 0.03, Eigen::Vector4d(0.925, 0.054, 0.964,
  // 1),
  //                      1);

  displayTrajWithColor(traj_cmd_, 0.05, Eigen::Vector4d(0, 1, 0, 1), 2);
}

void cmdCallback() {
  /* no publishing before receive traj_ */
  if (!receive_traj_) return;

  rclcpp::Time time_now = rclcpp::Clock().now();
  double t_cur = (time_now - start_time_).seconds();

  Eigen::Vector3d pos{0, 0, 0}, vel{0, 0, 0}, acc{0, 0, 0}, pos_f{0, 0, 0};
  double yaw{}, yawdot{};

  if (t_cur < traj_duration_ && t_cur >= 0.0) {
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);
    yaw = traj_[3].evaluateDeBoorT(t_cur)[0];
    yawdot = traj_[4].evaluateDeBoorT(t_cur)[0];

    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);

  } else if (t_cur >= traj_duration_) {
    /* hover when finish traj_ */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();
    yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
    yawdot = traj_[4].evaluateDeBoorT(traj_duration_)[0];

    pos_f = pos;

  } else {
    cout << "[Traj server]: invalid time." << endl;
  }

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw;
  cmd.yaw_dot = yawdot;

  auto pos_err = pos_f - pos;
  // if (pos_err.norm() > 1e-3) {
  //   cmd.yaw = atan2(pos_err(1), pos_err(0));
  // } else {
  //   cmd.yaw = last_yaw_;
  // }
  // cmd.yaw_dot = 1.0;

  last_yaw_ = cmd.yaw;

  pos_cmd_pub->publish(cmd);

  // draw cmd

  // drawCmd(pos, vel, 0, Eigen::Vector4d(0, 1, 0, 1));
  // drawCmd(pos, acc, 1, Eigen::Vector4d(0, 0, 1, 1));

  Eigen::Vector3d dir(cos(yaw), sin(yaw), 0.0);
  drawCmd(pos, 2 * dir, 2, Eigen::Vector4d(1, 1, 0, 0.7));
  // drawCmd(pos, pos_err, 3, Eigen::Vector4d(1, 1, 0, 0.7));

  traj_cmd_.push_back(pos);
  if (traj_cmd_.size() > 10000) traj_cmd_.erase(traj_cmd_.begin(), traj_cmd_.begin() + 1000);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node;

  auto bspline_sub = node->create_subscription<quadrotor_msgs::msg::Bspline>("planning/bspline", 10, bsplineCallback);
  auto replan_sub = node->create_subscription<std_msgs::msg::Empty>("planning/replan", 10, replanCallback);
  auto new_sub = node->create_subscription<std_msgs::msg::Empty>("planning/new", 10, newCallback);
  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>("/odom_world", 50, odomCallbck);

  cmd_vis_pub = node->create_publisher<visualization_msgs::msg::Marker>("planning/position_cmd_vis", 10);
  pos_cmd_pub = node->create_publisher<quadrotor_msgs::msg::PositionCommand>("/position_cmd", 50);
  traj_pub = node->create_publisher<visualization_msgs::msg::Marker>("planning/travel_traj", 10);

  auto cmd_timer = node->create_wall_timer(std::chrono::milliseconds(10), cmdCallback);
  auto vis_timer = node->create_wall_timer(std::chrono::milliseconds(250), visCallback);

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  node->declare_parameter("traj_server/time_forward", -1.0);
  node->get_parameter("traj_server/time_forward", time_forward_);
  last_yaw_ = 0.0;

  rclcpp::Rate sleepRate(1);
  sleepRate.sleep();

  RCLCPP_WARN(node->get_logger(), "[Traj server]: ready.");

  rclcpp::spin(node);

  return 0;
}
