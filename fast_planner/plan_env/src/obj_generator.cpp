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

#include "rclcpp/rclcpp.hpp"

#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <random>
#include <string>

#include <plan_env/linear_obj_model.hpp>
using namespace std;

int obj_num;
double _xy_size, _h_size, _vel, _yaw_dot, _acc_r1, _acc_r2, _acc_z, _scale1, _scale2, _interval;

rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obj_pub;            // visualize marker
vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_pubs;  // obj pose (from optitrack)
vector<LinearObjModel> obj_models;

random_device rd;
default_random_engine eng(rd());
uniform_real_distribution<double> rand_pos;
uniform_real_distribution<double> rand_h;
uniform_real_distribution<double> rand_vel;
uniform_real_distribution<double> rand_acc_r;
uniform_real_distribution<double> rand_acc_t;
uniform_real_distribution<double> rand_acc_z;
uniform_real_distribution<double> rand_color;
uniform_real_distribution<double> rand_scale;
uniform_real_distribution<double> rand_yaw_dot;
uniform_real_distribution<double> rand_yaw;

rclcpp::Time time_update, time_change;

void updateCallback();
void visualizeObj(int id);

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("dynamic_obj");

  /* ---------- initialize ---------- */
  //https://blog.csdn.net/qq_45701501/article/details/118611735
  node->declare_parameter<int>("obj_generator/obj_num", 10);
  node->declare_parameter<double>("obj_generator/xy_size", 15.0);
  node->declare_parameter<double>("obj_generator/h_size", 5.0);
  node->declare_parameter<double>("obj_generator/vel", 5.0);
  node->declare_parameter<double>("obj_generator/yaw_dot", 5.0);
  node->declare_parameter<double>("obj_generator/acc_r1", 4.0);
  node->declare_parameter<double>("obj_generator/acc_r2", 6.0);
  node->declare_parameter<double>("obj_generator/acc_z", 3.0);
  node->declare_parameter<double>("obj_generator/scale1", 1.5);
  node->declare_parameter<double>("obj_generator/scale2", 2.5);
  node->declare_parameter<double>("obj_generator/interval", 2.5);

  node->get_parameter("obj_generator/obj_num", obj_num);
  node->get_parameter("obj_generator/xy_size", _xy_size);
  node->get_parameter("obj_generator/h_size", _h_size);
  node->get_parameter("obj_generator/vel", _vel);
  node->get_parameter("obj_generator/yaw_dot", _yaw_dot);
  node->get_parameter("obj_generator/acc_r1", _acc_r1);
  node->get_parameter("obj_generator/acc_r2", _acc_r2);
  node->get_parameter("obj_generator/acc_z", _acc_z);
  node->get_parameter("obj_generator/scale1", _scale1);
  node->get_parameter("obj_generator/scale2", _scale2);
  node->get_parameter("obj_generator/interval", _interval);

  obj_pub = node->create_publisher<visualization_msgs::msg::Marker>("/dynamic/obj", 10);
  for (int i = 0; i < obj_num; ++i) {
    auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/dynamic/pose_" + to_string(i), 10);
    pose_pubs.push_back(pose_pub);
  }

  //https://roboticsbackend.com/write-minimal-ros2-cpp-node/
  auto update_timer = node->create_wall_timer(std::chrono::milliseconds(1000 / 3), updateCallback);
  cout << "[dynamic]: initialize with " + to_string(obj_num) << " moving obj." << endl;
  //https://www.reddit.com/r/ROS/comments/hzmlhw/error_with_using_sleep_with_the_duration_client/
  rclcpp::Rate sleepRate(1.0);
  sleepRate.sleep();

  rand_color = uniform_real_distribution<double>(0.0, 1.0);
  rand_pos = uniform_real_distribution<double>(-_xy_size, _xy_size);
  rand_h = uniform_real_distribution<double>(0.0, _h_size);
  rand_vel = uniform_real_distribution<double>(-_vel, _vel);
  rand_acc_t = uniform_real_distribution<double>(0.0, 6.28);
  rand_acc_r = uniform_real_distribution<double>(_acc_r1, _acc_r2);
  rand_acc_z = uniform_real_distribution<double>(-_acc_z, _acc_z);
  rand_scale = uniform_real_distribution<double>(_scale1, _scale2);
  rand_yaw = uniform_real_distribution<double>(0.0, 2 * 3.141592);
  rand_yaw_dot = uniform_real_distribution<double>(-_yaw_dot, _yaw_dot);

  /* ---------- give initial value of each obj ---------- */
  for (int i = 0; i < obj_num; ++i) {
    LinearObjModel model;
    Eigen::Vector3d pos(rand_pos(eng), rand_pos(eng), rand_h(eng));
    Eigen::Vector3d vel(rand_vel(eng), rand_vel(eng), 0.0);
    Eigen::Vector3d color(rand_color(eng), rand_color(eng), rand_color(eng));
    Eigen::Vector3d scale(rand_scale(eng), 1.5 * rand_scale(eng), rand_scale(eng));
    double yaw = rand_yaw(eng);
    double yaw_dot = rand_yaw_dot(eng);

    double r, t, z;
    r = rand_acc_r(eng);
    t = rand_acc_t(eng);
    z = rand_acc_z(eng);
    Eigen::Vector3d acc(r * cos(t), r * sin(t), z);

    model.initialize(pos, vel, acc, yaw, yaw_dot, color, scale);
    model.setLimits(Eigen::Vector3d(_xy_size, _xy_size, _h_size), Eigen::Vector2d(0.0, _vel),
                    Eigen::Vector2d(0, 0));
    obj_models.push_back(model);
  }

  //https://github.com/ros-controls/ros2_controllers/commit/e3f57ed938c9e17643e54a2c58026529bc481561
  time_update = rclcpp::Clock().now();
  time_change = rclcpp::Clock().now();

  /* ---------- start loop ---------- */
  rclcpp::spin(node);

  return 0;
}

void updateCallback() {
  rclcpp::Time time_now = rclcpp::Clock().now();

  /* ---------- change input ---------- */
  double dtc = (time_now - time_change).seconds();
  if (dtc > _interval) {
    for (int i = 0; i < obj_num; ++i) {
      /* ---------- use acc input ---------- */
      // double r, t, z;
      // r = rand_acc_r(eng);
      // t = rand_acc_t(eng);
      // z = rand_acc_z(eng);
      // Eigen::Vector3d acc(r * cos(t), r * sin(t), z);
      // obj_models[i].setInput(acc);

      /* ---------- use vel input ---------- */
      double vx, vy, vz, yd;
      vx = rand_vel(eng);
      vy = rand_vel(eng);
      vz = 0.0;
      yd = rand_yaw_dot(eng);

      obj_models[i].setInput(Eigen::Vector3d(vx, vy, vz));
      obj_models[i].setYawDot(yd);
    }
    time_change = time_now;
  }

  /* ---------- update obj state ---------- */
  double dt = (time_now - time_update).seconds();
  time_update = time_now;
  for (int i = 0; i < obj_num; ++i) {
    obj_models[i].update(dt);
    visualizeObj(i);
  }

  /* ---------- collision ---------- */
  for (int i = 0; i < obj_num; ++i)
    for (int j = i + 1; j < obj_num; ++j) {
      bool collision = LinearObjModel::collide(obj_models[i], obj_models[j]);
      if (collision) {
        double yd1 = rand_yaw_dot(eng);
        double yd2 = rand_yaw_dot(eng);
        obj_models[i].setYawDot(yd1);
        obj_models[j].setYawDot(yd2);
      }
    }
}

void visualizeObj(int id) {
  Eigen::Vector3d pos, color, scale;
  pos = obj_models[id].getPosition();
  color = obj_models[id].getColor();
  scale = obj_models[id].getScale();
  double yaw = obj_models[id].getYaw();

  Eigen::Matrix3d rot;
  rot << cos(yaw), -sin(yaw), 0.0, sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 1.0;

  Eigen::Quaterniond qua;
  qua = rot;

  /* ---------- rviz ---------- */
  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = "world";
  //  msg->header.stamp = this->now(); //todo eric replace allwith this
  mk.header.stamp = rclcpp::Clock().now();
  mk.type = visualization_msgs::msg::Marker::CUBE;
  mk.action = visualization_msgs::msg::Marker::ADD;
  mk.id = id;

  mk.scale.x = scale(0), mk.scale.y = scale(1), mk.scale.z = scale(2);
  mk.color.a = 0.5, mk.color.r = color(0), mk.color.g = color(1), mk.color.b = color(2);

  mk.pose.orientation.w = qua.w();
  mk.pose.orientation.x = qua.x();
  mk.pose.orientation.y = qua.y();
  mk.pose.orientation.z = qua.z();

  mk.pose.position.x = pos(0), mk.pose.position.y = pos(1), mk.pose.position.z = pos(2);

  obj_pub->publish(mk);

  /* ---------- pose ---------- */
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "world";
  //pose.header.seq = id;
  pose.pose.position.x = pos(0), pose.pose.position.y = pos(1), pose.pose.position.z = pos(2);
  pose.pose.orientation.w = 1.0;
  pose_pubs[id]->publish(pose);
}
