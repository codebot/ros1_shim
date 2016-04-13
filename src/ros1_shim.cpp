#include "ros/ros.h"
#include <cstdio>
#include "rclcpp/rclcpp.hpp"

static ros::Shim g_shim;

void ros::init(int &argc, char **argv, const std::string &node_name, uint32_t /*options*/)
{
  rclcpp::init(argc, argv);
  g_shim.node_name = node_name;
  g_shim.node = rclcpp::node::Node::make_shared(node_name);
}

ros::Shim *ros::Shim::get_shim() { return &g_shim; }

void ros::spinOnce()
{
  rclcpp::spin_some(g_shim.node);
}

ros::Rate::Rate(double hz)
: ros2_rate(hz)
{
}

bool ros::Rate::sleep()
{
  return ros2_rate.sleep();
}

bool ros::ok()
{
  return rclcpp::ok();
}

void ros::spin()
{
  rclcpp::spin(g_shim.node);
}
