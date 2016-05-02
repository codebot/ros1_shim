#include "ros/ros.h"
#include "ros/this_node.h"
#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "ros/time.h"
#include <boost/io/ios_state.hpp>

static ros::Shim g_shim;

void ros::init(int &argc, char **argv, const std::string &node_name, uint32_t /*options*/)
{
  ros::Time::init();
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

/*
bool ros::NodeHandle::ok()
{
  return rclcpp::ok();
}
*/

const std::string &ros::this_node::getName()
{
  return g_shim.node_name;
}

std::ostream &operator<<(std::ostream &os,
    const builtin_interfaces::msg::Time &rhs)
{
  os << rhs.sec << "." << std::setw(9) << std::setfill('0') << rhs.nanosec;
  return os;
}
