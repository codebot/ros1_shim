#ifndef ROS_PARAM_H
#define ROS_PARAM_H

#include "ros/ros.h"

namespace ros
{

namespace param
{

template <class T>
void get(const std::string &name __attribute__((unused)), T &param)
{
  // should probably do something here sometime
}

}

}

#endif
