#ifndef _ROS_H_
#define _ROS_H_

#include "ros/node_handle.h"
#include "hROSHardware.h"

namespace ros
{
  typedef NodeHandle_<hROSHardware> NodeHandle;
}

#endif
