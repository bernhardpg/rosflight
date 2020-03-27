#ifndef CONTROLLER1_H
#define CONTROLLER1_H

#include <ros/ros.h>
#include "rosflight_msgs/Command.h"
#include <nav_msgs/Odometry.h>

namespace controller
{
  class Controller
  {
    public:
      Controller(ros::NodeHandle *nh);
    private:
      ros::NodeHandle _nh;

      ros::Subscriber _odom_subscriber;
      ros::Publisher _command_publisher;

      void odomCallback(const nav_msgs::OdometryConstPtr &msg);
  };
}

#endif
