#ifndef CONTROLLER1_H
#define CONTROLLER1_H

#include <ros/ros.h>
#include "rosflight_msgs/Command.h"
#include <nav_msgs/Odometry.h>

namespace controller
{
  typedef struct
  {
    // Position
    double pn; // N
    double pe; // E
    double pd; // D

    double x_dot;
    double y_dot;
    double z_dot;
  } state_t;

 typedef struct
  {
    double x;
    double y;
    double z;
    double F;
  } input_t;

  class Controller
  {
    public:
      Controller(ros::NodeHandle *nh);
    private:
      ros::NodeHandle _nh;

      ros::Subscriber _odom_subscriber;
      ros::Publisher _command_publisher;

      void odomCallback(const nav_msgs::OdometryConstPtr &msg);
      void computeControl();
      void publishCommand();

      state_t _x; // TODO change to estimate
      state_t _x_ref;
      input_t _u;
      double _eq_thrust;

      double _kp;
      double _kd;
  };
}

#endif
