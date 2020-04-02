#ifndef CONTROLLER1_H
#define CONTROLLER1_H

#include <ros/ros.h>
#include "rosflight_msgs/Command.h"
#include <nav_msgs/Odometry.h>
#include <cmath>

namespace controller
{
  typedef struct
  {
    // Position
    double pn; // N
    double pe; // E
    double pd; // D

    double vn;
    double ve;
    double vd;

    // Using yaw-pitch-roll Euler angles
    double phi; // roll
    double theta; // pitch
    double psi; // yaw
  } state_t;

 typedef struct
  {
    double phi;
    double theta;
    double psi;
    double F;

    // Virtual inputs
    double n;
    double e;
    double d;
  } input_t;

 typedef struct
 {
  double pn;
  double pe;
  double pd;

  double dn;
  double de;
  double dd;
 } gain_t;

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

      double saturate(double v); // TODO move somewhere else

      state_t _x; // TODO change to estimate
      state_t _x_ref;
      input_t _u;
      gain_t _k;

      double _m;
      double _g;
      double _eq_thrust;
      double _max_thrust;
  };
}

#endif
