#include "rosflight/controller/controller.h"

namespace controller {
  Controller::Controller(ros::NodeHandle *nh) : _nh(*nh) {
    _x_ref.pd = -1.0; // TODO ref height
    _kp = 0.1;
    _kd = 0.1;
    _eq_thrust = 0.542;
    _u = {0, 0, 0, 0};

    _odom_subscriber = _nh.subscribe("/multirotor/truth/NED", 1000,
        &Controller::odomCallback, this);
    _command_publisher = _nh.advertise<rosflight_msgs::Command>(
        "/command", 1000);

    ROS_INFO("Controller node init complete");
  }

  void Controller::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    // NOTE Given in NED by simulation
    _x.pn = msg->pose.pose.position.x;
    _x.pe = msg->pose.pose.position.y;
    _x.pd = msg->pose.pose.position.z;

    _x.x_dot = msg->twist.twist.linear.x;
    _x.y_dot = msg->twist.twist.linear.y;
    _x.z_dot = msg->twist.twist.linear.z;
    
    computeControl();
    publishCommand();
  }

  void Controller::computeControl()
  {
    double e_pd = _x_ref.pd - _x.pd;
    _u.F = - _kp * e_pd + _kd * _x.z_dot + _eq_thrust;

    ROS_INFO("e_pd: %f, _x.z_dot: %f", e_pd, _x.z_dot);
  }
  
  void Controller::publishCommand()
  {
    rosflight_msgs::Command _command;
    _command.header.stamp = ros::Time::now();
    _command.ignore = rosflight_msgs::Command::IGNORE_NONE;
    _command.mode = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
    _command.F = _u.F;
    _command.x = _u.x;
    _command.y = _u.y;
    _command.z = _u.z;

    _command_publisher.publish(_command);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "norobo_controller");
  ros::NodeHandle nh;
  controller::Controller c = controller::Controller(&nh);
  ros::spin();

  return 0;
}
