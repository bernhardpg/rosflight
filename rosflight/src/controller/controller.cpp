#include "rosflight/controller/controller.h"

namespace controller {
  Controller::Controller(ros::NodeHandle *nh) : _nh(*nh) {
    _odom_subscriber = _nh.subscribe("/multirotor/truth/NED", 1000,
        &Controller::odomCallback, this);
    _command_publisher = _nh.advertise<rosflight_msgs::Command>(
        "/command", 1000);

    ROS_INFO("Controller node init complete");
  }

  void Controller::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    rosflight_msgs::Command _command;
    _command.header.stamp = ros::Time::now();
    _command.ignore = rosflight_msgs::Command::IGNORE_NONE;
    _command.mode = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
    _command.F = 0.9;
    _command.x = 0.;
    _command.y = 0.;
    _command.z = 0.;

    _command_publisher.publish(_command);

    //ROS_INFO("%f", msg->pose.pose.position.x);

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
