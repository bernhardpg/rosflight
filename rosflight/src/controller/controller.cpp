#include "rosflight/controller/controller.h"

namespace controller {
  Controller::Controller(ros::NodeHandle *nh) : _nh(*nh) {
    _odom_subscriber = _nh.subscribe("/multirotor/truth/NED", 1000, &Controller::odomCallback, this);
    _command_publisher = _nh.advertise<rosflight_msgs::Command>("/command", 1000);

    ROS_INFO("Controller node init complete");
  }

  void Controller::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    ROS_INFO("%f", msg->pose.pose.position.x);
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
