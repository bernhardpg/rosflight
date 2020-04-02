#include "rosflight/controller/controller.h"

namespace controller {
  Controller::Controller(ros::NodeHandle *nh) : _nh(*nh) {
    _x_ref.pd = -1.0; // TODO ref height
    
    // P gains
    _k.pn = 0.1;
    _k.pe = 0.1;
    _k.pd = 14;

    // D gains
    _k.dn = 0.1;
    _k.de = 0.1;
    _k.dd = 7.0;
    _eq_thrust = 0.542;
    _max_thrust = 14.961 * 4; // From gazebo sim, 4 rotors
    _u = {0, 0, 0, 0};

    _odom_subscriber = _nh.subscribe("/multirotor/truth/NED", 1000,
        &Controller::odomCallback, this);
    _command_publisher = _nh.advertise<rosflight_msgs::Command>(
        "/command", 1000);

    ROS_INFO("Control1ler node init complete");
  }

  void Controller::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    // NOTE Given in NED by simulation
    _x.pn = msg->pose.pose.position.x;
    _x.pe = msg->pose.pose.position.y;
    _x.pd = msg->pose.pose.position.z;

    _x.vn = msg->twist.twist.linear.x;
    _x.ve = msg->twist.twist.linear.y;
    _x.vd = msg->twist.twist.linear.z;

    // TODO define more generally
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    // TODO move somewhere else
    // source: rosflight docs
    _x.phi = atan2(
        2 * (qw * qx + qy * qz),
        1 - 2 * (pow(qx, 2) + pow(qy, 2))
        ); 
    _x.theta = asin(2 * (qw * qy - qz * qx));
    _x.psi = atan2(
        2 * (qw * qz + qx * qy),
        1 - 2 * (pow(qy, 2) + pow(qz, 2))
        ); 
  

    //ROS_INFO("%f %f %f", _x.phi, _x.theta, _x.psi);
    
    computeControl();
    publishCommand();
  }

  void Controller::computeControl()
  {
    // Altitude PD control + eq feedforward
    double e_pd = _x.pd - _x_ref.pd;
    _u.d = - _k.pd * e_pd - _k.dd * _x.vd; // + _eq_thrust

    // Position PD control
    // TODO replace with error, so far only stabilizes the origin
    _u.n = -_k.pn * _x.pn - _k.dn * _x.vn;
    _u.e = -_k.pe * _x.pe - _k.de * _x.ve;

    // Control inputs
    double m = 2.0;
    double g = 9.8;
    
    _u.F = m * ((g - _u.d) / (cos(_x.phi) * cos(_x.theta))); // N
    
    // Attitude
    ROS_INFO("e_pd: %f, ud: %f, _x.phi: %f, x.theta: %f",
        e_pd, _u.d, _x.phi, _x.theta);
  }

  double Controller::saturate(double v)
  {
    v = v > 1.0 ? 1.0 : (
        v < 0.0 ? 0.0 : v
        );

    return v;
  }
  
  void Controller::publishCommand()
  {
    rosflight_msgs::Command _command;
    _command.header.stamp = ros::Time::now();
    _command.ignore = rosflight_msgs::Command::IGNORE_NONE;
    _command.mode = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
    // F from 0 to 1, scale by max thrust
    _command.F = saturate(_u.F / _max_thrust);
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
