#include "rosflight/controller/controller.h"

namespace controller {
  Controller::Controller(ros::NodeHandle *nh) : _nh(*nh) {
    // References
    _x_ref.pd = -1.0; // TODO ref height
    _max_tilt = 0.35; // TODO just picked
    
    // Model parameters
    _m = 2.0; // kg
    _g = 9.8; // m/s**2
    _max_thrust = 14.961 * 4; // From gazebo sim, 4 rotors

    // Controller gains
    _k.pn = 2.0;
    _k.pe = 2.0;
    _k.pd = 14;
    _k.dn = 4.0;
    _k.de = 4.0;
    _k.dd = 7.0;
    _k.ppsi = 1.0;

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
  

    ROS_INFO("%f %f %f", _x.phi, _x.theta, _x.psi);
    
    computeControl();
    publishCommand();
  }

  void Controller::computeControl()
  {
    // OUTER CONTROL LOOP
    // Position PD control
    // TODO replace with error, so far only stabilizes the origin
    _u.n = -_k.pn * _x.pn - _k.dn * _x.vn;
    _u.e = -_k.pe * _x.pe - _k.de * _x.ve;

    // Altitude PD control
    double e_pd = _x.pd - _x_ref.pd;
    _u.d = - _k.pd * e_pd - _k.dd * _x.vd; // + _eq_thrust

    // CONTROL INPUTS
    _u.F = _m * ((_g - _u.d) / (cos(_x.phi) * cos(_x.theta))); // N
    _u.phi = atan2(
        ((cos(_x.theta) * _u.e )
        + (sin(_x.theta) * sin(_x.psi) * (_g - _u.d))),
        (_g - _u.d) * cos(_x.psi)
        ); // TODO this gets singular
    _u.theta = atan2( 
        -(cos(_x.psi) * _u.n + sin(_x.psi) * _u.e),
        (_g - _u.d)
        );
    /*
    _u.phi = atan(
        (cos(_x.theta) * _u.e ) / ((_g - _u.d) * cos(_x.psi))
        + sin(_x.theta) * tan(_x.psi)
        );
    _u.theta = atan(
        -(_u.n + tan(_x.psi) * _u.e)
        / ((_g - _u.d) * (cos(_x.psi) + tan(_x.psi) * sin(_x.psi)))
        );
        */

    // TODO yaw problem: oscillates like crazy for psi = pi/2
    // singularity maybe?
    
    _u.psi = -_k.ppsi * _x.psi; // TODO only P controller so far

    ROS_INFO("un: %f, ue: %f, u_phi: %f, u_theta: %f",
        _u.n, _u.e, _u.phi, _u.theta);

  }

  double Controller::saturate(double v, double min, double max)
  {
    v = v > max ? max : (
        v < min ? min : v
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
    _command.F = saturate(_u.F / _max_thrust, 0.0, 1.0);
    _command.x = saturate(_u.phi, -_max_tilt, _max_tilt);
    _command.y = saturate(_u.theta, -_max_tilt, _max_tilt);
    _command.z = _u.psi;

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
