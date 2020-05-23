//
// Created by lars on 3/27/20.
//

#include "Quaternion.h"
#include <math.h>
#include "Eigen/Core"

#ifndef SRC_QUATERNIONMATH_H
#define SRC_QUATERNIONMATH_H


// using namespace Eigen;

class QuaternionMath{
  public:
  static void conjugate(Quaternion q1) {
    q1.x = -q1.x;
    q1.y = -q1.y;
    q1.z = -q1.z;
  }

  static void normalise(Quaternion q1) {
    double n = sqrt(q1.x*q1.x + q1.y*q1.y + q1.z*q1.z + q1.w*q1.w);
    q1.x /= n;
    q1.y /= n;
    q1.z /= n;
    q1.w /= n;
  }

   static void scale(Quaternion q1, double s){
    q1.x *= s;
    q1.y *= s;
    q1.z *= s;
    q1.w *= s;
  }

  static Quaternion multiply(Quaternion q1,Quaternion q2) {
    double x =  q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
    double y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
    double z =  q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
    double w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;
    return Quaternion{w,x,y,z};
  }

  static Eigen::Vector3d quaternion_to_euler(double x, double y, double z, double w){
    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + y * y);
    double roll = atan2(t0, t1);
    double t2 = +2.0 * (w * y - z * x);
    t2 = t2 > +1.0 ? +1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    double pitch = asin(t2);
    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (y * y + z * z);
    double yaw = atan2(t3, t4);
    return Eigen::Vector3d(roll, pitch, yaw);
  }

};

#endif //SRC_QUATERNIONMATH_H
