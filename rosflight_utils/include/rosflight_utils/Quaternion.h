//
// Created by lars on 3/27/20.
//

#ifndef SRC_QUATERNION_H
#define SRC_QUATERNION_H

class Quaternion {
  public:
  double x, y, z, w;
  Quaternion (double w, double x,double y,double z){
    this->w = w;
    this->x = x;
    this->y = y;
    this->z = z;
  }
};

#endif //SRC_QUATERNION_H
