#ifndef AMBIENT_H_
#define AMBIENT_H_

#include "../LightSource/lightSource.h"
class Ambient : public LightSource {
   public:
    Ambient(Eigen::Vector3d I_F)
        : LightSource(I_F, lightTypes::AMBIENT) {}
    double getDistance(Eigen::Vector3d P_I);
    Eigen::Vector3d getPF();
    void translate(double x, double y, double z, Eigen::Matrix4d wc);
    void returnToWorld(Eigen::Matrix4d cw);
    void setTheta(double new_theta);
    void changeDirection(double x, double y, double z, Eigen::Matrix4d wc);
};
#endif

//  Point(Eigen::Vector3d I_F, Eigen::Vector3d P_F)
//         : LightSource(I_F, lightTypes::POINT), P_F(P_F) {}