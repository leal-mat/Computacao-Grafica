#ifndef DIRECTIONAL_H_
#define DIRECTIONAL_H_

#include <iostream>

#include "../LightSource/lightSource.h"
class Directional : public LightSource {
   public:
    Directional(Eigen::Vector3d I_F)
        : LightSource(I_F, lightTypes::DIRECTIONAL) {
        this->D_F = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);
    }
    Eigen::Vector4d getDF();
    Eigen::Vector3d getPF();
    std::tuple<Eigen::Vector3d, Eigen::Vector3d> calculateL(Eigen::Vector3d,
                                                            Eigen::Vector3d);
    double getDistance(Eigen::Vector3d);
    void translate(double x, double y, double z, Eigen::Matrix4d wc);
    void returnToWorld(Eigen::Matrix4d cw);
    void setTheta(double new_theta);
    void changeDirection(double x, double y, double z, Eigen::Matrix4d wc);

   private:
    Eigen::Vector4d D_F;
};

#endif
