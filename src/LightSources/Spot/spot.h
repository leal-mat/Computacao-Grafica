#ifndef SPOT_H_
#define SPOT_H_
#define M_PI 3.14159265358979323846

#include "../LightSource/lightSource.h"
class Spot : public LightSource {
   public:
    Spot(Eigen::Vector3d I_F, Eigen::Vector4d P_I, double angle)
        : LightSource(I_F, lightTypes::SPOTLIGHT), P_F(P_F), theta(angle) {
        this->P_I = P_I;
        this->theta = angle * M_PI / 180;
        this->D_S = Eigen::Vector4d(0.0, 0.0, 0.0, 0);
    };
    Eigen::Vector4d getDS();
    Eigen::Vector3d getPF();
    double getTheta();
    void setTheta(double new_theta);
    std::tuple<Eigen::Vector3d, Eigen::Vector3d> calculateL(Eigen::Vector3d,
                                                            Eigen::Vector3d);
    double getDistance(Eigen::Vector3d P_I);
    void translate(double x, double y, double z, Eigen::Matrix4d wc);
    void changeDirection(double x, double y, double z, Eigen::Matrix4d wc);
    void returnToWorld(Eigen::Matrix4d cw);

   private:
    Eigen::Vector4d D_S;  // direcao da luz
    Eigen::Vector4d P_F;  // posicao da luz
    Eigen::Vector4d P_I;  // posicao que a luz aponta
    double theta;
};
#endif