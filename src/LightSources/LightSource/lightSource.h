#ifndef LIGHTSOURCE_H_
#define LIGHTSOURCE_H_

#include <eigen3/Eigen/Dense>
#include <tuple>

class LightSource {
   public:
    enum class lightTypes { POINT,
                            SPOTLIGHT,
                            DIRECTIONAL,
                            AMBIENT };

    LightSource(Eigen::Vector3d I_F, lightTypes ltype) : I_F(I_F), lightType(ltype) {}
    Eigen::Vector3d getIF();
    void setIF(double i1, double i2, double i3);
    virtual Eigen::Vector3d getPF() = 0;
    virtual std::tuple<Eigen::Vector3d, Eigen::Vector3d> calculateL(
        Eigen::Vector3d, Eigen::Vector3d) {
        return std::make_tuple(Eigen::Vector3d(0.0, 0.0, 0.0),
                               Eigen::Vector3d(0.0, 0.0, 0.0));
    }
    virtual double getDistance(Eigen::Vector3d) = 0;
    virtual void translate(double x, double y, double z, Eigen::Matrix4d wc) = 0;
    virtual void returnToWorld(Eigen::Matrix4d cw) = 0;
    virtual void setTheta(double new_theta) = 0;
    virtual void changeDirection(double x, double y, double z, Eigen::Matrix4d wc) = 0;
    lightTypes lightType;

   private:
    Eigen::Vector3d I_F;
};
#endif