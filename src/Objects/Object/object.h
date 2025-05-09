#ifndef OBJECT_H_
#define OBJECT_H_

#include <eigen3/Eigen/Dense>
#include <memory>
#include <tuple>

#include "../../Display/displayStructs.h"
#include "../../Matrix/matrix.h"
#include "../../Utils/utilsStructs.h"

class Object {
   public:
    virtual std::tuple<double, double> intersectRay(Eigen::Vector3d,
                                                    Eigen::Vector3d) = 0;
    virtual Eigen::Vector3d getNormal(Eigen::Vector3d) = 0;
    virtual void returnToWorld(Eigen::Matrix4d cw, bool isReflection) = 0;
    virtual void backToCamera(Eigen::Matrix4d wc) = 0;
    utilsStructs::materialK getK();
    void modifyK(Eigen::Vector3d K, int option);
    double getM();
    utilsStructs::Texel getPixel(double x, double z);
    virtual void translate(double x, double y, double z, Eigen::Matrix4d wc) = 0;
    virtual void scale(double x, double y, double z) = 0;
    virtual void shear(double delta, matrix::SHEAR_AXIS axis) = 0;
    virtual void rotate(double theta, matrix::AXIS axis) = 0;
    virtual void reflection(matrix::REFLECTION_AXIS axis, std::vector<std::shared_ptr<Object>> &objects, Eigen::Matrix4d wc) = 0;
    std::tuple<double, double, double> getCoordinates();
    void setCoordinates(double newX, double newY, double newZ);
    utilsStructs::OBJ_TYPE getType();

   protected:
    Object(utilsStructs::materialK k, double m, utilsStructs::OBJ_TYPE type)
        : K(k), m(m), type(type){};

    utilsStructs::materialK K;
    double m;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    utilsStructs::OBJ_TYPE type;
    std::vector<std::vector<utilsStructs::Texel>> imageRGB;
};

#endif