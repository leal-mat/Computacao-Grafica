#ifndef SPHERE_H_
#define SPHERE_H_

#include <eigen3/Eigen/Dense>
#include <memory>
#include <tuple>

#include "../../Display/displayStructs.h"
#include "../../Utils/utilsStructs.h"
#include "../Object/object.h"

class Sphere : public Object {
   public:
    Sphere(utilsStructs::materialK k, double shininess, double radius,
           Eigen::Vector3d center)
        : Object(k, shininess, utilsStructs::OBJ_TYPE::SPHERE), radius(radius), center(center){};
    double getRadius();
    Eigen::Vector3d getCenter();
    Eigen::Vector3d getNormal(Eigen::Vector3d P_I);
    void returnToWorld(Eigen::Matrix4d cw, bool isReflection);
    void backToCamera(Eigen::Matrix4d wc);
    std::tuple<double, double> intersectRay(Eigen::Vector3d, Eigen::Vector3d);

    /*void applyMatrixVertices(Eigen::Matrix4d);
    void applyMatrixNormals(Eigen::Matrix4d);*/

    void scale(double s, double opt = 0.0, double opt2 = 0.0);

    void shear(double delta, matrix::SHEAR_AXIS axis);
    void translate(double x, double y, double z, Eigen::Matrix4d wc);
    void rotate(double theta, matrix::AXIS axis);
    void reflection(matrix::REFLECTION_AXIS axis, std::vector<std::shared_ptr<Object>> &objects, Eigen::Matrix4d wc);

   private:
    Eigen::Vector3d center;
    double radius;
    Eigen::Matrix4d wc;
};
#endif