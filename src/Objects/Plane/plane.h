#ifndef PLANE_H_
#define PLANE_H_
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <tuple>

#include "../Object/object.h"

class Plane : public Object {
   public:
    Plane(utilsStructs::materialK k, double shininess, Eigen::Vector3d pPI,
          Eigen::Vector3d nBar)
        : Object(k, shininess, utilsStructs::OBJ_TYPE::PLANE), pPI(pPI), nBar(nBar){};

    std::tuple<double, double> intersectRay(Eigen::Vector3d, Eigen::Vector3d);
    Eigen::Vector3d getNormal(Eigen::Vector3d);
    void returnToWorld(Eigen::Matrix4d cw, bool isReflection);
    void backToCamera(Eigen::Matrix4d wc);
    void scale(double x, double y, double z);
    void translate(double x, double y, double z, Eigen::Matrix4d wc);
    void shear(double delta, matrix::SHEAR_AXIS axis);
    void rotate(double theta, matrix::AXIS axis);
    void reflection(matrix::REFLECTION_AXIS axis, std::vector<std::shared_ptr<Object>> &objects, Eigen::Matrix4d wc);

   private:
    Eigen::Vector3d pPI;
    Eigen::Vector3d nBar;
};

#endif