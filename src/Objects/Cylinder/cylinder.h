#ifndef CYLINDER_H_
#define CYLINDER_H_

#include <eigen3/Eigen/Dense>
#include <memory>
#include <tuple>

#include "../../Display/displayStructs.h"
#include "../../Utils/utilsStructs.h"
#include "../Object/object.h"
#include "../Plane/plane.h"

class Cylinder : public Object {
    enum INTERSECTION_TYPE { BASE,
                             TOP,
                             SURFACE,
                             NONE };

   public:
    Cylinder(utilsStructs::materialK k, double shininess, double radius,
             Eigen::Vector3d center, double height, Eigen::Vector3d dCil)
        : Object(k, shininess, utilsStructs::OBJ_TYPE::CYLINDER),
          radius(radius),
          center(center),
          height(height),
          cylinderDir(dCil),
          intersectionType(INTERSECTION_TYPE::NONE){};

    double getRadius();
    Eigen::Vector3d getCenter();
    Eigen::Vector3d getNormal(Eigen::Vector3d P_I);
    void returnToWorld(Eigen::Matrix4d cw, bool isReflection);
    void backToCamera(Eigen::Matrix4d wc);
    std::tuple<double, double> intersectRay(Eigen::Vector3d, Eigen::Vector3d);

    void scale(double radiusScale, double heightScale, double opt = 0.0);
    void shear(double delta, matrix::SHEAR_AXIS axis);
    void translate(double x, double y, double z, Eigen::Matrix4d wc);
    void rotate(double theta, matrix::AXIS axis);
    void reflection(matrix::REFLECTION_AXIS axis, std::vector<std::shared_ptr<Object>> &objects, Eigen::Matrix4d wc);
    void generateLids();

   private:
    double radius;
    double height;
    INTERSECTION_TYPE intersectionType;
    Eigen::Vector3d center;
    Eigen::Vector3d cylinderDir;
    Eigen::Vector3d topCenter;
    Eigen::Matrix<double, 3, 3> M;
    std::unique_ptr<Plane> topLid;
    std::unique_ptr<Plane> baseLid;
    Eigen::Vector4d coordinatesAux;

    double onSurface(Eigen::Vector3d, Eigen::Vector3d, double, double);
};
#endif