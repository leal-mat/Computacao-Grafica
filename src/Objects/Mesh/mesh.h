#ifndef MESH_H_
#define MESH_H_

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>
#include <tuple>
#include <vector>

#include "../Object/object.h"
#include "../Sphere/sphere.h"

class Mesh : public Object {
   public:
    Mesh(utilsStructs::materialK k, double shininess, std::string path, std::shared_ptr<Sphere> cluster = nullptr);
    Mesh(utilsStructs::materialK k, double shininess,
         std::vector<Eigen::Vector4d> vertices,
         std::vector<Eigen::Vector4d> normals, std::vector<Eigen::Vector3d> edges,
         std::vector<Eigen::Vector4d> faces, std::shared_ptr<Sphere> cluster = nullptr);
    std::tuple<double, double> intersectRay(Eigen::Vector3d, Eigen::Vector3d);
    Eigen::Vector3d getNormal(Eigen::Vector3d P_I);
    void returnToWorld(Eigen::Matrix4d cw, bool isReflection);
    void backToCamera(Eigen::Matrix4d wc);
    void applyMatrixVertices(Eigen::Matrix4d);
    void applyMatrixNormals(Eigen::Matrix4d);
    void scale(double x, double y, double z);
    void shear(double delta, matrix::SHEAR_AXIS axis);
    void translate(double x, double y, double z, Eigen::Matrix4d wc);
    void rotate(double theta, matrix::AXIS axis);
    void reflection(matrix::REFLECTION_AXIS axis, std::vector<std::shared_ptr<Object>> &objects, Eigen::Matrix4d wc);
    /*std::vector<Eigen::Vector4d> getNormals();
    std::vector<Eigen::Vector4d> getVertices();*/

   private:
    std::vector<Eigen::Vector4d> vertices;
    std::vector<Eigen::Vector4d> normals;
    std::vector<Eigen::Vector3d> edges;
    std::vector<Eigen::Vector4d> faces;
    Eigen::Vector3d normal;
    Eigen::Vector4d coordinatesAux;
    std::string objName;
    std::shared_ptr<Sphere> cluster;
};

#endif