#include "plane.h"

Eigen::Vector3d Plane::getNormal(Eigen::Vector3d P_I) { return this->nBar; }

std::tuple<double, double> Plane::intersectRay(Eigen::Vector3d O,
                                               Eigen::Vector3d D) {
    Eigen::Vector3d w = O - this->pPI;
    double t = -(w.dot(this->nBar) / D.dot(this->nBar));
    t = t - 0.0001;
    return std::make_tuple(t, t);
}

void Plane::scale(double x, double y, double z) {
    // std::cout << "Eu sou inutil :P\n";
    return;
}

void Plane::translate(double x, double y, double z, Eigen::Matrix4d wc) {
    Eigen::Vector4d auxPPI(this->pPI(0), this->pPI(1), this->pPI(2), 1.0);
    Eigen::Vector4d auxNBAR(this->nBar(0), this->nBar(1), this->nBar(2), 0.0);
    this->pPI = (wc * auxPPI).head<3>();
    this->nBar = ((wc * auxNBAR).normalized()).head<3>();
}

void Plane::returnToWorld(Eigen::Matrix4d cw, bool isReflection) {
    Eigen::Vector4d auxPPI(this->pPI(0), this->pPI(1), this->pPI(2), 1.0);
    Eigen::Vector4d auxNBAR(this->nBar(0), this->nBar(1), this->nBar(2), 0.0);
    this->pPI = (cw * auxPPI).head<3>();
    this->nBar = ((cw * auxNBAR).normalized()).head<3>();
}

void Plane::shear(double delta, matrix::SHEAR_AXIS axis) {
    return;
}

void Plane::rotate(double theta, matrix::AXIS axis) {
    // std::cout << "Eu sou inutil\n";
    return;
}

void Plane::reflection(matrix::REFLECTION_AXIS axis, std::vector<std::shared_ptr<Object>> &objects, Eigen::Matrix4d wc) {
    return;
}

void Plane::backToCamera(Eigen::Matrix4d wc) {
    Eigen::Vector4d auxPPI(this->pPI(0), this->pPI(1), this->pPI(2), 1.0);
    Eigen::Vector4d auxNBAR(this->nBar(0), this->nBar(1), this->nBar(2), 0.0);
    this->pPI = (wc * auxPPI).head<3>();
    this->nBar = ((wc * auxNBAR).normalized()).head<3>();
    return;
}