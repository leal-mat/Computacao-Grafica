#include "point.h"

#include <iostream>

Eigen::Vector3d Point::getPF() { return this->P_F; }

std::tuple<Eigen::Vector3d, Eigen::Vector3d> Point::calculateL(
    Eigen::Vector3d P_I, Eigen::Vector3d n) {
    Eigen::Vector3d l(0, 0, 0);
    Eigen::Vector3d P_F = this->getPF();
    l = (P_F - P_I).normalized();

    return std::make_tuple(l, this->getIF());
}

double Point::getDistance(Eigen::Vector3d P_I) {
    return (this->P_F - P_I).norm();
}

void Point::translate(double x, double y, double z, Eigen::Matrix4d wc) {
    Eigen::Vector4d auxPF(x, y, z, 1.0);
    auxPF = wc * auxPF;
    this->P_F = auxPF.head<3>();
}

void Point::returnToWorld(Eigen::Matrix4d cw) {
    Eigen::Vector4d auxPF(this->P_F(0), this->P_F(1), this->P_F(2), 1.0);
    auxPF = cw * auxPF;
    this->P_F = auxPF.head<3>();
}

void Point::setTheta(double new_angle) {
    return;
}

void Point::changeDirection(double x, double y, double z, Eigen::Matrix4d wc) {
    return;
}