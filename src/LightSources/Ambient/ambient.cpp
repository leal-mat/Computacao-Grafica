#include "ambient.h"

// Eigen::Vector3d Ambient::getIA() { return this->I_A; }
Eigen::Vector3d Ambient::getPF() { return Eigen::Vector3d(0, 0, 0); }
double Ambient::getDistance(Eigen::Vector3d P_I) { return -1; }
void Ambient::translate(double x, double y, double z, Eigen::Matrix4d wc) {
    return;
}
void Ambient::returnToWorld(Eigen::Matrix4d cw) {
    return;
}

void Ambient::setTheta(double new_theta) {
    return;
}

void Ambient::changeDirection(double x, double y, double z, Eigen::Matrix4d wc) {
    return;
}