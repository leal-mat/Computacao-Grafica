#include "directional.h"

Eigen::Vector4d Directional::getDF() { return this->D_F; }
std::tuple<Eigen::Vector3d, Eigen::Vector3d> Directional::calculateL(
    Eigen::Vector3d P_I, Eigen::Vector3d n) {
    Eigen::Vector3d l(0, 0, 0);
    Eigen::Vector3d I_F = this->getIF();
    l = (-1 * this->getDF().head<3>()).normalized();
    if (this->getDF().head<3>().dot(n) >= 0.0001) {
        I_F = I_F * 0.0;
    }
    return std::make_tuple(l, I_F);
}
Eigen::Vector3d Directional::getPF() {
    return Eigen::Vector3d(INFINITY, INFINITY, INFINITY);
};

double Directional::getDistance(Eigen::Vector3d) { return INFINITY; }
void Directional::translate(double x, double y, double z, Eigen::Matrix4d wc) {
    if (x != INFINITY) {
        Eigen::Vector4d auxDF(x, y, z, 0);
        this->D_F = (wc * auxDF).normalized();
    }
    return;
}
void Directional::returnToWorld(Eigen::Matrix4d cw) {
    this->D_F = (cw * this->D_F).normalized();
    return;
}

void Directional::setTheta(double new_angle) {
    return;
}

void Directional::changeDirection(double x, double y, double z, Eigen::Matrix4d wc) {
    Eigen::Vector4d auxDF(x, y, z, 0);
    this->D_F = (wc * auxDF).normalized();
    return;
}