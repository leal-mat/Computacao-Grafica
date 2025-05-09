#include "spot.h"

Eigen::Vector4d Spot::getDS() { return this->D_S; }

Eigen::Vector3d Spot::getPF() { return this->P_F.head<3>(); }

double Spot::getTheta() { return this->theta; }

std::tuple<Eigen::Vector3d, Eigen::Vector3d> Spot::calculateL(
    Eigen::Vector3d P_I, Eigen::Vector3d n) {
    Eigen::Vector3d I_F = this->getIF();
    Eigen::Vector3d l(0, 0, 0);
    l = (this->getPF().head<3>() - P_I).normalized();
    double clds = l.dot(-(this->getDS().head<3>()).normalized());
    if (clds >= std::cos(this->theta)) {
        I_F = I_F * clds;
    } else {
        I_F = I_F * 0.0;
    }
    return std::make_tuple(l, I_F);
}

double Spot::getDistance(Eigen::Vector3d P_I) {
    return ((this->getPF() - P_I)).norm();
}

void Spot::translate(double x, double y, double z, Eigen::Matrix4d wc) {
    Eigen::Vector4d auxPF(x, y, z, 1.0);
    this->P_F = wc * auxPF;

    Eigen::Vector4d auxPI(this->P_I(0), this->P_I(1), this->P_I(2), 1.0);
    this->P_I = wc * auxPI;

    this->D_S = (this->P_I - this->P_F).normalized();
    return;
}

void Spot::changeDirection(double x, double y, double z, Eigen::Matrix4d wc) {
    Eigen::Vector4d auxPI(x, y, z, 1.0);
    auxPI = wc * auxPI;
    this->P_I = auxPI;
    this->D_S = (auxPI - this->P_F).normalized();
    return;
}

void Spot::returnToWorld(Eigen::Matrix4d cw) {
    Eigen::Vector4d auxPF(this->P_F(0), this->P_F(1), this->P_F(2), 1.0);
    auxPF = cw * auxPF;
    this->P_F = auxPF;

    Eigen::Vector4d auxDS(this->D_S(0), this->D_S(1), this->D_S(2), 1.0);
    auxDS = cw * auxDS;
    this->D_S = auxDS.normalized();

    Eigen::Vector4d auxPointTo(this->P_I(0), this->P_I(1), this->P_I(2), 1.0);
    auxPointTo = cw * auxPointTo;
    this->P_I = auxPointTo;
    return;
}

void Spot::setTheta(double new_theta) {
    this->theta = new_theta * M_PI / 180;
    return;
}