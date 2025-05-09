#include "lightSource.h"

Eigen::Vector3d LightSource::getIF() { return this->I_F; }

void LightSource::setIF(double i1, double i2, double i3) {
    this->I_F(0) = i1;
    this->I_F(1) = i2;
    this->I_F(2) = i3;
}