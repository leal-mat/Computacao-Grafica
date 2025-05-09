#include "object.h"

#include <iostream>

utilsStructs::materialK Object::getK() { return K; }

double Object::getM() { return m; }

utilsStructs::Texel Object::getPixel(double x, double z) {
    // utilsStructs::Texel tex(0,0,0);

    int imgW = imageRGB.size();
    int imgH = imageRGB[0].size();

    double fx = (x - (-200.0)) / 400.0;
    double fz = (z - (-400.0)) / 400.0;

    int indexX = int(0 + fx * (imgW - 1));
    int indexZ = int(0 + fz * (imgH - 1));

    return imageRGB[indexX][indexZ];
}

std::tuple<double, double, double> Object::getCoordinates() {
    return std::make_tuple(this->x, this->y, this->z);
}

utilsStructs::OBJ_TYPE Object::getType() {
    return this->type;
}

void Object::modifyK(Eigen::Vector3d K, int option) {
    switch (option) {
        case 1: {
            this->K.Ka = K;
            break;
        }
        case 2: {
            this->K.Kd = K;
            break;
        }
        case 3: {
            this->K.Ke = K;
            break;
        }
        case 4: {
            this->K.Ka = K;
            this->K.Kd = K;
            this->K.Ke = K;
            break;
        }

        default:
            break;
    }
}

void Object::setCoordinates(double newX, double newY, double newZ) {
    this->x = newX;
    this->y = newY;
    this->z = newZ;
    return;
}