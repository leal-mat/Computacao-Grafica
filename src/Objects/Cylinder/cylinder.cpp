#include "cylinder.h"

#include <eigen3/Eigen/Dense>
#include <iostream>

std::tuple<double, double> Cylinder::intersectRay(Eigen::Vector3d O,
                                                  Eigen::Vector3d D) {
    double a, b, c;
    double delta;
    double r = this->radius;
    double inf = std::numeric_limits<double>::infinity();
    double validPoint = inf;

    Eigen::Vector3d w = O - this->getCenter();

    a = D.transpose() * M * D;
    b = 2 * w.transpose() * M * D;
    c = (w.transpose() * M * w) - std::pow(radius, 2);

    delta = (b * b) - (4 * a * c);

    if (delta < 0) {
        return std::make_tuple(validPoint, validPoint);
    }

    double t1 = (-b + std::sqrt(delta)) / (2.0 * a);
    double t2 = (-b - std::sqrt(delta)) / (2.0 * a);

    Eigen::Vector3d P_I;

    std::tuple<double, double> tBottom = baseLid->intersectRay(O, D);
    std::tuple<double, double> tUpper = topLid->intersectRay(O, D);

    // Segunda verifica��o - Raio/Plano da base

    P_I = O + std::get<0>(tBottom) * D;
    double test1 = (P_I - center).norm();
    if (test1 > 0 && test1 <= radius) {
        intersectionType = BASE;
        validPoint = std::get<0>(tBottom);
    }

    // Terceira verifica��o - Raio/Plano do topo

    P_I = O + std::get<0>(tUpper) * D;
    double test2 = (P_I - topCenter).norm();
    if (test2 > 0 && test2 <= radius) {
        if (validPoint > std::get<0>(tUpper)) {
            intersectionType = TOP;
            validPoint = std::get<0>(tUpper);
        }
    }

    // Primeira verifica��o - Raio/Superf�cie
    double t = onSurface(O, D, t1, t2);

    if (validPoint > t) {
        intersectionType = SURFACE;
        validPoint = t;
    }

    return std::make_tuple(validPoint, validPoint);
}

double Cylinder::getRadius() { return radius; }

Eigen::Vector3d Cylinder::getCenter() { return center; }

Eigen::Vector3d Cylinder::getNormal(Eigen::Vector3d P_I) {
    if (intersectionType == TOP) {
        return cylinderDir;
    }

    if (intersectionType == BASE) {
        return -cylinderDir;
    }

    Eigen::Vector3d n = M * (P_I - center);
    return n / n.norm();
}

double Cylinder::onSurface(Eigen::Vector3d O, Eigen::Vector3d D, double t1,
                           double t2) {
    Eigen::Vector3d P_I_1;
    Eigen::Vector3d P_I_2;
    double validPoint = std::numeric_limits<double>::infinity();
    P_I_1 = O + t1 * D;
    P_I_2 = O + t2 * D;

    double test1 = (P_I_1 - center).dot(cylinderDir);
    double test2 = (P_I_2 - center).dot(cylinderDir);

    if (test1 > 0 && test1 <= height) {
        if (t1 > 0) {
            validPoint = t1;
        }
    }
    if (test2 > 0 && test2 <= height) {
        if (t2 > 0 && validPoint > t2) {
            validPoint = t2;
        }
    }
    return validPoint;
}

void Cylinder::scale(double radiusScale, double heightScale, double opt) {
    this->height *= heightScale;
    this->radius *= radiusScale;
    return;
}

void Cylinder::shear(double delta, matrix::SHEAR_AXIS axis) {
    //std::cout << "Eu sou inutil\n";
    return;
}
void Cylinder::translate(double x, double y, double z, Eigen::Matrix4d wc) {
    this->x = x;
    this->y = y;
    this->z = z;
    Eigen::Vector4d auxCenter(x, y, z, 1);
    Eigen::Vector4d newCenter = wc * auxCenter;
    this->center = newCenter.head<3>();

    Eigen::Vector4d auxDir(cylinderDir(0), cylinderDir(1), cylinderDir(2), 0);
    Eigen::Vector4d newDir = wc * auxDir;
    this->cylinderDir = (newDir.head<3>()).normalized();
    generateLids();
    return;
}
void Cylinder::rotate(double theta, matrix::AXIS axis) {
    Eigen::Matrix4d m = matrix::rotate(theta, axis);
    Eigen::Vector4d centerAux = Eigen::Vector4d(this->center(0), this->center(1), this->center(2), 1);
    Eigen::Vector4d cylDirAux = Eigen::Vector4d(this->cylinderDir(0), this->cylinderDir(1), this->cylinderDir(2), 0);
    Eigen::Vector4d topCenterAux = Eigen::Vector4d(this->topCenter(0), this->topCenter(1), this->topCenter(2), 1);

    centerAux = m * centerAux;
    cylDirAux = m * cylDirAux;
    topCenterAux = m * topCenterAux;

    this->center = centerAux.head<3>();
    this->cylinderDir = cylDirAux.head<3>();
    this->topCenter = topCenterAux.head<3>();

    return;
}

void Cylinder::reflection(matrix::REFLECTION_AXIS axis, std::vector<std::shared_ptr<Object>> &objects, Eigen::Matrix4d wc) {
    Eigen::Matrix4d m = matrix::reflection(axis);
    // Cylinder reflectedCylinder(this->getK(), this->getM(), this->radius, this->center, this->height, this->cylinderDir);
    std::shared_ptr<Cylinder> reflectedCylinder = std::make_shared<Cylinder>(
        Cylinder(this->getK(), this->getM(), this->radius, this->center, this->height, this->cylinderDir));

    Eigen::Vector4d centerAux = Eigen::Vector4d(reflectedCylinder->center(0), reflectedCylinder->center(1), reflectedCylinder->center(2), 1);
    Eigen::Vector4d cylDirAux = Eigen::Vector4d(reflectedCylinder->cylinderDir(0), reflectedCylinder->cylinderDir(1), reflectedCylinder->cylinderDir(2), 0);
    Eigen::Vector4d topCenterAux = Eigen::Vector4d(reflectedCylinder->topCenter(0), reflectedCylinder->topCenter(1), reflectedCylinder->topCenter(2), 1);

    centerAux = m * centerAux;
    cylDirAux = m * cylDirAux;
    topCenterAux = m * topCenterAux;

    centerAux = wc * centerAux;
    cylDirAux = wc * cylDirAux;
    topCenterAux = wc * topCenterAux;

    reflectedCylinder->center = centerAux.head<3>();
    reflectedCylinder->cylinderDir = cylDirAux.head<3>();
    reflectedCylinder->topCenter = topCenterAux.head<3>();

    this->coordinatesAux(0) = this->x;
    this->coordinatesAux(1) = this->y;
    this->coordinatesAux(2) = this->z;
    this->coordinatesAux = m * this->coordinatesAux;
    //std::cout << this->coordinatesAux(0) << " " << this->coordinatesAux(1) << " " << this->coordinatesAux(2) << "\n";

    reflectedCylinder->x = this->coordinatesAux(0);
    reflectedCylinder->y = this->coordinatesAux(1);
    reflectedCylinder->z = this->coordinatesAux(2);

    // reflectedCylinder->x = reflectedCylinder->center(0);
    // reflectedCylinder->y = reflectedCylinder->center(1);
    // reflectedCylinder->z = reflectedCylinder->center(2);

    reflectedCylinder->generateLids();

    objects.push_back(reflectedCylinder);
}

void Cylinder::generateLids() {
    // std::cout << this->cylinderDir << "\n";
    this->M = Eigen::Matrix<double, 3, 3>::Identity() -
              this->cylinderDir * this->cylinderDir.transpose();

    this->topCenter = this->center + this->height * this->cylinderDir;

    this->baseLid =
        std::make_unique<Plane>(this->K, this->m, center, -cylinderDir);

    this->topLid =
        std::make_unique<Plane>(this->K, this->m, topCenter, cylinderDir);
    return;
}

void Cylinder::returnToWorld(Eigen::Matrix4d cw, bool isReflection) {
    Eigen::Vector4d centerAux = Eigen::Vector4d(this->center(0), this->center(1), this->center(2), 1);
    Eigen::Vector4d cylDirAux = Eigen::Vector4d(this->cylinderDir(0), this->cylinderDir(1), this->cylinderDir(2), 0);
    Eigen::Vector4d topCenterAux = Eigen::Vector4d(this->topCenter(0), this->topCenter(1), this->topCenter(2), 1);
    topCenterAux = cw * topCenterAux;
    centerAux = cw * centerAux;
    cylDirAux = (cw * cylDirAux).normalized();
    this->center = centerAux.head<3>();
    this->cylinderDir = cylDirAux.head<3>();
    this->topCenter = topCenterAux.head<3>();
    return;
}

void Cylinder::backToCamera(Eigen::Matrix4d wc) {
    Eigen::Vector4d centerAux = Eigen::Vector4d(this->center(0), this->center(1), this->center(2), 1);
    Eigen::Vector4d cylDirAux = Eigen::Vector4d(this->cylinderDir(0), this->cylinderDir(1), this->cylinderDir(2), 0);
    Eigen::Vector4d topCenterAux = Eigen::Vector4d(this->topCenter(0), this->topCenter(1), this->topCenter(2), 1);
    topCenterAux = wc * topCenterAux;
    centerAux = wc * centerAux;
    cylDirAux = wc * cylDirAux;
    this->center = centerAux.head<3>();
    this->cylinderDir = cylDirAux.head<3>();
    this->topCenter = topCenterAux.head<3>();
    this->x = this->center(0);
    this->y = this->center(1);
    this->z = this->center(2);
    return;
}