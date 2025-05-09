#include "cone.h"

#include <eigen3/Eigen/Dense>
#include <iostream>

std::tuple<double, double> Cone::intersectRay(Eigen::Vector3d O,
                                              Eigen::Vector3d D) {
    double a, b, c;
    double delta;
    double r = this->radius;
    double inf = std::numeric_limits<double>::infinity();
    double validPoint = inf;

    Eigen::Vector3d w = vertex - O;

    double powHeight = std::pow(height, 2);
    double dConeDir = D.dot(coneDir);
    double cos2Theta = powHeight / (std::pow(radius, 2) + powHeight);

    a = std::pow(dConeDir, 2) - D.dot(D) * cos2Theta;
    b = 2 * ((w.dot(D) * cos2Theta) - (w.dot(coneDir) * dConeDir));
    c = std::pow(w.dot(coneDir), 2) - w.dot(w) * cos2Theta;

    delta = (b * b) - (4 * a * c);
    if (delta < 0) {
        return std::make_tuple(validPoint, validPoint);
    }

    Eigen::Vector3d P_I;

    std::tuple<double, double> tBottom = baseLid->intersectRay(O, D);
    double t1;
    double t2;
    if (a != 0) {
        t1 = (-b + std::sqrt(delta)) / (2 * a);
        t2 = (-b - std::sqrt(delta)) / (2 * a);
    } else {
        t1 = -c / b;
        t2 = t1;
    }

    P_I = O + std::get<0>(tBottom) * D;
    double test1 = (P_I - center).norm();
    if (test1 > 0 && test1 <= radius) {
        intersectionType = BASE;
        validPoint = std::get<0>(tBottom);
    }

    double tSurface = onSurface(O, D, t1, t2);

    if (tSurface < validPoint) {
        intersectionType = SURFACE;
        validPoint = tSurface;
    }

    return std::make_tuple(validPoint, validPoint);
}

double Cone::getRadius() { return radius; }

Eigen::Vector3d Cone::getCenter() { return center; }

Eigen::Vector3d Cone::getNormal(Eigen::Vector3d P_I) {
    if (intersectionType == BASE) {
        return -coneDir;
    }
    Eigen::Vector3d VPI = (vertex - P_I);
    Eigen::Vector3d nBar = VPI.cross(coneDir);
    Eigen::Vector3d normal = nBar.cross(VPI);
    return normal / normal.norm();
}

double Cone::onSurface(Eigen::Vector3d O, Eigen::Vector3d D, double t1,
                       double t2) {
    Eigen::Vector3d P_I_1;
    Eigen::Vector3d P_I_2;
    double validPoint = std::numeric_limits<double>::infinity();
    P_I_1 = O + t1 * D;
    P_I_2 = O + t2 * D;

    double test1 = (vertex - P_I_1).dot(coneDir);
    double test2 = (vertex - P_I_2).dot(coneDir);

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

void Cone::scale(double radiusScale, double heightScale, double opt) {
    this->height *= heightScale;
    this->radius *= radiusScale;
    return;
}

void Cone::shear(double delta, matrix::SHEAR_AXIS axis) {
    Eigen::Matrix4d m = matrix::shear(delta, axis);
    return;
}
void Cone::translate(double x, double y, double z, Eigen::Matrix4d wc) {
    this->x = x;
    this->y = y;
    this->z = z;
    Eigen::Vector4d auxCenter(x, y, z, 1);
    Eigen::Vector4d newCenter = wc * auxCenter;
    this->center = newCenter.head<3>();

    Eigen::Vector4d auxVertex4d(this->vertex(0), this->vertex(1), this->vertex(2),
                                1.0);
    // this->vertex = (wc * auxVertex4d).head<3>();

    Eigen::Vector4d auxConeDir4d(this->coneDir(0), this->coneDir(1),
                                 this->coneDir(2), 0.0);

    this->coneDir = (wc * auxConeDir4d).head<3>().normalized();

    this->vertex = this->center + this->coneDir * this->height;

    generateLids();
    return;
}
void Cone::rotate(double theta, matrix::AXIS axis) {
    Eigen::Matrix4d m = matrix::rotate(theta, axis);
    Eigen::Vector4d centerAux = Eigen::Vector4d(this->center(0), this->center(1), this->center(2), 1);
    Eigen::Vector4d coneDirAux = Eigen::Vector4d(this->coneDir(0), this->coneDir(1), this->coneDir(2), 0);
    Eigen::Vector4d vertexAux = Eigen::Vector4d(this->vertex(0), this->vertex(1), this->vertex(2), 1);

    centerAux = m * centerAux;
    coneDirAux = m * coneDirAux;
    vertexAux = m * vertexAux;

    this->center = centerAux.head<3>();
    this->coneDir = coneDirAux.head<3>();
    this->vertex = vertexAux.head<3>();

    // centerAux = wc * centerAux;
    // coneDirAux = wc * coneDirAux;
    // vertexAux = wc * vertexAux;
    return;
}
void Cone::reflection(matrix::REFLECTION_AXIS axis, std::vector<std::shared_ptr<Object>> &objects, Eigen::Matrix4d wc) {
    Eigen::Matrix4d m = matrix::reflection(axis);
    std::shared_ptr<Cone> reflectedCone = std::make_shared<Cone>(
        Cone(this->getK(), this->getM(), this->radius, this->center, this->height, this->coneDir));

    Eigen::Vector4d centerAux = Eigen::Vector4d(reflectedCone->center(0), reflectedCone->center(1), reflectedCone->center(2), 1);
    Eigen::Vector4d coneDirAux = Eigen::Vector4d(reflectedCone->coneDir(0), reflectedCone->coneDir(1), reflectedCone->coneDir(2), 0);
    Eigen::Vector4d vertexAux = Eigen::Vector4d(reflectedCone->vertex(0), reflectedCone->vertex(1), reflectedCone->vertex(2), 1);

    centerAux = m * centerAux;
    coneDirAux = m * coneDirAux;
    vertexAux = m * vertexAux;

    centerAux = wc * centerAux;
    coneDirAux = wc * coneDirAux;
    vertexAux = wc * vertexAux;

    reflectedCone->center = centerAux.head<3>();
    reflectedCone->coneDir = coneDirAux.head<3>();
    reflectedCone->vertex = vertexAux.head<3>();

    this->coordinatesAux(0) = this->x;
    this->coordinatesAux(1) = this->y;
    this->coordinatesAux(2) = this->z;
    this->coordinatesAux = m * this->coordinatesAux;
    std::cout << this->coordinatesAux(0) << " " << this->coordinatesAux(1) << " " << this->coordinatesAux(2) << "\n";

    reflectedCone->x = this->coordinatesAux(0);
    reflectedCone->y = this->coordinatesAux(1);
    reflectedCone->z = this->coordinatesAux(2);

    // reflectedCone->x = reflectedCone->center(0);
    // reflectedCone->y = reflectedCone->center(1);
    // reflectedCone->z = reflectedCone->center(2);

    reflectedCone->generateLids();

    objects.push_back(reflectedCone);
}

void Cone::generateLids() {
    this->baseLid =
        std::make_unique<Plane>(this->K, this->m, this->center, -this->coneDir);

    return;
}

void Cone::returnToWorld(Eigen::Matrix4d cw, bool isReflection) {
    Eigen::Vector4d centerAux = Eigen::Vector4d(this->center(0), this->center(1), this->center(2), 1);
    Eigen::Vector4d coneDirAux = Eigen::Vector4d(this->coneDir(0), this->coneDir(1), this->coneDir(2), 0);
    Eigen::Vector4d vertexAux = Eigen::Vector4d(this->vertex(0), this->vertex(1), this->vertex(2), 1);
    vertexAux = cw * vertexAux;
    centerAux = cw * centerAux;
    coneDirAux = (cw * coneDirAux).normalized();
    this->center = centerAux.head<3>();
    this->coneDir = coneDirAux.head<3>();
    this->vertex = vertexAux.head<3>();
    return;
}

void Cone::backToCamera(Eigen::Matrix4d wc) {
    Eigen::Vector4d centerAux = Eigen::Vector4d(this->center(0), this->center(1), this->center(2), 1);
    Eigen::Vector4d coneDirAux = Eigen::Vector4d(this->coneDir(0), this->coneDir(1), this->coneDir(2), 0);
    Eigen::Vector4d vertexAux = Eigen::Vector4d(this->vertex(0), this->vertex(1), this->vertex(2), 1);
    vertexAux = wc * vertexAux;
    centerAux = wc * centerAux;
    coneDirAux = wc * coneDirAux;
    this->center = centerAux.head<3>();
    this->coneDir = coneDirAux.head<3>();
    this->vertex = vertexAux.head<3>();
    this->x = this->center(0);
    this->y = this->center(1);
    this->z = this->center(2);
    return;
}