#include "sphere.h"

#include <eigen3/Eigen/Dense>
#include <iostream>

std::tuple<double, double> Sphere::intersectRay(Eigen::Vector3d O,
                                                Eigen::Vector3d D) {
    double a, b, c;
    double delta;
    double r = this->radius;
    double inf = std::numeric_limits<double>::infinity();
    Eigen::Vector3d w = O - this->getCenter();

    a = D.dot(D);
    b = 2 * (w.dot(D));
    c = w.dot(w) - (r * r);

    delta = (b * b) - (4 * a * c);

    if (delta < 0) {
        return std::make_tuple(inf, inf);
    }

    double t1 = (-b + std::sqrt(delta)) / (2.0 * a);
    double t2 = (-b - std::sqrt(delta)) / (2.0 * a);

    return std::make_tuple(t1, t2);
}

double Sphere::getRadius() { return radius; }

Eigen::Vector3d Sphere::getCenter() { return center; }

Eigen::Vector3d Sphere::getNormal(Eigen::Vector3d P_I) {
    return ((P_I - this->center) / this->radius);
}

void Sphere::scale(double s, double opt, double opt2) {
    this->radius = this->radius * s;
    return;
}

void Sphere::translate(double x, double y, double z, Eigen::Matrix4d wc) {
    this->x = x;
    this->y = y;
    this->z = z;
    Eigen::Vector4d aux_center(x, y, z, 1);
    Eigen::Vector4d new_center = wc * aux_center;
    this->center(0) = new_center(0);
    this->center(1) = new_center(1);
    this->center(2) = new_center(2);
    this->wc = wc;
    return;
}

void Sphere::shear(double delta, matrix::SHEAR_AXIS axis) {
    //std::cout << "Eu sou inutil\n";
    return;
}

void Sphere::rotate(double theta, matrix::AXIS axis) {
    //std::cout << "Eu sou inutil\n";
    return;
}

void Sphere::returnToWorld(Eigen::Matrix4d cw, bool isReflection) {
    Eigen::Vector4d aux_center(this->center(0), this->center(1), this->center(2), 1);
    Eigen::Vector4d new_center = cw * aux_center;
    this->center(0) = new_center(0);
    this->center(1) = new_center(1);
    this->center(2) = new_center(2);
    // this->wc = wc;
}

void Sphere::backToCamera(Eigen::Matrix4d wc) {
    Eigen::Vector4d aux_center(this->center(0), this->center(1), this->center(2), 1);
    Eigen::Vector4d new_center = wc * aux_center;
    this->center(0) = new_center(0);
    this->center(1) = new_center(1);
    this->center(2) = new_center(2);
    return;
}

void Sphere::reflection(matrix::REFLECTION_AXIS axis, std::vector<std::shared_ptr<Object>> &objects, Eigen::Matrix4d wc) {
    Eigen::Matrix4d m = matrix::reflection(axis);
    Sphere reflectedSphere(this->K, this->m, this->radius, this->center);

    Eigen::Vector4d aux_center(this->center(0), this->center(1), this->center(2), 1);
    Eigen::Vector4d new_center = m * aux_center;
    reflectedSphere.center(0) = new_center(0);
    reflectedSphere.center(1) = new_center(1);
    reflectedSphere.center(2) = new_center(2);
    reflectedSphere.x = new_center(0);
    reflectedSphere.y = new_center(1);
    reflectedSphere.z = new_center(2);
    reflectedSphere.backToCamera(wc);
    objects.push_back(std::make_shared<Sphere>(reflectedSphere));

    return;
}