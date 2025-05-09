#include "utils.h"

#include <iostream>

namespace utils {
double inf = std::numeric_limits<double>::infinity();
int BACKGROUND_COLOR = 100;

std::tuple<double, std::shared_ptr<Object>> closestIntersection(
    Eigen::Vector3d O, Eigen::Vector3d D, double tMin, double tMax,
    std::vector<std::shared_ptr<Object>> objects) {
    double closestT = inf;
    std::shared_ptr<Object> closestObject;
    for (std::shared_ptr<Object> object : objects) {
        auto [t1, t2] = (object->intersectRay(O, D));

        if (t1 > tMin && t1 < tMax && t1 < closestT) {
            closestT = t1;
            closestObject = object;
        }
        if (t2 > tMin && t2 < tMax && t2 < closestT) {
            closestT = t2;
            closestObject = object;
        }
    }
    return std::make_tuple(closestT, closestObject);
}

bool isLightBlocked(std::vector<std::shared_ptr<Object>> objects,
                    Eigen::Vector3d P_I, std::shared_ptr<LightSource> lS,
                    Eigen::Vector3d l) {
    for (std::shared_ptr<Object> object : objects) {
        auto [t1, t2] = object->intersectRay(P_I, l);
        double t = -inf;
        double normalizedV = 0.0;
        t = std::min(t1, t2);

        if (t < 0.0) {
            t = (t == t2) ? t = t1 : t = t2;
            if (t < 0.0) {
                t = -inf;
            }
        }
        normalizedV = lS->getDistance(P_I);

        if (t > 0.0001 && t < normalizedV) {
            return true;
        }
    }
    return false;
}

std::tuple<double, double, double> calculateLighting(
    std::vector<std::shared_ptr<LightSource>> lightSources,
    displayStructs::Camera camera, double t,
    std::shared_ptr<Object> closestObject,
    std::vector<std::shared_ptr<Object>> objects) {
    utilsStructs::materialK K = closestObject->getK();

    Eigen::Vector3d I_A(0, 0, 0);
    Eigen::Vector3d I_D(0, 0, 0);
    Eigen::Vector3d I_E(0, 0, 0);

    for (std::shared_ptr<LightSource> lS : lightSources) {
        Eigen::Vector3d currentIA(0.0, 0.0, 0.0);
        Eigen::Vector3d I_F(0.0, 0.0, 0.0);
        if (lS->lightType == LightSource::lightTypes::AMBIENT) {
            currentIA = lS->getIF();
            I_A(0) += currentIA(0) * K.Ka(0);
            I_A(1) += currentIA(1) * K.Ka(1);
            I_A(2) += currentIA(2) * K.Ka(2);
        } else {
            I_F = lS->getIF();
            Eigen::Vector3d P_I(0, 0, 0);
            Eigen::Vector3d n(0, 0, 0);
            Eigen::Vector3d l(0, 0, 0);
            Eigen::Vector3d r(0, 0, 0);
            Eigen::Vector3d v(0, 0, 0);

            P_I = camera.O + (camera.D) * t;
            n = closestObject->getNormal(P_I);
            std::tuple<Eigen::Vector3d, Eigen::Vector3d> L_IF = lS->calculateL(P_I, n);

            l = std::get<0>(L_IF);
            I_F = std::get<1>(L_IF);

            r = 2 * ((l.dot(n)) * n) - l;
            v = (camera.D / camera.D.norm()) * (-1);

            double F_D = std::max(l.dot(n), 0.0);
            double F_E = std::pow(std::max(r.dot(v), 0.0), closestObject->getM());

            bool isBlocked =
                isLightBlocked(objects, P_I, lS, l);
            if (isBlocked) {
                I_F(0) = 0;
                I_F(1) = 0;
                I_F(2) = 0;
            }
            I_D(0) += I_F(0) * K.Kd(0) * F_D;
            I_D(1) += I_F(1) * K.Kd(1) * F_D;
            I_D(2) += I_F(2) * K.Kd(2) * F_D;

            I_E(0) += I_F(0) * K.Ke(0) * F_E;
            I_E(1) += I_F(1) * K.Ke(1) * F_E;
            I_E(2) += I_F(2) * K.Ke(2) * F_E;
        }
    }

    double I_1 = (I_A(0) + I_D(0) + I_E(0));
    double I_2 = (I_A(1) + I_D(1) + I_E(1));
    double I_3 = (I_A(2) + I_D(2) + I_E(2));

    double maxI = std::max(std::max(I_1, I_2), I_3);

    if (maxI > 1) {
        return std::make_tuple(I_1 / maxI, I_2 / maxI, I_3 / maxI);
    }
    return std::make_tuple(I_1, I_2, I_3);
}

utilsStructs::Color traceRay(
    displayStructs::Camera camera,
    std::vector<std::shared_ptr<LightSource>> lightSources,
    std::vector<std::shared_ptr<Object>> objects, int x, int y) {
    auto [closestT, closestObject] =
        closestIntersection(camera.O, camera.D, 0, inf, objects);

    if (closestObject == nullptr) {
        return utilsStructs::Color(BACKGROUND_COLOR);
    }
    if (closestT != inf) {
        auto [R, G, B] = calculateLighting(lightSources, camera, closestT,
                                           closestObject, objects);
        return utilsStructs::Color(int(R * 255), int(G * 255), int(B * 255));
    }
    return utilsStructs::Color(BACKGROUND_COLOR);
}
}  // namespace utils
