#ifndef UTILS_H_
#define UTILS_H_

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <limits>
#include <memory>
#include <tuple>
#include <vector>

#include "../Display/displayStructs.h"
#include "../LightSources/LightSource/lightSource.h"
#include "../Objects/Object/object.h"
#include "utilsStructs.h"

namespace utils {

utilsStructs::Color traceRay(
    displayStructs::Camera camera,
    std::vector<std::shared_ptr<LightSource>>,
    std::vector<std::shared_ptr<Object>>, int x, int y);

std::tuple<double, std::shared_ptr<Object>> closestIntersection(
    Eigen::Vector3d, Eigen::Vector3d, double, double,
    std::vector<std::shared_ptr<Object>>);

std::tuple<double, double, double> calculateLighting(
    std::vector<std::shared_ptr<LightSource>> lightSources,
    displayStructs::Camera camera, double t,
    std::shared_ptr<Object> closestObject,
    std::vector<std::shared_ptr<Object>> objects);

bool isLightBlocked(std::vector<std::shared_ptr<Object>> objects,
                    Eigen::Vector3d P_I,
                    std::shared_ptr<LightSource> lS,
                    Eigen::Vector3d l);
}  // namespace utils

#endif