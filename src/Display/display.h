#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <limits>
#include <memory>
#include <tuple>
#include <vector>

#include "../LightSources/LightSource/lightSource.h"
#include "../Objects/Object/object.h"
#include "../Utils/utils.h"
#include "displayStructs.h"

class Sphere;
namespace display {
std::vector<unsigned char> scene(
    displayStructs::Viewport, displayStructs::Camera,
    std::vector<std::shared_ptr<LightSource>>,
    std::vector<std::shared_ptr<Object>>);
}  // namespace display

#endif