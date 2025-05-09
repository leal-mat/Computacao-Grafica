#ifndef MATRIX_H_
#define MATRIX_H_

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <memory>
#include <tuple>

namespace matrix {
enum AXIS { X,
            Y,
            Z };
enum class SHEAR_AXIS { XY,
                        XZ,
                        YX,
                        YZ,
                        ZX,
                        ZY };
enum class REFLECTION_AXIS { XY,
                             XZ,
                             YZ };

Eigen::Matrix4d scale(double x, double y, double z);

Eigen::Matrix4d shear(double delta, SHEAR_AXIS axis);

Eigen::Matrix4d translate(double x, double y, double z);

Eigen::Matrix4d rotate(double theta, AXIS axis);

Eigen::Matrix4d reflection(REFLECTION_AXIS axis);

Eigen::Matrix4d lookAt(Eigen::Vector3d eye, Eigen::Vector3d at,
                       Eigen::Vector3d up);

Eigen::Matrix4d cwMatrix(Eigen::Vector3d eye, Eigen::Vector3d at,
                         Eigen::Vector3d up);
}  // namespace matrix

#endif
