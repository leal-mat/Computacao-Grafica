#ifndef DISPLAYSTRUCTS_H_
#define DISPLAYSTRUCTS_H_

#include <eigen3/Eigen/Dense>

namespace displayStructs {
struct Viewport {
    double width;
    double height;
    double nRows;
    double nColumns;
    double dWindow;

    Viewport(double w, double h, double r, double c, double dW)
        : width(w), height(h), nRows(r), nColumns(c), dWindow(dW) {}
};

struct Camera {
    Eigen::Vector3d O;
    Eigen::Vector3d D;
    Camera(Eigen::Vector3d O, Eigen::Vector3d D) : O(O), D(D) {}
};

}  // namespace displayStructs
#endif