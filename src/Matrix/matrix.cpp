#include "matrix.h"
#define M_PI 3.14159265358979323846

namespace matrix {
Eigen::Matrix4d scale(double x, double y, double z) {
    Eigen::Matrix4d m;
    m << x, 0, 0, 0, 0, y, 0, 0, 0, 0, z, 0, 0, 0, 0, 1;
    return m;
}

Eigen::Matrix4d shear(double delta, SHEAR_AXIS axis) {
    Eigen::Matrix4d m;
    delta = delta * M_PI / 180.0;
    switch (axis) {
        case matrix::SHEAR_AXIS::XY:
            m << 1, 0, 0, 0, std::tan(delta), 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
            break;
        case matrix::SHEAR_AXIS::XZ:
            m << 1, 0, 0, 0, 0, 1, 0, 0, std::tan(delta), 0, 1, 0, 0, 0, 0, 1;
            break;
        case matrix::SHEAR_AXIS::YX:
            m << 1, std::tan(delta), 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
            break;
        case matrix::SHEAR_AXIS::YZ:
            m << 1, 0, 0, 0, 0, 1, 0, 0, 0, std::tan(delta), 1, 0, 0, 0, 0, 1;
            break;
        case matrix::SHEAR_AXIS::ZX:
            m << 1, 0, std::tan(delta), 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
            break;
        case matrix::SHEAR_AXIS::ZY:
            m << 1, 0, 0, 0, 0, 1, std::tan(delta), 0, 0, 0, 1, 0, 0, 0, 0, 1;
            break;
        default:
            break;
    }
    return m;
}

Eigen::Matrix4d translate(double x, double y, double z) {
    Eigen::Matrix4d m;
    m << 1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1;
    return m;
}

Eigen::Matrix4d rotate(double theta, AXIS axis) {
    Eigen::Matrix4d m;
    theta = theta * M_PI / 180.0;
    if (axis == AXIS::X) {
        m << 1, 0, 0, 0, 0, std::cos(theta), -std::sin(theta), 0, 0,
            std::sin(theta), std::cos(theta), 0, 0, 0, 0, 1;
    } else if (axis == AXIS::Y) {
        m << std::cos(theta), 0, std::sin(theta), 0, 0, 1, 0, 0, -std::sin(theta),
            0, std::cos(theta), 0, 0, 0, 0, 1;
    } else if (axis == AXIS::Z) {
        m << std::cos(theta), -std::sin(theta), 0, 0, std::sin(theta),
            std::cos(theta), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    }
    return m;
}

Eigen::Matrix4d reflection(REFLECTION_AXIS axis) {
    Eigen::Matrix4d m;
    switch (axis) {
        case matrix::REFLECTION_AXIS::XY:
            std::cout << "CASO 1\n";
            m << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, -1, 0,
                0, 0, 0, 1;
            break;
        case matrix::REFLECTION_AXIS::XZ:
            std::cout << "CASO 2\n";
            m << 1, 0, 0, 0,
                0, -1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
            break;
        case matrix::REFLECTION_AXIS::YZ:
            std::cout << "CASO 3\n";
            m << -1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
            break;
        default:
            break;
    }
    return m;
}

Eigen::Matrix4d lookAt(Eigen::Vector3d eye, Eigen::Vector3d at,
                       Eigen::Vector3d up) {
    Eigen::Matrix4d wc;
    Eigen::Matrix4d cw;
    Eigen::Vector3d ic(0.0, 0.0, 0.0);
    Eigen::Vector3d jc(0.0, 0.0, 0.0);
    Eigen::Vector3d kc(0.0, 0.0, 0.0);

    kc = (eye - at).normalized();

    Eigen::Vector3d vup = up - eye;

    ic = (vup.cross(kc)).normalized();

    jc = kc.cross(ic);

    // Matrix WC
    wc << ic(0), ic(1), ic(2), -(ic.dot(eye)), jc(0), jc(1), jc(2),
        -(jc.dot(eye)), kc(0), kc(1), kc(2), -(kc.dot(eye)), 0, 0, 0, 1;

    // std::cout << wc << "\n";

    // Matrix CW
    /*cw << ic(0), jc(0), kc(0), eye(0), ic(1), jc(1), kc(1), eye(1), ic(2),
       jc(2), kc(2), eye(2), 0, 0, 0, 1;*/

    return wc;
}

Eigen::Matrix4d cwMatrix(Eigen::Vector3d eye, Eigen::Vector3d at,
                         Eigen::Vector3d up) {
    Eigen::Matrix4d cw;
    Eigen::Vector3d ic(0.0, 0.0, 0.0);
    Eigen::Vector3d jc(0.0, 0.0, 0.0);
    Eigen::Vector3d kc(0.0, 0.0, 0.0);

    kc = (eye - at).normalized();

    Eigen::Vector3d vup = up - eye;

    ic = (vup.cross(kc)).normalized();

    jc = kc.cross(ic);

    cw << ic(0), jc(0), kc(0), eye(0), ic(1), jc(1), kc(1), eye(1), ic(2),
        jc(2), kc(2), eye(2), 0, 0, 0, 1;

    return cw;
}
}  // namespace matrix