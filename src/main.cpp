#define _USE_MATH_DEFINES

#include <SDL2/SDL.h>

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>
#include <tuple>

#include "Canvas/canvas.h"
#include "Display/display.h"
#include "Display/displayStructs.h"
#include "LightSources/Ambient/ambient.h"
#include "LightSources/Directional/directional.h"
#include "LightSources/LightSource/lightSource.h"
#include "LightSources/Point/point.h"
#include "LightSources/Spot/spot.h"
#include "Matrix/matrix.h"
#include "Objects/Cone/cone.h"
#include "Objects/Cylinder/cylinder.h"
#include "Objects/Mesh/mesh.h"
#include "Objects/Plane/plane.h"
#include "Objects/Sphere/sphere.h"
#include "Scene/scene.h"
#include "Utils/utils.h"
#include "Utils/utilsStructs.h"

int main()
{
    double radius = 60.0;
    double dWindow = 25;
    // double x = 0;
    // double y = 0;
    // double z = -(dWindow + radius);

    bool isPerspective = true;
    double canvasWidth = 500;
    double canvasHeight = 500;
    double viewPortWidth = isPerspective ? 60 : 1500;
    double viewPortHeight = isPerspective ? 60 : 1500;
    double nRow = 500;
    double nCol = 500;
    double lx = 350.0;
    double ly = 210.0;
    double lz = 1200.0;

    // double I_A = 0.3;

    Eigen::Vector4d O(lx, ly, lz, 1.0);
    Eigen::Vector4d D(0.0, 0.0, 0.0, 1.0);
    // Eigen::Vector3d at(450.0, 97.5, 500.0);
    Eigen::Vector3d at(450.0, 97.5, 550.0);
    // Eigen::Vector3d at(lx, ly, 1.0);
    // Eigen::Vector3d up(lx, ly + 100.0, lz);
    Eigen::Vector3d up(lx, ly + 100.0, lz);
    Eigen::Matrix4d wc = matrix::lookAt(O.head<3>(), at, up);
    Eigen::Matrix4d cw = matrix::cwMatrix(O.head<3>(), at, up);
    // std::cout << up - O.head<3>() << "\n";
    O = wc * O;
    displayStructs::Camera camera(O.head<3>(), D.head<3>());
    // Eigen::Vector3d center(x, y, z);

    Eigen::Vector3d I_F_1(0.3, 0.3, 0.3);

    Eigen::Vector3d I_F_2(0.1, 0.1, 0.1);
    Eigen::Vector4d D_F_2(1.0, 0.0, 0.0, 0.0);

    Eigen::Vector3d I_F_3(0.9, 0.9, 0.9);
    Eigen::Vector4d P_I_3(900.0, 360.0, 300.0, 1.0); // Para onde a luz vai apontar
    // P_I_3 = wc * P_I_3;
    Eigen::Vector4d P_S_3(450.0, 310.0, 420.0, 1.0); // Posicao da luz spot no mundo
    P_S_3 = wc * P_S_3;
    // double theta = 30;

    // sphere
    Eigen::Vector3d Ke_1(0.854, 0.647, 0.125);
    Eigen::Vector3d Ka_1(0.854, 0.647, 0.125);
    Eigen::Vector3d Kd_1(0.854, 0.647, 0.125);

    Eigen::Vector3d Ke_sphere1(0.949, 0.027, 0.027);
    Eigen::Vector3d Ka_sphere1(0.949, 0.027, 0.027);
    Eigen::Vector3d Kd_sphere1(0.949, 0.027, 0.027);

    Eigen::Vector3d Ke_sphere2(0.000, 0.120, 1.000);
    Eigen::Vector3d Ka_sphere2(0.000, 0.120, 1.000);
    Eigen::Vector3d Kd_sphere2(0.000, 0.120, 1.000);

    // Eigen::Vector3d Ke_2(222.0 / 255.0, 184.0 / 255.0, 135.0 / 255.0);
    // Eigen::Vector3d Ka_2(222.0 / 255.0, 184.0 / 255.0, 135.0 / 255.0);
    // Eigen::Vector3d Kd_2(222.0 / 255.0, 184.0 / 255.0, 135.0 / 255.0);

    // background plane
    Eigen::Vector3d Ke_3(0.0, 0.933, 0.0);
    Eigen::Vector3d Ka_3(0.0, 0.933, 0.0);
    Eigen::Vector3d Kd_3(0.0, 0.933, 0.0);

    utilsStructs::materialK K_3(Ke_3, Ka_3, Kd_3);

    utilsStructs::materialK K_sphere1(Ke_sphere1, Ka_sphere1, Kd_sphere1);
    utilsStructs::materialK K_sphere2(Ke_sphere2, Ka_sphere2, Kd_sphere2);

    displayStructs::Viewport viewport(viewPortWidth, viewPortHeight, nRow, nCol,
                                      dWindow);

    std::vector<std::shared_ptr<LightSource>> lightSources;
    std::vector<std::shared_ptr<Object>> objects;

    Eigen::Vector3d center1(0.0, 0.0, 0.0);
    Eigen::Vector3d center2(0.0, 0.0, 0.0);
    Eigen::Vector3d center3(0, -60, -200);
    Eigen::Vector3d center4(0, 20, -150);

    double m_1 = 10;
    double m_2 = 1;
    // double m_3 = 1;

    Sphere sphere1(K_sphere1, m_1, radius, center1);
    Sphere sphere2(K_sphere2, m_1, radius, center1);

    // Chao
    Eigen::Vector4d floor_pos(0.0, 0.0, 0.0, 1);
    Eigen::Vector4d floor_dir(0.0, 1.0, 0.0, 0.0);
    Plane floor(K_3, m_2, (wc * floor_pos).head<3>(),
                ((wc * floor_dir).head<3>()).normalized());

    sphere1.scale(1.0);
    sphere1.translate(450.0, 117.5 + 2.5 + 5.0 + 40.0 + 1.0, 1000, wc);

    sphere2.scale(1.0);
    sphere2.translate(250.0, 117.5 + 2.5 + 5.0 + 40.0 + 1.0, 1000, wc);

    objects.push_back(std::make_shared<Plane>(floor));

    objects.push_back(std::make_shared<Sphere>(sphere1));
    objects.push_back(std::make_shared<Sphere>(sphere2));

    Point ponctualLight1(I_F_1);
    ponctualLight1.translate(300.0, 200.0, 1100.0, wc);

    // Directional directionalLight1(I_F_2);
    // directionalLight1.translate(-1.0, 0.0, 0, wc);

    // Spot spotLight1(I_F_3, P_I_3, 15.0);
    // spotLight1.translate(470.0, 350.0, 630.0, wc);

    lightSources.push_back(std::make_shared<Point>(ponctualLight1));
    lightSources.push_back(std::make_shared<Ambient>(Ambient(Eigen::Vector3d(0.3, 0.3, 0.3))));
    // lightSources.push_back(std::make_shared<Directional>(directionalLight1));
    // lightSources.push_back(std::make_shared<Spot>(spotLight1));

    std::shared_ptr<Object> pickedObj = nullptr;
    std::shared_ptr<Scene> scene = std::make_shared<Scene>(Scene(viewport, camera, lightSources, objects, isPerspective));
    Canvas canvas(canvasWidth, canvasHeight, scene);

    canvas.init();
    canvas.update();

    canvas.eventLoop(pickedObj);
    return 0;
}
