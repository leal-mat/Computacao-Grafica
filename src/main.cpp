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
    double lx = 450.0;
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

    Eigen::Vector3d Ke_cherry(0.949, 0.027, 0.027);
    Eigen::Vector3d Ka_cherry(0.949, 0.027, 0.027);
    Eigen::Vector3d Kd_cherry(0.949, 0.027, 0.027);

    // Eigen::Vector3d Ke_2(222.0 / 255.0, 184.0 / 255.0, 135.0 / 255.0);
    // Eigen::Vector3d Ka_2(222.0 / 255.0, 184.0 / 255.0, 135.0 / 255.0);
    // Eigen::Vector3d Kd_2(222.0 / 255.0, 184.0 / 255.0, 135.0 / 255.0);

    // background plane
    Eigen::Vector3d Ke_3(0.0, 0.933, 0.0);
    Eigen::Vector3d Ka_3(0.0, 0.933, 0.0);
    Eigen::Vector3d Kd_3(0.0, 0.933, 0.0);

    utilsStructs::materialK K_3(Ke_3, Ka_3, Kd_3);

    utilsStructs::materialK K_cherry(Ke_cherry, Ka_cherry, Kd_cherry);

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

    Sphere cherry1(K_cherry, m_1, radius, center1);

    // Chao
    Eigen::Vector4d floor_pos(0.0, 0.0, 0.0, 1);
    Eigen::Vector4d floor_dir(0.0, 1.0, 0.0, 0.0);
    Plane floor(K_3, m_2, (wc * floor_pos).head<3>(),
                ((wc * floor_dir).head<3>()).normalized());

    cherry1.scale(2.0);
    cherry1.translate(450.0, 117.5 + 2.5 + 5.0 + 40.0 + 1.0, 260, wc);

    objects.push_back(std::make_shared<Plane>(floor));

    objects.push_back(std::make_shared<Sphere>(cherry1));

    Point ponctualLight1(I_F_1);
    ponctualLight1.translate(300.0, 100.0, 1000.0, wc);

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

    std::thread inputThread([&]()
                            {
        int selected;
        while (true) {
            std::cout << "1 - Translate " << std::endl;
            std::cout << "2 - Scale " << std::endl;
            std::cout << "3 - Rotate " << std::endl;
            std::cout << "4 - Shear " << std::endl;
            std::cout << "5 - Mirroring " << std::endl;
            std::cout << "6 - Modify Material " << std::endl;
            std::cout << "7 - Reposition Camera " << std::endl;
            std::cout << "8 - Manage Light Sources " << std::endl;
            std::cout << "9 - Switch Camera Projection" << std::endl;
            std::cout << "10 - Change viewport" << std::endl;
            std::cout << "11 - Change Focal Length (D)" << std::endl;

            std::cin >> selected;
            Eigen::Matrix4d m;

            if (pickedObj == nullptr && selected >= 1 && selected <= 6) {
                std::cout << "No selected object" << std::endl;
                continue;
            }

            switch (selected) {
                case 1: {
                    double x, y, z;
                    std::cout << "Set coordinates (X,Y,Z): ";
                    std::cin >> x;
                    std::cin >> y;
                    std::cin >> z;

                    pickedObj->returnToWorld(cw, false);
                    pickedObj->translate(x, y, z, wc);
                    canvas.update();
                    break;
                }
                case 2: {
                    double x = 0.0, y = x, z = x;

                    if (pickedObj->getType() == utilsStructs::OBJ_TYPE::CONE) {
                        std::cout << "Set scale (Radius, Height): ";
                        std::cin >> x;
                        std::cin >> y;
                    }
                    if (pickedObj->getType() == utilsStructs::OBJ_TYPE::CYLINDER) {
                        std::cout << "Set scale (Radius, Height): ";
                        std::cin >> x;
                        std::cin >> y;
                    }
                    if (pickedObj->getType() == utilsStructs::OBJ_TYPE::MESH) {
                        std::cout << "Set scale (X,Y,Z): ";
                        std::cin >> x;
                        std::cin >> y;
                        std::cin >> z;
                    }
                    if (pickedObj->getType() == utilsStructs::OBJ_TYPE::PLANE) {
                        std::cout << "Invalid transformation for plane" << std::endl;
                        break;
                    }
                    if (pickedObj->getType() == utilsStructs::OBJ_TYPE::SPHERE) {
                        std::cout << "Set scale: ";
                        std::cin >> x;
                    }

                    pickedObj->returnToWorld(cw, false);

                    pickedObj->scale(x, y, z);

                    std::tuple<double, double, double> coordinates = pickedObj->getCoordinates();
                    pickedObj->translate(std::get<0>(coordinates), std::get<1>(coordinates), std::get<2>(coordinates), wc);

                    canvas.update();
                    break;
                }

                case 3: {
                    double angle;
                    int axis;
                    if (pickedObj->getType() != utilsStructs::OBJ_TYPE::SPHERE) {
                        std::cout << "Rotating Angle: ";
                        std::cin >> angle;
                        std::cout << "Select Axis: ";
                        std::cout << "0 - X" << std::endl;
                        std::cout << "1 - Y" << std::endl;
                        std::cout << "2 - Z" << std::endl;
                        std::cin >> axis;

                        matrix::AXIS axisEnum = static_cast<matrix::AXIS>(axis);

                        pickedObj->returnToWorld(cw, false);

                        pickedObj->rotate(angle, axisEnum);

                        std::tuple<double, double, double> coordinates = pickedObj->getCoordinates();
                        std::cout << std::get<0>(coordinates) << " " << std::get<1>(coordinates) << " " << std::get<2>(coordinates) << "\n";
                        pickedObj->translate(std::get<0>(coordinates), std::get<1>(coordinates), std::get<2>(coordinates), wc);
                        canvas.update();
                    }
                    break;
                }

                case 4:
                    double angle;
                    int axis;
                    if (pickedObj->getType() == utilsStructs::OBJ_TYPE::MESH) {
                        std::cout << "Shearing Angle: ";
                        std::cin >> angle;
                        std::cout << "Select Axis: ";
                        std::cout << "0 - XY" << std::endl;
                        std::cout << "1 - XZ" << std::endl;
                        std::cout << "2 - YX" << std::endl;
                        std::cout << "3 - YZ" << std::endl;
                        std::cout << "4 - ZX" << std::endl;
                        std::cout << "5 - ZY" << std::endl;
                        std::cin >> axis;

                        matrix::SHEAR_AXIS axisEnum = static_cast<matrix::SHEAR_AXIS>(axis);

                        pickedObj->returnToWorld(cw, false);

                        pickedObj->shear(angle, axisEnum);

                        std::tuple<double, double, double> coordinates = pickedObj->getCoordinates();
                        pickedObj->translate(std::get<0>(coordinates), std::get<1>(coordinates), std::get<2>(coordinates), wc);
                        canvas.update();
                    }
                    break;

                case 5: {
                    int axis;
                    std::cout << "Select Axis: " << std::endl;
                    std::cout << "0 - XY" << std::endl;
                    std::cout << "1 - XZ" << std::endl;
                    std::cout << "2 - YZ" << std::endl;
                    std::cin >> axis;

                    matrix::REFLECTION_AXIS axisEnum = static_cast<matrix::REFLECTION_AXIS>(axis);
                    pickedObj->returnToWorld(cw, true);
                    pickedObj->reflection(axisEnum, scene->objects, wc);
                    pickedObj->backToCamera(wc);
                    canvas.update();
                    break;
                }

                case 6: {
                    int matOption;
                    double p1, p2, p3;

                    std::cout << "Modify Material" << std::endl;
                    std::cout << "1 - Ambient" << std::endl;
                    std::cout << "2 - Difuse" << std::endl;
                    std::cout << "3 - Specular" << std::endl;
                    std::cout << "4 - All" << std::endl;
                    std::cin >> matOption;
                    std::cout << "Enter new properties (0 to 1): (p1, p2, p3): " << std::endl;
                    std::cin >> p1;
                    std::cin >> p2;
                    std::cin >> p3;

                    pickedObj->modifyK(Eigen::Vector3d(p1, p2, p3), matOption);
                    canvas.update();
                    break;
                }

                case 7: {
                    double x_O, y_O, z_O;
                    double x_at, y_at, z_at;
                    double x_up, y_up, z_up;
                    std::cout << "Insert new CAMERA coordinates (x, y, z): ";
                    std::cin >> x_O;
                    std::cin >> y_O;
                    std::cin >> z_O;

                    std::cout << "Insert new AT coordinates (x, y, z): ";
                    std::cin >> x_at;
                    std::cin >> y_at;
                    std::cin >> z_at;

                    std::cout << "Insert new UP coordinates (x, y, z): ";
                    std::cin >> x_up;
                    std::cin >> y_up;
                    std::cin >> z_up;

                    O(0) = x_O;
                    O(1) = y_O;
                    O(2) = z_O;

                    at(0) = x_at;
                    at(1) = y_at;
                    at(2) = z_at;

                    up(0) = x_up;
                    up(1) = y_up;
                    up(2) = z_up;

                    for (std::shared_ptr<Object> obj : scene->objects) {
                        obj->returnToWorld(cw, false);
                    }

                    for (std::shared_ptr<LightSource> lightSource : scene->lightSources) {
                        lightSource->returnToWorld(cw);
                    }

                    wc = matrix::lookAt(O.head<3>(), at, up);
                    cw = matrix::cwMatrix(O.head<3>(), at, up);
                    O = wc * O;
                    std::cout << O(0) << " " << O(1) << " " << O(2) << "\n";
                    scene->setOrigin(O(0), O(1), O(2));

                    for (std::shared_ptr<Object> obj : scene->objects) {
                        obj->translate(std::get<0>(obj->getCoordinates()), std::get<1>(obj->getCoordinates()), std::get<2>(obj->getCoordinates()), wc);
                    }

                    for (std::shared_ptr<LightSource> lightSource : scene->lightSources) {
                        Eigen::Vector3d pf = lightSource->getPF();
                        lightSource->translate(pf(0), pf(1), pf(2), wc);
                    }
                    canvas.update();
                    break;
                }

                case 8: {
                    int option = 0;
                    for (std::shared_ptr<LightSource> lightSource : scene->lightSources) {
                        if (lightSource->lightType == LightSource::lightTypes::POINT) {
                            std::cout << "Light source " << option << ": "
                                      << "POINT" << std::endl;
                        }
                        if (lightSource->lightType == LightSource::lightTypes::AMBIENT) {
                            std::cout << "Light source " << option << ": "
                                      << "AMBIENT" << std::endl;
                        }
                        if (lightSource->lightType == LightSource::lightTypes::SPOTLIGHT) {
                            std::cout << "Light source " << option << ": "
                                      << "SPOTLIGHT" << std::endl;
                        }
                        if (lightSource->lightType == LightSource::lightTypes::DIRECTIONAL) {
                            std::cout << "Light source " << option << ": "
                                      << "DIRECTIONAL" << std::endl;
                        }
                        option++;
                    }
                    int l, operation;
                    double intensity1, intensity2, intensity3;
                    std::cout << "Choose a light source: " << std::endl;
                    std::cin >> l;
                    std::cout << "1 - Change light intensity " << std::endl;
                    std::cout << "2 - Light specific operations " << std::endl;
                    std::cin >> operation;

                    if (operation == 1) {
                        std::cout << "Set Intensity (intensity1 intensity2 intensity3): " << std::endl;
                        std::cin >> intensity1;
                        std::cin >> intensity2;
                        std::cin >> intensity3;
                        scene->lightSources[l]->setIF(intensity1, intensity2, intensity3);
                    }
                    if (operation == 2 && scene->lightSources[l]->lightType != LightSource::lightTypes::AMBIENT) {
                        if (scene->lightSources[l]->lightType == LightSource::lightTypes::POINT) {
                            double x, y, z;
                            scene->lightSources[l]->returnToWorld(cw);
                            std::cout << "Set new position (x y z): " << std::endl;
                            std::cin >> x;
                            std::cin >> y;
                            std::cin >> z;
                            scene->lightSources[l]->translate(x, y, z, wc);
                        }
                        if (scene->lightSources[l]->lightType == LightSource::lightTypes::DIRECTIONAL) {
                            // Mudar a direcao
                            double x, y, z;
                            scene->lightSources[l]->returnToWorld(cw);
                            std::cout << "Set new direction (x y z): " << std::endl;
                            std::cin >> x;
                            std::cin >> y;
                            std::cin >> z;
                            scene->lightSources[l]->changeDirection(x, y, z, wc);
                        }
                        if (scene->lightSources[l]->lightType == LightSource::lightTypes::SPOTLIGHT) {
                            double spot_op;
                            std::cout << "Choose an operation: " << std::endl;
                            std::cout << "1 - Modify opening angle" << std::endl;
                            std::cout << "2 - Change light source position" << std::endl;
                            std::cout << "3 - Change spot location(Change location to where light is pointing)" << std::endl;
                            std::cin >> spot_op;
                            if (spot_op == 1) {
                                double new_angle;
                                std::cout << "Enter new angle value:" << std::endl;
                                std::cin >> new_angle;
                                scene->lightSources[l]->setTheta(new_angle);
                            }
                            if (spot_op == 2) {
                                double x, y, z;
                                scene->lightSources[l]->returnToWorld(cw);
                                std::cout << "Set new position (x y z): " << std::endl;
                                std::cin >> x;
                                std::cin >> y;
                                std::cin >> z;
                                scene->lightSources[l]->translate(x, y, z, wc);
                            }
                            if (spot_op == 3) {
                                double x, y, z;
                                scene->lightSources[l]->returnToWorld(cw);
                                std::cout << "Set new position to point (x y z): " << std::endl;
                                std::cin >> x;
                                std::cin >> y;
                                std::cin >> z;
                                scene->lightSources[l]->changeDirection(x, y, z, wc);
                                Eigen::Vector3d pos = scene->lightSources[l]->getPF();
                                scene->lightSources[l]->translate(pos(0), pos(1), pos(2), wc);
                            }
                        }
                    } else {
                        std::cout << "There are no specific operations for this light type\n";
                    }

                    canvas.update();
                    break;
                }
                case 9: {
                    scene->switchProjection();
                    std::string projection = scene->getProjection() ? "Perspective" : "Ortogonal";
                    canvas.update();
                    std::cout << "Projection changed to: " << projection << std::endl;
                    break;
                }
                case 10: {
                    double w, h;
                    std::cout << "Insert new dimensions (width, height): " << std::endl;
                    std::cin >> w;
                    std::cin >> h;
                    scene->resizeViewport(w, h);
                    canvas.update();
                    break;
                }

                case 11: {
                    double d;
                    std::cout << "Enter a new Focal Length value: " << std::endl;
                    std::cin >> d;
                    scene->setFocalLength(d);
                    canvas.update();
                    break;
                }

                default:
                    std::cout << "Invalid Option" << std::endl;
                    break;
            }
            // system("clear");
        } });
    std::cout << "Out of the while" << std::endl;
    canvas.eventLoop(pickedObj);
    return 0;
}
