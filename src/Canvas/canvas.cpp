#include "canvas.h"

#include <iostream>

void Canvas::init() {
    if (SDL_Init(SDL_INIT_EVERYTHING) >= 0) {
        this->window = SDL_CreateWindow("Scene", SDL_WINDOWPOS_UNDEFINED,
                                        SDL_WINDOWPOS_UNDEFINED, this->canvasWidth,
                                        this->canvasHeight, SDL_WINDOW_ALLOW_HIGHDPI);

        if (window != nullptr) {
            this->screenSurf = SDL_GetWindowSurface(this->window);
        }
    }
}
void Canvas::update() {
    std::vector<unsigned char> pixelVector = this->scene->display();
    unsigned char *pixelArray = pixelVector.data();
    SDL_Rect offset;
    // Give the offsets to the rectangle
    offset.x = 0;
    offset.y = 0;

    surf = SDL_CreateRGBSurfaceFrom((void *)pixelArray, this->canvasWidth, this->canvasHeight, 24,
                                    3 * this->canvasWidth, 0x000000ff, 0x0000ff00,
                                    0x00ff0000, 0xff000000);
    // Blit the surface
    SDL_BlitSurface(this->surf, NULL, this->screenSurf, &offset);
    SDL_FreeSurface(this->surf);
    SDL_UpdateWindowSurface(this->window);
}
void Canvas::eventLoop(std::shared_ptr<Object> &pickedObj) {
    double xj, yj;
    displayStructs::Viewport vw = scene->getViewport();
    double deltaX = vw.width / vw.nColumns;
    double deltaY = vw.height / vw.nRows;
    SDL_Event event;

    while (true) {
        if (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                break;
            }
            if (event.type == SDL_MOUSEBUTTONDOWN) {
                xj = (-vw.width / 2.0) + (deltaX / 2.0) +
                     (event.motion.x * deltaX);
                yj = (vw.height / 2.0) - (deltaY / 2.0) -
                     (event.motion.y * deltaY);
                Eigen::Vector4d pickedD(xj, yj, -vw.dWindow, 0);
                if (scene->getProjection()) {
                    Eigen::Vector4d auxO(0, 0, 0, 1);
                    Eigen::Vector3d cam = scene->getCamera();
                    auxO(0) = cam(0);
                    auxO(1) = cam(1);
                    auxO(2) = cam(2);
                    Eigen::Vector4d direction = pickedD - auxO;
                    pickedObj = scene->pick(cam, direction.head<3>(), scene->getObjects());
                } else {
                    pickedObj = scene->pick(pickedD.head<3>(), Eigen::Vector3d(0.0, 0.0, -1.0), scene->getObjects());
                }

                if (pickedObj != nullptr) {
                    utilsStructs::materialK k = pickedObj->getK();
                    std::tuple<double, double, double> coordinates = pickedObj->getCoordinates();

                    std::cout << "Current coordinates: "
                              << "X: " << get<0>(coordinates) << " Y: " << get<1>(coordinates) << " Z: " << get<2>(coordinates) << std::endl;

                    std::cout << "Current Kd: " << k.Kd(0) << " " << k.Kd(1) << " " << k.Kd(2) << std::endl;
                } else {
                    std::cout << "No object found" << std::endl;
                }
            }
        }
    }
}

std::shared_ptr<Scene> Canvas::getScene() {
    return this->scene;
}