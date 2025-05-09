#include "scene.h"

#include <chrono>
#include <iostream>

#include "../Matrix/matrix.h"

std::vector<unsigned char> Scene::display() {
    double deltaX = viewport.width / viewport.nColumns;
    double deltaY = viewport.height / viewport.nRows;
    double x, y;
    std::vector<unsigned char> pixelVector;
    Eigen::Vector3d orthDir(0, 0, -1);

    auto start = std::chrono::steady_clock::now();
    for (int r = 0; r < viewport.nRows; r++) {
        y = (viewport.height / 2) - (deltaY / 2) - (r * deltaY);
        for (int c = 0; c < viewport.nColumns; c++) {
            x = -(viewport.width / 2) + (deltaX / 2) + (c * deltaX);

            utilsStructs::Color color(0);

            if (this->isPerspective) {
                // this->camera.O = this->originBackup;
                this->camera.D(0) = x - this->camera.O(0);
                this->camera.D(1) = y - this->camera.O(1);
                this->camera.D(2) = -viewport.dWindow;
                color =
                    utils::traceRay(this->camera, lightSources, objects, int(x), int(y));
            } else {
                this->orthCamera.O(0) = x;
                this->orthCamera.O(1) = y;
                this->orthCamera.O(2) = -viewport.dWindow;
                this->orthCamera.D = orthDir;
                color =
                    utils::traceRay(this->orthCamera, lightSources, objects, int(x), int(y));
            }
            pixelVector.push_back(color.R);
            pixelVector.push_back(color.G);
            pixelVector.push_back(color.B);
        }
    }
    std::cout << "Elapsed(ms)="
              << std::chrono::duration_cast<std::chrono::milliseconds>(
                     std::chrono::steady_clock::now() - start)
                         .count() /
                     1000.0
              << std::endl;
    return pixelVector;
}

std::shared_ptr<Object> Scene::pick(Eigen::Vector3d O, Eigen::Vector3d D, std::vector<std::shared_ptr<Object>> objects) {
    double inf = INFINITY;
    // double t;
    // std::shared_ptr<Object> closestObject = nullptr;

    auto [closestT, closestObject] = utils::closestIntersection(O, D, 0, inf, objects);
    if (closestT == INFINITY) {
        closestObject = nullptr;
    }
    return closestObject;
}

std::vector<std::shared_ptr<Object>> Scene::getObjects() {
    return this->objects;
}

displayStructs::Viewport Scene::getViewport() {
    return this->viewport;
}

Eigen::Vector3d Scene::getCamera() {
    return this->camera.O;
}

bool Scene::getProjection() {
    return this->isPerspective;
}

void Scene::switchProjection() {
    this->isPerspective = !isPerspective;
    this->viewport.width = this->isPerspective ? 60 : 1500;
    this->viewport.height = this->isPerspective ? 60 : 1500;
}

void Scene::setOrigin(double x, double y, double z) {
    this->camera.O(0) = x;
    this->camera.O(1) = y;
    this->camera.O(2) = z;
}

void Scene::setFocalLength(double d) {
    this->viewport.dWindow = d;
}

void Scene::resizeViewport(double width, double height) {
    this->viewport.width = width;
    this->viewport.height = height;
}