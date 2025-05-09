#ifndef CANVAS_H_
#define CANVAS_H_
#include <SDL2/SDL.h>

#include <tuple>

#include "../Scene/scene.h"

class Canvas {
   public:
    Canvas(int canvasWidth, int canvasHeight, std::shared_ptr<Scene> scene) : scene(scene) {
        this->canvasWidth = canvasWidth;
        this->canvasHeight = canvasHeight;
    }
    ~Canvas() {
        if (this->screenSurf != nullptr)
            SDL_FreeSurface(this->screenSurf);
        if (this->surf != nullptr)
            SDL_FreeSurface(surf);
        if (this->window != nullptr)
            SDL_DestroyWindow(this->window);

        SDL_Quit();
    }

    void init();
    void update();
    void eventLoop(std::shared_ptr<Object>& pickedObj);
    std::shared_ptr<Scene> getScene();

   private:
    int canvasWidth;
    int canvasHeight;
    std::shared_ptr<Scene> scene;
    SDL_Surface* surf = nullptr;
    SDL_Surface* screenSurf = nullptr;
    SDL_Window* window = nullptr;
};

#endif