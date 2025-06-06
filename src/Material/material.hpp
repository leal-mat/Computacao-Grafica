#ifndef MATERIAL_H_
#define MATERIAL_H_

#include "../Utils/utilsStructs.h"

class material
{
public:
    // int 1 = Metal
    // int 2 = Lambertian
    material(int materialType, utilsStructs::Color albedo) : materialType(materialType), albedo(albedo) {};
    ~material() = default;
    bool scatter();

private:
    utilsStructs::Color albedo;
    int materialType;
};

#endif MATERIAL_H_