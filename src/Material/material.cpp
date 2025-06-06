#include "material.hpp"
#include <iostream>

bool material::scatter()
{
    if (materialType == 1)
    {
        // Metal
        // Implement metal scattering logic here
    }
    else if (materialType == 2)
    {
        // Lambertian
        // Implement Lambertian scattering logic here
    }
    else
    {
        // Handle unknown material type
        std::cout << "Unknown material type: " << materialType << std::endl;
        return false;
    }
    return true;
}