#pragma once

#include <vector>
#include "Triangle.hpp"
#include "Light.hpp"

// Scene holds all the triangles and a single light for now.
// Later you can add more lights, materials, etc.
struct Scene {
    std::vector<Triangle> triangles;
    Light light;

    void addTriangle(const Triangle& t) {
        triangles.push_back(t);
    }
};
