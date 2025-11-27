#pragma once

#include <vector>
#include "Triangle.hpp"

struct Scene {
    std::vector<Triangle> triangles;

    void addTriangle(const Triangle& t) {
        triangles.push_back(t);
    }
};
