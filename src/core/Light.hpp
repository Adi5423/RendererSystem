#pragma once

#include "math/Vec3.hpp"

// Simple point light.
// position: where the light is in the scene.
// color: RGB color (usually (1,1,1) for white).
// intensity: scalar multiplier for brightness.
struct Light {
    Vec3 position;
    Vec3 color;
    double intensity;

    Light()
        : position(0.0, 5.0, 0.0),
        color(1.0, 1.0, 1.0),
        intensity(1.0) {
    }

    Light(const Vec3& pos, const Vec3& col, double inten)
        : position(pos), color(col), intensity(inten) {
    }
};
