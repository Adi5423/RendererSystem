#pragma once

#include "math/Vec3.hpp"

struct Ray {
    Vec3 origin;      // s in the explanation
    Vec3 direction;   // m in the explanation (should be normalized)

    Ray() = default;
    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d) {}

    // Point along the ray at parameter t (lambda)
    Vec3 at(double t) const {
        return origin + direction * t;
    }
};
