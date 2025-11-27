#pragma once

#include "math/Vec3.hpp"

// A single triangle in 3D space with a flat color.
// t1, t2, t3 are the three vertices.
// color is the base RGB color (each component in [0,1]).
struct Triangle {
    Vec3 t1;
    Vec3 t2;
    Vec3 t3;
    Vec3 color;   // base color (e.g., red = (1,0,0))

    Triangle() = default;

    Triangle(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& col)
        : t1(a), t2(b), t3(c), color(col) {
    }
};
