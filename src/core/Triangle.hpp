#pragma once

#include "math/Vec3.hpp"

struct Triangle {
    Vec3 t1;
    Vec3 t2;
    Vec3 t3;

    Triangle() = default;
    Triangle(const Vec3& a, const Vec3& b, const Vec3& c)
        : t1(a), t2(b), t3(c) {
    }
};
