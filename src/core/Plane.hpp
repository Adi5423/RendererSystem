#pragma once

struct Plane {
    double a, b, c, k;

    Plane() : a(0), b(0), c(0), k(0) {}
    Plane(double a_, double b_, double c_, double k_)
        : a(a_), b(b_), c(c_), k(k_) {
    }
};
