#pragma once

#include "Triangle.hpp"
#include "Plane.hpp"
#include "math/Vec3.hpp"

// Computes the plane of a triangle using the cross-product method
inline Plane computePlane(const Triangle& tri) {
    Vec3 a = tri.t2 - tri.t1;
    Vec3 b = tri.t3 - tri.t1;

    Vec3 n = cross(a, b);  // normal vector: (a, b, c)

    double k = -dot(n, tri.t1);

    return Plane(n.x, n.y, n.z, k);
}

// Intersect ray with plane
// returns true if intersection exists (λ >= 0), false otherwise
inline bool intersectPlane(
    const Ray& ray,
    const Plane& p,
    double& outLambda)
{
    Vec3 n(p.a, p.b, p.c);

    double denom = dot(n, ray.direction);
    if (std::abs(denom) < 1e-8) {
        return false; // Ray parallel to plane
    }

    double numer = -(dot(n, ray.origin) + p.k);
    double lambda = numer / denom;

    if (lambda < 0)
        return false; // Behind camera

    outLambda = lambda;
    return true;
}

inline bool sameSide(const Vec3& p1, const Vec3& p2, const Vec3& a, const Vec3& b)
{
    Vec3 ab = b - a;
    Vec3 cp1 = cross(ab, p1 - a);
    Vec3 cp2 = cross(ab, p2 - a);
    return dot(cp1, cp2) >= 0;
}

inline bool pointInTriangle(const Vec3& p, const Triangle& t)
{
    return
        sameSide(p, t.t1, t.t2, t.t3) &&
        sameSide(p, t.t2, t.t1, t.t3) &&
        sameSide(p, t.t3, t.t1, t.t2);
}
