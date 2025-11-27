#pragma once

#include <cmath>

#include "Triangle.hpp"
#include "Plane.hpp"
#include "math/Vec3.hpp"
#include "Ray.hpp"

// ----------------------------------------------------------
// Plane computation from a triangle
// ----------------------------------------------------------
//
// Given a triangle t, we compute its plane:
//
//   a = t2 - t1
//   b = t3 - t1
//   n = a x b
//   k = - n · t1
//
// So the plane equation is:
//   n.x * X + n.y * Y + n.z * Z + k = 0
//
inline Plane computePlane(const Triangle& tri) {
    Vec3 a = tri.t2 - tri.t1;
    Vec3 b = tri.t3 - tri.t1;

    Vec3 n = cross(a, b);   // normal vector components (a,b,c)

    double k = -dot(n, tri.t1);

    return Plane(n.x, n.y, n.z, k);
}

// ----------------------------------------------------------
// Compute normalized triangle normal (for shading)
// ----------------------------------------------------------
//
// Normal is cross-product of two edges, normalized to unit length.
//
inline Vec3 computeTriangleNormal(const Triangle& tri) {
    Vec3 a = tri.t2 - tri.t1;
    Vec3 b = tri.t3 - tri.t1;
    Vec3 n = cross(a, b);
    return n.normalized();
}

// ----------------------------------------------------------
// Ray–Plane intersection
// ----------------------------------------------------------
//
// Ray:   r(t) = origin + direction * t
// Plane: n · X + k = 0, where n = (a,b,c)
//
// Solve for t:
//   t = - (n · origin + k) / (n · direction)
//
// Returns true if intersection exists with t >= 0.
//
inline bool intersectPlane(const Ray& ray, const Plane& p, double& outLambda) {
    Vec3 n(p.a, p.b, p.c);

    double denom = dot(n, ray.direction);
    if (std::abs(denom) < 1e-8) {
        // Ray is (almost) parallel to plane => no intersection
        return false;
    }

    double numer = -(dot(n, ray.origin) + p.k);
    double lambda = numer / denom;

    if (lambda < 0.0) {
        // Intersection is behind the camera
        return false;
    }

    outLambda = lambda;
    return true;
}

// ----------------------------------------------------------
// Point-in-triangle test using "same side" method
// ----------------------------------------------------------
//
// sameSide(p1, p2, a, b) is true if p1 and p2 are on the same
// side of the line a->b (in 3D sense using cross + dot).
//
inline bool sameSide(const Vec3& p1, const Vec3& p2, const Vec3& a, const Vec3& b) {
    Vec3 ab = b - a;
    Vec3 cp1 = cross(ab, p1 - a);
    Vec3 cp2 = cross(ab, p2 - a);
    return dot(cp1, cp2) >= 0.0;
}

// Check if point p is inside triangle t.
inline bool pointInTriangle(const Vec3& p, const Triangle& t) {
    return
        sameSide(p, t.t1, t.t2, t.t3) &&
        sameSide(p, t.t2, t.t1, t.t3) &&
        sameSide(p, t.t3, t.t1, t.t2);
}

// ----------------------------------------------------------
// Ray–Triangle intersection (full test)
// ----------------------------------------------------------
//
// 1) Intersect the ray with the plane of the triangle.
// 2) If hit, check whether the intersection point lies inside
//    the triangle using pointInTriangle().
//
// Returns true if the ray hits the triangle in front of the
// ray (lambda >= 0). If true, outLambda is set to the hit t,
// and outHitPoint (if not null) is set to the hit position.
//
inline bool intersectTriangle(
    const Ray& ray,
    const Triangle& tri,
    double& outLambda,
    Vec3* outHitPoint = nullptr
) {
    // 1) Plane intersection
    Plane p = computePlane(tri);

    double lambda = 0.0;
    if (!intersectPlane(ray, p, lambda)) {
        return false;
    }

    // 2) Get intersection point on the plane
    Vec3 hit = ray.at(lambda);

    // 3) Check if it's inside the triangle
    if (!pointInTriangle(hit, tri)) {
        return false;
    }

    outLambda = lambda;
    if (outHitPoint) {
        *outHitPoint = hit;
    }

    return true;
}
