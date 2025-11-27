#include <fstream>
#include <iostream>
#include <limits>

#include "math/Vec3.hpp"
#include "core/Camera.hpp"
#include "core/Ray.hpp"
#include "core/Scene.hpp"
#include "core/Geometry.hpp"

// Clamp a value to [0,1]
static double clamp01(double x) {
    if (x < 0.0) return 0.0;
    if (x > 1.0) return 1.0;
    return x;
}

// ----------------------------------------------------------
// Shadow test
// ----------------------------------------------------------
//
// Cast a ray from the hit point towards the light.
// If we hit ANY triangle before reaching the light, the point
// is in shadow.
//
// We offset the ray origin slightly along the light direction
// to avoid "self-intersection" (shadow acne).
//
bool isInShadow(const Vec3& point, const Scene& scene, const Triangle* selfTri) {
    // Vector from hit point to light
    Vec3 toLight = scene.light.position - point;
    double distToLight = toLight.length();
    if (distToLight <= 0.0) {
        return false;
    }

    Vec3 dirToLight = toLight / distToLight;   // normalized

    // Small epsilon to avoid self-intersection due to floating point error
    const double EPS = 1e-4;
    Vec3 origin = point + dirToLight * EPS;

    Ray shadowRay(origin, dirToLight);

    // Test against all triangles
    for (const Triangle& tri : scene.triangles) {
        // Optional: skip the triangle that we are already on
        if (&tri == selfTri) continue;

        double lambda = 0.0;
        if (!intersectTriangle(shadowRay, tri, lambda)) {
            continue;
        }

        // If the hit is between the point and the light, we are in shadow
        if (lambda > 0.0 && lambda < distToLight) {
            return true;
        }
    }

    return false;
}

// ----------------------------------------------------------
// Ray tracer: trace a single ray and return color
// ----------------------------------------------------------
//
// This now supports:
//   * Closest-hit triangle
//   * Lambert lighting (N·L)
//   * Distance attenuation
//   * Hard shadows (shadow rays)
//   * Ambient term
//
Vec3 traceRay(const Ray& ray, const Scene& scene, const Vec3& camPos) {
    double closestDist = std::numeric_limits<double>::infinity();
    Vec3 hitColor(0.0, 0.0, 0.0);
    bool hitSomething = false;
    const Triangle* hitTriPtr = nullptr;
    Vec3 hitPoint;

    // ------------------------------------------------------
    // Find closest intersection
    // ------------------------------------------------------
    for (const Triangle& tri : scene.triangles) {
        double lambda = 0.0;
        Vec3 localHit;

        if (!intersectTriangle(ray, tri, lambda, &localHit)) {
            continue;
        }

        double dist = (localHit - camPos).length();

        if (dist < closestDist) {
            closestDist = dist;
            hitSomething = true;
            hitTriPtr = &tri;
            hitPoint = localHit;
        }
    }

    // If we didn't hit anything, draw background sky gradient
    if (!hitSomething) {
        Vec3 d = ray.direction.normalized();
        double t = 0.5 * (d.y + 1.0); // map y from [-1,1] to [0,1]
        Vec3 skyTop(0.2, 0.4, 0.8);   // bluish
        Vec3 skyBottom(0.0, 0.0, 0.0); // black
        Vec3 skyColor = (1.0 - t) * skyBottom + t * skyTop;
        skyColor.x = clamp01(skyColor.x);
        skyColor.y = clamp01(skyColor.y);
        skyColor.z = clamp01(skyColor.z);
        return skyColor;
    }

    // ------------------------------------------------------
    // We have a hit: compute shading
    // ------------------------------------------------------
    const Triangle& hitTri = *hitTriPtr;

    // Base values
    Vec3 baseColor = hitTri.color;
    Vec3 lightColor = scene.light.color;

    // Normal at triangle (flat shaded)
    Vec3 N = computeTriangleNormal(hitTri);

    // Direction to light
    Vec3 L = (scene.light.position - hitPoint).normalized();

    // Lambert term
    double lambert = dot(N, L);
    if (lambert < 0.0) lambert = 0.0;

    // Distance attenuation (same as before)
    double dist = closestDist;
    double attenuation = 1.0 / (1.0 + 0.15 * dist * dist);

    // Ambient term (always on a little bit, so shadows are not pure black)
    const double ambient = 0.15;

    // Shadow check: if in shadow, we remove the direct light
    bool shadow = isInShadow(hitPoint, scene, hitTriPtr);

    double lightFactor = ambient;
    if (!shadow) {
        lightFactor += lambert * scene.light.intensity * attenuation;
    }

    // Combine base color and light
    hitColor = Vec3(
        baseColor.x * lightColor.x * lightFactor,
        baseColor.y * lightColor.y * lightFactor,
        baseColor.z * lightColor.z * lightFactor
    );

    // Clamp to [0,1]
    hitColor.x = clamp01(hitColor.x);
    hitColor.y = clamp01(hitColor.y);
    hitColor.z = clamp01(hitColor.z);

    return hitColor;
}

int main() {
    const int W = 400;
    const int H = 300;

    // --------------------------------------------------
    // Camera: positioned back a bit, looking along +Z
    // --------------------------------------------------
    Camera cam(
        Vec3(0.0, 0.5, -4.0),  // position
        0.0,                   // yaw
        0.0,                   // pitch
        W, H,
        1.0,                   // viewplane distance
        1.2                    // viewplane width
    );

    // --------------------------------------------------
    // Scene setup
    // --------------------------------------------------
    Scene scene;

    // Light slightly above and to the right of the camera
    scene.light = Light(
        Vec3(2.0, 4.0, -1.0),   // position
        Vec3(1.0, 1.0, 1.0),    // color (white)
        3.0                     // intensity
    );

    // Create a pyramid
    Vec3 top(0.0, 1.5, 3.5);
    Vec3 bl(-1.0, 0.0, 4.5);
    Vec3 br(1.0, 0.0, 4.5);
    Vec3 br2(1.0, 0.0, 2.5);
    Vec3 bl2(-1.0, 0.0, 2.5);

    // Colors for each triangle (RGB)
    Vec3 baseColor1(0.8, 0.2, 0.2); // reddish
    Vec3 baseColor2(0.8, 0.8, 0.2); // yellowish
    Vec3 sideColor1(0.2, 0.8, 0.2); // greenish
    Vec3 sideColor2(0.2, 0.8, 0.8); // cyan
    Vec3 sideColor3(0.2, 0.2, 0.8); // blue
    Vec3 sideColor4(0.8, 0.2, 0.8); // magenta

    // Base (square split into 2 triangles)
    scene.addTriangle(Triangle(bl, br, br2, baseColor1));
    scene.addTriangle(Triangle(bl, br2, bl2, baseColor2));

    // Sides
    scene.addTriangle(Triangle(top, bl, br, sideColor1));
    scene.addTriangle(Triangle(top, br, br2, sideColor2));
    scene.addTriangle(Triangle(top, br2, bl2, sideColor3));
    scene.addTriangle(Triangle(top, bl2, bl, sideColor4));

    // --------------------------------------------------
    // Open output file
    // --------------------------------------------------
    std::ofstream out("render_shadow.ppm");
    if (!out) {
        std::cerr << "Failed to open output file.\n";
        return 1;
    }

    // PPM header
    out << "P3\n" << W << " " << H << "\n255\n";

    // --------------------------------------------------
    // Render loop
    // --------------------------------------------------
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            // Generate camera ray for this pixel
            Ray ray = cam.generateRay(x, y);

            // Trace ray into scene (with shadows)
            Vec3 color = traceRay(ray, scene, cam.position);

            // Convert [0,1] to [0,255]
            int ir = static_cast<int>(255.99 * color.x);
            int ig = static_cast<int>(255.99 * color.y);
            int ib = static_cast<int>(255.99 * color.z);

            out << ir << " " << ig << " " << ib << "\n";
        }
    }

    out.close();
    std::cout << "Rendered shadowed image to render_shadow.ppm\n";

    return 0;
}
