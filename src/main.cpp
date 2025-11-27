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

// Trace a single ray into the scene and return its color.
Vec3 traceRay(const Ray& ray, const Scene& scene, const Vec3& camPos) {
    double closestDist = std::numeric_limits<double>::infinity();
    Vec3 finalColor(0.0, 0.0, 0.0);
    bool hitSomething = false;

    for (const Triangle& tri : scene.triangles) {
        // 1) Compute the plane of this triangle
        Plane p = computePlane(tri);

        // 2) Intersect ray with the plane
        double lambda = 0.0;
        if (!intersectPlane(ray, p, lambda)) {
            continue; // No intersection with plane
        }

        // 3) Get intersection point
        Vec3 hit = ray.at(lambda);

        // 4) Check if intersection is inside the triangle
        if (!pointInTriangle(hit, tri)) {
            continue;
        }

        // 5) Compute distance from camera to hit point
        double dist = (hit - camPos).length();

        // If this hit is closer than anything we've seen before, shade it
        if (dist < closestDist) {
            closestDist = dist;
            hitSomething = true;

            // --- Lambert shading ---

            // Normal of triangle (constant across surface)
            Vec3 N = computeTriangleNormal(tri);

            // Direction from hit point to light
            Vec3 L = (scene.light.position - hit).normalized();

            // Basic Lambert term
            double lambert = dot(N, L);
            if (lambert < 0.0) lambert = 0.0;

            // Distance-based attenuation (soft)
            double attenuation = 1.0 / (1.0 + 0.15 * dist * dist);

            // Combine:
            // finalColor = triangleColor * lightColor * lambert * intensity * attenuation
            Vec3 baseColor = tri.color;
            Vec3 lightCol = scene.light.color;

            finalColor =
                baseColor *
                (lambert * scene.light.intensity * attenuation) *
                lightCol.x;  // since lightCol is usually (1,1,1), we can just scale by x

            // If you want proper RGB light, use:
            // finalColor = Vec3(
            //     baseColor.x * lightCol.x * lambert * scene.light.intensity * attenuation,
            //     baseColor.y * lightCol.y * lambert * scene.light.intensity * attenuation,
            //     baseColor.z * lightCol.z * lambert * scene.light.intensity * attenuation
            // );
        }
    }

    if (!hitSomething) {
        // Background: simple gradient based on ray direction (optional)
        Vec3 d = ray.direction.normalized();
        double t = 0.5 * (d.y + 1.0); // map y from [-1,1] to [0,1]
        Vec3 skyTop(0.2, 0.4, 0.8);   // bluish
        Vec3 skyBottom(0.0, 0.0, 0.0); // black
        finalColor = (1.0 - t) * skyBottom + t * skyTop;
    }

    // Clamp to [0,1]
    finalColor.x = clamp01(finalColor.x);
    finalColor.y = clamp01(finalColor.y);
    finalColor.z = clamp01(finalColor.z);

    return finalColor;
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
    //
    // Coordinates are chosen so that:
    // - top is higher (Y)
    // - base is a square-ish shape in front of camera
    //
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
    std::ofstream out("render_color.ppm");
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

            // Trace ray into scene
            Vec3 color = traceRay(ray, scene, cam.position);

            // Convert [0,1] to [0,255]
            int ir = static_cast<int>(255.99 * color.x);
            int ig = static_cast<int>(255.99 * color.y);
            int ib = static_cast<int>(255.99 * color.z);

            out << ir << " " << ig << " " << ib << "\n";
        }
    }

    out.close();
    std::cout << "Rendered color image to render_color.ppm\n";

    return 0;
}
