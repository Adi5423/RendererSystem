#include <fstream>
#include <iostream>
#include <limits>

#include "core/Camera.hpp"
#include "core/Scene.hpp"
#include "core/Geometry.hpp"

int main() {
    const int W = 400;
    const int H = 300;

    Camera cam(
        Vec3(0, 0, -3),   // camera pulled back a bit
        0.0,              // yaw
        0.0,              // pitch
        W, H,
        1.0,              // viewplane dist
        1.0               // width
    );

    Scene scene;

    // --------------------------
    // Create a pyramid (4 triangles)
    // --------------------------

    Vec3 top(0, 1, 3);
    Vec3 bl(-1, -1, 4);
    Vec3 br(1, -1, 4);
    Vec3 br2(1, -1, 2);
    Vec3 bl2(-1, -1, 2);

    // Pyramid base is a square split into 2 triangles
    scene.addTriangle(Triangle(bl, br, br2));
    scene.addTriangle(Triangle(bl, br2, bl2));

    // 3 sides
    scene.addTriangle(Triangle(top, bl, br));
    scene.addTriangle(Triangle(top, br, br2));
    scene.addTriangle(Triangle(top, br2, bl2));
    scene.addTriangle(Triangle(top, bl2, bl));

    // --------------------------
    // Prepare output
    // --------------------------
    std::ofstream out("render.ppm");
    out << "P3\n" << W << " " << H << "\n255\n";

    // --------------------------
    // Render loop
    // --------------------------

    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {

            Ray ray = cam.generateRay(x, y);

            double closestDist = std::numeric_limits<double>::infinity();
            double brightness = 0.0;

            // test against all triangles
            for (const Triangle& t : scene.triangles) {
                Plane p = computePlane(t);

                double lambda;
                if (!intersectPlane(ray, p, lambda))
                    continue;

                Vec3 hit = ray.at(lambda);

                if (!pointInTriangle(hit, t))
                    continue;

                double dist = (hit - cam.position).length();
                if (dist < closestDist) {
                    closestDist = dist;
                    brightness = 1.0 / dist;
                }
            }

            int col = static_cast<int>(255.99 * brightness);
            if (col > 255) col = 255;

            out << col << " " << col << " " << col << "\n";
        }
    }

    out.close();
    std::cout << "Render saved to render.ppm\n";
}
