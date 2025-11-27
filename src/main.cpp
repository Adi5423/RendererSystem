#include <fstream>
#include <iostream>
#include <string>

#include "math/Vec3.hpp"
#include "core/Camera.hpp"
#include "core/Ray.hpp"

int main() {
    const int imageWidth = 400;
    const int imageHeight = 300;

    // Camera at origin, looking straight along +Z
    Camera cam(
        Vec3(0.0, 0.0, 0.0),  // position
        0.6,                  // yaw in radians
        0.3,                  // pitch in radians
        imageWidth,
        imageHeight,
        1.0,                  // view plane distance
        0.5                   // view plane width
    );

    // Output file
    const std::string filename = "output.ppm";
    std::ofstream out(filename);

    if (!out) {
        std::cerr << "Failed to open output file: " << filename << "\n";
        return 1;
    }

    // PPM header (plain P3)
    out << "P3\n" << imageWidth << " " << imageHeight << "\n255\n";

    // For each pixel, generate a ray and color it based on its direction (for testing)
    for (int y = 0; y < imageHeight; ++y) {
        for (int x = 0; x < imageWidth; ++x) {
            Ray r = cam.generateRay(x, y);

            // Map direction to color just for debugging:
            // dir in [-1,1] -> color in [0,1]
            Vec3 d = r.direction;
            double rCol = 0.5 * (d.x + 1.0);
            double gCol = 0.5 * (d.y + 1.0);
            double bCol = 0.5 * (d.z + 1.0);

            int ir = static_cast<int>(255.99 * rCol);
            int ig = static_cast<int>(255.99 * gCol);
            int ib = static_cast<int>(255.99 * bCol);

            out << ir << " " << ig << " " << ib << "\n";
        }
    }

    out.close();
    std::cout << "Wrote test image to " << filename << "\n";

    return 0;
}
