#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <cstdint>
#include <iostream>
#include <limits>
#include <vector>
#include <cmath>

#include "math/Vec3.hpp"
#include "core/Ray.hpp"
#include "core/Camera.hpp"
#include "core/Scene.hpp"
#include "core/Geometry.hpp"
#include "platform/Win32Window.hpp"

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
bool isInShadow(const Vec3& point, const Scene& scene, const Triangle* selfTri) {
    Vec3 toLight = scene.light.position - point;
    double distToLight = toLight.length();
    if (distToLight <= 0.0) {
        return false;
    }

    Vec3 dirToLight = toLight / distToLight;
    const double EPS = 1e-4;
    Vec3 origin = point + dirToLight * EPS;

    Ray shadowRay(origin, dirToLight);

    for (const Triangle& tri : scene.triangles) {
        if (&tri == selfTri) continue;

        double lambda = 0.0;
        if (!intersectTriangle(shadowRay, tri, lambda)) {
            continue;
        }

        if (lambda > 0.0 && lambda < distToLight) {
            return true;
        }
    }

    return false;
}

// ----------------------------------------------------------
// Ray tracer: trace a single ray and return color
// ----------------------------------------------------------
Vec3 traceRay(const Ray& ray, const Scene& scene, const Vec3& camPos) {
    double closestDist = std::numeric_limits<double>::infinity();
    Vec3 hitColor(0.0, 0.0, 0.0);
    bool hitSomething = false;
    const Triangle* hitTriPtr = nullptr;
    Vec3 hitPoint;

    // Find closest intersection
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

    if (!hitSomething) {
        // Background gradient
        Vec3 d = ray.direction.normalized();
        double t = 0.5 * (d.y + 1.0);
        Vec3 skyTop(0.2, 0.4, 0.8);
        Vec3 skyBottom(0.0, 0.0, 0.0);
        Vec3 skyColor = (1.0 - t) * skyBottom + t * skyTop;
        skyColor.x = clamp01(skyColor.x);
        skyColor.y = clamp01(skyColor.y);
        skyColor.z = clamp01(skyColor.z);
        return skyColor;
    }

    const Triangle& hitTri = *hitTriPtr;

    Vec3 baseColor = hitTri.color;
    Vec3 lightColor = scene.light.color;

    Vec3 N = computeTriangleNormal(hitTri);
    Vec3 L = (scene.light.position - hitPoint).normalized();

    double lambert = dot(N, L);
    if (lambert < 0.0) lambert = 0.0;

    double dist = closestDist;
    double attenuation = 1.0 / (1.0 + 0.15 * dist * dist);

    const double ambient = 0.15;

    bool shadow = isInShadow(hitPoint, scene, hitTriPtr);

    double lightFactor = ambient;
    if (!shadow) {
        lightFactor += lambert * scene.light.intensity * attenuation;
    }

    hitColor = Vec3(
        baseColor.x * lightColor.x * lightFactor,
        baseColor.y * lightColor.y * lightFactor,
        baseColor.z * lightColor.z * lightFactor
    );

    hitColor.x = clamp01(hitColor.x);
    hitColor.y = clamp01(hitColor.y);
    hitColor.z = clamp01(hitColor.z);

    return hitColor;
}

// ----------------------------------------------------------
// Render whole scene into a framebuffer (0x00BBGGRR pixels)
// ----------------------------------------------------------
void renderSceneToBuffer(
    int width,
    int height,
    Camera& cam,
    const Scene& scene,
    std::uint32_t* framebuffer)
{
    // Make sure camera knows about current image size
    cam.setImageSize(width, height);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            Ray ray = cam.generateRay(x, y);
            Vec3 color = traceRay(ray, scene, cam.position);

            int ir = static_cast<int>(255.99 * color.x);
            int ig = static_cast<int>(255.99 * color.y);
            int ib = static_cast<int>(255.99 * color.z);

            if (ir < 0) ir = 0; if (ir > 255) ir = 255;
            if (ig < 0) ig = 0; if (ig > 255) ig = 255;
            if (ib < 0) ib = 0; if (ib > 255) ib = 255;

            // Convert to 0x00BBGGRR for Win32 DIB
            std::uint32_t pixel =
                (static_cast<std::uint32_t>(ib)) |
                (static_cast<std::uint32_t>(ig) << 8) |
                (static_cast<std::uint32_t>(ir) << 16);

            framebuffer[y * width + x] = pixel;
        }
    }
}

// ----------------------------------------------------------
// Update camera based on keyboard input
// ----------------------------------------------------------
//
// Controls:
//   W/S: move forward/back
//   A/D: strafe left/right
//   Q/E: move down/up
//   Arrow Left/Right: yaw
//   Arrow Up/Down:    pitch
//
void updateCamera(Camera& cam, double dt) {
    const double moveSpeed = 2.5;      // units per second
    const double rotSpeed = 1.5;      // radians per second

    auto keyDown = [](int vk) -> bool {
        return (GetAsyncKeyState(vk) & 0x8000) != 0;
        };

    // Forward direction on XZ plane
    Vec3 forward(std::sin(cam.yaw), 0.0, std::cos(cam.yaw));
    // Right vector on XZ plane
    Vec3 right(std::cos(cam.yaw), 0.0, -std::sin(cam.yaw));

    double moveStep = moveSpeed * dt;
    double rotStep = rotSpeed * dt;

    // Translation
    if (keyDown('W')) {
        cam.position += forward * moveStep;
    }
    if (keyDown('S')) {
        cam.position -= forward * moveStep;
    }
    if (keyDown('A')) {
        cam.position -= right * moveStep;
    }
    if (keyDown('D')) {
        cam.position += right * moveStep;
    }
    if (keyDown('Q')) {
        cam.position.y -= moveStep;
    }
    if (keyDown('E')) {
        cam.position.y += moveStep;
    }

    // Rotation
    if (keyDown(VK_LEFT)) {
        cam.yaw -= rotStep;
    }
    if (keyDown(VK_RIGHT)) {
        cam.yaw += rotStep;
    }
    if (keyDown(VK_UP)) {
        cam.pitch += rotStep;
    }
    if (keyDown(VK_DOWN)) {
        cam.pitch -= rotStep;
    }

    // Clamp pitch to avoid flipping upside-down
    const double maxPitch = 1.5; // ~86 degrees
    if (cam.pitch > maxPitch) cam.pitch = maxPitch;
    if (cam.pitch < -maxPitch) cam.pitch = -maxPitch;
}

// ----------------------------------------------------------
// Entry point
// ----------------------------------------------------------
int main() {
    // Hybrid resolutions
    const int LOW_W = 200;
    const int LOW_H = 150;
    const int HIGH_W = 400;
    const int HIGH_H = 300;

    // --------------------------------------------------
    // Create window
    // --------------------------------------------------
    Win32Window window;
    if (!window.create("NoPkg-Ren Ray Tracer (Interactive)", 800, 600)) {
        std::cerr << "Failed to create window.\n";
        return 1;
    }

    // --------------------------------------------------
    // Scene setup
    // --------------------------------------------------
    Scene scene;

    // Light above and to the right of the scene
    scene.light = Light(
        Vec3(2.0, 5.0, -2.0),   // position
        Vec3(1.0, 1.0, 1.0),    // color
        3.0                     // intensity
    );

    // Pyramid
    Vec3 top(0.0, 1.5, 3.5);
    Vec3 bl(-1.0, 0.0, 4.5);
    Vec3 br(1.0, 0.0, 4.5);
    Vec3 br2(1.0, 0.0, 2.5);
    Vec3 bl2(-1.0, 0.0, 2.5);

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

    // Ground plane (big quad as two triangles)
    Vec3 g1(-5.0, -0.001, 8.0);
    Vec3 g2(5.0, -0.001, 8.0);
    Vec3 g3(5.0, -0.001, 0.0);
    Vec3 g4(-5.0, -0.001, 0.0);
    Vec3 groundColor(0.4, 0.4, 0.4);

    scene.addTriangle(Triangle(g1, g2, g3, groundColor));
    scene.addTriangle(Triangle(g1, g3, g4, groundColor));

    // --------------------------------------------------
    // Camera
    // --------------------------------------------------
    Camera cam(
        Vec3(0.0, 1.0, -4.0),  // position
        0.0,                   // yaw
        0.0,                   // pitch
        LOW_W, LOW_H,          // start with low-res
        1.0,                   // viewplane dist
        1.2                    // viewplane width
    );

    // --------------------------------------------------
    // Framebuffers for low and high res
    // --------------------------------------------------
    std::vector<std::uint32_t> fbLow(LOW_W * LOW_H);
    std::vector<std::uint32_t> fbHigh(HIGH_W * HIGH_H);

    // --------------------------------------------------
    // Timing setup
    // --------------------------------------------------
    LARGE_INTEGER freq;
    LARGE_INTEGER prevCounter;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&prevCounter);

    bool running = true;
    bool highResRequested = false;

    std::cout << "Controls:\n"
        << "  W/S: move forward/back\n"
        << "  A/D: strafe left/right\n"
        << "  Q/E: move down/up\n"
        << "  Arrow Keys: rotate camera\n"
        << "  R: render high-res frame\n"
        << "  Close window to exit\n";

    // --------------------------------------------------
    // Main loop
    // --------------------------------------------------
    while (running) {
        // Handle OS messages
        if (!window.processMessages()) {
            running = false;
            break;
        }

        // Compute delta time
        LARGE_INTEGER curCounter;
        QueryPerformanceCounter(&curCounter);
        double dt = static_cast<double>(curCounter.QuadPart - prevCounter.QuadPart)
            / static_cast<double>(freq.QuadPart);
        prevCounter = curCounter;

        // Update camera from input
        updateCamera(cam, dt);

        // Check for high-res request (R key)
        if (GetAsyncKeyState('R') & 0x8000) {
            highResRequested = true;
        }

        if (highResRequested) {
            // Render a high-res frame (blocking)
            renderSceneToBuffer(HIGH_W, HIGH_H, cam, scene, fbHigh.data());
            window.present(fbHigh.data(), HIGH_W, HIGH_H);
            highResRequested = false;
        }
        else {
            // Render low-res interactive frame
            renderSceneToBuffer(LOW_W, LOW_H, cam, scene, fbLow.data());
            window.present(fbLow.data(), LOW_W, LOW_H);
        }

        // Small sleep to avoid maxing out CPU (optional)
        Sleep(1);
    }

    return 0;
}
