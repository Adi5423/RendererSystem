#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <algorithm> // For std::min

#include <cstdint>
#include <iostream>
#include <limits>
#include <vector>
#include <thread>


#include "math/Vec3.hpp"
#include "core/Ray.hpp"
#include "core/Camera.hpp"
#include "core/Scene.hpp"
#include "core/Geometry.hpp"
#include "platform/Win32Window.hpp"

// ----------------------------------------------------------
// Utility: clamp [0,1]
// ----------------------------------------------------------
static double clamp01(double x) {
    if (x < 0.0) return 0.0;
    if (x > 1.0) return 1.0;
    return x;
}

// ----------------------------------------------------------
// Simple per-thread RNG (xorshift32)
// ----------------------------------------------------------
inline double rand01(std::uint32_t& state) {
    // xorshift32
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    // Keep only lower 24 bits for fraction
    std::uint32_t v = state & 0xFFFFFFu;
    return static_cast<double>(v) / static_cast<double>(0x1000000u); // [0,1)
}

// ----------------------------------------------------------
// Shadow test
// ----------------------------------------------------------
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
// Render low-res scene into framebuffer (0x00BBGGRR pixels)
// ----------------------------------------------------------
void renderLowRes(
    int width,
    int height,
    Camera& cam,
    const Scene& scene,
    std::uint32_t* framebuffer)
{
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

            std::uint32_t pixel =
                (static_cast<std::uint32_t>(ib)) |
                (static_cast<std::uint32_t>(ig) << 8) |
                (static_cast<std::uint32_t>(ir) << 16);

            framebuffer[y * width + x] = pixel;
        }
    }
}

// ----------------------------------------------------------
// Progressive high-res sample (multi-threaded)
// ----------------------------------------------------------
//
// Each call adds ONE new sample per pixel and updates:
//   - fbAccum : floating-point accumulated color
//   - framebuffer : 8-bit display buffer (0x00BBGGRR)
//
// sampleIndex = current number of samples already accumulated.
// New sample is folded like:
//   newAcc = (oldAcc * sampleIndex + sampleColor) / (sampleIndex + 1)
// ----------------------------------------------------------
void renderHighResSample(
    int width,
    int height,
    Camera& cam,
    const Scene& scene,
    std::vector<Vec3>& fbAccum,
    std::uint32_t* framebuffer,
    int sampleIndex)
{
    cam.setImageSize(width, height);

    // ---------------------------------------
    // MSVC-safe hardware concurrency fetch
    // ---------------------------------------
    unsigned int nt = std::thread::hardware_concurrency();
    int numThreads = nt > 0 ? (int)nt : 1;

    std::vector<std::thread> threads;
    threads.reserve(numThreads);

    int blockHeight = (height + numThreads - 1) / numThreads;

    for (int t = 0; t < numThreads; ++t) {
        int startY = t * blockHeight;
        int endY = (std::min)(height, startY + blockHeight);
        if (startY >= endY) break;

        // Per-thread RNG seed
        std::uint32_t seed = (std::uint32_t)GetTickCount64() ^ (t * 0x9E3779B1u);

        // ---------------------------------------
        // MSVC-friendly lambda:
        // capture all by reference [&]
        // copy needed small variables explicitly
        // ---------------------------------------
        threads.emplace_back(
            [&, sampleIndex, startY, endY, width, height, seed]() mutable
            {
                for (int y = startY; y < endY; ++y) {
                    for (int x = 0; x < width; ++x) {

                        // Sub-pixel jitter
                        double jx = rand01(seed) - 0.5;
                        double jy = rand01(seed) - 0.5;

                        Ray ray = cam.generateRayJittered(x, y, jx, jy);
                        Vec3 color = traceRay(ray, scene, cam.position);

                        int idx = y * width + x;

                        // Progressive accumulation
                        Vec3 oldAcc = fbAccum[idx];
                        double s = (double)sampleIndex;
                        Vec3 newAcc = (oldAcc * s + color) / (s + 1.0);
                        fbAccum[idx] = newAcc;

                        // Convert to 8-bit
                        int ir = (int)(255.99 * clamp01(newAcc.x));
                        int ig = (int)(255.99 * clamp01(newAcc.y));
                        int ib = (int)(255.99 * clamp01(newAcc.z));

                        if (ir < 0) ir = 0; if (ir > 255) ir = 255;
                        if (ig < 0) ig = 0; if (ig > 255) ig = 255;
                        if (ib < 0) ib = 0; if (ib > 255) ib = 255;

                        framebuffer[idx] =
                            (uint32_t)ib |
                            ((uint32_t)ig << 8) |
                            ((uint32_t)ir << 16);
                    }
                }
            });
    }

    for (auto& th : threads) th.join();
}

// ----------------------------------------------------------
// Update camera based on keyboard input
// ----------------------------------------------------------
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

    // Clamp pitch
    const double maxPitch = 1.5; // ~86 degrees
    if (cam.pitch > maxPitch) cam.pitch = maxPitch;
    if (cam.pitch < -maxPitch) cam.pitch = -maxPitch;
}

// ----------------------------------------------------------
// Entry point
// ----------------------------------------------------------
int main() {
    const int LOW_W = 200;
    const int LOW_H = 150;
    const int HIGH_W = 400;
    const int HIGH_H = 300;

    const double targetFPS = 60.0;
    const double targetDelta = 1.0 / targetFPS;

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

    scene.light = Light(
        Vec3(2.0, 5.0, -2.0),
        Vec3(1.0, 1.0, 1.0),
        3.0
    );

    Vec3 top(0.0, 1.5, 3.5);
    Vec3 bl(-1.0, 0.0, 4.5);
    Vec3 br(1.0, 0.0, 4.5);
    Vec3 br2(1.0, 0.0, 2.5);
    Vec3 bl2(-1.0, 0.0, 2.5);

    Vec3 baseColor1(0.8, 0.2, 0.2);
    Vec3 baseColor2(0.8, 0.8, 0.2);
    Vec3 sideColor1(0.2, 0.8, 0.2);
    Vec3 sideColor2(0.2, 0.8, 0.8);
    Vec3 sideColor3(0.2, 0.2, 0.8);
    Vec3 sideColor4(0.8, 0.2, 0.8);

    scene.addTriangle(Triangle(bl, br, br2, baseColor1));
    scene.addTriangle(Triangle(bl, br2, bl2, baseColor2));
    scene.addTriangle(Triangle(top, bl, br, sideColor1));
    scene.addTriangle(Triangle(top, br, br2, sideColor2));
    scene.addTriangle(Triangle(top, br2, bl2, sideColor3));
    scene.addTriangle(Triangle(top, bl2, bl, sideColor4));

    // Ground
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
        Vec3(0.0, 1.0, -4.0),
        0.0,
        0.0,
        LOW_W, LOW_H,
        1.0,
        1.2
    );

    // --------------------------------------------------
    // Framebuffers
    // --------------------------------------------------
    std::vector<std::uint32_t> fbLow(LOW_W * LOW_H);
    std::vector<std::uint32_t> fbHigh(HIGH_W * HIGH_H);
    std::vector<Vec3>          fbHighAccum(HIGH_W * HIGH_H, Vec3(0.0, 0.0, 0.0));

    // --------------------------------------------------
    // Timing
    // --------------------------------------------------
    LARGE_INTEGER freq;
    QueryPerformanceFrequency(&freq);

    LARGE_INTEGER prevCounter;
    QueryPerformanceCounter(&prevCounter);

    bool running = true;
    bool highResMode = false;
    int  highResSamples = 0;
    bool rLastDown = false;

    std::cout << "Controls:\n"
        << "  W/S: move forward/back\n"
        << "  A/D: strafe left/right\n"
        << "  Q/E: move down/up\n"
        << "  Arrow Keys: rotate camera\n"
        << "  Hold R: progressive high-res render\n"
        << "  Release R: back to low-res interactive\n";

    while (running) {
        LARGE_INTEGER frameStart;
        QueryPerformanceCounter(&frameStart);

        if (!window.processMessages()) {
            running = false;
            break;
        }

        double dt = static_cast<double>(frameStart.QuadPart - prevCounter.QuadPart)
            / static_cast<double>(freq.QuadPart);
        prevCounter = frameStart;
        if (dt > 0.1) dt = 0.1;

        // Camera update only in low-res mode (you can change this if you want camera
        // to move while high-res is refining, but that will blur the final image).
        if (!highResMode) {
            updateCamera(cam, dt);
        }

        // R key: hold for high-res progressive
        bool rDown = (GetAsyncKeyState('R') & 0x8000) != 0;
        if (rDown && !rLastDown) {
            // R pressed: start high-res mode
            highResMode = true;
            highResSamples = 0;
            std::fill(fbHighAccum.begin(), fbHighAccum.end(), Vec3(0.0, 0.0, 0.0));
        }
        if (!rDown && rLastDown) {
            // R released: stop high-res mode
            highResMode = false;
        }
        rLastDown = rDown;

        if (highResMode) {
            // Add one sample per pixel each frame (multi-threaded)
            renderHighResSample(HIGH_W, HIGH_H, cam, scene, fbHighAccum, fbHigh.data(), highResSamples);
            highResSamples++;
            window.present(fbHigh.data(), HIGH_W, HIGH_H);
        }
        else {
            // Low-res interactive
            renderLowRes(LOW_W, LOW_H, cam, scene, fbLow.data());
            window.present(fbLow.data(), LOW_W, LOW_H);
        }

        LARGE_INTEGER frameEnd;
        QueryPerformanceCounter(&frameEnd);

        double frameTime = static_cast<double>(frameEnd.QuadPart - frameStart.QuadPart)
            / static_cast<double>(freq.QuadPart);
        double remaining = targetDelta - frameTime;
        if (remaining > 0.0) {
            DWORD sleepMs = static_cast<DWORD>(remaining * 1000.0);
            if (sleepMs > 0) {
                Sleep(sleepMs);
            }
        }
    }

    return 0;
}
