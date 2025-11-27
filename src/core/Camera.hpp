#pragma once

#include <cmath>
#include "math/Vec3.hpp"
#include "core/Ray.hpp"

struct Camera {
    Vec3 position;   // camera position in world space
    double yaw;      // around Y axis (in radians)
    double pitch;    // around X axis (in radians)

    double viewPlaneDist;   // distance from camera to view plane (e.g. 1.0)
    double viewPlaneWidth;  // physical width of view plane (e.g. 0.5)
    double viewPlaneHeight; // determined by aspect ratio

    int imageWidth;
    int imageHeight;

    Camera(
        const Vec3& pos,
        double yawRadians,
        double pitchRadians,
        int imgW,
        int imgH,
        double planeDist = 1.0,
        double planeWidth = 0.5
    )
        : position(pos)
        , yaw(yawRadians)
        , pitch(pitchRadians)
        , viewPlaneDist(planeDist)
        , viewPlaneWidth(planeWidth)
        , viewPlaneHeight(0.0)
        , imageWidth(imgW)
        , imageHeight(imgH)
    {
        setImageSize(imgW, imgH);
    }

    // Update the image size and recompute the view plane height.
    void setImageSize(int w, int h) {
        imageWidth = w;
        imageHeight = h;

        double aspect = static_cast<double>(imageHeight) / static_cast<double>(imageWidth);
        viewPlaneHeight = viewPlaneWidth * aspect;
    }

    // Rotate a vector around the Y axis (yaw)
    Vec3 rotateY(const Vec3& v, double angle) const {
        double c = std::cos(angle);
        double s = std::sin(angle);
        return Vec3(
            v.x * c + v.z * s,
            v.y,
            -v.x * s + v.z * c
        );
    }

    // Rotate a vector around the X axis (pitch)
    Vec3 rotateX(const Vec3& v, double angle) const {
        double c = std::cos(angle);
        double s = std::sin(angle);
        return Vec3(
            v.x,
            v.y * c - v.z * s,
            v.y * s + v.z * c
        );
    }

    // ----------------------------
    // Default: center-of-pixel ray
    // ----------------------------
    Ray generateRay(int px, int py) const {
        return generateRayJittered(px, py, 0.0, 0.0);
    }

    // ------------------------------------------------------
    // Jittered ray: sub-pixel sampling
    //
    // px, py: integer pixel indices
    // jx, jy: jitter in range [-0.5, 0.5] for anti-aliasing
    //
    // Effective sample position:
    //   (px + 0.5 + jx, py + 0.5 + jy)
    // ------------------------------------------------------
    Ray generateRayJittered(int px, int py, double jx, double jy) const {
        double u = (static_cast<double>(px) + 0.5 + jx) / static_cast<double>(imageWidth);
        double v = (static_cast<double>(py) + 0.5 + jy) / static_cast<double>(imageHeight);

        double x = (u - 0.5) * viewPlaneWidth;
        double y = (v - 0.5) * viewPlaneHeight;
        double z = viewPlaneDist;  // forward

        Vec3 dir(x, -y, z); // -y so that top of image is positive up

        dir = rotateY(dir, yaw);
        dir = rotateX(dir, pitch);

        dir = dir.normalized();
        return Ray(position, dir);
    }
};
