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

    // Generate a ray for pixel (px, py) in [0, imageWidth/Height)
    Ray generateRay(int px, int py) const {
        // Convert pixel coords to [0,1] then to [-0.5, +0.5] in view plane
        double u = (static_cast<double>(px) + 0.5) / static_cast<double>(imageWidth);
        double v = (static_cast<double>(py) + 0.5) / static_cast<double>(imageHeight);

        double x = (u - 0.5) * viewPlaneWidth;
        double y = (v - 0.5) * viewPlaneHeight;
        double z = viewPlaneDist;  // forward

        // Base direction before rotation
        Vec3 dir(x, -y, z); // -y so that top of image is positive up

        // Apply yaw then pitch
        dir = rotateY(dir, yaw);
        dir = rotateX(dir, pitch);

        dir = dir.normalized();
        return Ray(position, dir);
    }
};
