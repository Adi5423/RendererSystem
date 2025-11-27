#pragma once

#include <cmath>
#include <iostream>

struct Vec3 {
    double x;
    double y;
    double z;

    // Constructors
    Vec3() : x(0.0), y(0.0), z(0.0) {}
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    // Basic arithmetic
    Vec3 operator+(const Vec3& other) const {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }

    Vec3 operator-(const Vec3& other) const {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }

    Vec3 operator-() const {
        return Vec3(-x, -y, -z);
    }

    Vec3 operator*(double s) const {
        return Vec3(x * s, y * s, z * s);
    }

    Vec3 operator/(double s) const {
        return Vec3(x / s, y / s, z / s);
    }

    Vec3& operator+=(const Vec3& other) {
        x += other.x; y += other.y; z += other.z;
        return *this;
    }

    Vec3& operator-=(const Vec3& other) {
        x -= other.x; y -= other.y; z -= other.z;
        return *this;
    }

    Vec3& operator*=(double s) {
        x *= s; y *= s; z *= s;
        return *this;
    }

    Vec3& operator/=(double s) {
        x /= s; y /= s; z /= s;
        return *this;
    }

    // Length and normalization
    double length() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    double lengthSquared() const {
        return x * x + y * y + z * z;
    }

    Vec3 normalized() const {
        double len = length();
        if (len == 0.0) return Vec3(0.0, 0.0, 0.0);
        return *this / len;
    }
};

// Scalar * Vec3
inline Vec3 operator*(double s, const Vec3& v) {
    return Vec3(v.x * s, v.y * s, v.z * s);
}

// Dot product
inline double dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

// Cross product
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return Vec3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

// Simple stream output for debugging
inline std::ostream& operator<<(std::ostream& os, const Vec3& v) {
    os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}
