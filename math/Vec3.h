#pragma once

#include <cmath>

struct Vec3 {
    float x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& other) const { return {x + other.x, y + other.y, z + other.z}; }
    Vec3 operator-(const Vec3& other) const { return {x - other.x, y - other.y, z - other.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(float scalar) const { return {x * scalar, y * scalar, z * scalar}; }

    Vec3 operator/(float scalar) const {
        float inv = (scalar != 0.0f) ? (1.0f / scalar) : 0.0f;
        return {x * inv, y * inv, z * inv};
    }

    Vec3& operator+=(const Vec3& other) {
        x += other.x; y += other.y; z += other.z;
        return *this;
    }

    Vec3& operator-=(const Vec3& other) {
        x -= other.x; y -= other.y; z -= other.z;
        return *this;
    }

    Vec3& operator*=(float scalar) {
        x *= scalar; y *= scalar; z *= scalar;
        return *this;
    }

    float lengthSq() const { return x * x + y * y + z * z; }

    float length() const { return std::sqrt(lengthSq()); }

    static float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

    static Vec3 cross(const Vec3& a, const Vec3& b) {
        return {
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x,
        };
    }

    Vec3 normalized(float eps = 1e-8f) const {
        float lsq = lengthSq();
        if (lsq <= eps * eps) return {0.0f, 0.0f, 0.0f};
        float inv = 1.0f / std::sqrt(lsq);
        return *this * inv;
    }
};

inline Vec3 operator*(float scalar, const Vec3& v) {
    return v * scalar;
}