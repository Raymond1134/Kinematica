#pragma once
#include "Vec3.h"

struct Mat3 {
    Vec3 cols[3];

    Mat3() {
        cols[0] = {0, 0, 0};
        cols[1] = {0, 0, 0};
        cols[2] = {0, 0, 0};
    }

    Mat3(const Vec3& c0, const Vec3& c1, const Vec3& c2) {
        cols[0] = c0;
        cols[1] = c1;
        cols[2] = c2;
    }

    static Mat3 identity() {
        return Mat3(
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1}
        );
    }

    static Mat3 zero() {
        return Mat3();
    }

    Vec3 operator*(const Vec3& v) const {
        return cols[0] * v.x + cols[1] * v.y + cols[2] * v.z;
    }

    Mat3 operator+(const Mat3& other) const {
        return Mat3(cols[0] + other.cols[0], cols[1] + other.cols[1], cols[2] + other.cols[2]);
    }

    Mat3 operator-(const Mat3& other) const {
        return Mat3(cols[0] - other.cols[0], cols[1] - other.cols[1], cols[2] - other.cols[2]);
    }

    Mat3& operator+=(const Mat3& other) {
        cols[0] += other.cols[0];
        cols[1] += other.cols[1];
        cols[2] += other.cols[2];
        return *this;
    }

    Mat3 inverse() const {
        Vec3 r0 = {cols[0].x, cols[1].x, cols[2].x};
        Vec3 r1 = {cols[0].y, cols[1].y, cols[2].y};
        Vec3 r2 = {cols[0].z, cols[1].z, cols[2].z};

        Vec3 c0 = Vec3::cross(r1, r2);
        Vec3 c1 = Vec3::cross(r2, r0);
        Vec3 c2 = Vec3::cross(r0, r1);

        float det = Vec3::dot(r0, c0);
        if (std::abs(det) < 1e-6f) return Mat3::zero();

        float invDet = 1.0f / det;
        return Mat3(c0 * invDet, c1 * invDet, c2 * invDet).transpose();
    }

    Mat3 transpose() const {
        return Mat3(
            {cols[0].x, cols[1].x, cols[2].x},
            {cols[0].y, cols[1].y, cols[2].y},
            {cols[0].z, cols[1].z, cols[2].z}
        );
    }
};
