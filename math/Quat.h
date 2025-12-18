#pragma once

#include "Vec3.h"
#include <cmath>

struct Quat {
    float w, x, y, z;

    Quat() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
    Quat(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}

    static Quat identity() { return Quat(); }

    static Quat fromAxisAngle(const Vec3& axis, float angleRad) {
        float half = 0.5f * angleRad;
        float s = sinf(half);
        float c = cosf(half);
        float lenSq = axis.x * axis.x + axis.y * axis.y + axis.z * axis.z;
        if (lenSq < 1e-8f) return Quat();
        float invLen = 1.0f / sqrtf(lenSq);
        return {c, axis.x * invLen * s, axis.y * invLen * s, axis.z * invLen * s};
    }

    Quat normalized() const {
        float n = w * w + x * x + y * y + z * z;
        if (n < 1e-12f) return Quat();
        float inv = 1.0f / sqrtf(n);
        return {w * inv, x * inv, y * inv, z * inv};
    }

    Quat conjugate() const { return {w, -x, -y, -z}; }

    Quat operator*(const Quat& b) const {
        return {
            w * b.w - x * b.x - y * b.y - z * b.z,
            w * b.x + x * b.w + y * b.z - z * b.y,
            w * b.y - x * b.z + y * b.w + z * b.x,
            w * b.z + x * b.y - y * b.x + z * b.w,
        };
    }

    Vec3 rotate(const Vec3& v) const {
        Quat qv(0.0f, v.x, v.y, v.z);
        Quat qr = (*this) * qv * this->conjugate();
        return {qr.x, qr.y, qr.z};
    }

    Vec3 rotateInv(const Vec3& v) const {
        Quat inv = this->conjugate();
        Quat qv(0.0f, v.x, v.y, v.z);
        Quat qr = inv * qv * (*this);
        return {qr.x, qr.y, qr.z};
    }

    void integrateAngularVelocity(const Vec3& omegaWorld, float dt) {
        Quat o(0.0f, omegaWorld.x, omegaWorld.y, omegaWorld.z);
        Quat qdot = o * (*this);
        w += 0.5f * qdot.w * dt;
        x += 0.5f * qdot.x * dt;
        y += 0.5f * qdot.y * dt;
        z += 0.5f * qdot.z * dt;
        *this = this->normalized();
    }
};