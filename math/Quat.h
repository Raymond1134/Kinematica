#pragma once

#include "Vec3.h"
#include <cmath>

struct Quat {
    static Quat lookRotation(const Vec3& forward, const Vec3& up) {
        Vec3 f = forward;
        float fLen = sqrtf(f.x*f.x + f.y*f.y + f.z*f.z);
        if (fLen < 1e-6f) return Quat();
        f = f / fLen;
        Vec3 u = up;
        float uLen = sqrtf(u.x*u.x + u.y*u.y + u.z*u.z);
        if (uLen < 1e-6f) u = {0,1,0};
        u = u / uLen;
        Vec3 r = Vec3::cross(u, f);
        float rLen = sqrtf(r.x*r.x + r.y*r.y + r.z*r.z);
        if (rLen < 1e-6f) r = {1,0,0};
        r = r / rLen;
        u = Vec3::cross(f, r);
        float m00 = r.x, m01 = r.y, m02 = r.z;
        float m10 = u.x, m11 = u.y, m12 = u.z;
        float m20 = f.x, m21 = f.y, m22 = f.z;
        float t = m00 + m11 + m22;
        Quat q;
        if (t > 0.0f) {
            float s = sqrtf(t + 1.0f) * 2.0f;
            q.w = 0.25f * s;
            q.x = (m21 - m12) / s;
            q.y = (m02 - m20) / s;
            q.z = (m10 - m01) / s;
        } else if ((m00 > m11) && (m00 > m22)) {
            float s = sqrtf(1.0f + m00 - m11 - m22) * 2.0f;
            q.w = (m21 - m12) / s;
            q.x = 0.25f * s;
            q.y = (m01 + m10) / s;
            q.z = (m02 + m20) / s;
        } else if (m11 > m22) {
            float s = sqrtf(1.0f + m11 - m00 - m22) * 2.0f;
            q.w = (m02 - m20) / s;
            q.x = (m01 + m10) / s;
            q.y = 0.25f * s;
            q.z = (m12 + m21) / s;
        } else {
            float s = sqrtf(1.0f + m22 - m00 - m11) * 2.0f;
            q.w = (m10 - m01) / s;
            q.x = (m02 + m20) / s;
            q.y = (m12 + m21) / s;
            q.z = 0.25f * s;
        }
        return q.normalized();
    }

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