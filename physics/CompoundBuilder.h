#pragma once
#include "collision/Collider.h"
#include <vector>

class CompoundBuilder {
public:
    CompoundBuilder() = default;

    CompoundBuilder& AddBox(const Vec3& pos, const Vec3& size, const Quat& rot = Quat::identity()) {
        CompoundChild child;
        child.collider = Collider::createBox(size);
        child.localPosition = pos;
        child.localOrientation = rot;
        children.push_back(child);
        return *this;
    }

    CompoundBuilder& AddSphere(const Vec3& pos, float radius, const Quat& rot = Quat::identity()) {
        CompoundChild child;
        child.collider = Collider::createSphere(radius);
        child.localPosition = pos;
        child.localOrientation = rot;
        children.push_back(child);
        return *this;
    }

    CompoundBuilder& AddCapsule(const Vec3& pos, float radius, float height, const Quat& rot = Quat::identity()) {
        CompoundChild child;
        child.collider = Collider::createCapsule(radius, height);
        child.localPosition = pos;
        child.localOrientation = rot;
        children.push_back(child);
        return *this;
    }

    const std::vector<CompoundChild>& Build() const {
        return children;
    }

private:
    std::vector<CompoundChild> children;
};
