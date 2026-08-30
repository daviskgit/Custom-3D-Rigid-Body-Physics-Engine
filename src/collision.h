// collision.h - narrowphase: GJK boolean + closest info, EPA penetration depth,
// and face clipping to turn the EPA normal into a multi-point contact manifold.
#pragma once
#include "math.h"
#include "shapes.h"

namespace pe {

// Axis-aligned bounding box in world space (broadphase currency).
struct AABB {
    Vec3 min{0, 0, 0}, max{0, 0, 0};
    bool overlaps(const AABB& b) const {
        return min.x <= b.max.x && max.x >= b.min.x &&
               min.y <= b.max.y && max.y >= b.min.y &&
               min.z <= b.max.z && max.z >= b.min.z;
    }
};

// Pose of a collider in the world: p + R * local.
struct Pose {
    Vec3 p{0, 0, 0};
    Mat3 R;
    Vec3 toWorld(const Vec3& local) const { return p + R * local; }
    Vec3 toLocal(const Vec3& world) const { return R.transposed() * (world - p); }
};

AABB computeAABB(const ConvexShape& s, const Pose& pose);

struct ContactPoint {
    Vec3 position{0, 0, 0}; // world, midway between the surfaces
    Real penetration = 0;   // >0 means overlapping
    uint64_t featureId = 0; // stable-ish id for warm starting
    // solver scratch (persisted across frames for warm starting)
    Real normalImpulse = 0;
    Real tangentImpulse[2] = {0, 0};
};

struct Manifold {
    bool colliding = false;
    Vec3 normal{0, 1, 0}; // world, points from A to B
    int count = 0;
    ContactPoint points[4];
};

// Full narrowphase: GJK to detect overlap, EPA for the normal + depth,
// then reference/incident face clipping for the manifold.
Manifold collide(const ConvexShape& A, const Pose& poseA,
                 const ConvexShape& B, const Pose& poseB);

} // namespace pe
