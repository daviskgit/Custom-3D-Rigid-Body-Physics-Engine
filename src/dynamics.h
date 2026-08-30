// dynamics.h - rigid bodies and the simulation world. The world owns a fixed-rate
// step: integrate -> broadphase -> narrowphase -> sequential-impulse solve.
#pragma once
#include <cstdint>
#include <unordered_map>
#include <vector>
#include "broadphase.h"
#include "collision.h"
#include "shapes.h"

namespace pe {

struct RigidBody {
    // State
    Vec3 position{0, 0, 0};
    Quat orientation;
    Vec3 linearVelocity{0, 0, 0};
    Vec3 angularVelocity{0, 0, 0};

    // Mass properties (inverse form so static bodies are just zeros)
    Real invMass = 0;
    Mat3 invInertiaLocal = Mat3::zero();
    Mat3 invInertiaWorld = Mat3::zero();

    // Material
    Real restitution = Real(0.1);
    Real friction = Real(0.5);

    ConvexShape shape;
    Vec3 color{0.8f, 0.8f, 0.8f};

    bool isStatic() const { return invMass == 0; }
    Pose pose() const { return Pose{position, orientation.toMat3()}; }
    void updateInertiaWorld() {
        Mat3 R = orientation.toMat3();
        invInertiaWorld = R * invInertiaLocal * R.transposed();
    }
    void applyImpulse(const Vec3& imp, const Vec3& r) {
        linearVelocity += imp * invMass;
        angularVelocity += invInertiaWorld * cross(r, imp);
    }
};

RigidBody makeBoxBody(const Vec3& halfExtents, Real mass);
RigidBody makeStaticBox(const Vec3& halfExtents);

class World {
public:
    World();

    int addBody(const RigidBody& b);
    RigidBody& body(int i) { return bodies_[i]; }
    const std::vector<RigidBody>& bodies() const { return bodies_; }
    int bodyCount() const { return (int)bodies_.size(); }

    void step(Real dt);

    Vec3 gravity{0, Real(-9.81), 0};
    int solverIterations = 10;
    Real baumgarte = Real(0.2);   // position-correction stiffness
    Real penetrationSlop = Real(0.005);
    Real restitutionThreshold = Real(1.0); // below this closing speed, no bounce

    // diagnostics
    int lastPairCount = 0;
    int lastContactCount = 0;

private:
    struct Contact {
        int a, b;
        Manifold manifold;
    };
    struct ConstraintPoint {
        Vec3 rA, rB;
        Vec3 normal;
        Vec3 tangent[2];
        Real normalMass = 0;
        Real tangentMass[2] = {0, 0};
        Real bias = 0;            // Baumgarte + restitution
        Real normalImpulse = 0;   // accumulated (warm-started)
        Real tangentImpulse[2] = {0, 0};
        uint64_t key = 0;
    };
    struct Constraint {
        int a, b;
        Vec3 normal;
        int count = 0;
        ConstraintPoint pts[4];
    };

    void integrateVelocities(Real dt);
    void integratePositions(Real dt);
    void detectContacts();
    void prepareConstraints(Real dt);
    void warmStart();
    void solveVelocity();

    struct CachedImpulse {
        Real normalImpulse = 0;
        Real tangentImpulse[2] = {0, 0};
    };

    std::vector<RigidBody> bodies_;
    std::vector<AABB> aabbs_;
    SpatialHash broadphase_;
    std::vector<Contact> contacts_;
    std::vector<Constraint> constraints_;
    std::unordered_map<uint64_t, CachedImpulse> impulseCache_; // per-feature warm start
};

} // namespace pe
