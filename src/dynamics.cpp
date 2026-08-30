#include "dynamics.h"
#include <algorithm>
#include <cmath>

namespace pe {

RigidBody makeBoxBody(const Vec3& he, Real mass) {
    RigidBody b;
    b.shape = makeBox(he);
    b.invMass = mass > 0 ? Real(1) / mass : 0;
    // Solid-box inertia tensor about the center of mass, in body axes.
    Real ix = mass / 3 * (he.y * he.y + he.z * he.z);
    Real iy = mass / 3 * (he.x * he.x + he.z * he.z);
    Real iz = mass / 3 * (he.x * he.x + he.y * he.y);
    Mat3 inertia = Mat3::diagonal(ix, iy, iz);
    b.invInertiaLocal = mass > 0 ? inertia.inverse() : Mat3::zero();
    b.updateInertiaWorld();
    return b;
}

RigidBody makeStaticBox(const Vec3& he) {
    RigidBody b;
    b.shape = makeBox(he);
    b.invMass = 0;
    b.invInertiaLocal = Mat3::zero();
    b.color = {0.35f, 0.35f, 0.4f};
    return b;
}

World::World() : broadphase_(Real(2.5)) {}

int World::addBody(const RigidBody& b) {
    bodies_.push_back(b);
    aabbs_.push_back(computeAABB(b.shape, b.pose()));
    return (int)bodies_.size() - 1;
}

void World::integrateVelocities(Real dt) {
    for (auto& b : bodies_) {
        if (b.isStatic()) continue;
        b.linearVelocity += gravity * dt;
        b.updateInertiaWorld();
    }
}

void World::integratePositions(Real dt) {
    for (auto& b : bodies_) {
        if (b.isStatic()) continue;
        b.position += b.linearVelocity * dt;
        b.orientation = integrateOrientation(b.orientation, b.angularVelocity, dt);
    }
    for (int i = 0; i < (int)bodies_.size(); ++i)
        aabbs_[i] = computeAABB(bodies_[i].shape, bodies_[i].pose());
}

void World::detectContacts() {
    // AABBs padded slightly so the broadphase catches contacts about to form.
    std::vector<AABB> padded = aabbs_;
    const Real pad = Real(0.05);
    for (auto& box : padded) { box.min -= Vec3{pad, pad, pad}; box.max += Vec3{pad, pad, pad}; }
    broadphase_.rebuild(padded);
    lastPairCount = (int)broadphase_.pairs().size();

    contacts_.clear();
    for (auto& pr : broadphase_.pairs()) {
        RigidBody& A = bodies_[pr.a];
        RigidBody& B = bodies_[pr.b];
        if (A.isStatic() && B.isStatic()) continue;
        Manifold m = collide(A.shape, A.pose(), B.shape, B.pose());
        if (m.colliding && m.count > 0) contacts_.push_back({pr.a, pr.b, m});
    }
    lastContactCount = (int)contacts_.size();
}

void World::prepareConstraints(Real dt) {
    constraints_.clear();
    constraints_.reserve(contacts_.size());

    for (auto& c : contacts_) {
        RigidBody& A = bodies_[c.a];
        RigidBody& B = bodies_[c.b];
        Constraint con;
        con.a = c.a; con.b = c.b;
        con.normal = c.manifold.normal;
        con.count = c.manifold.count;

        // Build an orthonormal tangent basis once per manifold.
        Vec3 t1 = std::fabs(con.normal.x) < Real(0.9) ? cross(con.normal, {1, 0, 0})
                                                        : cross(con.normal, {0, 1, 0});
        t1 = normalize(t1);
        Vec3 t2 = cross(con.normal, t1);

        for (int i = 0; i < con.count; ++i) {
            const ContactPoint& cp = c.manifold.points[i];
            ConstraintPoint& p = con.pts[i];
            p.rA = cp.position - A.position;
            p.rB = cp.position - B.position;
            p.normal = con.normal;
            p.tangent[0] = t1; p.tangent[1] = t2;
            p.key = (uint64_t(c.a) << 48) ^ (uint64_t(c.b) << 32) ^ cp.featureId;

            Real knFull = A.invMass + B.invMass +
                          dot(A.invInertiaWorld * cross(p.rA, con.normal), cross(p.rA, con.normal)) +
                          dot(B.invInertiaWorld * cross(p.rB, con.normal), cross(p.rB, con.normal));
            p.normalMass = knFull > 0 ? Real(1) / knFull : 0;
            for (int t = 0; t < 2; ++t) {
                Real kt = A.invMass + B.invMass +
                          dot(A.invInertiaWorld * cross(p.rA, p.tangent[t]), cross(p.rA, p.tangent[t])) +
                          dot(B.invInertiaWorld * cross(p.rB, p.tangent[t]), cross(p.rB, p.tangent[t]));
                p.tangentMass[t] = kt > 0 ? Real(1) / kt : 0;
            }

            // Baumgarte position bias + restitution bias, combined additively.
            Real correction = std::max(cp.penetration - penetrationSlop, Real(0));
            p.bias = baumgarte / dt * correction;

            Vec3 relVel = (B.linearVelocity + cross(B.angularVelocity, p.rB)) -
                          (A.linearVelocity + cross(A.angularVelocity, p.rA));
            Real closingSpeed = dot(relVel, con.normal);
            Real rest = std::max(A.restitution, B.restitution);
            if (closingSpeed < -restitutionThreshold) p.bias += -rest * closingSpeed;

            // Warm start from the previous frame's impulse on the same feature.
            auto it = impulseCache_.find(p.key);
            if (it != impulseCache_.end()) {
                p.normalImpulse = it->second.normalImpulse;
                p.tangentImpulse[0] = it->second.tangentImpulse[0];
                p.tangentImpulse[1] = it->second.tangentImpulse[1];
            }
        }
        constraints_.push_back(con);
    }
}

void World::warmStart() {
    for (auto& con : constraints_) {
        RigidBody& A = bodies_[con.a];
        RigidBody& B = bodies_[con.b];
        for (int i = 0; i < con.count; ++i) {
            ConstraintPoint& p = con.pts[i];
            Vec3 imp = p.normal * p.normalImpulse + p.tangent[0] * p.tangentImpulse[0] +
                      p.tangent[1] * p.tangentImpulse[1];
            if (!A.isStatic()) A.applyImpulse(-imp, p.rA);
            if (!B.isStatic()) B.applyImpulse(imp, p.rB);
        }
    }
}

void World::solveVelocity() {
    for (int iter = 0; iter < solverIterations; ++iter) {
        for (auto& con : constraints_) {
            RigidBody& A = bodies_[con.a];
            RigidBody& B = bodies_[con.b];
            for (int i = 0; i < con.count; ++i) {
                ConstraintPoint& p = con.pts[i];

                // Friction first against the previous normal impulse (standard
                // sequential-impulse ordering quirk is fine at these iteration counts).
                Real mu = std::sqrt(A.friction * B.friction);
                for (int t = 0; t < 2; ++t) {
                    Vec3 relVel = (B.linearVelocity + cross(B.angularVelocity, p.rB)) -
                                  (A.linearVelocity + cross(A.angularVelocity, p.rA));
                    Real vt = dot(relVel, p.tangent[t]);
                    Real lambda = -vt * p.tangentMass[t];
                    Real maxF = mu * p.normalImpulse;
                    Real newImp = clampf(p.tangentImpulse[t] + lambda, -maxF, maxF);
                    lambda = newImp - p.tangentImpulse[t];
                    p.tangentImpulse[t] = newImp;
                    Vec3 imp = p.tangent[t] * lambda;
                    if (!A.isStatic()) A.applyImpulse(-imp, p.rA);
                    if (!B.isStatic()) B.applyImpulse(imp, p.rB);
                }

                Vec3 relVel = (B.linearVelocity + cross(B.angularVelocity, p.rB)) -
                              (A.linearVelocity + cross(A.angularVelocity, p.rA));
                Real vn = dot(relVel, p.normal);
                Real lambda = -p.normalMass * (vn - p.bias);
                Real newImp = std::max(p.normalImpulse + lambda, Real(0));
                lambda = newImp - p.normalImpulse;
                p.normalImpulse = newImp;
                Vec3 imp = p.normal * lambda;
                if (!A.isStatic()) A.applyImpulse(-imp, p.rA);
                if (!B.isStatic()) B.applyImpulse(imp, p.rB);
            }
        }
    }

    // Persist impulses for next frame's warm start, keyed by contact feature.
    impulseCache_.clear();
    for (auto& con : constraints_)
        for (int i = 0; i < con.count; ++i) {
            CachedImpulse& c = impulseCache_[con.pts[i].key];
            c.normalImpulse = con.pts[i].normalImpulse;
            c.tangentImpulse[0] = con.pts[i].tangentImpulse[0];
            c.tangentImpulse[1] = con.pts[i].tangentImpulse[1];
        }
}

void World::step(Real dt) {
    integrateVelocities(dt);
    detectContacts();
    prepareConstraints(dt);
    warmStart();
    solveVelocity();
    integratePositions(dt);
}

} // namespace pe
