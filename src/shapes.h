// shapes.h - convex geometry described purely by a support function plus an
// explicit face list. GJK/EPA only need support(); the face list is used to
// clip a full contact manifold for box-like resting contact.
#pragma once
#include <vector>
#include "math.h"

namespace pe {

struct Face {
    Vec3 normal;              // outward, body space
    std::vector<int> indices; // vertex loop, CCW seen from outside
};

struct ConvexShape {
    std::vector<Vec3> verts;   // body space
    std::vector<Face> faces;
    Vec3 halfExtents{0, 0, 0}; // set for boxes; used for the analytic inertia tensor
    bool isBox = false;

    // Farthest vertex along dir (body space). Linear scan is fine for small hulls.
    Vec3 supportLocal(const Vec3& dir) const {
        int best = 0;
        Real bestDot = dot(verts[0], dir);
        for (int i = 1; i < (int)verts.size(); ++i) {
            Real d = dot(verts[i], dir);
            if (d > bestDot) { bestDot = d; best = i; }
        }
        return verts[best];
    }
};

inline ConvexShape makeBox(const Vec3& he) {
    ConvexShape s;
    s.isBox = true;
    s.halfExtents = he;
    s.verts = {
        {-he.x, -he.y, -he.z}, {he.x, -he.y, -he.z}, {he.x, he.y, -he.z}, {-he.x, he.y, -he.z},
        {-he.x, -he.y, he.z},  {he.x, -he.y, he.z},  {he.x, he.y, he.z},  {-he.x, he.y, he.z},
    };
    auto face = [](Vec3 n, int a, int b, int c, int d) {
        Face f; f.normal = n; f.indices = {a, b, c, d}; return f;
    };
    s.faces = {
        face({0, 0, -1}, 0, 3, 2, 1),
        face({0, 0, 1}, 4, 5, 6, 7),
        face({-1, 0, 0}, 0, 4, 7, 3),
        face({1, 0, 0}, 1, 2, 6, 5),
        face({0, -1, 0}, 0, 1, 5, 4),
        face({0, 1, 0}, 3, 7, 6, 2),
    };
    return s;
}

} // namespace pe
