#include "collision.h"
#include <algorithm>
#include <vector>

namespace pe {

// ---------------------------------------------------------------------------
AABB computeAABB(const ConvexShape& s, const Pose& pose) {
    AABB box;
    Vec3 p0 = pose.toWorld(s.verts[0]);
    box.min = box.max = p0;
    for (size_t i = 1; i < s.verts.size(); ++i) {
        Vec3 w = pose.toWorld(s.verts[i]);
        box.min.x = std::min(box.min.x, w.x); box.max.x = std::max(box.max.x, w.x);
        box.min.y = std::min(box.min.y, w.y); box.max.y = std::max(box.max.y, w.y);
        box.min.z = std::min(box.min.z, w.z); box.max.z = std::max(box.max.z, w.z);
    }
    return box;
}

// ---------------------------------------------------------------------------
// Minkowski-difference support point with witnesses on each body.
namespace {

struct SP {
    Vec3 v; // support of A-B
    Vec3 a; // witness on A (world)
};

SP support(const ConvexShape& A, const Pose& pa,
           const ConvexShape& B, const Pose& pb, const Vec3& dir) {
    Vec3 la = pa.R.transposed() * dir;
    Vec3 lb = pb.R.transposed() * (-dir);
    Vec3 wa = pa.toWorld(A.supportLocal(la));
    Vec3 wb = pb.toWorld(B.supportLocal(lb));
    return {wa - wb, wa};
}

// ---- GJK ------------------------------------------------------------------
// Returns true if the origin is contained in the Minkowski difference.
// On overlap, `simplex` holds the enclosing tetrahedron for EPA.
bool gjk(const ConvexShape& A, const Pose& pa, const ConvexShape& B, const Pose& pb,
         std::vector<SP>& simplex) {
    Vec3 dir{1, 0, 0};
    simplex.clear();
    simplex.push_back(support(A, pa, B, pb, dir));
    dir = -simplex[0].v;

    for (int iter = 0; iter < 64; ++iter) {
        if (lengthSq(dir) < Real(1e-14)) return true; // origin on simplex
        SP p = support(A, pa, B, pb, dir);
        if (dot(p.v, dir) < 0) return false;          // no overlap along dir
        simplex.push_back(p);

        // ---- doSimplex: reduce and pick next search direction --------------
        auto& s = simplex;
        if (s.size() == 2) {
            Vec3 a = s[1].v, b = s[0].v;
            Vec3 ab = b - a, ao = -a;
            if (dot(ab, ao) > 0) dir = cross(cross(ab, ao), ab);
            else { s = {s[1]}; dir = ao; }
        } else if (s.size() == 3) {
            Vec3 a = s[2].v, b = s[1].v, c = s[0].v;
            Vec3 ab = b - a, ac = c - a, ao = -a;
            Vec3 abc = cross(ab, ac);
            if (dot(cross(abc, ac), ao) > 0) {
                if (dot(ac, ao) > 0) { s = {s[0], s[2]}; dir = cross(cross(ac, ao), ac); }
                else { s = {s[1], s[2]}; Vec3 abv = b - a; if (dot(abv, ao) > 0) dir = cross(cross(abv, ao), abv); else { s = {s[2]}; dir = ao; } }
            } else if (dot(cross(ab, abc), ao) > 0) {
                s = {s[1], s[2]}; Vec3 abv = b - a;
                if (dot(abv, ao) > 0) dir = cross(cross(abv, ao), abv); else { s = {s[2]}; dir = ao; }
            } else {
                if (dot(abc, ao) > 0) dir = abc;
                else { s = {s[1], s[0], s[2]}; dir = -abc; }
            }
        } else { // tetrahedron
            Vec3 a = s[3].v, b = s[2].v, c = s[1].v, d = s[0].v;
            Vec3 ao = -a;
            Vec3 abc = cross(b - a, c - a);
            Vec3 acd = cross(c - a, d - a);
            Vec3 adb = cross(d - a, b - a);
            if (dot(abc, ao) > 0) { s = {s[1], s[2], s[3]}; dir = abc; }
            else if (dot(acd, ao) > 0) { s = {s[0], s[1], s[3]}; dir = acd; }
            else if (dot(adb, ao) > 0) { s = {s[2], s[0], s[3]}; dir = adb; }
            else return true; // origin inside tetrahedron
        }
    }
    return false;
}

// ---- EPA ---------------------------------------------------------------------
struct Tri { int a, b, c; Vec3 n; Real dist; };

Tri makeTri(const std::vector<SP>& v, int a, int b, int c) {
    Tri t{a, b, c, {}, 0};
    t.n = normalize(cross(v[b].v - v[a].v, v[c].v - v[a].v));
    t.dist = dot(t.n, v[a].v);
    if (t.dist < 0) { t.n = -t.n; t.dist = -t.dist; std::swap(t.b, t.c); }
    return t;
}

// Expands the GJK tetrahedron toward the surface of the Minkowski difference.
// Fills normal (unit) and depth; also a coarse contact point via barycentrics.
bool epa(const ConvexShape& A, const Pose& pa, const ConvexShape& B, const Pose& pb,
         std::vector<SP> verts, Vec3& outNormal, Real& outDepth, Vec3& outPointA) {
    if (verts.size() < 4) return false;
    std::vector<Tri> tris = {
        makeTri(verts, 0, 1, 2), makeTri(verts, 0, 2, 3),
        makeTri(verts, 0, 3, 1), makeTri(verts, 1, 3, 2),
    };

    Tri best{};
    for (int iter = 0; iter < 96; ++iter) {
        int bi = 0;
        for (int i = 1; i < (int)tris.size(); ++i)
            if (tris[i].dist < tris[bi].dist) bi = i;
        best = tris[bi];

        SP p = support(A, pa, B, pb, best.n);
        Real d = dot(p.v, best.n);
        if (d - best.dist < Real(1e-4)) break;

        int newIdx = (int)verts.size();
        verts.push_back(p);

        // Remove faces visible from the new point, record the horizon.
        std::vector<std::pair<int, int>> horizon;
        std::vector<Tri> keep;
        for (auto& t : tris) {
            if (dot(t.n, p.v - verts[t.a].v) > Real(1e-9)) {
                auto edge = [&](int i, int j) {
                    for (auto it = horizon.begin(); it != horizon.end(); ++it)
                        if (it->first == j && it->second == i) { horizon.erase(it); return; }
                    horizon.push_back({i, j});
                };
                edge(t.a, t.b); edge(t.b, t.c); edge(t.c, t.a);
            } else {
                keep.push_back(t);
            }
        }
        if (horizon.empty()) break;
        for (auto& e : horizon) keep.push_back(makeTri(verts, e.first, e.second, newIdx));
        tris.swap(keep);
        if (tris.size() > 256) break;
    }

    outNormal = best.n;
    outDepth = best.dist;

    // Barycentric projection of the origin onto the closest triangle for a
    // fallback contact point (used only when face clipping is unavailable).
    Vec3 pnt = best.n * best.dist;
    Vec3 A0 = verts[best.a].v, B0 = verts[best.b].v, C0 = verts[best.c].v;
    Vec3 v0 = B0 - A0, v1 = C0 - A0, v2 = pnt - A0;
    Real d00 = dot(v0, v0), d01 = dot(v0, v1), d11 = dot(v1, v1);
    Real d20 = dot(v2, v0), d21 = dot(v2, v1);
    Real den = d00 * d11 - d01 * d01;
    Real vb = den != 0 ? (d11 * d20 - d01 * d21) / den : 0;
    Real wb = den != 0 ? (d00 * d21 - d01 * d20) / den : 0;
    Real ub = 1 - vb - wb;
    outPointA = verts[best.a].a * ub + verts[best.b].a * vb + verts[best.c].a * wb;
    return true;
}

// ---- Sutherland-Hodgman clipping of the incident face -----------------------
struct WorldFace { std::vector<Vec3> v; Vec3 n; };

WorldFace faceToWorld(const ConvexShape& s, const Pose& pose, int fi) {
    WorldFace wf;
    wf.n = pose.R * s.faces[fi].normal;
    for (int idx : s.faces[fi].indices) wf.v.push_back(pose.toWorld(s.verts[idx]));
    return wf;
}

int bestFace(const ConvexShape& s, const Pose& pose, const Vec3& dir) {
    int best = 0; Real bd = -1e30f;
    for (int i = 0; i < (int)s.faces.size(); ++i) {
        Real d = dot(pose.R * s.faces[i].normal, dir);
        if (d > bd) { bd = d; best = i; }
    }
    return best;
}

} // namespace

// ---------------------------------------------------------------------------
Manifold collide(const ConvexShape& A, const Pose& poseA,
                 const ConvexShape& B, const Pose& poseB) {
    Manifold m;
    std::vector<SP> simplex;
    if (!gjk(A, poseA, B, poseB, simplex)) return m;

    Vec3 n; Real depth; Vec3 pointA;
    if (!epa(A, poseA, B, poseB, simplex, n, depth, pointA)) return m;

    // Orient the normal from A to B using the center offset (robust for stacks).
    if (dot(n, poseB.p - poseA.p) < 0) n = -n;
    m.colliding = true;
    m.normal = n;

    // Pick the reference face (most aligned with +/- n), clip the incident face.
    int fa = bestFace(A, poseA, n);
    int fb = bestFace(B, poseB, -n);
    Real sepA = dot(poseA.R * A.faces[fa].normal, n);
    Real sepB = dot(poseB.R * B.faces[fb].normal, -n);

    bool refIsA = sepA >= sepB;
    WorldFace ref = refIsA ? faceToWorld(A, poseA, fa) : faceToWorld(B, poseB, fb);
    Vec3 refN = refIsA ? ref.n : ref.n; // outward of the reference face
    const ConvexShape& incShape = refIsA ? B : A;
    const Pose& incPose = refIsA ? poseB : poseA;
    int incFi = bestFace(incShape, incPose, -refN);
    WorldFace inc = faceToWorld(incShape, incPose, incFi);

    // Clip the incident polygon against each side plane of the reference face.
    std::vector<Vec3> poly = inc.v;
    Vec3 centroid{0, 0, 0};
    for (auto& p : ref.v) centroid += p;
    centroid *= Real(1) / ref.v.size();

    for (size_t i = 0; i < ref.v.size(); ++i) {
        Vec3 a = ref.v[i], b = ref.v[(i + 1) % ref.v.size()];
        Vec3 planeN = normalize(cross(b - a, refN));
        Real planeD = dot(planeN, a);
        if (dot(planeN, centroid) - planeD > 0) { planeN = -planeN; planeD = -planeD; }

        std::vector<Vec3> out;
        for (size_t k = 0; k < poly.size(); ++k) {
            Vec3 cur = poly[k], nxt = poly[(k + 1) % poly.size()];
            Real dc = dot(planeN, cur) - planeD;
            Real dn = dot(planeN, nxt) - planeD;
            if (dc <= 0) out.push_back(cur);
            if ((dc < 0) != (dn < 0)) {
                Real t = dc / (dc - dn);
                out.push_back(cur + (nxt - cur) * t);
            }
        }
        poly.swap(out);
        if (poly.empty()) break;
    }

    Vec3 refV0 = ref.v[0];
    struct Cand { Vec3 pos; Real pen; };
    std::vector<Cand> cands;
    for (auto& x : poly) {
        Real sd = dot(refN, x - refV0); // <0 means below the reference face
        if (sd <= Real(0.01)) {
            Vec3 onFace = x - refN * sd;
            cands.push_back({(x + onFace) * Real(0.5), -sd});
        }
    }

    if (cands.empty()) {
        // Fallback: single EPA point (e.g. edge-edge contact).
        m.count = 1;
        m.points[0].position = pointA - n * (depth * Real(0.5));
        m.points[0].penetration = depth;
        m.points[0].featureId = 1;
        return m;
    }

    std::sort(cands.begin(), cands.end(),
              [](const Cand& a, const Cand& b) { return a.pen > b.pen; });
    int cnt = std::min<int>(4, (int)cands.size());
    m.count = cnt;
    for (int i = 0; i < cnt; ++i) {
        m.points[i].position = cands[i].pos;
        m.points[i].penetration = cands[i].pen;
        m.points[i].featureId =
            (uint64_t(fa) << 40) ^ (uint64_t(incFi) << 20) ^ uint64_t(i + 1);
    }
    return m;
}

} // namespace pe
