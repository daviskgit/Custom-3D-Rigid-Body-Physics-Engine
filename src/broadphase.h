// broadphase.h - uniform-grid spatial hash. Each body's AABB is rasterised into
// grid cells; only bodies sharing a cell are tested, turning the O(N^2) pair
// sweep into roughly O(N) for scenes with bounded local density.
#pragma once
#include <cstdint>
#include <unordered_map>
#include <vector>
#include "collision.h"

namespace pe {

struct BroadphasePair { int a, b; };

class SpatialHash {
public:
    explicit SpatialHash(Real cellSize = Real(2)) : cell_(cellSize) {}

    void rebuild(const std::vector<AABB>& boxes);
    const std::vector<BroadphasePair>& pairs() const { return pairs_; }

private:
    struct CellKey {
        int x, y, z;
        bool operator==(const CellKey& o) const { return x == o.x && y == o.y && z == o.z; }
    };
    struct CellHash {
        size_t operator()(const CellKey& k) const {
            // three large primes, xor-folded
            return (size_t)(k.x * 73856093) ^ (size_t)(k.y * 19349663) ^ (size_t)(k.z * 83492791);
        }
    };

    Real cell_;
    std::unordered_map<CellKey, std::vector<int>, CellHash> grid_;
    std::vector<BroadphasePair> pairs_;
};

} // namespace pe
