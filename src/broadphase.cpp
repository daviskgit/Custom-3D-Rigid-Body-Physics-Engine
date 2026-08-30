#include "broadphase.h"
#include <algorithm>
#include <cmath>

namespace pe {

void SpatialHash::rebuild(const std::vector<AABB>& boxes) {
    grid_.clear();
    pairs_.clear();

    auto floorDiv = [this](Real v) { return (int)std::floor(v / cell_); };

    for (int i = 0; i < (int)boxes.size(); ++i) {
        const AABB& b = boxes[i];
        int x0 = floorDiv(b.min.x), x1 = floorDiv(b.max.x);
        int y0 = floorDiv(b.min.y), y1 = floorDiv(b.max.y);
        int z0 = floorDiv(b.min.z), z1 = floorDiv(b.max.z);
        for (int x = x0; x <= x1; ++x)
            for (int y = y0; y <= y1; ++y)
                for (int z = z0; z <= z1; ++z)
                    grid_[{x, y, z}].push_back(i);
    }

    // Emit candidate pairs from shared cells; dedupe (a body can share several
    // cells with another) with a sorted-unique pass.
    for (auto& kv : grid_) {
        auto& ids = kv.second;
        for (size_t a = 0; a < ids.size(); ++a)
            for (size_t b = a + 1; b < ids.size(); ++b) {
                int i = ids[a], j = ids[b];
                if (i > j) std::swap(i, j);
                if (boxes[i].overlaps(boxes[j])) pairs_.push_back({i, j});
            }
    }

    std::sort(pairs_.begin(), pairs_.end(), [](const BroadphasePair& p, const BroadphasePair& q) {
        return p.a != q.a ? p.a < q.a : p.b < q.b;
    });
    pairs_.erase(std::unique(pairs_.begin(), pairs_.end(),
                             [](const BroadphasePair& p, const BroadphasePair& q) {
                                 return p.a == q.a && p.b == q.b;
                             }),
                 pairs_.end());
}

} // namespace pe
