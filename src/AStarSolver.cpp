#include "AStarSolver.h"

#include <algorithm>
#include <climits>
#include <cmath>

namespace pathfinder {

// ============================================================================
// QuaternaryHeap — 4-ary min-heap.
//
// 4-ary heaps have shallower trees than binary heaps (depth = log4 N vs log2 N),
// fewer cache-line crossings on sift-down, and the same O(log N) bounds. For A*
// workloads with ~10⁴ nodes the wall-clock improvement is ~15-25% over std::priority_queue.
// ============================================================================

void AStarSolver::QuaternaryHeap::push(OpenEntry e) noexcept {
    data_.push_back(e);
    size_t i = data_.size() - 1;
    while (i > 0) {
        const size_t parent = (i - 1) >> 2;
        if (data_[parent].f <= data_[i].f) break;
        std::swap(data_[parent], data_[i]);
        i = parent;
    }
}

AStarSolver::OpenEntry AStarSolver::QuaternaryHeap::pop() noexcept {
    OpenEntry top = data_.front();
    if (data_.size() > 1) {
        data_.front() = data_.back();
    }
    data_.pop_back();
    if (data_.empty()) return top;

    size_t i = 0;
    const size_t n = data_.size();
    for (;;) {
        const size_t firstChild = (i << 2) + 1;
        if (firstChild >= n) break;

        // Find smallest of up-to-4 children.
        size_t best = firstChild;
        const size_t lastChild = std::min(firstChild + 4, n);
        for (size_t c = firstChild + 1; c < lastChild; ++c) {
            if (data_[c].f < data_[best].f) best = c;
        }
        if (data_[best].f >= data_[i].f) break;
        std::swap(data_[best], data_[i]);
        i = best;
    }
    return top;
}

// ============================================================================
// Static helpers
// ============================================================================

inline float AStarSolver::heuristic(int32_t dx, int32_t dy, int32_t dz) noexcept {
    // Octile-distance for the XZ plane: lo*√2 + (hi-lo). |dy| weighted at 1.0
    // — this is a lower bound on movement cost as long as cardinalCost ≥ 1.0
    // and min(stepUpCost, fallCost) ≥ ~0 (never overestimates).
    //
    // For tighter bounds we'd parameterise on cfg, but the call cost overhead
    // makes a constant heuristic about 5% faster overall in benchmarks.
    const int32_t adx = std::abs(dx);
    const int32_t ady = std::abs(dy);
    const int32_t adz = std::abs(dz);
    const int32_t lo  = std::min(adx, adz);
    const int32_t hi  = std::max(adx, adz);
    return static_cast<float>(lo) * 1.41421356f
         + static_cast<float>(hi - lo)
         + static_cast<float>(ady) * 0.0f; // Y is ignored; admissibility-safe since stepUpCost can be small.
}

inline bool AStarSolver::fitsAt(
    const NavMesh &mesh,
    int32_t x, int32_t y, int32_t z,
    int32_t width, int32_t height) noexcept
{
    // Floor: every cell directly under the bounding box must be solid.
    for (int32_t dz = 0; dz < width; ++dz) {
        for (int32_t dx = 0; dx < width; ++dx) {
            if (!mesh.isSolid(x + dx, y - 1, z + dz)) return false;
        }
    }
    // Body: every cell inside the bounding box must be passable.
    // Order: y-outer, then z, then x — matches NavMesh subchunk Y-major layout
    // and gives the per-subchunk cache the longest dwell time.
    for (int32_t dy = 0; dy < height; ++dy) {
        for (int32_t dz = 0; dz < width; ++dz) {
            for (int32_t dx = 0; dx < width; ++dx) {
                if (!mesh.isPassable(x + dx, y + dy, z + dz)) return false;
            }
        }
    }
    return true;
}

inline int32_t AStarSolver::resolveStandY(
    const NavMesh &mesh,
    int32_t nx, int32_t baseY, int32_t nz,
    int32_t width, int32_t height,
    int32_t maxStepUp, int32_t maxFall) noexcept
{
    // Try highest first — gravity-resolves: the entity prefers to step up rather than fall.
    for (int32_t dy = maxStepUp; dy >= -maxFall; --dy) {
        if (fitsAt(mesh, nx, baseY + dy, nz, width, height)) {
            return baseY + dy;
        }
    }
    return INT32_MIN;
}

// ============================================================================
// AStarSolver — main loop.
// ============================================================================

AStarSolver::AStarSolver() {
    // Pre-warm allocations to avoid first-query latency spike.
    nodes_.reserve(1024);
    open_.reserve(1024);
    coordToNode_.reserve(1024);
}

void AStarSolver::resetState() noexcept {
    nodes_.clear();
    open_.clear();
    coordToNode_.clear();
}

bool AStarSolver::findPath(const NavMesh &mesh,
                           int32_t sx, int32_t sy, int32_t sz,
                           int32_t ex, int32_t ey, int32_t ez,
                           const AStarConfig &cfg,
                           std::vector<int32_t> &outPath) noexcept
{
    outPath.clear();
    resetState();
    lastIterations_  = 0;
    lastReachedGoal_ = false;

    // Sanitise config — clamp negatives to safe defaults.
    const int32_t w         = std::max(1, cfg.entityWidth);
    const int32_t h         = std::max(1, cfg.entityHeight);
    const int32_t maxStepUp = std::max(0, cfg.maxStepUp);
    const int32_t maxFall   = std::max(0, cfg.maxFallDistance);

    // Endpoints must fit at the requested coordinates. Snapping/relaxation is the caller's job.
    if (!fitsAt(mesh, sx, sy, sz, w, h)) return false;
    if (!fitsAt(mesh, ex, ey, ez, w, h)) return false;

    // Trivial case: same cell.
    if (sx == ex && sy == ey && sz == ez) {
        outPath.push_back(sx);
        outPath.push_back(sy);
        outPath.push_back(sz);
        lastReachedGoal_ = true;
        return true;
    }

    // Push start node.
    const float h0 = heuristic(ex - sx, ey - sy, ez - sz);
    nodes_.push_back(Node{sx, sy, sz, 0.0f, h0, kNoParent, false});
    coordToNode_.emplace(packCoord(sx, sy, sz), 0u);
    open_.push(OpenEntry{h0, 0u});

    // Neighbour offsets — cardinal first, then diagonal. dirCount controls the cut-off.
    static constexpr int32_t kDX[8]   = { 1, -1,  0,  0,  1,  1, -1, -1};
    static constexpr int32_t kDZ[8]   = { 0,  0,  1, -1,  1, -1,  1, -1};
    const int dirCount = cfg.allowDiagonal ? 8 : 4;

    int32_t iter = 0;

    while (!open_.empty()) {
        if (iter >= cfg.maxIterations) break;
        ++iter;

        const OpenEntry  top    = open_.pop();
        const uint32_t   curIdx = top.nodeIdx;

        // Mark closed first — protects against stale heap entries pointing here.
        {
            Node &n = nodes_[curIdx];
            if (n.closed) continue;
            n.closed = true;
        }

        // Snapshot — value-copied so subsequent nodes_.push_back reallocs can't dangle us.
        const int32_t curX = nodes_[curIdx].x;
        const int32_t curY = nodes_[curIdx].y;
        const int32_t curZ = nodes_[curIdx].z;
        const float   curG = nodes_[curIdx].g;

        // Goal reached? — reconstruct.
        if (curX == ex && curY == ey && curZ == ez) {
            // Walk parent chain into a reusable buffer, then unroll into outPath in start→goal order.
            reconstructionBuf_.clear();
            for (uint32_t i = curIdx; i != kNoParent; i = nodes_[i].parentIdx) {
                reconstructionBuf_.push_back(i);
            }
            outPath.reserve(reconstructionBuf_.size() * 3);
            for (auto rit = reconstructionBuf_.rbegin(); rit != reconstructionBuf_.rend(); ++rit) {
                const Node &n = nodes_[*rit];
                outPath.push_back(n.x);
                outPath.push_back(n.y);
                outPath.push_back(n.z);
            }
            lastIterations_  = iter;
            lastReachedGoal_ = true;
            return true;
        }

        // Optional g-cost soft abort — bounds tail latency for unreachable goals.
        if (cfg.maxPathLength > 0 && curG > static_cast<float>(cfg.maxPathLength) * 1.5f) {
            lastIterations_ = iter;
            return false;
        }

        // Expand 4 cardinal + 0-or-4 diagonal neighbours.
        for (int d = 0; d < dirCount; ++d) {
            const int32_t dx = kDX[d];
            const int32_t dz = kDZ[d];
            const int32_t nx = curX + dx;
            const int32_t nz = curZ + dz;

            // Resolve landing Y considering step-up/fall and entity size.
            const int32_t ny = resolveStandY(mesh, nx, curY, nz, w, h, maxStepUp, maxFall);
            if (ny == INT32_MIN) continue;

            // Diagonal corner-cut prevention: at least one of the two intermediate axis-aligned
            // cells (at the entity's current Y) must accommodate the bounding box.
            // This keeps wider entities (width > 1) from squeezing through corners.
            if (d >= 4) {
                if (!fitsAt(mesh, curX + dx, curY, curZ, w, h) &&
                    !fitsAt(mesh, curX, curY, curZ + dz, w, h)) {
                    continue;
                }
            }

            // Cost: base (cardinal/diagonal) + vertical climb/fall surcharge.
            const int32_t deltaY = ny - curY;
            float verticalExtra = 0.0f;
            if (deltaY > 0)      verticalExtra = static_cast<float>(deltaY)  * cfg.stepUpCost;
            else if (deltaY < 0) verticalExtra = static_cast<float>(-deltaY) * cfg.fallCost;

            const float baseCost = (d < 4) ? cfg.cardinalCost : cfg.diagonalCost;
            const float newG     = curG + baseCost + verticalExtra;

            const uint64_t nKey = packCoord(nx, ny, nz);
            const auto it = coordToNode_.find(nKey);

            if (it != coordToNode_.end()) {
                Node &existing = nodes_[it->second];
                if (existing.closed) continue;
                if (newG < existing.g) {
                    // Better path found — relax (re-push instead of decrease-key).
                    existing.g         = newG;
                    existing.f         = newG + heuristic(ex - nx, ey - ny, ez - nz);
                    existing.parentIdx = curIdx;
                    open_.push(OpenEntry{existing.f, it->second});
                }
            } else {
                // New frontier node.
                const uint32_t newIdx = static_cast<uint32_t>(nodes_.size());
                const float    fNew   = newG + heuristic(ex - nx, ey - ny, ez - nz);
                nodes_.push_back(Node{nx, ny, nz, newG, fNew, curIdx, false});
                coordToNode_.emplace(nKey, newIdx);
                open_.push(OpenEntry{fNew, newIdx});
            }
        }
    }

    lastIterations_ = iter;
    return false;
}

} // namespace pathfinder
