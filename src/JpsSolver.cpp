#include "JpsSolver.h"

#include <algorithm>
#include <climits>
#include <cmath>

namespace pathfinder {

// ============================================================================
// QuaternaryHeap (4-ary min-heap) — same shape as AStarSolver's
// ============================================================================

void JpsSolver::Heap::push(OpenEntry e) noexcept {
    data_.push_back(e);
    size_t i = data_.size() - 1;
    while (i > 0) {
        const size_t parent = (i - 1) >> 2;
        if (data_[parent].f <= data_[i].f) break;
        std::swap(data_[parent], data_[i]);
        i = parent;
    }
}

JpsSolver::OpenEntry JpsSolver::Heap::pop() noexcept {
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
// Helpers
// ============================================================================

namespace {

/// Octile distance for the XZ plane.
[[gnu::always_inline]] inline float octile(int32_t dx, int32_t dz) noexcept {
    const int32_t adx = std::abs(dx);
    const int32_t adz = std::abs(dz);
    const int32_t lo  = std::min(adx, adz);
    const int32_t hi  = std::max(adx, adz);
    return static_cast<float>(lo) * 1.41421356f + static_cast<float>(hi - lo);
}

} // namespace

// ============================================================================
// JpsSolver
// ============================================================================

JpsSolver::JpsSolver() {
    nodes_.reserve(1024);
    open_.reserve(1024);
    coordToNode_.reserve(1024);
}

void JpsSolver::resetState() noexcept {
    nodes_.clear();
    open_.clear();
    coordToNode_.clear();
}

// ----------------------------------------------------------------------------
// jump() — recursive walk that returns the next jump point in (dx, dz).
//
// 2D-JPS rules adapted to 2.5D:
//   - "walkable" at (x, z) = entity bbox fits at any Y within step-up/fall of curY
//   - cardinal direction's forced neighbour: blocked-shoulder + reachable-detour
//   - diagonal direction: also kicks off cardinal jumps from the new position
//
// Iterative for the same-direction continuation; recurses only when expanding
// the cardinal components of a diagonal step. Recursion depth is bounded by the
// XZ distance of the path (typically <100), well within stack limits.
// ----------------------------------------------------------------------------
bool JpsSolver::jump(int32_t curX, int32_t curY, int32_t curZ,
                     int32_t dx, int32_t dz,
                     int32_t &outX, int32_t &outY, int32_t &outZ) const noexcept
{
    const NavMesh &mesh = *ctx_.mesh;

    while (true) {
        // ---- Step in (dx, dz). --------------------------------------------------------------
        const int32_t nx = curX + dx;
        const int32_t nz = curZ + dz;
        const int32_t ny = mesh.resolveStandY(nx, curY, nz, ctx_.w, ctx_.h, ctx_.maxStepUp, ctx_.maxFall);
        if (ny == INT32_MIN) return false;

        // ---- Goal? -------------------------------------------------------------------------
        if (nx == ctx_.ex && ny == ctx_.ey && nz == ctx_.ez) {
            outX = nx; outY = ny; outZ = nz;
            return true;
        }

        // ---- Forced-neighbour detection. ---------------------------------------------------
        bool forced = false;
        if (dx != 0 && dz != 0) {
            // Diagonal: forced if a perpendicular cell behind us was blocked AND the cell
            // beyond it (on our same axis) is now reachable from (nx, nz).
            if (!walkable(curX, curY, nz) && walkable(nx, curY, nz)) forced = true;
            if (!walkable(nx, curY, curZ) && walkable(nx, curY, nz)) forced = true;
            // The two checks above cover the asymmetric forced cases; their ordering matches
            // the diagonal step's "left shoulder / right shoulder" intuition.
        } else if (dx != 0) {
            // Cardinal X. Forced if (nx, z±1) is reachable but (curX, z±1) was blocked.
            if (!walkable(curX, curY, nz - 1) && walkable(nx, curY, nz - 1)) forced = true;
            if (!walkable(curX, curY, nz + 1) && walkable(nx, curY, nz + 1)) forced = true;
        } else { // dz != 0
            if (!walkable(curX - 1, curY, curZ) && walkable(curX - 1, curY, nz)) forced = true;
            if (!walkable(curX + 1, curY, curZ) && walkable(curX + 1, curY, nz)) forced = true;
        }

        if (forced) {
            outX = nx; outY = ny; outZ = nz;
            return true;
        }

        // ---- Diagonal: recurse on cardinal axes from the new position. -----------------------
        if (dx != 0 && dz != 0) {
            int32_t tX, tY, tZ;
            if (jump(nx, ny, nz, dx, 0, tX, tY, tZ)) {
                outX = nx; outY = ny; outZ = nz;
                return true;
            }
            if (jump(nx, ny, nz, 0, dz, tX, tY, tZ)) {
                outX = nx; outY = ny; outZ = nz;
                return true;
            }
        }

        // ---- Continue same direction. -------------------------------------------------------
        curX = nx;
        curY = ny;
        curZ = nz;
    }
}

// ----------------------------------------------------------------------------
// findPath
// ----------------------------------------------------------------------------
bool JpsSolver::findPath(const NavMesh &mesh,
                         int32_t sx, int32_t sy, int32_t sz,
                         int32_t ex, int32_t ey, int32_t ez,
                         const AStarConfig &cfg,
                         std::vector<int32_t> &outPath) noexcept
{
    outPath.clear();
    resetState();
    lastIterations_  = 0;
    lastReachedGoal_ = false;

    const int32_t w         = std::max(1, cfg.entityWidth);
    const int32_t h         = std::max(1, cfg.entityHeight);
    const int32_t maxStepUp = std::max(0, cfg.maxStepUp);
    const int32_t maxFall   = std::max(0, cfg.maxFallDistance);

    if (!mesh.fitsBox(sx, sy, sz, w, h)) return false;
    if (!mesh.fitsBox(ex, ey, ez, w, h)) return false;

    if (sx == ex && sy == ey && sz == ez) {
        outPath.push_back(sx);
        outPath.push_back(sy);
        outPath.push_back(sz);
        lastReachedGoal_ = true;
        return true;
    }

    // Stash the per-query context so jump()/walkable() don't need 8-arg signatures.
    ctx_ = QueryCtx{&mesh, ex, ey, ez, w, h, maxStepUp, maxFall};

    const float h0 = octile(ex - sx, ez - sz);
    nodes_.push_back(Node{sx, sy, sz, 0.0f, h0, kNoParent, false});
    coordToNode_.emplace(packCoord(sx, sy, sz), 0u);
    open_.push(OpenEntry{h0, 0u});

    static constexpr int32_t kDX[8] = { 1, -1,  0,  0,  1,  1, -1, -1};
    static constexpr int32_t kDZ[8] = { 0,  0,  1, -1,  1, -1,  1, -1};

    int32_t iter = 0;

    while (!open_.empty()) {
        if (iter >= cfg.maxIterations) break;
        ++iter;

        const OpenEntry top = open_.pop();
        const uint32_t curIdx = top.nodeIdx;

        {
            Node &n = nodes_[curIdx];
            if (n.closed) continue;
            n.closed = true;
        }

        const int32_t curX = nodes_[curIdx].x;
        const int32_t curY = nodes_[curIdx].y;
        const int32_t curZ = nodes_[curIdx].z;
        const float   curG = nodes_[curIdx].g;

        // Goal reached?
        if (curX == ex && curY == ey && curZ == ez) {
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

        // ---- Determine search directions based on parent direction. ----
        // Start node (no parent): search all 8.
        // Otherwise: skip strictly-backward directions to avoid revisiting.
        int32_t pDx = 0, pDz = 0;
        if (nodes_[curIdx].parentIdx != kNoParent) {
            const Node &parent = nodes_[nodes_[curIdx].parentIdx];
            const int32_t dxRaw = curX - parent.x;
            const int32_t dzRaw = curZ - parent.z;
            pDx = (dxRaw == 0) ? 0 : (dxRaw > 0 ? 1 : -1);
            pDz = (dzRaw == 0) ? 0 : (dzRaw > 0 ? 1 : -1);
        }

        for (int d = 0; d < 8; ++d) {
            const int32_t dx = kDX[d];
            const int32_t dz = kDZ[d];

            // Skip directions that point strictly backward relative to parent.
            if ((pDx != 0 || pDz != 0) && dx == -pDx && dz == -pDz) continue;
            if (pDx != 0 && pDz == 0 && dx == -pDx && dz == 0)      continue;
            if (pDz != 0 && pDx == 0 && dx == 0 && dz == -pDz)      continue;

            int32_t jx, jy, jz;
            if (!jump(curX, curY, curZ, dx, dz, jx, jy, jz)) continue;

            // Add the jump point to the open set if it's better than what we have.
            const uint64_t k = packCoord(jx, jy, jz);
            const float    distXZ = octile(jx - curX, jz - curZ);
            const float    newG   = curG + distXZ;

            const auto it = coordToNode_.find(k);
            if (it != coordToNode_.end()) {
                Node &existing = nodes_[it->second];
                if (existing.closed) continue;
                if (newG < existing.g) {
                    existing.g         = newG;
                    existing.f         = newG + octile(ex - jx, ez - jz);
                    existing.parentIdx = curIdx;
                    open_.push(OpenEntry{existing.f, it->second});
                }
            } else {
                const uint32_t newIdx = static_cast<uint32_t>(nodes_.size());
                const float    fNew   = newG + octile(ex - jx, ez - jz);
                nodes_.push_back(Node{jx, jy, jz, newG, fNew, curIdx, false});
                coordToNode_.emplace(k, newIdx);
                open_.push(OpenEntry{fNew, newIdx});
            }
        }
    }

    lastIterations_ = iter;
    return false;
}

} // namespace pathfinder
