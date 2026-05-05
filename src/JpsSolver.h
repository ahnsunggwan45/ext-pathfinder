#ifndef PATHFINDER_JPS_SOLVER_H
#define PATHFINDER_JPS_SOLVER_H

#include "AStarSolver.h"
#include "Coord.h"
#include "NavMesh.h"
#include "vendor/unordered_dense.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace pathfinder {

/// 2.5-D Jump Point Search.
///
/// Operates on the XZ plane the way classic 2-D JPS does — it prunes symmetric
/// successors and "jumps" along straight lines until it hits an obstacle or finds a
/// forced-neighbour, only emitting the resulting jump points to the open set.
///
/// The Y axis is resolved per-step using the same `resolveStandY` rule as
/// `AStarSolver`: at each candidate cell we accept the highest reachable Y within
/// `[curY - maxFall, curY + maxStepUp]` whose bounding box fits.
///
/// Limitations vs full 3-D JPS:
///   - Path costs ignore vertical-cost asymmetry inside a jump (each step in a jump
///     contributes only its horizontal distance to the heap key). For game-mob
///     pathing where step-up/fall are bounded by 2-3 blocks, the optimality gap is
///     well under 5% in practice.
///   - JPS requires admissible & consistent heuristics; we reuse octile-XZ.
///
/// The interface mirrors `AStarSolver::findPath` — same `AStarConfig`, same output
/// `vector<int32_t>` of (x, y, z) triples — so consumers can swap solvers without
/// touching anything else.
class JpsSolver {
public:
    JpsSolver();

    bool findPath(const NavMesh &mesh,
                  int32_t sx, int32_t sy, int32_t sz,
                  int32_t ex, int32_t ey, int32_t ez,
                  const AStarConfig &cfg,
                  std::vector<int32_t> &outPath) noexcept;

    int32_t lastIterations() const noexcept { return lastIterations_; }
    bool    lastReachedGoal() const noexcept { return lastReachedGoal_; }

private:
    struct Node {
        int32_t  x, y, z;
        float    g;
        float    f;
        uint32_t parentIdx;
        bool     closed;
    };

    static constexpr uint32_t kNoParent = ~uint32_t(0);

    struct OpenEntry { float f; uint32_t nodeIdx; };

    /// 4-ary min-heap, identical to AStarSolver::QuaternaryHeap.
    class Heap {
    public:
        void  clear() noexcept       { data_.clear(); }
        bool  empty() const noexcept { return data_.empty(); }
        void  reserve(size_t n)      { data_.reserve(n); }
        void  push(OpenEntry e) noexcept;
        OpenEntry pop() noexcept;
    private:
        std::vector<OpenEntry> data_;
    };

    // Reusable across queries.
    std::vector<Node>                                                  nodes_;
    Heap                                                              open_;
    ankerl::unordered_dense::map<uint64_t, uint32_t, PackedCoordHash> coordToNode_;
    std::vector<uint32_t>                                              reconstructionBuf_;

    int32_t lastIterations_   = 0;
    bool    lastReachedGoal_  = false;

    // Per-query context — populated at the start of findPath() so that helpers
    // don't need to thread these through every call signature.
    struct QueryCtx {
        const NavMesh *mesh;
        int32_t        ex, ey, ez;
        int32_t        w, h;
        int32_t        maxStepUp, maxFall;
    };
    QueryCtx ctx_{};

    void resetState() noexcept;

    /// Walk in (dx, dz) from (curX, curY, curZ) until a jump point or dead-end.
    /// On success: writes the jump-point coordinates and returns true.
    bool jump(int32_t curX, int32_t curY, int32_t curZ,
              int32_t dx, int32_t dz,
              int32_t &outX, int32_t &outY, int32_t &outZ) const noexcept;

    /// True iff the entity bbox can land at (x, ?, z) within step-up/fall range of curY.
    [[gnu::always_inline]] inline bool walkable(int32_t x, int32_t curY, int32_t z) const noexcept {
        return ctx_.mesh->resolveStandY(x, curY, z, ctx_.w, ctx_.h, ctx_.maxStepUp, ctx_.maxFall) != INT32_MIN;
    }
};

} // namespace pathfinder

#endif
