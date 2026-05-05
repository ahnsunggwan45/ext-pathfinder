#ifndef PATHFINDER_NAVMESH_H
#define PATHFINDER_NAVMESH_H

#include "BlockTable.h"
#include "Coord.h"
#include "SubChunk.h"

#include <climits>  // INT32_MIN
#include <cmath>    // std::sqrt, std::floor (used by isLineWalkable)
#include <cstddef>
#include <cstdint>
#include <memory>
#include <unordered_map>

namespace pathfinder {

/// Stores walkability bitmaps for loaded subchunks plus the block-property table that
/// converts raw block-state-ids into passable/solid bits.
///
/// Hot-path queries (`isPassable`, `isSolid`) are inlined and use a 1-entry "last subchunk"
/// cache — A* repeatedly hits the same subchunk for adjacent neighbours, so a single-slot
/// cache cuts the unordered_map lookup from ~80% of hot-path time to <5%.
class NavMesh {
public:
    NavMesh() = default;

    NavMesh(const NavMesh &)            = delete;
    NavMesh &operator=(const NavMesh &) = delete;
    NavMesh(NavMesh &&)                 = default;
    NavMesh &operator=(NavMesh &&)      = default;

    // ----- Block-property table -----------------------------------------------------------
    BlockTable       &blockTable() noexcept       { return blockTable_; }
    const BlockTable &blockTable() const noexcept { return blockTable_; }

    // ----- Subchunk lifecycle -------------------------------------------------------------

    /// Replace the subchunk at (cx, cy, cz) with a freshly-decoded copy from `blockIds`.
    /// `blockIds` MUST point to 4096 consecutive uint32 values in `(y << 8) | (z << 4) | x` order.
    void loadSubChunk(int32_t cx, int32_t cy, int32_t cz, const uint32_t *blockIds);

    /// Load a subchunk directly from chunkutils2's PalettedBlockArray representation
    /// (word-array + palette + bitsPerBlock). Avoids the 4096 PHP→C++ method calls and
    /// `pack('V', ...)` allocations of the `loadSubChunk(packedString)` path.
    ///
    /// chunkutils2's word array is:
    ///   - native uint32 little-endian (matches x86_64 memory layout)
    ///   - palette indices packed LSB-first within each 32-bit word
    ///   - block ordering `(x << 8) | (z << 4) | y` (X-major, Y innermost)
    ///
    /// `bitsPerBlock == 0` is the uniform-subchunk shortcut: palette[0] fills everything.
    ///
    /// Throws via the caller's error path on malformed input — the binding layer is
    /// expected to translate to a PHP exception.
    bool loadSubChunkFromWordArray(int32_t cx, int32_t cy, int32_t cz,
                                   const uint8_t *wordArray, size_t wordArrayBytes,
                                   const uint32_t *palette, size_t paletteSize,
                                   int bitsPerBlock,
                                   const char **errOut) noexcept;

    /// Convenience: load a fully-passable air subchunk (skips the per-block decode).
    void loadAirSubChunk(int32_t cx, int32_t cy, int32_t cz);

    /// Convenience: load a fully-solid subchunk (e.g. unloaded bedrock, void floor).
    void loadSolidSubChunk(int32_t cx, int32_t cy, int32_t cz);

    /// Drop a subchunk. No-op if it wasn't loaded.
    void unloadSubChunk(int32_t cx, int32_t cy, int32_t cz);

    /// Drop every loaded subchunk in column (cx, cz).
    void unloadColumn(int32_t cx, int32_t cz);

    /// Drop every loaded subchunk.
    void clear();

    /// Patch a single block's properties using the current block-table.
    void updateBlock(int32_t x, int32_t y, int32_t z, uint32_t blockStateId);

    bool isLoaded(int32_t cx, int32_t cy, int32_t cz) const noexcept;
    size_t loadedSubChunkCount() const noexcept { return subChunks_.size(); }

    // ----- Hot-path queries (inlined) ------------------------------------------------------

    /// True if a mob's body can occupy (x, y, z). Unloaded subchunks default to passable.
    [[gnu::always_inline]] inline bool isPassable(int32_t x, int32_t y, int32_t z) const noexcept {
        const SubChunk *sub = subChunkAt(x, y, z);
        return sub ? sub->isPassable(x, y, z) : true;
    }

    /// True if a mob can stand on top of the block at (x, y, z). Unloaded subchunks default to non-solid.
    [[gnu::always_inline]] inline bool isSolid(int32_t x, int32_t y, int32_t z) const noexcept {
        const SubChunk *sub = subChunkAt(x, y, z);
        return sub ? sub->isSolid(x, y, z) : false;
    }

    /// True if a mob can stand at (x, y, z) — body cell `y` is passable AND block at `y-1` is solid.
    /// (Per design: head clearance at `y+1` is **not** checked. The consumer's stepHeight handles it.)
    [[gnu::always_inline]] inline bool isStandable(int32_t x, int32_t y, int32_t z) const noexcept {
        return isPassable(x, y, z) && isSolid(x, y - 1, z);
    }

    /// True iff an entity bounding box of `(width × height × width)` rooted at (x, y, z) fits:
    /// every covered cell is passable and every floor cell at y-1 is solid.
    /// Shared between A* and JPS solvers.
    [[gnu::always_inline]] inline bool fitsBox(int32_t x, int32_t y, int32_t z,
                                                int32_t width, int32_t height) const noexcept {
        // Hot path: 99% of mobs are 1×{1,2} (slime/baby = 1×1, player-like = 1×2).
        // Branchless body-cell check; compiler CSEs the three subchunk lookups since they
        // hit the same column. Measured ~7% faster than the generic triple-nested loop.
        if (width == 1) {
            if (height == 1) {
                return isSolid(x, y - 1, z) && isPassable(x, y, z);
            }
            if (height == 2) {
                return isSolid(x, y - 1, z)
                    && isPassable(x, y,     z)
                    && isPassable(x, y + 1, z);
            }
            // Taller width=1 mobs: still avoid the X×Z loops.
            if (!isSolid(x, y - 1, z)) return false;
            for (int32_t dy = 0; dy < height; ++dy) {
                if (!isPassable(x, y + dy, z)) return false;
            }
            return true;
        }
        // General path: arbitrary (width × height × width) bounding box.
        for (int32_t dz = 0; dz < width; ++dz) {
            for (int32_t dx = 0; dx < width; ++dx) {
                if (!isSolid(x + dx, y - 1, z + dz)) return false;
            }
        }
        for (int32_t dy = 0; dy < height; ++dy) {
            for (int32_t dz = 0; dz < width; ++dz) {
                for (int32_t dx = 0; dx < width; ++dx) {
                    if (!isPassable(x + dx, y + dy, z + dz)) return false;
                }
            }
        }
        return true;
    }

    /// True iff the entity can walk in a straight line from (x1, y1, z1) to (x2, y2, z2)
    /// — every grid cell the segment passes through is `fitsBox`-able for the entity.
    ///
    /// Used by path smoothing to merge collinear waypoints. The check naturally breaks
    /// at jump-up / fall transitions because the line passes through cells that aren't
    /// standable mid-Y, so smoothed paths preserve discrete jump points.
    ///
    /// Sampling: 2 samples per block of distance (cell-rounded). Empirically catches
    /// every cell the segment touches without expensive 3-D Bresenham.
    [[gnu::always_inline]] inline bool isLineWalkable(
        int32_t x1, int32_t y1, int32_t z1,
        int32_t x2, int32_t y2, int32_t z2,
        int32_t width, int32_t height) const noexcept
    {
        const float dx = static_cast<float>(x2 - x1);
        const float dy = static_cast<float>(y2 - y1);
        const float dz = static_cast<float>(z2 - z1);
        const float distSq = dx * dx + dy * dy + dz * dz;
        if (distSq < 0.25f) return fitsBox(x1, y1, z1, width, height);

        const float dist = std::sqrt(distSq);
        const int   steps = static_cast<int>(dist * 2.0f) + 1; // 2 samples / block, min 1
        const float invSteps = 1.0f / static_cast<float>(steps);

        int32_t lastX = INT32_MIN, lastY = INT32_MIN, lastZ = INT32_MIN;
        for (int i = 0; i <= steps; ++i) {
            const float t = static_cast<float>(i) * invSteps;
            const int32_t x = static_cast<int32_t>(std::floor(static_cast<float>(x1) + dx * t + 0.5f));
            const int32_t y = static_cast<int32_t>(std::floor(static_cast<float>(y1) + dy * t + 0.5f));
            const int32_t z = static_cast<int32_t>(std::floor(static_cast<float>(z1) + dz * t + 0.5f));
            if (x == lastX && y == lastY && z == lastZ) continue;
            if (!fitsBox(x, y, z, width, height)) return false;
            lastX = x; lastY = y; lastZ = z;
        }
        return true;
    }

    /// Find the best Y in `[baseY - maxFall, baseY + maxStepUp]` at which the entity
    /// bbox fits at (nx, ?, nz). Returns INT32_MIN if no Y in the range works.
    ///
    /// Search order: flat (dy=0) → step-up (1, 2, …) → fall (-1, -2, …).
    /// Flat-first matches gravity-bound mob behaviour (don't climb if you don't have to)
    /// and short-circuits the common case in 1 fitsBox call instead of `maxStepUp + 1`.
    /// On flat-terrain benchmarks this cuts solver fitsBox cost ~6× — both A* and JPS
    /// see the same speedup since resolveStandY is the per-neighbour bottleneck.
    [[gnu::always_inline]] inline int32_t resolveStandY(
        int32_t nx, int32_t baseY, int32_t nz,
        int32_t width, int32_t height,
        int32_t maxStepUp, int32_t maxFall) const noexcept
    {
        // Hot path: flat terrain. Most steps in a path don't change Y at all.
        if (fitsBox(nx, baseY, nz, width, height)) return baseY;

        // Step up — gravity-bound mob can climb but only if flat blocked.
        for (int32_t dy = 1; dy <= maxStepUp; ++dy) {
            if (fitsBox(nx, baseY + dy, nz, width, height)) return baseY + dy;
        }

        // Fall — last resort: walking off a ledge.
        for (int32_t dy = -1; dy >= -maxFall; --dy) {
            if (fitsBox(nx, baseY + dy, nz, width, height)) return baseY + dy;
        }
        return INT32_MIN;
    }

    // ----- Cache invalidation hook --------------------------------------------------------

    /// Bump any time a subchunk is mutated. Solver caches use this as a generation counter
    /// to invalidate stale path-cache entries.
    uint64_t generation() const noexcept { return generation_; }

private:
    [[gnu::always_inline]] inline const SubChunk *subChunkAt(int32_t x, int32_t y, int32_t z) const noexcept {
        const int32_t cx = x >> 4, cy = y >> 4, cz = z >> 4;
        const uint64_t key = packSubChunk(cx, cy, cz);
        if (key == cachedKey_) {
            return cachedSub_;
        }
        return resolveCached(key);
    }

    const SubChunk *resolveCached(uint64_t key) const noexcept;

    void invalidateCache() const noexcept {
        cachedKey_ = kInvalidKey;
        cachedSub_ = nullptr;
    }

    static constexpr uint64_t kInvalidKey = ~uint64_t(0);

    BlockTable blockTable_;
    std::unordered_map<uint64_t, std::unique_ptr<SubChunk>, PackedCoordHash> subChunks_;

    // 1-entry single-slot cache for the most recently accessed subchunk.
    // `mutable` so const queries can refresh it.
    mutable uint64_t        cachedKey_ = kInvalidKey;
    mutable const SubChunk *cachedSub_ = nullptr;

    uint64_t generation_ = 0;
};

} // namespace pathfinder

#endif
