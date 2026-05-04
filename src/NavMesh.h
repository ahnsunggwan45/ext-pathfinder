#ifndef PATHFINDER_NAVMESH_H
#define PATHFINDER_NAVMESH_H

#include "BlockTable.h"
#include "Coord.h"
#include "SubChunk.h"

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
