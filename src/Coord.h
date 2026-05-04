#ifndef PATHFINDER_COORD_H
#define PATHFINDER_COORD_H

#include <cstddef>
#include <cstdint>
#include <functional>

namespace pathfinder {

/// Pack three 21-bit signed integers into a 64-bit key.
/// Range per axis: [-2^20, 2^20 - 1] = [-1048576, 1048575] — far beyond any Minecraft world.
[[gnu::always_inline]] static inline uint64_t packCoord(int32_t x, int32_t y, int32_t z) noexcept {
    return (static_cast<uint64_t>(x) & 0x1FFFFFULL) << 42
         | (static_cast<uint64_t>(y) & 0x1FFFFFULL) << 21
         | (static_cast<uint64_t>(z) & 0x1FFFFFULL);
}

/// Identity hash for already-packed 64-bit keys (skip default mixing — coords are well-distributed).
struct PackedCoordHash {
    [[gnu::always_inline]] size_t operator()(uint64_t k) const noexcept {
        // Mix once with splitmix64 finalizer to defeat adversarial coord clustering on the lower 21 bits.
        k ^= k >> 30;
        k *= 0xbf58476d1ce4e5b9ULL;
        k ^= k >> 27;
        k *= 0x94d049bb133111ebULL;
        k ^= k >> 31;
        return static_cast<size_t>(k);
    }
};

/// Subchunk-key packing (chunk-space coordinates).
[[gnu::always_inline]] static inline uint64_t packSubChunk(int32_t cx, int32_t cy, int32_t cz) noexcept {
    return packCoord(cx, cy, cz);
}

} // namespace pathfinder

#endif
