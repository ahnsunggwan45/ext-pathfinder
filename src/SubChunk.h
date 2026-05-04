#ifndef PATHFINDER_SUBCHUNK_H
#define PATHFINDER_SUBCHUNK_H

#include <bitset>
#include <cstdint>
#include <cstring>

namespace pathfinder {

/// 16×16×16 walkability bitmaps for one subchunk.
///
/// Two bitmaps:
///   `passable_` — body can occupy this cell (true = no collision).
///   `solid_`    — block has a standable top surface (true = mob can stand on top of this).
///
/// Each bitset is 4096 bits (512 bytes) — total 1 KiB per subchunk.
/// Indexing: `(y << 8) | (z << 4) | x` — Y-major so horizontal A* iteration stays cache-friendly.
class SubChunk {
public:
    static constexpr int SIZE   = 16;
    static constexpr int VOLUME = SIZE * SIZE * SIZE; // 4096

    SubChunk() = default;

    [[gnu::always_inline]] static constexpr int index(int x, int y, int z) noexcept {
        return ((y & 15) << 8) | ((z & 15) << 4) | (x & 15);
    }

    [[gnu::always_inline]] inline bool isPassable(int x, int y, int z) const noexcept {
        return passable_.test(index(x, y, z));
    }
    [[gnu::always_inline]] inline bool isSolid(int x, int y, int z) const noexcept {
        return solid_.test(index(x, y, z));
    }

    [[gnu::always_inline]] inline bool isPassableIdx(int idx) const noexcept { return passable_.test(idx); }
    [[gnu::always_inline]] inline bool isSolidIdx(int idx) const noexcept    { return solid_.test(idx); }

    [[gnu::always_inline]] inline void setIdx(int idx, bool passable, bool solid) noexcept {
        passable_.set(idx, passable);
        solid_.set(idx, solid);
    }

    void fillSolid() noexcept {
        passable_.reset();
        solid_.set();
    }
    void fillAir() noexcept {
        passable_.set();
        solid_.reset();
    }

    bool noneSolid() const noexcept    { return solid_.none(); }
    bool nonePassable() const noexcept { return passable_.none(); }

private:
    std::bitset<VOLUME> passable_;
    std::bitset<VOLUME> solid_;
};

} // namespace pathfinder

#endif
