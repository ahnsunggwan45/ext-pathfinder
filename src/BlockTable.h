#ifndef PATHFINDER_BLOCKTABLE_H
#define PATHFINDER_BLOCKTABLE_H

#include <cstddef>
#include <cstdint>
#include <unordered_map>

namespace pathfinder {

/// Per-block-state-id properties used by the navmesh.
///
/// `passable` — a mob's body can occupy this cell (no collision: air, water, tall grass).
/// `solid`    — a mob can stand on top of this block (full collision top: stone, dirt, slabs-top-half).
///
/// `passable` and `solid` are intentionally orthogonal — water is `passable=true, solid=false`,
/// a fence is `passable=false, solid=true`, a regular full block is `passable=false, solid=true`.
struct BlockProperty {
    bool passable;
    bool solid;
};

/// Lookup table from PocketMine full-block-state-id (uint32) to walkability properties.
/// Default for unset IDs is `{passable=false, solid=true}` — i.e. "treat unknown as a regular full block".
class BlockTable {
public:
    BlockTable() = default;

    void set(uint32_t blockStateId, bool passable, bool solid) {
        props_[blockStateId] = BlockProperty{passable, solid};
    }

    void clear() noexcept { props_.clear(); }

    [[gnu::always_inline]] inline BlockProperty get(uint32_t blockStateId) const noexcept {
        const auto it = props_.find(blockStateId);
        return it != props_.end() ? it->second : kDefault;
    }

    size_t size() const noexcept { return props_.size(); }

private:
    static constexpr BlockProperty kDefault{false, true};
    std::unordered_map<uint32_t, BlockProperty> props_;
};

} // namespace pathfinder

#endif
