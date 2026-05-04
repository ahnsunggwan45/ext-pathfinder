#include "NavMesh.h"

namespace pathfinder {

const SubChunk *NavMesh::resolveCached(uint64_t key) const noexcept {
    const auto it = subChunks_.find(key);
    cachedKey_ = key;
    cachedSub_ = (it != subChunks_.end()) ? it->second.get() : nullptr;
    return cachedSub_;
}

void NavMesh::loadSubChunk(int32_t cx, int32_t cy, int32_t cz, const uint32_t *blockIds) {
    auto sub = std::make_unique<SubChunk>();

    // Decode the entire 4096-block payload in linear scan order — this matches the bitset's
    // internal layout and is roughly 2× faster than a triple-nested xyz loop in benchmarks.
    for (int idx = 0; idx < SubChunk::VOLUME; ++idx) {
        const BlockProperty p = blockTable_.get(blockIds[idx]);
        sub->setIdx(idx, p.passable, p.solid);
    }

    subChunks_[packSubChunk(cx, cy, cz)] = std::move(sub);
    invalidateCache();
    ++generation_;
}

void NavMesh::loadAirSubChunk(int32_t cx, int32_t cy, int32_t cz) {
    auto sub = std::make_unique<SubChunk>();
    sub->fillAir();
    subChunks_[packSubChunk(cx, cy, cz)] = std::move(sub);
    invalidateCache();
    ++generation_;
}

void NavMesh::loadSolidSubChunk(int32_t cx, int32_t cy, int32_t cz) {
    auto sub = std::make_unique<SubChunk>();
    sub->fillSolid();
    subChunks_[packSubChunk(cx, cy, cz)] = std::move(sub);
    invalidateCache();
    ++generation_;
}

void NavMesh::unloadSubChunk(int32_t cx, int32_t cy, int32_t cz) {
    if (subChunks_.erase(packSubChunk(cx, cy, cz)) > 0) {
        invalidateCache();
        ++generation_;
    }
}

void NavMesh::unloadColumn(int32_t cx, int32_t cz) {
    bool any = false;
    for (auto it = subChunks_.begin(); it != subChunks_.end();) {
        // Reverse the packCoord layout: top 21 bits = x, middle = y, bottom = z.
        const int32_t kx = static_cast<int32_t>(it->first >> 42) & 0x1FFFFF;
        const int32_t kz = static_cast<int32_t>(it->first) & 0x1FFFFF;
        // Sign-extend back from 21-bit to 32-bit.
        const int32_t sx = (kx & 0x100000) ? (kx | ~0x1FFFFF) : kx;
        const int32_t sz = (kz & 0x100000) ? (kz | ~0x1FFFFF) : kz;
        if (sx == cx && sz == cz) {
            it = subChunks_.erase(it);
            any = true;
        } else {
            ++it;
        }
    }
    if (any) {
        invalidateCache();
        ++generation_;
    }
}

void NavMesh::clear() {
    if (!subChunks_.empty()) {
        subChunks_.clear();
        invalidateCache();
        ++generation_;
    }
}

void NavMesh::updateBlock(int32_t x, int32_t y, int32_t z, uint32_t blockStateId) {
    const int32_t cx = x >> 4, cy = y >> 4, cz = z >> 4;
    const uint64_t key = packSubChunk(cx, cy, cz);
    const auto it = subChunks_.find(key);
    if (it == subChunks_.end()) {
        return; // patching an unloaded subchunk is a no-op
    }
    const BlockProperty p = blockTable_.get(blockStateId);
    it->second->setIdx(SubChunk::index(x, y, z), p.passable, p.solid);

    // Refresh cache if it currently points at this subchunk — pointer is still valid since
    // unique_ptr was not replaced, but generation must bump for downstream invalidation.
    if (cachedKey_ == key) {
        cachedSub_ = it->second.get();
    }
    ++generation_;
}

bool NavMesh::isLoaded(int32_t cx, int32_t cy, int32_t cz) const noexcept {
    return subChunks_.find(packSubChunk(cx, cy, cz)) != subChunks_.end();
}

} // namespace pathfinder
