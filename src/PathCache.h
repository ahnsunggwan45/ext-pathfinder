#ifndef PATHFINDER_PATHCACHE_H
#define PATHFINDER_PATHCACHE_H

#include <cstddef>
#include <cstdint>
#include <list>
#include <unordered_map>
#include <utility>
#include <vector>

namespace pathfinder {

/// A path-cache key: endpoint coordinates plus entity bounding-box dimensions.
/// All eight fields are part of the identity — same start/end with a different-sized entity
/// is a different path.
struct PathKey {
    int32_t sx, sy, sz;
    int32_t ex, ey, ez;
    int32_t width, height;

    bool operator==(const PathKey &o) const noexcept {
        return sx == o.sx && sy == o.sy && sz == o.sz
            && ex == o.ex && ey == o.ey && ez == o.ez
            && width == o.width && height == o.height;
    }
};

struct PathKeyHash {
    [[gnu::always_inline]] size_t operator()(const PathKey &k) const noexcept {
        // FNV-1a over the raw bytes — well-distributed for integer-tuple keys.
        // 32 bytes for a PathKey, fully unrolled by the compiler.
        uint64_t h = 0xcbf29ce484222325ULL;
        const uint8_t *p = reinterpret_cast<const uint8_t *>(&k);
        for (size_t i = 0; i < sizeof(PathKey); ++i) {
            h ^= p[i];
            h *= 0x100000001b3ULL;
        }
        return static_cast<size_t>(h);
    }
};

/// Bounded LRU path cache.
///
/// Each entry remembers the NavMesh `generation` at insertion time. On lookup, if the cached
/// generation no longer matches the live one the entry is dropped silently — this turns
/// invalidation into a single integer compare, so block updates don't have to walk the cache.
class PathCache {
public:
    struct Entry {
        std::vector<int32_t> path; ///< flat (x0,y0,z0, x1,y1,z1, …); empty when `found = false`
        bool                 found;
        uint64_t             generation;
    };

    explicit PathCache(size_t maxSize = 1024) : maxSize_(maxSize ? maxSize : 1) {}

    void setMaxSize(size_t n) {
        maxSize_ = n ? n : 1;
        evict();
    }
    size_t maxSize() const noexcept { return maxSize_; }
    size_t size() const noexcept    { return index_.size(); }

    /// Look up `key`. Returns `true` on a fresh hit; writes the cached entry into `out`.
    /// On stale hit (generation mismatch), evicts and returns `false`.
    bool get(const PathKey &key, uint64_t currentGen, Entry &out) {
        const auto it = index_.find(key);
        if (it == index_.end()) return false;
        if (it->second->entry.generation != currentGen) {
            // Stale — drop it lazily so we don't waste CPU on eager invalidation.
            lru_.erase(it->second);
            index_.erase(it);
            return false;
        }
        // Promote to MRU.
        lru_.splice(lru_.begin(), lru_, it->second);
        out = it->second->entry;
        return true;
    }

    /// Store / overwrite an entry for `key`.
    void put(const PathKey &key, std::vector<int32_t> path, bool found, uint64_t currentGen) {
        const auto it = index_.find(key);
        if (it != index_.end()) {
            it->second->entry.path       = std::move(path);
            it->second->entry.found      = found;
            it->second->entry.generation = currentGen;
            lru_.splice(lru_.begin(), lru_, it->second);
            return;
        }
        lru_.push_front(LruNode{key, Entry{std::move(path), found, currentGen}});
        index_.emplace(key, lru_.begin());
        evict();
    }

    void clear() noexcept {
        index_.clear();
        lru_.clear();
    }

    /// Wipe entries whose stored generation no longer matches `currentGen`.
    /// Called rarely — normal eviction is lazy on `get`.
    size_t purgeStale(uint64_t currentGen) {
        size_t purged = 0;
        for (auto it = lru_.begin(); it != lru_.end();) {
            if (it->entry.generation != currentGen) {
                index_.erase(it->key);
                it = lru_.erase(it);
                ++purged;
            } else {
                ++it;
            }
        }
        return purged;
    }

private:
    struct LruNode {
        PathKey key;
        Entry   entry;
    };

    void evict() {
        while (lru_.size() > maxSize_) {
            index_.erase(lru_.back().key);
            lru_.pop_back();
        }
    }

    // List-based LRU: front = MRU, back = LRU.
    std::list<LruNode>                                                   lru_;
    std::unordered_map<PathKey, std::list<LruNode>::iterator, PathKeyHash> index_;
    size_t                                                               maxSize_;
};

} // namespace pathfinder

#endif
