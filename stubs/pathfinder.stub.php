<?php

/**
 * IDE stubs for the `pathfinder` PHP extension.
 *
 * This file is NOT loaded at runtime — the extension provides these classes natively.
 * Drop it in your IDE's "external libraries" or include it in your composer autoload's
 * `files` list (with the `if (false)` guard to keep it dormant).
 */

namespace pathfinder;

if (false) {

/**
 * High-performance A* navigation mesh backed by a C++ extension.
 *
 * Lifecycle:
 *   1. Build the block-property table once at startup with `setBlockProperties()`.
 *   2. Stream sub-chunks in/out as worlds load via `loadSubChunk()` / `unloadSubChunk()`.
 *   3. Patch live block changes via `updateBlock()` — the path cache invalidates lazily
 *      via the navmesh generation counter.
 *   4. Query paths with `findPath()`. Pass `entityWidth` / `entityHeight` so the bounding
 *      box check accounts for the actual mob size × scale.
 */
final class NavMesh {

    public function __construct() {}

    /**
     * Mark a single block-state-id as `[passable, solid]`.
     *
     * - `passable` — a mob's body can occupy the cell (no collision).
     * - `solid`    — a mob can stand on top of the block.
     *
     * Default for unset IDs is `[passable=false, solid=true]` — i.e. "treat unknown as full block".
     */
    public function setBlockProperty(int $blockStateId, bool $passable, bool $solid): void {}

    /**
     * Batch variant of {@see self::setBlockProperty()}.
     *
     * @param array<int, array{0: bool, 1: bool}> $properties Map of `blockStateId => [passable, solid]`.
     */
    public function setBlockProperties(array $properties): void {}

    public function clearBlockTable(): void {}

    /**
     * Load a 16×16×16 sub-chunk's block IDs as a packed binary string.
     *
     * `$packedBlockIds` MUST be exactly 16384 bytes (4096 × `uint32`) in
     * `(y << 8) | (z << 4) | x` order.
     *
     * For PocketMine you'll usually want {@see self::loadSubChunkFromWordArray()}
     * instead — it skips the 4096-iteration `pack('V', ...)` loop.
     */
    public function loadSubChunk(int $cx, int $cy, int $cz, string $packedBlockIds): void {}

    /**
     * Load a sub-chunk directly from a chunkutils2 `PalettedBlockArray`'s internal
     * components — no per-block PHP loop. Roughly **30× faster** than the
     * `loadSubChunk(packedString)` path on subchunk-load events.
     *
     * Typical usage:
     * ```
     * $arr = $subChunk->getBlockLayers()[0]; // PalettedBlockArray from chunkutils2
     * $nav->loadSubChunkFromWordArray(
     *     $cx, $cy, $cz,
     *     $arr->getWordArray(),
     *     $arr->getPalette(),
     *     $arr->getBitsPerBlock(),
     * );
     * ```
     *
     * @param string         $wordArray    Binary string from `PalettedBlockArray::getWordArray()`.
     * @param array<int,int> $palette      Result of `PalettedBlockArray::getPalette()` — list of
     *                                     unique block-state IDs in palette-index order.
     * @param int            $bitsPerBlock Result of `PalettedBlockArray::getBitsPerBlock()`.
     */
    public function loadSubChunkFromWordArray(
        int $cx,
        int $cy,
        int $cz,
        string $wordArray,
        array $palette,
        int $bitsPerBlock,
    ): void {}

    /** Load a fully-passable air sub-chunk. */
    public function loadAirSubChunk(int $cx, int $cy, int $cz): void {}

    /** Load a fully-solid sub-chunk (e.g. world floor padding). */
    public function loadSolidSubChunk(int $cx, int $cy, int $cz): void {}

    public function unloadSubChunk(int $cx, int $cy, int $cz): void {}

    public function unloadColumn(int $cx, int $cz): void {}

    public function isLoaded(int $cx, int $cy, int $cz): bool {}

    /**
     * True iff every grid cell the segment from `(x1, y1, z1)` to `(x2, y2, z2)` passes
     * through can fit a `width × height × width` entity bounding box.
     *
     * Used by path smoothing to merge collinear A* waypoints — the check naturally breaks
     * at jump-up / fall transitions because the line passes through cells that aren't
     * standable mid-Y, so smoothed paths keep their discrete jump points.
     */
    public function isLineWalkable(
        int $x1, int $y1, int $z1,
        int $x2, int $y2, int $z2,
        int $width, int $height,
    ): bool {}

    /** Drop every sub-chunk and the path cache. */
    public function clear(): void {}

    /**
     * Patch a single block in-place. No-op if the containing sub-chunk isn't loaded.
     * Bumps the navmesh generation, so cached paths are invalidated lazily on next lookup.
     */
    public function updateBlock(int $x, int $y, int $z, int $blockStateId): void {}

    /**
     * Find a path from `(sx, sy, sz)` to `(ex, ey, ez)`.
     *
     * @param array{
     *     maxIterations?:   int,    // default 10000
     *     maxStepUp?:       int,    // default 2 — cells the mob auto-steps up
     *     maxFallDistance?: int,    // default 3 — cells the mob can safely fall
     *     allowDiagonal?:   bool,   // default true — 8-connected vs 4-connected horizontal
     *     entityWidth?:     int,    // default 1 — XZ extent in cells (= ceil(width × scale))
     *     entityHeight?:    int,    // default 2 — Y extent in cells (= ceil(height × scale))
     *     diagonalCost?:    float,  // default √2
     *     cardinalCost?:    float,  // default 1.0
     *     stepUpCost?:      float,  // default 0.5 per cell of vertical climb
     *     fallCost?:        float,  // default 0.4 per cell of vertical fall
     *     maxPathLength?:   int,    // default 0 (off) — abort if g-cost exceeds this
     *     useCache?:        bool,   // default false (must be enabled with setCacheSize() first)
     *     algorithm?:       string, // 'astar' (default) or 'jps' — 2.5D Jump Point Search
     * }|null $options
     * @return list<array{0: int, 1: int, 2: int}>|null List of `[x, y, z]` cells from start to goal,
     *                                                  or `null` if no path / endpoints invalid / cap hit.
     */
    public function findPath(int $sx, int $sy, int $sz, int $ex, int $ey, int $ez, ?array $options = null): ?array {}

    /** Monotonically-increasing counter; bumps on every navmesh mutation. */
    public function getGeneration(): int {}

    public function getLoadedSubChunkCount(): int {}

    /** A* iterations spent on the most recent `findPath()` call (for profiling). */
    public function getLastIterations(): int {}

    /** Set the LRU path-cache capacity. Pass 0 to disable caching entirely. */
    public function setCacheSize(int $maxEntries): void {}

    public function clearCache(): void {}

    public function getCacheSize(): int {}
}

}
