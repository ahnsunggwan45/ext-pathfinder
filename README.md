# ext-pathfinder

A high-performance A* pathfinding PHP extension for PocketMine-MP, written in C++17.

Modeled after [`ext-chunkutils2`](https://github.com/pmmp/ext-chunkutils2) — the C++ side
owns all hot-path data so the per-query PHP↔C++ boundary cost is one zval round trip,
not one per A* node.

## Why an extension

Pure-PHP A* spends most of its time on hashmap lookups, allocation churn and opcode
dispatch. Moving the inner loop to C++ buys roughly **20-30× in our benchmarks**:

| Scenario                            | PHP-native | ext-pathfinder | Speedup |
| ----------------------------------- | ---------- | -------------- | ------- |
| 10-cell straight (open field)       | 0.080 ms   | 0.004 ms       | 22×     |
| 50-cell straight (open field)       | 0.416 ms   | 0.017 ms       | 24×     |
| 50-cell diagonal (open field)       | 0.489 ms   | 0.022 ms       | 22×     |
| 60-cell maze with detours           | 7.105 ms   | 0.278 ms       | 26×     |

Measured on PHP 8.4.16 with tracing JIT, ZTS build. The PHP-native side is hand-rolled
with every reasonable optimisation (binary min-heap, flat int coords, string-keyed
hashmaps) — i.e. **better than a typical first-attempt plugin implementation**. That
makes these speedups a conservative lower bound; an idiomatic plugin using `Vector3`
objects and `SplPriorityQueue` would see significantly more.

Reproduce: `cd bench && php run.php` (see [`bench/README.md`](bench/README.md)).

## Architecture

```
PHP (PocketMine)                        C++ (ext-pathfinder)
──────────────────                      ───────────────────────
ChunkLoadEvent ──► loadSubChunk()  ──►  NavMesh::loadSubChunk()
                                         └─ decode 4096 block-state-ids → walkable/solid bitmaps
BlockUpdateEvent ► updateBlock()   ──►  NavMesh::updateBlock()
                                         └─ 1-bit patch + generation bump
findPath() ─────────────────────►       AStarSolver::findPath()
                                         ├─ 4-ary heap (open set)
                                         ├─ unordered_map (closed/coord index)
                                         └─ size-aware bbox collision
                                          ↑
                                       PathCache (LRU, generation-tagged)
```

### Core decisions

- **Y-major bitmap layout** — neighbour iteration in A* hits adjacent indices in the
  bitset, keeping the hot region in L1 cache.
- **Single-slot subchunk cache** on `NavMesh` — A* hits the same subchunk for ~80% of
  consecutive lookups; one cache line beats a hashmap probe.
- **4-ary heap for the open set** — shallower than binary, ~15-25% less wall-clock for
  typical A* sizes.
- **Lazy stale-entry invalidation** in the path cache — block updates don't have to walk
  the cache; stale entries are dropped when looked up.
- **Re-pushing instead of decrease-key** — the heap stays branch-free; stale entries
  short-circuit on the node's `closed` flag when popped.
- **Size-aware bounding box** — `findPath()` accepts `entityWidth` and `entityHeight`
  computed from the entity's `getSize() × getScale()`, so a 0.6×1.8 player and a 1.4×2.7
  iron golem path correctly without separate code paths.

## Building

### Linux / macOS

```bash
phpize
./configure --enable-pathfinder
make -j$(nproc)
sudo make install
```

Requires a C++17-capable compiler (GCC 8+, Clang 7+).

### Windows

Use the [PHP SDK](https://github.com/php/php-sdk-binary-tools) build environment:

```cmd
phpize
configure --enable-pathfinder=shared
nmake
```

For PocketMine, integrate via `pmmp/php-build-scripts` — add `pathfinder` to the
extension list in `compile.sh` / `windows-compile-vs.ps1`.

### Loading

Add to `php.ini`:

```ini
extension=pathfinder
```

## Usage

```php
<?php

use pathfinder\NavMesh;
use pocketmine\world\format\Chunk;

$nav = new NavMesh();

// 1. Configure block properties — done once at startup.
$nav->setBlockProperties([
    // blockStateId => [passable, solid]
    0   => [true,  false], // air
    1   => [false, true],  // stone
    2   => [false, true],  // dirt
    8   => [true,  false], // water (treat as passable)
    // ... populate from your block registry
]);

// 2. Stream sub-chunks as the world loads.
foreach ($chunk->getSubChunks() as $cy => $subChunk) {
    if ($subChunk->isEmptyFast()) {
        $nav->loadAirSubChunk($cx, $cy, $cz);
        continue;
    }
    $packed = pack_paletted_block_array($subChunk->getBlockLayers()[0]);
    $nav->loadSubChunk($cx, $cy, $cz, $packed);
}

// 3. Patch single-block changes.
$server->getPluginManager()->registerEvent(
    BlockUpdateEvent::class,
    function (BlockUpdateEvent $e) use ($nav): void {
        $b = $e->getBlock();
        $p = $b->getPosition();
        $nav->updateBlock($p->x, $p->y, $p->z, $b->getStateId());
    },
    EventPriority::MONITOR,
    $plugin
);

// 4. Query a path, sized for the actual entity.
$entity = ...; // some Living
$size   = $entity->getSize();
$scale  = $entity->getScale();

$path = $nav->findPath(
    (int) floor($entity->getPosition()->x), (int) floor($entity->getPosition()->y), (int) floor($entity->getPosition()->z),
    $targetX, $targetY, $targetZ,
    [
        'entityWidth'     => (int) ceil($size->getWidth()  * $scale),
        'entityHeight'    => (int) ceil($size->getHeight() * $scale),
        'maxStepUp'       => 2,
        'maxFallDistance' => 3,
        'maxIterations'   => 5000,
    ],
);

if ($path === null) {
    // No path within the iteration cap.
    return;
}

foreach ($path as [$x, $y, $z]) {
    // Feed waypoints to your entity controller.
}
```

## Packing PocketMine's PalettedBlockArray

The fastest path is a thin C/PHP helper that copies the resolved block-state-ids into a
binary string. A reference PHP implementation:

```php
function pack_paletted_block_array(\pocketmine\world\format\PalettedBlockArray $arr): string {
    $out = '';
    for ($i = 0; $i < 4096; $i++) {
        $x = $i & 15;
        $z = ($i >> 4) & 15;
        $y = ($i >> 8) & 15;
        $out .= pack('V', $arr->get($x, $y, $z));
    }
    return $out;
}
```

For real workloads, expose a native `getEntries()` from `chunkutils2` and write a single
`pack('V*', ...)` call — same result with one fewer loop.

## Configuration reference

See [stubs/pathfinder.stub.php](stubs/pathfinder.stub.php) for the full PHP surface
with phpdoc-typed options.

## License

LGPL-3.0 (matching `ext-chunkutils2` for upstream compatibility).
