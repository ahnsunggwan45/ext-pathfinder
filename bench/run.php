<?php
declare(strict_types=1);

/**
 * Head-to-head benchmark: pure-PHP A* vs ext-pathfinder.
 *
 * Run:
 *   /home/user/PHP-Binaries/bin/php7/bin/php run.php
 *
 * Both implementations share:
 *   - 8-connected horizontal moves with step-up/fall vertical resolution
 *   - Octile heuristic
 *   - Same neighbour costs (cardinal=1.0, diagonal=√2, step=0.5, fall=0.4)
 *   - Same map fixtures
 *   - Cache disabled on the C++ side so each call re-runs A*
 */

require __DIR__ . '/PhpAStar.php';

use pathfinder\bench\PhpNavMesh;

if (!class_exists('\\pathfinder\\NavMesh')) {
    fwrite(STDERR, "ext-pathfinder is not loaded. Run with the PHP binary that has the extension built in.\n");
    exit(1);
}

// ============================================================================
// Map fixtures
// ============================================================================

/**
 * Open field: a `$size × 16 × $size` cube of stone with the top layer (y=15) hollowed out.
 */
function buildOpenField(int $size = 64): array {
    $stone = str_repeat(pack('V', 1), 4096);

    $cpp = new \pathfinder\NavMesh();
    $cpp->setBlockProperty(0, true, false);
    $cpp->setBlockProperty(1, false, true);
    $cpp->setCacheSize(0); // disable cache for fair per-call timing

    $php = new PhpNavMesh();
    $php->setBlockProperty(0, true, false);
    $php->setBlockProperty(1, false, true);

    $chunks = (int) ceil($size / 16);
    for ($cx = 0; $cx < $chunks; $cx++) {
        for ($cz = 0; $cz < $chunks; $cz++) {
            $cpp->loadSubChunk($cx, 0, $cz, $stone);
            $php->loadSubChunk($cx, 0, $cz, $stone);
        }
    }

    for ($x = 0; $x < $size; $x++) {
        for ($z = 0; $z < $size; $z++) {
            $cpp->updateBlock($x, 15, $z, 0);
            $php->updateBlock($x, 15, $z, 0);
        }
    }

    return [$cpp, $php];
}

/**
 * Maze: same flat field, but with vertical walls inserted to force detours.
 * Walls run perpendicular to the X axis with a single passage offset on each.
 */
function buildMaze(int $size = 64, int $wallSpacing = 6): array {
    [$cpp, $php] = buildOpenField($size);

    $rng = new \Random\Randomizer(new \Random\Engine\Mt19937(42));
    for ($wallX = $wallSpacing; $wallX < $size; $wallX += $wallSpacing) {
        $passageZ = $rng->getInt(2, $size - 3);
        for ($z = 0; $z < $size; $z++) {
            if ($z === $passageZ || $z === $passageZ + 1) continue;
            $cpp->updateBlock($wallX, 15, $z, 1);
            $php->updateBlock($wallX, 15, $z, 1);
        }
    }
    return [$cpp, $php];
}

// ============================================================================
// Benchmark harness
// ============================================================================

function bench(callable $fn, int $iterations, int $warmup = 5): array {
    for ($i = 0; $i < $warmup; $i++) $fn();

    $samples = [];
    $start   = hrtime(true);
    for ($i = 0; $i < $iterations; $i++) {
        $t0 = hrtime(true);
        $fn();
        $samples[] = (hrtime(true) - $t0) / 1e6;
    }
    $totalMs = (hrtime(true) - $start) / 1e6;

    sort($samples);
    return [
        'mean'   => $totalMs / $iterations,
        'median' => $samples[(int) ($iterations / 2)],
        'p95'    => $samples[(int) ($iterations * 0.95)],
        'min'    => $samples[0],
    ];
}

function runScenario(string $name, $cpp, $php, array $args, int $cppIter, int $phpIter): void {
    echo "── $name\n";

    // Warm a single call to discover output (verify both produce the same path length)
    $cppPath = $cpp->findPath(...$args);
    $phpPath = $php->findPath(...$args);
    $cppLen  = $cppPath !== null ? count($cppPath) : -1;
    $phpLen  = $phpPath !== null ? count($phpPath) : -1;

    echo "  path length    : ext={$cppLen}, php={$phpLen}";
    if ($cppLen !== $phpLen) echo "  ⚠ length mismatch";
    echo "\n";

    $cppStats = bench(fn() => $cpp->findPath(...$args), $cppIter);
    $phpStats = bench(fn() => $php->findPath(...$args), $phpIter);

    $speedupMean   = $phpStats['mean']   / $cppStats['mean'];
    $speedupMedian = $phpStats['median'] / $cppStats['median'];

    printf("  PHP-native     : mean=%7.3f ms  median=%7.3f ms  p95=%7.3f ms  (%d runs)\n",
        $phpStats['mean'], $phpStats['median'], $phpStats['p95'], $phpIter);
    printf("  ext-pathfinder : mean=%7.3f ms  median=%7.3f ms  p95=%7.3f ms  (%d runs)\n",
        $cppStats['mean'], $cppStats['median'], $cppStats['p95'], $cppIter);
    printf("  speedup        : %.1f× mean   %.1f× median\n\n", $speedupMean, $speedupMedian);
}

// ============================================================================
// Run
// ============================================================================

echo "================================================================\n";
echo " ext-pathfinder vs PHP-native A* benchmark\n";
echo "================================================================\n";
echo " PHP version    : " . PHP_VERSION . "\n";
echo " JIT enabled    : " . var_export(opcache_get_status(false)['jit']['enabled'] ?? false, true) . "\n";
echo " OPcache buffer : " . round((opcache_get_status(false)['memory_usage']['used_memory'] ?? 0) / 1024 / 1024, 1) . " MiB used\n";
echo " hostname       : " . gethostname() . "\n";
echo "\n";

[$cpp, $php] = buildOpenField(64);

$opts = ['entityHeight' => 1, 'useCache' => false];

runScenario(
    "Test 1 / 10-cell short hop (open field)",
    $cpp, $php,
    [5, 15, 5, 15, 15, 5, $opts],
    cppIter: 1000, phpIter: 200
);

runScenario(
    "Test 2 / 50-cell straight (open field)",
    $cpp, $php,
    [5, 15, 5, 55, 15, 5, $opts],
    cppIter: 500, phpIter: 50
);

runScenario(
    "Test 3 / 50-cell diagonal (open field)",
    $cpp, $php,
    [5, 15, 5, 55, 15, 55, $opts],
    cppIter: 500, phpIter: 30
);

[$cpp, $php] = buildMaze(64, 8);

runScenario(
    "Test 4 / Maze 60-cell with detours",
    $cpp, $php,
    [2, 15, 2, 60, 15, 60, $opts],
    cppIter: 200, phpIter: 10
);

echo "Done.\n";
