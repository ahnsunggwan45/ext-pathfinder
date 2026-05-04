<?php
declare(strict_types=1);

/**
 * Head-to-head benchmark: pure-PHP A* vs ext-pathfinder.
 *
 * Run:
 *   /home/user/PHP-Binaries/bin/php7/bin/php run.php
 *
 * Increase sample size for tighter confidence:
 *   php run.php --multiplier=5     (5× more iterations)
 *   php run.php --multiplier=20    (20× — takes a few minutes)
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

// ----- CLI args ----------------------------------------------------------------------------

$multiplier = 1;
foreach ($argv as $arg) {
    if (preg_match('/^--multiplier=(\d+(?:\.\d+)?)$/', $arg, $m)) {
        $multiplier = (float) $m[1];
    }
}

// ============================================================================
// Map fixtures
// ============================================================================

function buildOpenField(int $size = 64): array {
    $stone = str_repeat(pack('V', 1), 4096);

    $cpp = new \pathfinder\NavMesh();
    $cpp->setBlockProperty(0, true, false);
    $cpp->setBlockProperty(1, false, true);
    $cpp->setCacheSize(0);

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

function buildMaze(int $size = 64, int $wallSpacing = 8): array {
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

/**
 * Batched-loop timing.
 *
 * For very fast calls (sub-microsecond), the cost of `hrtime()` itself can
 * skew per-call samples by 10-30%. Batching `$batchSize` calls under a single
 * timer amortises that overhead away. Each batch contributes one sample point
 * (= average call time inside the batch), so percentile statistics are
 * batch-level — fine for stability metrics, less precise than per-call sampling.
 */
function benchBatched(callable $fn, int $totalIterations, int $batchSize = 50, int $warmup = 100): array {
    for ($i = 0; $i < $warmup; $i++) $fn();

    $batches = (int) max(1, ceil($totalIterations / $batchSize));
    $samples = [];

    $totalStart = hrtime(true);
    for ($b = 0; $b < $batches; $b++) {
        $t0 = hrtime(true);
        for ($i = 0; $i < $batchSize; $i++) $fn();
        $samples[] = (hrtime(true) - $t0) / 1e6 / $batchSize;
    }
    $totalMs = (hrtime(true) - $totalStart) / 1e6;

    $actualIters = $batches * $batchSize;
    sort($samples);

    return [
        'mean'      => $totalMs / $actualIters,
        'median'    => $samples[(int) ($batches / 2)],
        'p95'       => $samples[(int) min($batches - 1, $batches * 0.95)],
        'min'       => $samples[0],
        'max'       => $samples[$batches - 1],
        'iterations' => $actualIters,
        'batches'   => $batches,
    ];
}

function fmt(array $s): string {
    return sprintf("mean=%7.3f ms  median=%7.3f ms  p95=%7.3f ms  min=%7.3f ms  (%d iter / %d batches)",
        $s['mean'], $s['median'], $s['p95'], $s['min'], $s['iterations'], $s['batches']);
}

function runScenario(string $name, $cpp, $php, array $args, int $cppIter, int $phpIter): void {
    echo "── $name\n";

    // Sanity check: both impls should produce paths of the same length.
    $cppPath = $cpp->findPath(...$args);
    $phpPath = $php->findPath(...$args);
    $cppLen  = $cppPath !== null ? count($cppPath) : -1;
    $phpLen  = $phpPath !== null ? count($phpPath) : -1;

    echo "  path length    : ext={$cppLen}, php={$phpLen}";
    if ($cppLen !== $phpLen) echo "  ⚠ length mismatch";
    echo "\n";

    $cppStats = benchBatched(fn() => $cpp->findPath(...$args), $cppIter);
    $phpStats = benchBatched(fn() => $php->findPath(...$args), $phpIter, batchSize: 10);

    $speedupMean   = $phpStats['mean']   / max($cppStats['mean'],   1e-9);
    $speedupMedian = $phpStats['median'] / max($cppStats['median'], 1e-9);

    echo "  PHP-native     : " . fmt($phpStats) . "\n";
    echo "  ext-pathfinder : " . fmt($cppStats) . "\n";
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
echo " multiplier     : ×$multiplier\n";
echo " hostname       : " . gethostname() . "\n";
echo "\n";

[$cpp, $php] = buildOpenField(64);
$opts = ['entityHeight' => 1, 'useCache' => false];

runScenario(
    "Test 1 / 10-cell short hop (open field)",
    $cpp, $php,
    [5, 15, 5, 15, 15, 5, $opts],
    cppIter: (int) (10000 * $multiplier),
    phpIter: (int) (1000  * $multiplier),
);

runScenario(
    "Test 2 / 50-cell straight (open field)",
    $cpp, $php,
    [5, 15, 5, 55, 15, 5, $opts],
    cppIter: (int) (5000 * $multiplier),
    phpIter: (int) (500  * $multiplier),
);

runScenario(
    "Test 3 / 50-cell diagonal (open field)",
    $cpp, $php,
    [5, 15, 5, 55, 15, 55, $opts],
    cppIter: (int) (5000 * $multiplier),
    phpIter: (int) (300  * $multiplier),
);

[$cpp, $php] = buildMaze(64, 8);

runScenario(
    "Test 4 / Maze 60-cell with detours",
    $cpp, $php,
    [2, 15, 2, 60, 15, 60, $opts],
    cppIter: (int) (2000 * $multiplier),
    phpIter: (int) (100  * $multiplier),
);

echo "Done.\n";
