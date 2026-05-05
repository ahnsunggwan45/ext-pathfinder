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
require __DIR__ . '/UserStyleAStar.php';

use pathfinder\bench\PhpNavMesh;
use pathfinder\bench\UserStyleAStar;
use pocketmine\math\Vector3;

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

    // 2-D walkability grid for the user-style A* (it asks `isWalkable($x, $z)` only).
    $walkableGrid = [];

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
            $walkableGrid[$x][$z] = true;
        }
    }

    // Build the user-style A* with closures that hit the same grid as the other impls.
    // We approximate the cost of `BaseMonster::isPassable()` (4× World::getBlockAt) by doing
    // a small amount of dummy work — three index reads — to keep the comparison in the same
    // ballpark as a real PocketMine call site without depending on the engine.
    $isWalkable = function (int $x, int $z) use (&$walkableGrid): bool {
        // Three dummy reads simulate the floor/body/head lookups in BaseMonster::isPassable.
        $a = $walkableGrid[$x][$z] ?? false;
        $b = $walkableGrid[$x][$z] ?? false;
        $c = $walkableGrid[$x][$z] ?? false;
        return $a && $b && $c;
    };
    $getWeight = static fn(int $x, int $z): int => 10;

    $userAStar = new UserStyleAStar($isWalkable, $getWeight);
    // Force the user-style A* to search until completion instead of bailing out at the
    // default `maxCost = 500` (which truncates the path on 50+-cell tests). Without this
    // the comparison is "ext finds full path" vs "user finds partial path", which is a
    // misleading speedup ratio.
    $userAStar->setMaxCost(50_000);

    return [$cpp, $php, $userAStar];
}

/**
 * Open field with periodic 1-block walls perpendicular to the path. The mob has to
 * jump over each wall, then walk, then jump again. Tests pathfinder behaviour under
 * the vanilla MC model (maxStepUp=1, stepUpCost=1.0).
 *
 * UserStyleAStar can't navigate this — it's strictly 2-D and treats blocked cells
 * as impassable. PhpNavMesh and ext-pathfinder both handle it natively.
 */
function buildJumpCourse(int $size = 64, int $wallEvery = 10): array {
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

    // Walking floor at y=15.
    for ($x = 0; $x < $size; $x++) {
        for ($z = 0; $z < $size; $z++) {
            $cpp->updateBlock($x, 15, $z, 0);
            $php->updateBlock($x, 15, $z, 0);
        }
    }

    // Periodic walls perpendicular to +X direction at y=15. Mob must jump over.
    // (y=16+ is unloaded → defaults to passable, so jumping clears the obstacle.)
    for ($wx = $wallEvery; $wx < $size - 5; $wx += $wallEvery) {
        for ($z = 0; $z < $size; $z++) {
            $cpp->updateBlock($wx, 15, $z, 1);
            $php->updateBlock($wx, 15, $z, 1);
        }
    }

    // The user-style A* is intentionally absent for this scenario — it's 2-D and
    // can't represent jumping over a wall. We pass a no-op stub to keep the
    // runScenario signature uniform; runScenario will detect the null path return.
    $rejectAll = static fn(int $x, int $z): bool => false;
    $userAStar = new UserStyleAStar($rejectAll, static fn(int $x, int $z): int => 10);
    $userAStar->setMaxCost(50_000);

    return [$cpp, $php, $userAStar];
}

function buildMaze(int $size = 64, int $wallSpacing = 8): array {
    [$cpp, $php, $userAStarOld] = buildOpenField($size);
    unset($userAStarOld); // we rebuild below with maze-aware grid

    $walkableGrid = [];
    for ($x = 0; $x < $size; $x++) {
        for ($z = 0; $z < $size; $z++) {
            $walkableGrid[$x][$z] = true;
        }
    }

    $rng = new \Random\Randomizer(new \Random\Engine\Mt19937(42));
    for ($wallX = $wallSpacing; $wallX < $size; $wallX += $wallSpacing) {
        $passageZ = $rng->getInt(2, $size - 3);
        for ($z = 0; $z < $size; $z++) {
            if ($z === $passageZ || $z === $passageZ + 1) continue;
            $cpp->updateBlock($wallX, 15, $z, 1);
            $php->updateBlock($wallX, 15, $z, 1);
            $walkableGrid[$wallX][$z] = false;
        }
    }

    $isWalkable = function (int $x, int $z) use (&$walkableGrid): bool {
        $a = $walkableGrid[$x][$z] ?? false;
        $b = $walkableGrid[$x][$z] ?? false;
        $c = $walkableGrid[$x][$z] ?? false;
        return $a && $b && $c;
    };
    $getWeight = static fn(int $x, int $z): int => 10;
    $userAStar = new UserStyleAStar($isWalkable, $getWeight);
    $userAStar->setMaxCost(50_000);

    return [$cpp, $php, $userAStar];
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

function runScenario(string $name, $cpp, $php, $userAStar, array $args, array $userArgs, int $cppIter, int $phpIter, int $userIter): void {
    echo "── $name\n";

    // Build per-algorithm option arrays for ext-pathfinder.
    $argsA = $args;
    $argsJ = $args;
    $argsA[6] = ($args[6] ?? []) + ['algorithm' => 'astar'];
    $argsJ[6] = ($args[6] ?? []) + ['algorithm' => 'jps'];

    // Sanity check: produce paths from all four impls.
    $aPath = $cpp->findPath(...$argsA);
    $jPath = $cpp->findPath(...$argsJ);
    $pPath = $php->findPath(...$args);
    $userAStar->setGoal($userArgs[1]);
    $uPath = $userAStar->calculate($userArgs[0]);
    $aLen  = $aPath !== null ? count($aPath) : -1;
    $jLen  = $jPath !== null ? count($jPath) : -1;
    $pLen  = $pPath !== null ? count($pPath) : -1;
    $uLen  = $uPath !== null ? count($uPath) : -1;

    echo "  path length    : a*={$aLen}, jps={$jLen}(jp), php={$pLen}, user={$uLen}\n";

    $aStats = benchBatched(fn() => $cpp->findPath(...$argsA), $cppIter);
    $jStats = benchBatched(fn() => $cpp->findPath(...$argsJ), $cppIter);
    $pStats = benchBatched(fn() => $php->findPath(...$args),  $phpIter, batchSize: 10);
    $uStats = benchBatched(function () use ($userAStar, $userArgs) {
        $userAStar->setGoal($userArgs[1]);
        $userAStar->calculate($userArgs[0]);
    }, $userIter, batchSize: 5);

    $vsAStar    = $pStats['mean'] / max($aStats['mean'], 1e-9);
    $vsJps      = $pStats['mean'] / max($jStats['mean'], 1e-9);
    $userVsExt  = $uStats['mean'] / max($aStats['mean'], 1e-9);

    echo "  user A* (cur)  : " . fmt($uStats) . "\n";
    echo "  PHP-native ref : " . fmt($pStats) . "\n";
    echo "  ext A*         : " . fmt($aStats) . "\n";
    echo "  ext JPS        : " . fmt($jStats) . "\n";
    printf("  speedup        : ref→A*=%.1f×,  ref→JPS=%.1f×,  user→ext A*=%.1f×\n\n",
           $vsAStar, $vsJps, $userVsExt);
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

[$cpp, $php, $userAStar] = buildOpenField(64);
$opts = ['entityHeight' => 1, 'useCache' => false];

runScenario(
    "Test 1 / 10-cell short hop (open field)",
    $cpp, $php, $userAStar,
    [5, 15, 5, 15, 15, 5, $opts],
    [new Vector3(5, 15, 5), new Vector3(15, 15, 5)],
    cppIter:  (int) (10000 * $multiplier),
    phpIter:  (int) (1000  * $multiplier),
    userIter: (int) (1000  * $multiplier),
);

runScenario(
    "Test 2 / 50-cell straight (open field)",
    $cpp, $php, $userAStar,
    [5, 15, 5, 55, 15, 5, $opts],
    [new Vector3(5, 15, 5), new Vector3(55, 15, 5)],
    cppIter:  (int) (5000 * $multiplier),
    phpIter:  (int) (500  * $multiplier),
    userIter: (int) (500  * $multiplier),
);

runScenario(
    "Test 3 / 50-cell diagonal (open field)",
    $cpp, $php, $userAStar,
    [5, 15, 5, 55, 15, 55, $opts],
    [new Vector3(5, 15, 5), new Vector3(55, 15, 55)],
    cppIter:  (int) (5000 * $multiplier),
    phpIter:  (int) (300  * $multiplier),
    userIter: (int) (300  * $multiplier),
);

[$cpp, $php, $userAStar] = buildMaze(64, 8);

runScenario(
    "Test 4 / Maze 60-cell with detours",
    $cpp, $php, $userAStar,
    [2, 15, 2, 60, 15, 60, $opts],
    [new Vector3(2, 15, 2), new Vector3(60, 15, 60)],
    cppIter:  (int) (2000 * $multiplier),
    phpIter:  (int) (100  * $multiplier),
    userIter: (int) (100  * $multiplier),
);

[$cpp, $php, $userAStar] = buildJumpCourse(64, 10);

// Vanilla MC config: 1-block jump only, no auto-step over full blocks.
$jumpOpts = [
    'entityHeight'    => 2,
    'entityWidth'     => 1,
    'maxStepUp'       => 1,
    'maxFallDistance' => 3,
    'stepUpCost'      => 1.0,
    'fallCost'        => 0.3,
    'useCache'        => false,
];

runScenario(
    "Test 5 / Jump course 50-cell (1-block walls every 10)",
    $cpp, $php, $userAStar,
    [5, 15, 5, 55, 15, 5, $jumpOpts],
    [new Vector3(5, 15, 5), new Vector3(55, 15, 5)],
    cppIter:  (int) (2000 * $multiplier),
    phpIter:  (int) (100  * $multiplier),
    userIter: (int) (50   * $multiplier), // user A* will fail out — tiny budget
);

echo "Done.\n";
