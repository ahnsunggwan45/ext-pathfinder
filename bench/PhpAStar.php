<?php
declare(strict_types=1);

namespace pathfinder\bench;

/**
 * Pure-PHP A* pathfinder for benchmark comparison.
 *
 * Mirrors ext-pathfinder's C++ AStarSolver as closely as PHP allows:
 *   - Same neighbour model (4 or 8-connected horizontal + step-up/fall)
 *   - Same octile heuristic
 *   - Same bbox collision (entity width × height)
 *   - Same cost weights (cardinal=1.0, diagonal=√2, step-up=0.5, fall=0.4)
 *
 * Optimisations applied (what a perf-conscious PHP dev would write):
 *   - Flat int coords, no Vector3 allocations
 *   - String-keyed hashmap for visited / g-score / parent
 *   - Hand-rolled binary min-heap (faster than SplPriorityQueue)
 *
 * If anything, this is *more* optimised than what most plugins would ship —
 * which makes the speedup vs ext-pathfinder a conservative lower bound.
 */
final class PhpNavMesh {
    /** @var array<string, bool> */
    private array $passable = [];
    /** @var array<string, bool> */
    private array $solid = [];
    /** @var array<int, array{0: bool, 1: bool}> */
    private array $blockProps = [];

    private int $lastIterations = 0;

    public function setBlockProperty(int $id, bool $passable, bool $solid): void {
        $this->blockProps[$id] = [$passable, $solid];
    }

    public function loadSubChunk(int $cx, int $cy, int $cz, string $packed): void {
        if (strlen($packed) !== 16384) {
            throw new \InvalidArgumentException("expected 16384 bytes (4096 × uint32), got " . strlen($packed));
        }
        $bx = $cx << 4;
        $by = $cy << 4;
        $bz = $cz << 4;
        $ids = unpack('V*', $packed); // 1-indexed array of 4096 uint32
        for ($i = 0; $i < 4096; $i++) {
            $id = $ids[$i + 1];
            $x = $bx | ($i & 15);
            $z = $bz | (($i >> 4) & 15);
            $y = $by | (($i >> 8) & 15);
            [$pass, $sol] = $this->blockProps[$id] ?? [false, true];
            $k = $x . '|' . $y . '|' . $z;
            $this->passable[$k] = $pass;
            $this->solid[$k]    = $sol;
        }
    }

    public function updateBlock(int $x, int $y, int $z, int $id): void {
        [$pass, $sol] = $this->blockProps[$id] ?? [false, true];
        $k = $x . '|' . $y . '|' . $z;
        $this->passable[$k] = $pass;
        $this->solid[$k]    = $sol;
    }

    public function getLastIterations(): int {
        return $this->lastIterations;
    }

    private function fitsAt(int $x, int $y, int $z, int $w, int $h): bool {
        // Floor: every (x..x+w-1, y-1, z..z+w-1) cell must be solid.
        for ($dz = 0; $dz < $w; $dz++) {
            for ($dx = 0; $dx < $w; $dx++) {
                $fk = ($x + $dx) . '|' . ($y - 1) . '|' . ($z + $dz);
                if (!($this->solid[$fk] ?? false)) return false;
            }
        }
        // Body: every (x..x+w-1, y..y+h-1, z..z+w-1) cell must be passable.
        for ($dy = 0; $dy < $h; $dy++) {
            for ($dz = 0; $dz < $w; $dz++) {
                for ($dx = 0; $dx < $w; $dx++) {
                    $bk = ($x + $dx) . '|' . ($y + $dy) . '|' . ($z + $dz);
                    if (!($this->passable[$bk] ?? true)) return false;
                }
            }
        }
        return true;
    }

    /**
     * @return list<array{0: int, 1: int, 2: int}>|null
     */
    public function findPath(int $sx, int $sy, int $sz, int $ex, int $ey, int $ez, array $opts = []): ?array {
        $w         = $opts['entityWidth']     ?? 1;
        $h         = $opts['entityHeight']    ?? 2;
        $maxStepUp = $opts['maxStepUp']       ?? 2;
        $maxFall   = $opts['maxFallDistance'] ?? 3;
        $maxIter   = $opts['maxIterations']   ?? 10000;
        $allowDiag = $opts['allowDiagonal']   ?? true;

        $this->lastIterations = 0;

        if (!$this->fitsAt($sx, $sy, $sz, $w, $h)) return null;
        if (!$this->fitsAt($ex, $ey, $ez, $w, $h)) return null;
        if ($sx === $ex && $sy === $ey && $sz === $ez) return [[$sx, $sy, $sz]];

        // Hand-rolled binary min-heap of [f, x, y, z] tuples.
        $heap     = [];
        $heapSize = 0;
        $closed   = [];
        $gScore   = [];
        $parent   = [];

        $startKey = $sx . '|' . $sy . '|' . $sz;
        $h0       = $this->heuristic($ex - $sx, $ey - $sy, $ez - $sz);
        $gScore[$startKey] = 0.0;
        $heap[$heapSize++] = [$h0, $sx, $sy, $sz];

        $dirs = $allowDiag ? [
            [ 1,  0, 1.0],         [-1,  0, 1.0],         [ 0,  1, 1.0],         [ 0, -1, 1.0],
            [ 1,  1, 1.41421356],  [ 1, -1, 1.41421356],  [-1,  1, 1.41421356],  [-1, -1, 1.41421356],
        ] : [
            [1, 0, 1.0], [-1, 0, 1.0], [0, 1, 1.0], [0, -1, 1.0],
        ];

        $iter = 0;

        while ($heapSize > 0 && $iter < $maxIter) {
            $iter++;

            // ----- heap pop ---------------------------------------------------------------
            $top = $heap[0];
            $heap[0] = $heap[--$heapSize];
            unset($heap[$heapSize]);
            if ($heapSize > 0) {
                $i = 0;
                while (true) {
                    $left = ($i << 1) + 1;
                    if ($left >= $heapSize) break;
                    $right = $left + 1;
                    $best  = ($right < $heapSize && $heap[$right][0] < $heap[$left][0]) ? $right : $left;
                    if ($heap[$best][0] >= $heap[$i][0]) break;
                    $tmp = $heap[$i]; $heap[$i] = $heap[$best]; $heap[$best] = $tmp;
                    $i = $best;
                }
            }

            [, $cx, $cy, $cz] = $top;
            $curKey = $cx . '|' . $cy . '|' . $cz;

            if (isset($closed[$curKey])) continue;
            $closed[$curKey] = true;

            // Goal reached?
            if ($cx === $ex && $cy === $ey && $cz === $ez) {
                $path = [];
                $k    = $curKey;
                while ($k !== null) {
                    $parts  = explode('|', $k);
                    $path[] = [(int) $parts[0], (int) $parts[1], (int) $parts[2]];
                    $k      = $parent[$k] ?? null;
                }
                $this->lastIterations = $iter;
                return array_reverse($path);
            }

            $curG = $gScore[$curKey];

            foreach ($dirs as [$dx, $dz, $baseCost]) {
                $nx = $cx + $dx;
                $nz = $cz + $dz;

                // Resolve landing Y. Flat-first ordering (matches ext-pathfinder ≥0.2.2):
                //   dy=0 → up (1, 2, …) → down (-1, -2, …)
                // The vast majority of steps in any path are flat — checking dy=0 first
                // short-circuits in 1 fitsAt call instead of `maxStepUp + 1`.
                $ny = null;
                if ($this->fitsAt($nx, $cy, $nz, $w, $h)) {
                    $ny = $cy;
                } else {
                    for ($dy = 1; $dy <= $maxStepUp; $dy++) {
                        if ($this->fitsAt($nx, $cy + $dy, $nz, $w, $h)) {
                            $ny = $cy + $dy;
                            break;
                        }
                    }
                    if ($ny === null) {
                        for ($dy = -1; $dy >= -$maxFall; $dy--) {
                            if ($this->fitsAt($nx, $cy + $dy, $nz, $w, $h)) {
                                $ny = $cy + $dy;
                                break;
                            }
                        }
                    }
                }
                if ($ny === null) continue;

                // Diagonal corner-cut prevention.
                if ($dx !== 0 && $dz !== 0) {
                    if (!$this->fitsAt($cx + $dx, $cy, $cz, $w, $h)
                        && !$this->fitsAt($cx, $cy, $cz + $dz, $w, $h)) {
                        continue;
                    }
                }

                $deltaY = $ny - $cy;
                $vert   = $deltaY > 0 ? $deltaY * 0.5 : ($deltaY < 0 ? -$deltaY * 0.4 : 0.0);
                $newG   = $curG + $baseCost + $vert;

                $nKey = $nx . '|' . $ny . '|' . $nz;
                if (isset($closed[$nKey])) continue;
                if (isset($gScore[$nKey]) && $gScore[$nKey] <= $newG) continue;

                $gScore[$nKey] = $newG;
                $parent[$nKey] = $curKey;
                $f             = $newG + $this->heuristic($ex - $nx, $ey - $ny, $ez - $nz);

                // ----- heap push ---------------------------------------------------------
                $heap[$heapSize++] = [$f, $nx, $ny, $nz];
                $i = $heapSize - 1;
                while ($i > 0) {
                    $p = ($i - 1) >> 1;
                    if ($heap[$p][0] <= $heap[$i][0]) break;
                    $tmp = $heap[$i]; $heap[$i] = $heap[$p]; $heap[$p] = $tmp;
                    $i = $p;
                }
            }
        }

        $this->lastIterations = $iter;
        return null;
    }

    private function heuristic(int $dx, int $dy, int $dz): float {
        $adx = $dx < 0 ? -$dx : $dx;
        $adz = $dz < 0 ? -$dz : $dz;
        $lo  = $adx < $adz ? $adx : $adz;
        $hi  = $adx > $adz ? $adx : $adz;
        return $lo * 1.41421356 + ($hi - $lo);
    }
}
