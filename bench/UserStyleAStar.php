<?php
declare(strict_types=1);

/**
 * Verbatim copy of MonsterPlugin's existing `AStarPathFinder.php` (with `Vector3`
 * stubbed for CLI), so the bench can compare ext-pathfinder against the *actual*
 * code currently shipping in BunnyFarm rather than the optimised reference impl.
 *
 * Notable differences from `bench/PhpAStar.php`:
 *   - SplPriorityQueue (anonymous class) instead of hand-rolled binary heap
 *   - Closures for isWalkable / getWeight (one extra indirection per neighbour)
 *   - Hash key stored as int via shifted XZ pack
 *   - 2-D only (no Y handling) — Y stays at the start's Y for the entire path
 *   - Best-effort: returns the closest reached node when the goal is unreachable
 *
 * The `Vector3` here is just enough surface for the algorithm to run.
 */

namespace pocketmine\math {
    if (!class_exists(Vector3::class)) {
        final class Vector3 {
            public function __construct(public float $x = 0, public float $y = 0, public float $z = 0) {}
            public function floor(): self {
                return new self((float) floor($this->x), (float) floor($this->y), (float) floor($this->z));
            }
            public function add(float $dx, float $dy, float $dz): self {
                return new self($this->x + $dx, $this->y + $dy, $this->z + $dz);
            }
        }
    }
}

namespace pathfinder\bench {

    use pocketmine\math\Vector3;

    /**
     * Verbatim port of `MonsterPlugin\ai\AStarPathFinder` — same algorithm,
     * same data structures, same `calculate()` signature.
     */
    final class UserStyleAStar {
        private ?Vector3 $goal = null;
        private int $maxCost = 500;

        public function __construct(
            private \Closure $isWalkable,
            private \Closure $getWeight
        ) {}

        public function setGoal(?Vector3 $target): void { $this->goal = $target?->floor(); }
        public function getGoal(): ?Vector3 { return $this->goal; }
        public function setMaxCost(int $maxCost): void { $this->maxCost = $maxCost; }

        /**
         * @return Vector3[]|null
         */
        public function calculate(Vector3 $start): ?array {
            if ($this->goal === null) return null;

            $startV = $start->floor();
            $goalV = $this->goal;
            $fixedY = $start->y;

            $startHash = ((int) $startV->x << 32) | ((int) $startV->z & 0xFFFFFFFF);
            $goalHash = ((int) $goalV->x << 32) | ((int) $goalV->z & 0xFFFFFFFF);

            $openList = new class extends \SplPriorityQueue {
                public function compare($priority1, $priority2): int { return $priority2 <=> $priority1; }
            };

            $gScores = [$startHash => 0.0];
            $parents = [$startHash => null];

            $bestHash = $startHash;
            $minH = $this->heuristic((int) $startV->x, (int) $startV->z, (int) $goalV->x, (int) $goalV->z);

            $openList->insert($startHash, $minH);

            $iterations = 0;
            while (!$openList->isEmpty() && $iterations++ < $this->maxCost) {
                $currentHash = $openList->extract();

                if ($currentHash === $goalHash) {
                    return $this->reconstructPath($parents, $currentHash, $fixedY);
                }

                $currentX = $currentHash >> 32;
                $currentZ = ($currentHash & 0xFFFFFFFF) << 32 >> 32;
                $currentG = $gScores[$currentHash];

                for ($dx = -1; $dx <= 1; $dx++) {
                    for ($dz = -1; $dz <= 1; $dz++) {
                        if ($dx === 0 && $dz === 0) continue;
                        $nx = $currentX + $dx;
                        $nz = $currentZ + $dz;
                        if (!($this->isWalkable)($nx, $nz)) continue;
                        if ($dx !== 0 && $dz !== 0) {
                            if (!($this->isWalkable)($currentX + $dx, $currentZ) ||
                                !($this->isWalkable)($currentX, $currentZ + $dz)) continue;
                        }
                        $neighborHash = ($nx << 32) | ($nz & 0xFFFFFFFF);
                        $dist = ($dx === 0 || $dz === 0) ? 1.0 : 1.414;
                        $weight = ($this->getWeight)($nx, $nz);
                        $tentativeG = $currentG + ($dist * $weight);

                        if (!isset($gScores[$neighborHash]) || $tentativeG < $gScores[$neighborHash]) {
                            $gScores[$neighborHash] = $tentativeG;
                            $parents[$neighborHash] = $currentHash;
                            $h = $this->heuristic($nx, $nz, (int) $goalV->x, (int) $goalV->z);
                            if ($h < $minH) {
                                $minH = $h;
                                $bestHash = $neighborHash;
                            }
                            $openList->insert($neighborHash, $tentativeG + $h);
                        }
                    }
                }
            }

            if ($bestHash !== $startHash) {
                return $this->reconstructPath($parents, $bestHash, $fixedY);
            }
            return null;
        }

        private function heuristic(int $x1, int $z1, int $x2, int $z2): float {
            return sqrt(($x1 - $x2) ** 2 + ($z1 - $z2) ** 2);
        }

        private function reconstructPath(array $parents, int $currentHash, float $y): array {
            $path = [];
            while ($currentHash !== null) {
                $x = $currentHash >> 32;
                $z = ($currentHash & 0xFFFFFFFF) << 32 >> 32;
                array_unshift($path, new Vector3($x + 0.5, $y, $z + 0.5));
                $currentHash = $parents[$currentHash] ?? null;
            }
            return $path;
        }
    }
}
