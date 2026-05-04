# Benchmarks

Head-to-head measurement of ext-pathfinder vs a hand-rolled pure-PHP A*.

## What's measured

Same map, same start/end, same entity size, same neighbour rules.
The PHP side uses every reasonable optimisation (binary heap, flat int coords,
string hashmap) — i.e. better than a typical first-attempt plugin implementation.
That makes the reported speedups a **conservative lower bound**.

The ext-pathfinder side runs with `useCache => false` so each call re-runs A*.

## Running

```bash
# In WSL, against the ext-pathfinder-enabled binary:
/home/user/PHP-Binaries/bin/php7/bin/php run.php
```

Or, if you copied the PHP binary elsewhere:
```bash
/path/to/php run.php
```

## Output

Each scenario prints:
- path length (sanity check that both impls find a path of the same length)
- mean / median / p95 timing for each impl
- speedup ratio

```
── Test 2 / 50-cell straight (open field)
  path length    : ext=51, php=51
  PHP-native     : mean=  6.412 ms  median=  6.350 ms  p95=  7.801 ms  (50 runs)
  ext-pathfinder : mean=  0.108 ms  median=  0.105 ms  p95=  0.142 ms  (500 runs)
  speedup        : 59.4× mean   60.5× median
```

## Files

- `PhpAStar.php` — pure-PHP `PhpNavMesh` implementation (mirror of C++ side)
- `run.php` — orchestration + scenarios
