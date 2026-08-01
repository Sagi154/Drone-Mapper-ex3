# Map3D behavior contract

Carried from ex2 (`../Drone-Mapper-ex2/docs/map3d_impl_contract.md`) and re-verified against
`inputs/map/*.npy`. In ex3, `Map3DImpl` belongs to `Simulator/src/` — plugins only ever see `IMap3D` /
`IMutableMap3D`.

## World → voxel mapping

For world position `pos` and `MapConfig { offset, resolution }`:

```text
index_axis = (pos_axis - offset_axis) / resolution
```

- Indices must be non-negative integers aligned to the grid; fractional positions are out of bounds.
- NumPy C-order layout: `linear = ix * (ny * nz) + iy * nz + iz` for shape `(nx, ny, nz)`.
- World axes map to array dimensions `x → 0`, `y → 1`, `z → 2`.

`isInBounds(pos)` is true when `pos` is inside `MapConfig.boundaries` (inclusive on each axis) **and**
the computed index is inside the array shape. When `MappingBounds` are all zero they are derived from
array shape, offset, and resolution.

| Method | Out of bounds | In bounds |
|--------|---------------|-----------|
| `atVoxel` | `VoxelOccupancy::OutOfBounds` | stored occupancy |
| `set` | no-op | writes occupancy |
| `isInBounds` | `false` | `true` |

## dtype rules — the instructor maps are NOT all the same dtype

Verified from the `.npy` headers and raw byte histograms in `inputs/map/`:

| File | Shape | dtype | Distinct stored values |
|------|-------|-------|------------------------|
| `scenario_small.npy` | `(20, 20, 20)` | **`int8`** (`\|i1`) | `0`, `1` |
| `scenario_big.npy` | `(30, 30, 30)` | **`int8`** (`\|i1`) | `0`, `1` |
| `scenario_house.npy` | `(29, 30, 31)` | **`uint8`** (`\|u1`) | `0`, `1`, `2`, `3`, `4`, `18`, `45` |

So the reader must **not** assume hidden maps are `uint8`. Dispatch on dtype (TinyNPY `NpyArray::Type()`
returns `'u'` vs `'i'`), and for **hidden/input** maps clamp any value `>= 1` to `Occupied` regardless of
dtype — course staff confirmed the extra values (`2`, `18`, `45`, …) in the house map still mean Occupied.

**Mutable output** maps are ours: write `int8` and store the full `VoxelOccupancy` enum (`-3` … `1`).
`VoxelOccupancy` is now explicitly `: signed char` in ex3, so a plain byte write is exact. On read of an
output map, an unrecognized positive value maps to `Unmapped`.

The trap: applying the output-map read rule to `scenario_small.npy` / `scenario_big.npy` happens to work
(they only contain `0`/`1`), while applying it to a future `int8` hidden map containing `3` would silently
turn walls into `Unmapped`. Key the behavior on **map role** (hidden vs output), passed in at
construction — not on dtype alone.

Regression tests worth keeping from ex2:

- `int8` hidden map with values `> 1` reads as `Occupied`
- `uint8` hidden map with values `2`, `3`, `4`, `18`, `45` reads as `Occupied`
- `int8` output map `-1` stays `Unmapped` (not clamped through an unsigned path)

## Instructor scenarios

`inputs/` is byte-identical to ex2's vendored instructor set, so the ex2 spawn fixes and boundary
handling still apply.

| Map | Used by | Notes |
|-----|---------|-------|
| `scenario_house.npy` | `house_simulation.yaml` | `map_axes_offset.height_offset: 150`; spawn `height_cm: 10` → world z 160 |
| `scenario_small.npy` | `small_simulation_room.yaml`, `small_simulation_out.yaml` | |
| `scenario_big.npy` | `large_simulation_room.yaml`, `large_simulation_out.yaml` | |

All simulation configs use `map_resolution_cm: 10`. `inputs/sim_compose.yaml` expands to
6 (simulation, mission) pairs × 2 drones × 2 lidars = **24 runs**.

### Mission boundaries are offset-relative (ex2 bug, do not regress)

Mission boundaries in `mission_config` YAML are in **local map coordinates**. When building the output
map config, add `simulation.map_axes_offset` to all six boundary values. For the house scenario
(`height_offset: 150`) this shifts the output map's world z from `[0, 60]` to `[150, 210]` — without it,
drone scan writes land outside the output map and the score collapses. The scorer must likewise be seeded
with the **world** spawn position (local + offset).

## `.cw` files

`inputs/map/*.cw` are gzipped NBT ClassicWorld exports produced by `inputs/map/npy_to_cw.py`, for
viewing maps in ClassiCube. Confirmed to be the same maps (`scenario_small.cw` = 20³,
`scenario_big.cw` = 30³, `benchmark_map.cw` = the house map with axes reordered to ClassicWorld's
`(Y, Z, X)` packing). Every simulation YAML still points at a `.npy`, so nothing needs to read `.cw` —
see `docs/open-questions.md`.

## Writing `.npy` test fixtures

```python
import numpy as np
arr = np.zeros((nx, ny, nz), dtype=np.uint8)   # or int8 for output-map fixtures
arr[ix, iy, iz] = 1
np.save("my_test_map.npy", arr)
```
