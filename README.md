# ConstellationSim

C++ satellite constellation propagation and trade-study framework. Runs single simulations or parallel Monte Carlo sweeps. Optional OpenGL visualizer with real-time playback control.

## Build

```bash
cmake -B build
cmake --build build -j$(nproc)
```

With visualization (requires VGL):
```bash
cmake -B build -DBUILD_VIZ=ON
cmake --build build -j$(nproc)
```

## Running a Simulation

```bash
./build/ConstellationSim config/earth_observation.json
./build/ConstellationSim config/earth_observation.json --visualize
```

**Visualizer controls:**
| Key / Mouse | Action |
|---|---|
| `Space` | Pause / resume |
| `1` `2` `3` `4` | Speed: 1×, 10×, 100×, 1000× |
| `+` / `-` | Step speed up / down |
| Left-drag | Orbit camera |
| Right-drag | Pan |
| Scroll | Zoom |

## Running a Monte Carlo Sweep

```bash
./build/ConstellationSim montecarlo/starlink_sweep.json
```

Results are written to `output/<experiment_name>/` as CSV.

---

## Creating a Scenario

### Single simulation — `config/my_scenario.json`

```json
{
  "simulation": {
    "name": "My Constellation",
    "duration_days": 3,
    "timestep_s": 30,
    "epoch_jd": 2451545.0
  },
  "constellation": {
    "type": "walker",
    "altitude_km": 550,
    "inclination_deg": 53,
    "total_satellites": 1584,
    "planes": 72,
    "phasing_factor": 13
  },
  "satellite": {
    "mass_kg": 260,
    "drag_coefficient": 2.2,
    "drag_area_m2": 2.0,
    "reflectivity": 1.3,
    "srp_area_m2": 2.0
  },
  "forces": {
    "gravity": true,
    "j2": true,
    "drag": true,
    "srp": true
  },
  "metrics": {
    "coverage": {
      "enabled": true,
      "grid_resolution_deg": 2,
      "min_elevation_deg": 25,
      "sample_interval_s": 300
    },
    "sunlight": true,
    "drag": true,
    "delta_v": true,
    "revisit": true
  },
  "output": {
    "directory": "output",
    "run_name": "my_constellation"
  }
}
```

### Monte Carlo sweep — `montecarlo/my_sweep.json`

```json
{
  "experiment": {
    "name": "altitude_sweep",
    "sampling": "grid",
    "threads": 0
  },
  "base_config": { ... },
  "sweep": {
    "altitude_km":     [400, 500, 600],
    "inclination_deg": [53, 70, 97],
    "planes":          [24, 48, 72]
  },
  "output": {
    "directory": "output",
    "experiment_name": "altitude_sweep"
  }
}
```

`"sampling": "grid"` runs every combination. `"sampling": "random"` samples `runs` points at random. Set `"threads": 0` to use all CPU cores.

**Sweepable parameters:** `altitude_km`, `inclination_deg`, `planes`, `sats_per_plane`, `total_satellites`, `phasing_factor`, `mass_kg`, `drag_coefficient`, `drag_area_m2`, `timestep_s`, `duration_days`, `min_elevation_deg`

---

## Output

Each run produces two CSV files inside `output/<run_name>/`:

| File | Contents |
|---|---|
| `summary.csv` | Coverage %, avg revisit time, drag ΔV, orbital lifetime |
| `satellites.csv` | Per-satellite SMA, eccentricity, inclination, drag ΔV, eclipse fraction |

Monte Carlo runs also produce `experiment_summary.csv` aggregating all runs.

## Physics

| Model | Notes |
|---|---|
| Two-body gravity | Central force, GM = 3.986004418×10¹⁴ m³/s² |
| J2 oblateness | First zonal harmonic, J2 = 1.08262668×10⁻³ |
| Atmospheric drag | USSA76, 23 layers to 700 km |
| Solar radiation pressure | Cannonball model, Vallado low-precision Sun position |

Integrator: RK4 with configurable timestep.
