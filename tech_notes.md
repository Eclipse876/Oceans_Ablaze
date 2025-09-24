# Modern Naval RTS — Technical Notes

## Overview
Top-down RTS inspired by *Sea Power: Naval Warfare in the Missile Age*.  
Two play modes:
- **Skirmish Set**: Player picks ships, formations, and fights AI.
- **Skirmish Dynamic**: Base building, research, ship construction, objective is to destroy opponent’s base.

Core ship classes: Carrier, Cruiser, Destroyer, Frigate, Patrol, Submarine, Gunboat, Cargo.  
Weapons: Anti-ship missiles, Anti-air missiles, Torpedoes, Rocket-assisted torpedoes, Cannons, CIWS, Machine guns.

---

## Ship Representation

### `ShipSpec` (defined in `config.py`)
```python
@dataclass
class ShipSpec:
    name: str
    length_m: float        # hull length
    beam_m: float          # hull width
    radius_m: float        # effective clearance radius for pathfinding
    safety_m: float        # extra buffer around hull
    max_speed_mps: float   # forward speed (m/s)
    max_accel_mps2: float  # longitudinal acceleration
    max_lat_accel_mps2: float  # lateral grip for turning
```

### Current presets
- **Frigate**: 150×20 m, radius 20 m, safety 15 m, ~18 m/s max
- **Destroyer**: 170×22 m, radius 22 m, safety 20 m, ~20 m/s max
- **Cruiser**: 200×25 m, radius 25 m, safety 20 m, ~22 m/s max
- **Carrier**: 300×40 m, radius 40 m, safety 30 m, ~16 m/s max
- **Patrol boat**: 50×10 m, radius 10 m, safety 10 m, ~28 m/s max
- **Submarine**: 120×12 m (approx), radius 12 m, safety 12 m

### `Ship` wrapper (in `app.py`)
```python
@dataclass
class Ship:
    spec: ShipSpec
    pos: Vec2
    vel: Vec2
```

---

## Movement Logic (`app.py`)

- **Always max speed** on intermediate waypoints.  
- **Slowdown only on final waypoint** (based on `SLOW_RADIUS_FINAL`).  
- **Pass-through** logic:
  - Pop waypoint if within `WAYPOINT_RADIUS`.
  - Or if the ship has passed beyond it along the path vector.  
- **Acceleration clamped** by `spec.max_accel_mps2 * dt`.  
- **Velocity capped** at `spec.max_speed_mps`.  
- **Final arrival**: if within stopping distance, snap to waypoint and zero velocity.  
- **Idle damping**: when no path, velocity gradually decays using accel clamp.

### Visuals
- Ship icons drawn as circles, scaled to `beam_m / 2`.  
- Velocity vector and travel line rendered thicker/brighter (white, width=7).  

---

## Pathfinding (`game/nav`)

### Grid & Clearance
- `GridSpec` defines world grid (cell size, dimensions, origin).
- `Grid` holds `water` mask (True=water).
- **Clearance mask** computed via distance field (`clearance_mask_for_ship`):
  - Obstacles = land.
  - Ship traversable if distance-to-land ≥ (radius_m + safety_m).
- Each ship has unique clearance (large ships stay further away).

### Algorithms
- **A* on boolean grid** (`astar.py`).
- **Hierarchical pathfinding** via clusters/portals (`hpa.py`) [not yet fully integrated].
- **Planner** (`planner.py`):
  - Converts world → grid cells.
  - Uses clearance mask for given ship.
  - Runs A* to get path.
  - **Smoothing** with string-pull LOS (`smooth.py`), sampling denser (`cell/5`).
  - **Waypoint decimation** tightened to ~12 m min spacing.

### Improvements made
- Ships **snap** start/goal to nearest traversable cell if blocked.
- Smoothing now avoids “threading the needle” by checking LOS every ~½ cell.
- More waypoints retained to prevent risky shortcuts.

---

## Runtime Safety

### LOS check to current waypoint
- Each frame, the ship checks if the straight line to its **current waypoint** is still clear (radius+safety).  
- If blocked, **reroute** immediately to the final goal.

### Coast repulsion
- Precomputed distance-to-land field in planner.
- Gradient sampled to get outward normal.  
- When within `(radius + safety + 25 m)`, desired velocity biased away from land.  
- Strength ramps up to full push inside `(radius + safety + 5 m)`.

### Proactive reroute (buffer inflation)
- If ship drifts within `(radius + safety + 10 m)` of land:
  - Trigger reroute to final goal with **inflated safety (+15 m)**.  
  - Ensures new path gives wider berth to coastline.

---

## Known Issues & Next Steps

- **Close passes near sharp corners** still possible, though ship no longer clips land.  
  - Possible mitigations:  
    - Reduce `cell_m` (finer nav grid, e.g. 4.0 m).  
    - Increase proactive safety inflation.  
    - Tune repulsion scaling.  
- **No avoidance of other ships yet** — only land is considered.  
- **Weapons, combat, and AI fleets** not yet integrated.  
- **Multiple ships**: structure is ready (`Ship` class, specs), but only one ship is actively controlled.

---

## Development History
- Initial: hardcoded movement & constants in `app.py`.
- Refactored into `ShipSpec` + `Ship` wrapper for per-class behaviors.
- Fixed vector math (`.normalized()` vs `normalize()` confusion in pyglet).
- Fixed waypoint logic (`is_final == 1`, not `0`).
- Brightened visuals for clarity.
- Integrated clearance-aware pathfinding and LOS-based replanning.
- Added coast repulsion & proactive reroute to avoid sticky cornering.

---
