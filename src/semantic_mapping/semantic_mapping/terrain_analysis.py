"""Geometric terrain descriptors and a conservative traversability state.

Inputs are occupied voxel centres only; free and unobserved columns are never
filled. Heights use the map frame's Z axis, which is assumed (not verified) to
be gravity aligned, so every descriptor inherits tilt or drift of the recorded
poses. At 0.35 m voxels heights are quantized: one voxel layer is the smallest
resolvable obstacle and roughness has a quantization floor of about 0.1 m.
"""

from dataclasses import dataclass, fields

import numpy as np

from semantic_mapping.semantic_fusion import CLASS_NAMES, voxel_keys


# Ordered so that a larger code is a worse (or better-known) state.
STATES = ('unobserved', 'unknown', 'traversable', 'caution', 'non_traversable')


@dataclass(frozen=True)
class TerrainSettings:
    cell_m: float = .35
    ground_window_m: float = 1.5
    max_ground_rise_deg: float = 35.
    plane_half_width_m: float = 1.05
    min_plane_cells: int = 6
    ground_clearance_m: float = .3
    overhang_clearance_m: float = 2.
    max_slope_deg: float = 20.
    max_roughness_m: float = .2
    caution_fraction: float = .5
    compliant_vegetation_max_height_m: float = 1.
    vegetation_cost: float = .6
    rigid_labels: tuple = ('trunk', 'obstacle')
    hazard_labels: tuple = ('water',)
    # 'lowest': lowest voxel per column (original). 'cloth': cloth simulation
    # filter (Zhang et al. 2016) on the inverted lowest surface; a stiff cloth
    # bridges over low vegetation/understory that hides the ground.
    # Cost of a confirmed hazard label (water). 1 = non-traversable (original);
    # lower values treat camera water on LiDAR-returned surfaces as caution.
    hazard_cost: float = 1.
    ground_method: str = 'lowest'
    cloth_step_m: float = .05
    cloth_rigidness: int = 3
    cloth_iterations: int = 80
    cloth_threshold_m: float = .3

    def __post_init__(self):
        positive = ['cell_m', 'ground_window_m', 'plane_half_width_m', 'ground_clearance_m',
                    'overhang_clearance_m', 'max_slope_deg', 'max_roughness_m',
                    'compliant_vegetation_max_height_m']
        if any(not np.isfinite(getattr(self, name)) or getattr(self, name) <= 0 for name in positive):
            raise ValueError('Terrain distances, heights and limits must be positive')
        if not 0 < self.max_ground_rise_deg < 90 or not 0 < self.max_slope_deg < 90:
            raise ValueError('Angles must lie between 0 and 90 degrees')
        if self.min_plane_cells < 3:
            raise ValueError('A plane needs at least three cells')
        if not 0 < self.caution_fraction < 1 or not 0 <= self.vegetation_cost <= 1:
            raise ValueError('Costs are fractions of the non-traversable limit')
        if self.overhang_clearance_m <= self.ground_clearance_m:
            raise ValueError('Overhang clearance must exceed ground clearance')
        if not 0 < self.hazard_cost <= 1:
            raise ValueError('hazard_cost must be in (0, 1]')
        if self.ground_method not in ('lowest', 'cloth'):
            raise ValueError("ground_method must be 'lowest' or 'cloth'")
        if (self.cloth_step_m <= 0 or self.cloth_rigidness < 1 or self.cloth_iterations < 1
                or self.cloth_threshold_m <= 0):
            raise ValueError('Invalid cloth settings')
        unknown = set(self.rigid_labels) | set(self.hazard_labels)
        if not unknown <= set(CLASS_NAMES[1:]) or 'vegetation' in unknown:
            raise ValueError('Rigid/hazard labels must be confirmed non-vegetation classes')

    @classmethod
    def from_config(cls, config):
        """Flatten the YAML sections; reject misspelled keys instead of ignoring them."""
        values = {}
        for section in ('vehicle', 'limits', 'analysis', 'semantics'):
            values.update(config.get(section) or {})
        values.pop('footprint_radius_m', None)
        names = {f.name for f in fields(cls)}
        extra = set(values) - names
        if extra:
            raise ValueError(f'Unknown terrain settings: {sorted(extra)}')
        for key in ('rigid_labels', 'hazard_labels'):
            if key in values:
                values[key] = tuple(values[key])
        return cls(**values)


def cells(distance, cell):
    return max(1, int(round(distance/cell)))


def box_sum(values, r):
    """Sum over each cell's (2r+1)^2 neighbourhood; outside the grid counts as zero."""
    padded = np.pad(np.asarray(values, dtype=np.float64), r)
    integral = np.zeros((padded.shape[0]+1, padded.shape[1]+1))
    integral[1:, 1:] = padded.cumsum(0).cumsum(1)
    w = 2*r+1
    return integral[w:, w:]-integral[:-w, w:]-integral[w:, :-w]+integral[:-w, :-w]


def box_min(values, r):
    padded = np.pad(values, r, constant_values=np.inf)
    return np.lib.stride_tricks.sliding_window_view(padded, (2*r+1, 2*r+1)).min(axis=(-2, -1))


@dataclass
class TerrainMap:
    settings: TerrainSettings
    origin: np.ndarray            # integer XY key of cell [0, 0]
    observed: np.ndarray          # column contains at least one occupied voxel
    ground_measured: np.ndarray   # lowest voxel accepted as ground
    ground_z: np.ndarray          # measured, or mean of nearby measured ground; NaN if none
    gradient: np.ndarray          # dz/dx, dz/dy of the local ground plane
    slope_deg: np.ndarray
    roughness_m: np.ndarray       # plane-fit residual standard deviation
    obstacle_height_m: np.ndarray # tallest voxel between ground and overhang clearance
    rigid: np.ndarray
    hazard: np.ndarray
    compliant: np.ndarray         # geometric obstacle explained only by confirmed vegetation
    geometric_cost: np.ndarray
    combined_cost: np.ndarray
    geometric_state: np.ndarray
    combined_state: np.ndarray
    voxel_height_m: np.ndarray    # per input voxel, above its column's ground

    def footprint(self, xy, radius_m):
        """Grid indices whose centres lie within a horizontal disk around one world
        XY position, always including the cell that contains the position."""
        cell = self.settings.cell_m
        r = int(np.ceil(radius_m/cell))
        offsets = np.stack(np.meshgrid(np.arange(-r, r+1), np.arange(-r, r+1), indexing='ij'), -1).reshape(-1, 2)
        centre = voxel_keys(np.asarray(xy, dtype=np.float64)[None], cell)[0]
        keys = centre+offsets
        inside = (np.hypot(*((keys+.5)*cell-np.asarray(xy)[:2]).T) <= radius_m) | (offsets == 0).all(1)
        index = keys[inside]-self.origin
        index = index[((index >= 0) & (index < self.observed.shape)).all(1)]
        return tuple(index.T)

    def extent(self):
        """Matplotlib imshow extent of the transposed grid (rows are Y)."""
        cell = self.settings.cell_m
        low = self.origin*cell
        high = (self.origin+self.observed.shape)*cell
        return [low[0], high[0], low[1], high[1]]


def states_from_cost(cost, observed, settings):
    state = np.where(observed, STATES.index('unknown'), STATES.index('unobserved'))
    known = observed & np.isfinite(cost)
    state[known & (cost < settings.caution_fraction)] = STATES.index('traversable')
    state[known & (cost >= settings.caution_fraction)] = STATES.index('caution')
    state[known & (cost >= 1)] = STATES.index('non_traversable')
    return state.astype(np.uint8)


def cloth_surface(lowest, observed, r, settings):
    """Ground height from a cloth dropped onto the upside-down lowest surface.

    Particles start just above the local ground envelope, fall by one step per
    iteration, relax toward their neighbours (rigidness passes) and stop when
    they touch an observed column. Pits in the inverted surface (objects on the
    real ground) are bridged. Returns NaN outside observed columns' reach.
    """
    inverted = np.where(observed, -lowest, np.nan)
    start = -box_min(lowest, r)  # highest inverted value nearby = lowest real ground
    reach = np.isfinite(start)
    cloth = np.where(reach, start+2*settings.cloth_step_m, 0.)
    movable = reach.copy()
    # Neighbour counts are fixed: cells outside the reach never take part.
    padded_reach = np.pad(reach, 1)
    neighbours = (padded_reach[:-2, 1:-1].astype(int)+padded_reach[2:, 1:-1]
                  + padded_reach[1:-1, :-2]+padded_reach[1:-1, 2:])
    relax = movable & (neighbours > 0)
    history = [int((movable & observed).sum())]
    for _ in range(settings.cloth_iterations):
        cloth[movable] -= settings.cloth_step_m
        for _ in range(settings.cloth_rigidness):
            padded = np.pad(np.where(reach, cloth, 0.), 1)
            mean = (padded[:-2, 1:-1]+padded[2:, 1:-1]+padded[1:-1, :-2]+padded[1:-1, 2:])/np.maximum(neighbours, 1)
            cloth = np.where(relax, mean, cloth)
        with np.errstate(invalid='ignore'):
            hit = movable & observed & (cloth <= inverted)
        cloth[hit] = inverted[hit]
        movable &= ~hit
        relax &= ~hit
        history.append(int((movable & observed).sum()))
        # Stop when almost no particle lands any more (<0.2% of columns in 5
        # iterations): the rest bridge gaps (objects on the ground) and only sag.
        if not history[-1] or (len(history) > 5 and history[-6]-history[-1] < .002*history[0]):
            break
    cloth[~reach] = np.nan
    return -cloth


def analyze(centres, labels, settings=TerrainSettings(), compliant=None):
    """Compute column descriptors and geometry-only and geometry+semantic states.

    Semantics only change a column when class evidence is confirmed: rigid or
    hazard labels make it non-traversable, and a geometric obstacle consisting
    entirely of confirmed vegetation below the compliant height is relaxed to
    the vegetation cost. Unknown-label voxels keep the geometric decision.
    ``compliant`` optionally marks further voxels as vegetation for that
    relaxation (e.g. LiDAR-permeable low vegetation); it never overrides rigid
    or hazard labels.
    """
    centres = np.asarray(centres, dtype=np.float64).reshape(-1, 3)
    labels = np.asarray(labels).reshape(-1)
    if len(centres) == 0 or len(centres) != len(labels):
        raise ValueError('Need one label per occupied voxel centre')
    if not np.isfinite(centres).all():
        raise ValueError('Nonfinite voxel centres')
    s, cell = settings, settings.cell_m
    keys = voxel_keys(centres[:, :2], cell)
    origin = keys.min(0)
    ix, iy = (keys-origin).T
    shape = tuple(int(n) for n in (keys-origin).max(0)+1)
    z = centres[:, 2]

    lowest = np.full(shape, np.inf)
    np.minimum.at(lowest, (ix, iy), z)
    observed = np.isfinite(lowest)
    r = cells(s.ground_window_m, cell)
    if s.ground_method == 'cloth':
        surface = cloth_surface(lowest, observed, r, s)
        with np.errstate(invalid='ignore'):
            measured = observed & (lowest-surface <= s.cloth_threshold_m)
        # The cloth classifies; heights come from the accepted ground nearby
        # (the cloth itself sags slightly over wide gaps).
        ground = np.where(measured, lowest, np.nan)
        count = box_sum(measured, r)
        total = box_sum(np.where(measured, lowest, 0.), r)
        fill = observed & ~measured
        ground[fill] = np.where(count[fill] > 0, total[fill]/np.maximum(count[fill], 1), surface[fill])
    else:
        # A column's lowest voxel is ground unless it rises far above the local
        # lower envelope, e.g. canopy-only columns next to observed ground.
        tolerance = np.tan(np.radians(s.max_ground_rise_deg))*r*cell+cell
        with np.errstate(invalid='ignore'):  # inf-inf in unobserved columns
            measured = observed & (lowest-box_min(lowest, r) <= tolerance)
        ground = np.where(measured, lowest, np.nan)
        count = box_sum(measured, r)
        total = box_sum(np.where(measured, lowest, 0.), r)
        fill = observed & ~measured & (count > 0)
        ground[fill] = total[fill]/count[fill]

    # Least-squares plane z = a x + b y + c over measured ground in each window.
    rp = cells(s.plane_half_width_m, cell)
    x, y = np.meshgrid((np.arange(shape[0])+.5)*cell, (np.arange(shape[1])+.5)*cell, indexing='ij')
    m = measured.astype(np.float64)
    gz = np.where(measured, lowest, 0.)
    n = box_sum(m, rp)
    with np.errstate(invalid='ignore', divide='ignore'):
        mx, my, mz = box_sum(m*x, rp)/n, box_sum(m*y, rp)/n, box_sum(gz, rp)/n
        cxx = box_sum(m*x*x, rp)/n-mx*mx
        cxy = box_sum(m*x*y, rp)/n-mx*my
        cyy = box_sum(m*y*y, rp)/n-my*my
        cxz = box_sum(x*gz, rp)/n-mx*mz
        cyz = box_sum(y*gz, rp)/n-my*mz
        czz = box_sum(gz*gz, rp)/n-mz*mz
        det = cxx*cyy-cxy*cxy
        a = (cxz*cyy-cyz*cxy)/det
        b = (cyz*cxx-cxz*cxy)/det
        residual = czz-a*cxz-b*cyz
    plane = np.isfinite(ground) & (n >= s.min_plane_cells) & (det > 1e-3*cell**4)
    gradient = np.where(plane[..., None], np.stack([a, b], -1), np.nan)
    slope = np.where(plane, np.degrees(np.arctan(np.hypot(a, b))), np.nan)
    roughness = np.where(plane, np.sqrt(np.clip(residual, 0, None)), np.nan)

    height = z-ground[ix, iy]
    band = (height > .5*cell) & (height < s.overhang_clearance_m)
    obstacle = np.where(np.isfinite(ground), 0., np.nan)
    np.maximum.at(obstacle, (ix[band], iy[band]), height[band])

    rigid_ids = [CLASS_NAMES.index(name) for name in s.rigid_labels]
    hazard_ids = [CLASS_NAMES.index(name) for name in s.hazard_labels]
    rigid, hazard, unexplained = (np.zeros(shape, dtype=bool) for _ in range(3))
    selected = band & np.isin(labels, rigid_ids)
    rigid[ix[selected], iy[selected]] = True
    # Hazards include the ground layer itself (e.g. a water surface).
    selected = np.isfinite(height) & (height < s.overhang_clearance_m) & np.isin(labels, hazard_ids)
    hazard[ix[selected], iy[selected]] = True
    vegetation = labels == CLASS_NAMES.index('vegetation')
    if compliant is not None:
        compliant = np.asarray(compliant, dtype=bool).reshape(-1)
        if len(compliant) != len(labels):
            raise ValueError('Need one compliant flag per voxel')
        vegetation = vegetation | (compliant & ~np.isin(labels, rigid_ids+hazard_ids))
    selected = band & (height > s.ground_clearance_m) & ~vegetation
    unexplained[ix[selected], iy[selected]] = True
    compliant = ((obstacle > s.ground_clearance_m) & ~unexplained
                 & (obstacle <= s.compliant_vegetation_max_height_m))

    slope_term = slope/s.max_slope_deg
    terms = [slope_term, roughness/s.max_roughness_m, obstacle/s.ground_clearance_m]
    geometric = np.clip(np.maximum.reduce(terms), 0, 1)
    terms[2] = np.where(compliant, s.vegetation_cost, terms[2])
    terms.append(np.maximum(np.where(rigid, 1., 0.), np.where(hazard, s.hazard_cost, 0.)))
    combined = np.clip(np.maximum.reduce(terms), 0, 1)
    return TerrainMap(s, origin, observed, measured, ground, gradient, slope, roughness,
                      obstacle, rigid, hazard, compliant, geometric, combined,
                      states_from_cost(geometric, observed, s), states_from_cost(combined, observed, s),
                      height)


def carpet(terrain, combined=True):
    """Column centres at their ground height, for display; columns without a
    ground reference are left out rather than placed at an invented height."""
    index = np.nonzero(np.isfinite(terrain.ground_z))
    xy = (np.column_stack(index)+terrain.origin+.5)*terrain.settings.cell_m
    xyz = np.column_stack([xy, terrain.ground_z[index]])
    state = (terrain.combined_state if combined else terrain.geometric_state)[index]
    cost = (terrain.combined_cost if combined else terrain.geometric_cost)[index]
    return xyz, state, cost, terrain.slope_deg[index]


def path_states(terrain, positions, radius_m, combined=True):
    """Worst state within each pose's footprint (unobserved if nothing is mapped)."""
    state = terrain.combined_state if combined else terrain.geometric_state
    worst = []
    for xy in np.asarray(positions, dtype=np.float64)[:, :2]:
        index = terrain.footprint(xy, radius_m)
        worst.append(int(state[index].max()) if len(index[0]) else STATES.index('unobserved'))
    return np.array(worst, dtype=np.uint8)


def vehicle_consistency(terrain, world_vehicle, radius_m):
    """Vehicle origin height above mapped ground and tilt against the ground normal.

    The vehicle and the map share the same recorded poses, so this checks
    internal consistency, not absolute gravity alignment or odometry accuracy.
    """
    heights, angles = [], []
    for pose in np.asarray(world_vehicle, dtype=np.float64):
        index = terrain.footprint(pose[:2, 3], radius_m)
        ground = terrain.ground_z[index]
        gradient = terrain.gradient[index]
        heights.append(np.nanmean(ground) if np.isfinite(ground).any() else np.nan)
        valid = np.isfinite(gradient).all(-1)
        if not valid.any():
            angles.append(np.nan)
            continue
        a, b = gradient[valid].mean(0)
        normal = np.array([-a, -b, 1.])/np.sqrt(a*a+b*b+1)
        up = pose[:3, 2]/np.linalg.norm(pose[:3, 2])
        angles.append(np.degrees(np.arccos(np.clip(up @ normal, -1, 1))))
    heights = np.asarray(world_vehicle)[:, 2, 3]-np.array(heights)
    return heights, np.array(angles)


def settings_for_run(config, resolution):
    """Terrain settings from a parsed YAML config, with the cell size of the map it analyses."""
    from dataclasses import replace
    return replace(TerrainSettings.from_config(config), cell_m=float(resolution))
