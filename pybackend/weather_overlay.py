from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Sequence, Set, Tuple


@dataclass(frozen=True)
class TrajectoryPoint:
    lat: float
    lon: float


def cell_center_from_latlon(lat: float, lon: float, cell_deg: float) -> Tuple[float, float]:
    """
    Quantize (lat, lon) into a cell index of size cell_deg and return the cell center.
    Using an index-based approach keeps behavior stable for negatives.
    """
    if cell_deg <= 0:
        raise ValueError("cell_deg must be > 0")
    # Guard against floating point boundary issues (e.g. 37.51/0.01 becoming 3750.999999...).
    # The epsilon is tiny relative to the grid size, only intended to stabilize exact-grid inputs.
    eps = cell_deg * 1e-9
    i = int(math.floor((lat + eps) / cell_deg))
    j = int(math.floor((lon + eps) / cell_deg))
    return ((i + 0.5) * cell_deg, (j + 0.5) * cell_deg)


def densify_trajectory(points: Sequence[TrajectoryPoint], cell_deg: float) -> List[TrajectoryPoint]:
    """
    Add intermediate points along segments so that the step in lat/lon does not skip cells.
    This is a simple linear densification in (lat, lon) space (good enough for small steps like 0.01°).
    """
    if len(points) <= 1:
        return list(points)

    out: List[TrajectoryPoint] = [points[0]]
    for a, b in zip(points, points[1:]):
        dlat = b.lat - a.lat
        dlon = b.lon - a.lon
        steps = int(max(abs(dlat), abs(dlon)) / cell_deg)
        if steps <= 0:
            out.append(b)
            continue
        # Insert steps points between, excluding a and including b at the end.
        for k in range(1, steps + 1):
            t = k / (steps + 1)
            out.append(TrajectoryPoint(lat=a.lat + dlat * t, lon=a.lon + dlon * t))
        out.append(b)
    return out


def cells_from_trajectory(
    points: Sequence[TrajectoryPoint],
    *,
    cell_deg: float = 0.01,
    densify: bool = True,
) -> List[Tuple[float, float]]:
    """
    Convert a trajectory into unique cell centers.
    """
    if not points:
        return []

    pts = densify_trajectory(points, cell_deg) if densify else list(points)
    seen: Set[Tuple[float, float]] = set()
    ordered: List[Tuple[float, float]] = []
    for p in pts:
        c = cell_center_from_latlon(p.lat, p.lon, cell_deg)
        if c not in seen:
            seen.add(c)
            ordered.append(c)
    return ordered


def corridor_expand(
    cells: Sequence[Tuple[float, float]],
    *,
    cell_deg: float,
    radius_cells: int,
) -> List[Tuple[float, float]]:
    """
    Expand a set of cell centers by a Chebyshev radius (square around each cell).
    Useful for making the overlay look like a heatmap around the trajectory.
    """
    if radius_cells <= 0:
        return list(cells)

    seen: Set[Tuple[float, float]] = set()
    out: List[Tuple[float, float]] = []
    for latc, lonc in cells:
        i = int(math.floor(latc / cell_deg))
        j = int(math.floor(lonc / cell_deg))
        for di in range(-radius_cells, radius_cells + 1):
            for dj in range(-radius_cells, radius_cells + 1):
                lat2 = (i + di + 0.5) * cell_deg
                lon2 = (j + dj + 0.5) * cell_deg
                c = (lat2, lon2)
                if c not in seen:
                    seen.add(c)
                    out.append(c)
    return out

