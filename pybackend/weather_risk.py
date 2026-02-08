from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Literal, Tuple

import math


Profile = Literal["safety", "comfort"]


def _clamp01(x: float) -> float:
    if x < 0.0:
        return 0.0
    if x > 1.0:
        return 1.0
    return float(x)


def _lin01(x: float, x0: float, x1: float) -> float:
    """0 at x<=x0, 1 at x>=x1, linear in between."""
    if x1 <= x0:
        return 1.0 if x >= x1 else 0.0
    return _clamp01((x - x0) / (x1 - x0))


def _gauss01(x: float, sigma: float) -> float:
    """Gaussian peak 1 at x=0, decays with sigma."""
    if sigma <= 0:
        return 0.0
    return float(math.exp(-0.5 * (x / sigma) ** 2))


DEFAULT_HOURLY_VARS: List[str] = [
    # Precip / wet
    "precipitation",
    "rain",
    "snowfall",
    "precipitation_probability",
    # Convective
    "weather_code",
    "cape",
    # Wind
    "wind_speed_10m",
    "wind_gusts_10m",
    "wind_direction_10m",
    # Visibility / icing proxy
    "visibility",
    "temperature_2m",
]


def score_point(
    weather: Dict[str, float],
    *,
    profile: Profile = "safety",
) -> Tuple[float, Dict[str, float], List[Dict[str, object]]]:
    """
    Returns:
      - total (0..100)
      - components (0..1 each)
      - drivers (top contributors)

    NOTE: This is a heuristic scoring model for decision-support UI, not meteorological truth.
    """
    # Pull values with defaults
    precip = float(weather.get("precipitation", 0.0) or 0.0)  # mm/h
    pop = float(weather.get("precipitation_probability", 0.0) or 0.0)  # %
    rain = float(weather.get("rain", 0.0) or 0.0)
    snowfall = float(weather.get("snowfall", 0.0) or 0.0)

    wcode = int(weather.get("weather_code", 0) or 0)
    cape = float(weather.get("cape", 0.0) or 0.0)  # J/kg

    wind = float(weather.get("wind_speed_10m", 0.0) or 0.0)  # unit depends on API; we treat as magnitude
    gust = float(weather.get("wind_gusts_10m", 0.0) or 0.0)

    vis = float(weather.get("visibility", 99999.0) or 99999.0)  # meters
    temp = float(weather.get("temperature_2m", 20.0) or 20.0)  # °C

    # Profile-specific sensitivity (lower thresholds => more sensitive)
    if profile == "comfort":
        wind_s = max(_lin01(wind, 6.0, 12.0), _lin01(gust, 10.0, 18.0))
        precip_s = max(_lin01(precip, 0.2, 2.0), _lin01(rain, 0.2, 2.0), _lin01(snowfall, 0.1, 1.0))
        pop_s = _lin01(pop, 20.0, 70.0)
        vis_s = _lin01(8000.0 - vis, 0.0, 4000.0)  # starts degrading below 8km
        cape_s = _lin01(cape, 200.0, 900.0)
        icing_sigma = 3.0
        weights = {
            "convective": 0.30,
            "wind": 0.30,
            "precip": 0.15,
            "visibility": 0.15,
            "icing_proxy": 0.10,
        }
    else:
        # safety (more conservative on low vis / high precip / thunderstorm)
        wind_s = max(_lin01(wind, 10.0, 20.0), _lin01(gust, 15.0, 30.0))
        precip_s = max(_lin01(precip, 0.5, 5.0), _lin01(rain, 0.5, 5.0), _lin01(snowfall, 0.2, 2.0))
        pop_s = _lin01(pop, 30.0, 80.0)
        vis_s = _lin01(5000.0 - vis, 0.0, 4000.0)  # starts degrading below 5km; 1 around 1km
        cape_s = _lin01(cape, 400.0, 1200.0)
        icing_sigma = 2.5
        weights = {
            "convective": 0.30,
            "wind": 0.20,
            "precip": 0.20,
            "visibility": 0.20,
            "icing_proxy": 0.10,
        }

    # Convective: thunderstorm weather codes dominate
    thunderstorm = 1.0 if wcode in (95, 96, 99) else 0.0
    convective_s = max(thunderstorm, cape_s)

    # Precip: combine intensity + probability
    precip_s = _clamp01(max(precip_s, 0.6 * pop_s))

    # Icing proxy: near 0C and precip present (heuristic)
    icing_temp_factor = _gauss01(temp - 0.0, icing_sigma)
    icing_s = _clamp01(icing_temp_factor * precip_s)

    components = {
        "precip": float(precip_s),
        "convective": float(convective_s),
        "wind": float(wind_s),
        "visibility": float(_clamp01(vis_s)),
        "icing_proxy": float(icing_s),
    }

    contrib = {k: components[k] * weights[k] for k in weights}
    total01 = _clamp01(sum(contrib.values()))
    total = float(round(total01 * 100.0, 2))

    # Drivers: top 3 contributors, include a short evidence snapshot
    drivers = sorted(contrib.items(), key=lambda kv: kv[1], reverse=True)[:3]
    driver_objs: List[Dict[str, object]] = []
    for k, c in drivers:
        ev: Dict[str, object] = {"component": k, "contribution": float(round(c * 100.0, 2)), "score": float(round(components[k], 3))}
        if k == "wind":
            ev.update({"wind_speed_10m": wind, "wind_gusts_10m": gust})
        elif k == "precip":
            ev.update({"precipitation": precip, "precipitation_probability": pop, "rain": rain, "snowfall": snowfall})
        elif k == "convective":
            ev.update({"weather_code": wcode, "cape": cape})
        elif k == "visibility":
            ev.update({"visibility": vis})
        elif k == "icing_proxy":
            ev.update({"temperature_2m": temp, "precipitation": precip})
        driver_objs.append(ev)

    return total, components, driver_objs

