from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import pandas as pd
import numpy as np


@dataclass(frozen=True)
class LocationHourly:
    lat: float
    lon: float
    hourly: pd.DataFrame  # columns: ["time", *variables]
    units: Dict[str, str]  # variable -> unit (best-effort)


def _ensure_list(x: Sequence[float] | float) -> List[float]:
    if isinstance(x, (list, tuple)):
        return list(x)
    return [float(x)]


def fetch_hourly(
    *,
    latitudes: Sequence[float] | float,
    longitudes: Sequence[float] | float,
    hourly_vars: Sequence[str] | str,
    forecast_days: int = 1,
    timezone_name: str = "UTC",
    cache_dir: str = ".cache",
    cache_expire_seconds: int = 3600,
    retries: int = 5,
    backoff_factor: float = 0.2,
) -> List[LocationHourly]:
    """
    Fetch hourly data for one or more locations using openmeteo_requests with cache+retry.

    This follows Open-Meteo's recommended client pattern:
    - requests_cache.CachedSession(cache_dir, expire_after=...)
    - retry_requests.retry(session, retries=..., backoff_factor=...)
    - openmeteo_requests.Client(session=...)

    Returns one LocationHourly per location, in the same order as the returned responses.
    """
    # Lazy imports so that unit tests (and environments without deps) can still import this module.
    try:
        import openmeteo_requests
        import requests_cache
        from retry_requests import retry
    except Exception as e:  # pragma: no cover
        raise RuntimeError(
            "Missing Open-Meteo client dependencies. Install: openmeteo-requests, requests-cache, retry-requests"
        ) from e

    lats = _ensure_list(latitudes)
    lons = _ensure_list(longitudes)
    if len(lats) != len(lons):
        raise ValueError("latitudes and longitudes must have the same length")

    if isinstance(hourly_vars, str):
        vars_list = [v.strip() for v in hourly_vars.split(",") if v.strip()]
    else:
        vars_list = [str(v).strip() for v in hourly_vars if str(v).strip()]
    if not vars_list:
        raise ValueError("hourly_vars must contain at least one variable")

    cache_session = requests_cache.CachedSession(cache_dir, expire_after=cache_expire_seconds)
    retry_session = retry(cache_session, retries=retries, backoff_factor=backoff_factor)
    openmeteo = openmeteo_requests.Client(session=retry_session)

    url = "https://api.open-meteo.com/v1/forecast"
    params = {
        "latitude": lats,
        "longitude": lons,
        "hourly": ",".join(vars_list),
        "forecast_days": int(forecast_days),
        "timezone": timezone_name,
    }

    responses = openmeteo.weather_api(url, params=params)

    out: List[LocationHourly] = []
    for response in responses:
        hourly = response.Hourly()

        times = pd.date_range(
            start=pd.to_datetime(hourly.Time(), unit="s", utc=True),
            end=pd.to_datetime(hourly.TimeEnd(), unit="s", utc=True),
            freq=pd.Timedelta(seconds=hourly.Interval()),
            inclusive="left",
        )

        data: Dict[str, object] = {"time": times}
        for idx, var in enumerate(vars_list):
            data[var] = hourly.Variables(idx).ValuesAsNumpy()

        df = pd.DataFrame(data=data)

        units: Dict[str, str] = {}
        # Best-effort units extraction (API wrapper differs by version; keep non-fatal)
        try:  # pragma: no cover
            hourly_units = response.HourlyUnits()
            for idx, var in enumerate(vars_list):
                units[var] = str(hourly_units.Variables(idx))
        except Exception:
            units = {}

        out.append(
            LocationHourly(
                lat=float(response.Latitude()),
                lon=float(response.Longitude()),
                hourly=df,
                units=units,
            )
        )

    return out


def pick_time_index(
    times: pd.Series | pd.DatetimeIndex,
    target_time: Optional[datetime],
    *,
    mode: str = "nearest",
) -> int:
    if isinstance(times, pd.Series):
        t = pd.DatetimeIndex(times)
    else:
        t = times

    if target_time is None:
        target_time = datetime.now(timezone.utc)

    ts = pd.Timestamp(target_time)
    target = ts.tz_convert("UTC") if ts.tzinfo else ts.tz_localize("UTC")

    if mode == "nearest":
        # TimedeltaIndex does not have .abs() in some pandas versions; use numpy.
        deltas = (t - target).to_numpy(dtype="timedelta64[ns]")
        return int(np.argmin(np.abs(deltas)))
    if mode == "floor":
        idx = int((t <= target).to_numpy().nonzero()[0].max()) if (t <= target).any() else 0
        return idx
    if mode == "ceil":
        idxs = (t >= target).to_numpy().nonzero()[0]
        return int(idxs.min()) if len(idxs) else len(t) - 1
    raise ValueError("mode must be one of: nearest, floor, ceil")

