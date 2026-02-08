import sys
import os
from datetime import datetime, timezone

import pandas as pd
from fastapi.testclient import TestClient
from unittest.mock import patch

# Add parent directory to path to import main
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from main import app  # noqa: E402


client = TestClient(app)


def test_weather_overlay_basic():
    # Fake hourly dataframe for two locations
    times = pd.date_range("2026-02-08T00:00:00Z", periods=3, freq="1h")
    df = pd.DataFrame(
        {
            "time": times,
            "surface_pressure": [1000.0, 1001.0, 1002.0],
        }
    )

    fake_loc_hourlies = [
        type(
            "LH",
            (),
            {"lat": 0.0, "lon": 0.0, "hourly": df, "units": {"surface_pressure": "hPa"}},
        )(),
        type(
            "LH",
            (),
            {"lat": 0.01, "lon": 0.01, "hourly": df, "units": {"surface_pressure": "hPa"}},
        )(),
    ]

    with patch("main.open_meteo_client.fetch_hourly", return_value=fake_loc_hourlies):
        resp = client.post(
            "/weather/overlay",
            json={
                "trajectory": [{"lat": 0.001, "lon": 0.001}, {"lat": 0.011, "lon": 0.011}],
                "hourly": ["surface_pressure"],
                "cell_deg": 0.01,
                "forecast_days": 1,
                "densify": False,
                "radius_cells": 0,
                "target_time": "2026-02-08T01:10:00Z",
                "time_mode": "nearest",
                "include_series": False,
            },
        )

    assert resp.status_code == 200
    data = resp.json()
    assert data["hourly"] == ["surface_pressure"]
    assert "surface_pressure" in data["stats"]
    assert len(data["cells"]) == 2
    # nearest time to 01:10 is 01:00 => value 1001.0
    for c in data["cells"]:
        assert c["values"]["surface_pressure"] == 1001.0
        # Default coord_mode is input_point: lat/lon should match input_* when available.
        assert c["lat"] == c["input_lat"]
        assert c["lon"] == c["input_lon"]
        assert c["cell_lat"] is not None
        assert c["cell_lon"] is not None


def test_weather_overlay_include_series():
    times = pd.date_range("2026-02-08T00:00:00Z", periods=2, freq="1h")
    df = pd.DataFrame({"time": times, "surface_pressure": [999.0, 1000.0]})
    fake_loc_hourlies = [
        type("LH", (), {"lat": 0.0, "lon": 0.0, "hourly": df, "units": {}})(),
    ]

    with patch("main.open_meteo_client.fetch_hourly", return_value=fake_loc_hourlies):
        resp = client.post(
            "/weather/overlay",
            json={
                "trajectory": [{"lat": 0.001, "lon": 0.001}],
                "hourly": ["surface_pressure"],
                "cell_deg": 0.01,
                "densify": False,
                "include_series": True,
            },
        )

    assert resp.status_code == 200
    data = resp.json()
    assert len(data["cells"]) == 1
    cell = data["cells"][0]
    assert cell["series"]["surface_pressure"] == [999.0, 1000.0]
    assert len(cell["series_time"]) == 2
    assert cell["lat"] == cell["input_lat"]
    assert cell["lon"] == cell["input_lon"]


def test_cell_quantization_boundary_does_not_collapse_adjacent_cells():
    # Regression: 37.51 / 0.01 can be represented as 3750.999999... and floor() would collapse it to 37.505.
    import weather_overlay as wo

    cell_deg = 0.01
    c1 = wo.cell_center_from_latlon(37.50, 126.90, cell_deg)
    c2 = wo.cell_center_from_latlon(37.51, 126.90, cell_deg)
    assert c1 != c2
    assert c1[0] == 37.505
    assert c2[0] == 37.515

