import os
import joblib
import pandas as pd
import numpy as np
from fastapi import FastAPI, HTTPException
from google.cloud import bigquery
from pydantic import BaseModel
from typing import Optional, List, Dict, Any, Literal
from typing import Optional, List, Dict, Any, Literal
from contextlib import asynccontextmanager
from datetime import datetime, timezone
import math

import open_meteo_client
import weather_overlay
import weather_risk

try:
    import openap  # type: ignore
except Exception:  # pragma: no cover
    openap = None

class PredictionResponse(BaseModel):
    icao: str
    phase: str
    features: List[Dict[str, Any]] # Changed to return list of features used
    confidence: Optional[float] = None


class TrajectoryPoint(BaseModel):
    lat: float
    lon: float
    t: Optional[datetime] = None


class WeatherOverlayRequest(BaseModel):
    trajectory: List[TrajectoryPoint]
    hourly: List[str] = ["surface_pressure"]
    forecast_days: int = 1
    cell_deg: float = 0.01
    # If True, inserts intermediate points between trajectory points so intermediate cells are included.
    # Default is False to return only the cells corresponding to the provided points.
    densify: bool = False
    radius_cells: int = 0  # expand around the trajectory for a heatmap-like look
    target_time: Optional[datetime] = None  # UTC recommended
    time_mode: Literal["nearest", "floor", "ceil"] = "nearest"
    include_series: bool = False  # if True, also return the full hourly series per cell
    risk_profile: Literal["safety", "comfort"] = "safety"
    include_risk_details: bool = False
    # Controls what lat/lon in the response represents.
    # - input_point: return the original input lat/lon (best for frontend), and include cell_lat/cell_lon for grid overlay.
    # - cell_center: return the quantized cell center as lat/lon (legacy behavior), input_lat/input_lon still provided when available.
    coord_mode: Literal["input_point", "cell_center"] = "input_point"


class WeatherCell(BaseModel):
    lat: float
    lon: float
    # If available, the original trajectory point that produced this cell (only reliable when densify=False and radius_cells=0).
    input_lat: Optional[float] = None
    input_lon: Optional[float] = None
    # Quantized cell center coordinates (useful for heatmap grid rendering even when lat/lon are input points).
    cell_lat: Optional[float] = None
    cell_lon: Optional[float] = None
    time: datetime
    values: Dict[str, float]
    series: Optional[Dict[str, List[float]]] = None
    series_time: Optional[List[datetime]] = None
    risk_score: Optional[float] = None
    risk_level: Optional[str] = None
    risk_color: Optional[str] = None
    risk_components: Optional[Dict[str, float]] = None
    risk_drivers: Optional[List[Dict[str, Any]]] = None


class WeatherOverlayResponse(BaseModel):
    cell_deg: float
    hourly: List[str]
    forecast_days: int
    time_mode: str
    target_time: datetime
    units: Dict[str, str] = {}
    stats: Dict[str, Dict[str, float]] = {}
    cells: List[WeatherCell]


class CO2RecommendRequest(BaseModel):
    start_lat: float
    start_lon: float
    end_lat: float
    end_lon: float

    n_perturbations: int = 15
    n_points: int = 10
    seed0: int = 42
    max_offset_km: float = 1.5
    smooth_window: int = 9
    wind_level: str = "300hPa"
    eval_time_utc: Optional[datetime] = None

    # Aircraft / emissions (cruise-only simplification)
    aircraft: str = "A320"
    cruise_alt_ft: float = 35000
    cruise_tas_kt: float = 450
    mass_kg: float = 65000
    co2_kg_per_kg_fuel: float = 3.16


class CO2Trajectory(BaseModel):
    name: str
    points: List[TrajectoryPoint]
    distance_nm: float
    time_hr: float
    fuel_kg: float
    co2_kg: float


class CO2RecommendResponse(BaseModel):
    best_name: str
    best_points: List[TrajectoryPoint]
    co2_best_kg: float
    co2_geodesic_kg: float
    co2_reduction_kg: float
    eval_time_utc: datetime
    wind_level: str
    trajectories: Optional[List[CO2Trajectory]] = None

# Global variables for model and scaler
model = None
scaler = None
bq_client = None

# Configuration
PROJECT_ID = "iitp-class-team-5-473114"
DATASET_ID = "Dataset"
TABLE_ID = "FirstRun"  # Or "FirstRun_merged_filtered_added" depending on your live data source
# Assumes BigQueryCred.json is in the BigQuery directory relative to pybackend
CREDENTIALS_PATH = os.path.join(os.path.dirname(__file__), "..", "BigQuery", "YourJsonFile.json") 

SEQUENCE_LENGTH = 30 # Time steps required by the model
FEATURE_COLUMNS = ['Altitude', 'GroundSpeed', 'VerticalRate', 'Track']

@asynccontextmanager
async def lifespan(app: FastAPI):
    global model, scaler, bq_client
    
    # Load Model and Scaler
    try:
        current_dir = os.path.dirname(__file__)
        model_path = os.path.join(current_dir, "flight_phase_rf_model.joblib")
        scaler_path = os.path.join(current_dir, "scaler.joblib")
        
        model = joblib.load(model_path)
        scaler = joblib.load(scaler_path)
        print("✅ Model and Scaler loaded successfully.")
    except Exception as e:
        print(f"❌ Failed to load model/scaler: {e}")
        # In production, you might want to raise e to stop startup, 
        # but for now we'll log it.
        # raise e

    # Initialize BigQuery Client
    try:
        # Check if credentials file exists, otherwise assume environment variable or default auth
        if os.path.exists(CREDENTIALS_PATH):
            os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = CREDENTIALS_PATH
            print(f"🔑 Using credentials from: {CREDENTIALS_PATH}")
        else:
            print(f"⚠️ Credentials file not found at {CREDENTIALS_PATH}. Using default auth.")
        
        bq_client = bigquery.Client(project=PROJECT_ID)
        print("✅ BigQuery Client initialized.")
    except Exception as e:
        print(f"⚠️ Failed to initialize BigQuery Client: {e}")
    
    yield
    
    # Clean up resources if needed (e.g. close db connections)
    print("🛑 Shutting down...")

app = FastAPI(title="Flight Phase Prediction API", lifespan=lifespan)


def get_latest_flight_data_sequence(icao: str, limit: int = 100):
    """
    Fetches the latest data sequence for a given ICAO hex code from BigQuery.
    Returns a DataFrame with columns: Altitude, GroundSpeed, VerticalRate, Track, Time_MSG_Generated
    """
    if not bq_client:
        raise HTTPException(status_code=503, detail="BigQuery client not initialized")

    # Fetch more data than needed to handle sparse rows (null values)
    query = f"""
        SELECT Altitude, GroundSpeed, VerticalRate, Track, Time_MSG_Generated
        FROM `{PROJECT_ID}.{DATASET_ID}.{TABLE_ID}`
        WHERE HexIdent = '{icao}'
        ORDER BY Time_MSG_Generated DESC
        LIMIT {limit}
    """
    
    try:
        query_job = bq_client.query(query)
        df = query_job.to_dataframe()
        
        if df.empty:
            return None
            
        return df
    except Exception as e:
        print(f"BigQuery Error: {e}")
        raise HTTPException(status_code=500, detail=f"BigQuery query failed: {str(e)}")

@app.get("/predict/{icao}", response_model=PredictionResponse)
async def predict_phase(icao: str):
    """
    Predicts the flight phase for a given ICAO hex code using a sequence of data.
    """
    if not model or not scaler:
        raise HTTPException(status_code=503, detail="Model not loaded")

    # 1. Get Data Sequence
    # Fetch 100 rows to ensure we have enough valid data after merging/filling
    raw_df = get_latest_flight_data_sequence(icao, limit=100)
    
    if raw_df is None or raw_df.empty:
        raise HTTPException(status_code=404, detail=f"No data found for aircraft {icao}")

    # 2. Preprocess Sequence
    try:
        # Sort by time ascending for processing (Model expects time series sequence)
        raw_df = raw_df.sort_values('Time_MSG_Generated', ascending=True)
        
        # Select only feature columns
        df_features = raw_df[FEATURE_COLUMNS].copy()
        
        # Handle sparse data: 
        # ADS-B often sends Altitude in one msg, Speed in another.
        # Use Forward Fill (ffill) then Backward Fill (bfill) to complete rows
        df_features = df_features.ffill().bfill()
        
        # If still have NaNs (e.g. all values for a column are missing), fill with 0
        df_features = df_features.fillna(0)
        
        # Ensure we have exactly SEQUENCE_LENGTH rows
        current_len = len(df_features)
        
        if current_len >= SEQUENCE_LENGTH:
            # Take the most recent SEQUENCE_LENGTH rows
            df_final = df_features.iloc[-SEQUENCE_LENGTH:]
        else:
            # Padding: duplicate the first row (or fill with 0) to reach SEQUENCE_LENGTH
            # Here we prepend the first row (pad at the beginning)
            padding_len = SEQUENCE_LENGTH - current_len
            first_row = df_features.iloc[0:1] # Keep as DataFrame
            padding = pd.concat([first_row] * padding_len, ignore_index=True)
            df_final = pd.concat([padding, df_features], ignore_index=True)
        
        # Scale the data
        scaled_features = scaler.transform(df_final) # Shape: (30, 4)
        
        # Flatten for the model: (1, 120)
        # Reshape to (1, 30*4)
        input_data = scaled_features.reshape(1, -1) 
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Preprocessing failed: {str(e)}")

    # 3. Predict
    try:
        prediction = model.predict(input_data)[0]
        
        # Map integer prediction to phase name
        phase_mapping = {0: 'Climb', 1: 'Cruise', 2: 'Descent', 3: 'Taxi'}
        # Handle numpy int64 types safely
        prediction_int = int(prediction)
        prediction = phase_mapping.get(prediction_int, f"Unknown ({prediction_int})")
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Prediction failed: {str(e)}")

    # Prepare features for response (convert to list of dicts)
    features_response = df_final.to_dict(orient='records')

    return PredictionResponse(
        icao=icao,
        phase=prediction,
        features=features_response
    )

@app.get("/health")
async def health_check():
    return {"status": "ok", "model_loaded": model is not None, "bq_connected": bq_client is not None}


@app.post("/weather/overlay", response_model=WeatherOverlayResponse)
async def weather_overlay_endpoint(req: WeatherOverlayRequest):
    if not req.trajectory:
        raise HTTPException(status_code=400, detail="trajectory must be non-empty")
    if not req.hourly:
        raise HTTPException(status_code=400, detail="hourly must be non-empty")
    if req.cell_deg <= 0:
        raise HTTPException(status_code=400, detail="cell_deg must be > 0")
    if req.forecast_days <= 0:
        raise HTTPException(status_code=400, detail="forecast_days must be > 0")

    def _coord_decimals(cell_deg: float) -> int:
        # For cell_deg=0.01, we want 3 decimals to represent cell centers (…x.xx5).
        # In general: ceil(-log10(cell_deg)) + 1
        if cell_deg <= 0:
            return 6
        return max(0, int(math.ceil(-math.log10(cell_deg))) + 1)

    decimals = _coord_decimals(req.cell_deg)

    # If we're not densifying and not expanding, we can keep a direct mapping from cell -> input point.
    input_point_by_cell: Dict[tuple[float, float], tuple[float, float]] = {}
    if (not req.densify) and req.radius_cells == 0:
        for p in req.trajectory:
            c = weather_overlay.cell_center_from_latlon(p.lat, p.lon, req.cell_deg)
            if c not in input_point_by_cell:
                input_point_by_cell[c] = (p.lat, p.lon)

    traj_pts = [weather_overlay.TrajectoryPoint(lat=p.lat, lon=p.lon) for p in req.trajectory]
    base_cells = weather_overlay.cells_from_trajectory(traj_pts, cell_deg=req.cell_deg, densify=req.densify)
    cells = (
        weather_overlay.corridor_expand(base_cells, cell_deg=req.cell_deg, radius_cells=req.radius_cells)
        if req.radius_cells > 0
        else base_cells
    )

    if not cells:
        raise HTTPException(status_code=400, detail="no cells generated from trajectory")

    lats = [c[0] for c in cells]
    lons = [c[1] for c in cells]

    try:
        loc_hourlies = open_meteo_client.fetch_hourly(
            latitudes=lats,
            longitudes=lons,
            hourly_vars=req.hourly,
            forecast_days=req.forecast_days,
            timezone_name="UTC",
        )
    except RuntimeError as e:
        # dependency missing
        raise HTTPException(status_code=503, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=502, detail=f"Open-Meteo fetch failed: {e}")

    # Build response
    target_time = req.target_time or datetime.now(timezone.utc)

    units: Dict[str, str] = {}
    # Use first response's units (all should match)
    for lh in loc_hourlies:
        if lh.units:
            units = lh.units
            break

    def _risk_level_and_color(score: float) -> tuple[str, str]:
        if score >= 80.0:
            return "HIGH", "#FF0000"
        if score >= 60.0:
            return "ELEVATED", "#FF8C00"
        if score >= 30.0:
            return "GUARDED", "#FFD700"
        return "LOW", "#00C853"

    # Stats across cells for each variable
    minmax: Dict[str, List[float]] = {v: [] for v in req.hourly}
    out_cells: List[WeatherCell] = []

    for (latc, lonc), lh in zip(cells, loc_hourlies):
        df = lh.hourly
        idx = open_meteo_client.pick_time_index(df["time"], req.target_time, mode=req.time_mode)
        tsel = df.loc[idx, "time"].to_pydatetime()

        values: Dict[str, float] = {}
        for v in req.hourly:
            val = float(df.loc[idx, v])
            values[v] = val
            minmax[v].append(val)

        series = None
        series_time = None
        if req.include_series:
            series = {v: [float(x) for x in df[v].to_list()] for v in req.hourly}
            series_time = [t.to_pydatetime() for t in df["time"].to_list()]

        risk_score, risk_components, risk_drivers = weather_risk.score_point(
            values,
            profile=req.risk_profile,
        )
        risk_level, risk_color = _risk_level_and_color(risk_score)

        in_latlon = input_point_by_cell.get((latc, lonc))
        cell_lat = float(round(latc, decimals))
        cell_lon = float(round(lonc, decimals))

        # Decide output coordinates
        if req.coord_mode == "input_point" and in_latlon is not None:
            out_lat = float(in_latlon[0])
            out_lon = float(in_latlon[1])
        else:
            out_lat = cell_lat
            out_lon = cell_lon

        out_cells.append(
            WeatherCell(
                lat=out_lat,
                lon=out_lon,
                input_lat=(float(in_latlon[0]) if in_latlon else None),
                input_lon=(float(in_latlon[1]) if in_latlon else None),
                cell_lat=cell_lat,
                cell_lon=cell_lon,
                time=tsel,
                values=values,
                series=series,
                series_time=series_time,
                risk_score=risk_score,
                risk_level=risk_level,
                risk_color=risk_color,
                risk_components=(risk_components if req.include_risk_details else None),
                risk_drivers=(risk_drivers if req.include_risk_details else None),
            )
        )

    stats: Dict[str, Dict[str, float]] = {}
    for v, vals in minmax.items():
        if vals:
            stats[v] = {"min": float(min(vals)), "max": float(max(vals))}

    return WeatherOverlayResponse(
        cell_deg=req.cell_deg,
        hourly=req.hourly,
        forecast_days=req.forecast_days,
        time_mode=req.time_mode,
        target_time=target_time,
        units=units,
        stats=stats,
        cells=out_cells,
    )


def _unit_vec_from_ll(lat_deg: float, lon_deg: float) -> np.ndarray:
    lat = math.radians(float(lat_deg))
    lon = math.radians(float(lon_deg))
    return np.array(
        [
            math.cos(lat) * math.cos(lon),
            math.cos(lat) * math.sin(lon),
            math.sin(lat),
        ],
        dtype=float,
    )


def _ll_from_unit_vec(v: np.ndarray) -> tuple[float, float]:
    v = v / float(np.linalg.norm(v))
    lat = math.degrees(math.asin(float(v[2])))
    lon = math.degrees(math.atan2(float(v[1]), float(v[0])))
    # normalize to [-180, 180)
    lon = (lon + 540.0) % 360.0 - 180.0
    return float(lat), float(lon)


def _great_circle_points(start_lat: float, start_lon: float, end_lat: float, end_lon: float, n_points: int) -> list[tuple[float, float]]:
    if n_points < 2:
        return [(float(start_lat), float(start_lon))]
    a = _unit_vec_from_ll(start_lat, start_lon)
    b = _unit_vec_from_ll(end_lat, end_lon)
    dot = float(np.clip(float(np.dot(a, b)), -1.0, 1.0))
    omega = math.acos(dot)
    if omega < 1e-12:
        return [(float(start_lat), float(start_lon)) for _ in range(int(n_points))]
    sin_omega = math.sin(omega)
    fracs = np.linspace(0.0, 1.0, int(n_points))
    pts: list[tuple[float, float]] = []
    for f in fracs:
        w1 = math.sin((1.0 - float(f)) * omega) / sin_omega
        w2 = math.sin(float(f) * omega) / sin_omega
        lat, lon = _ll_from_unit_vec(w1 * a + w2 * b)
        pts.append((lat, lon))
    pts[0] = (float(start_lat), float(start_lon))
    pts[-1] = (float(end_lat), float(end_lon))
    return pts


def _haversine_km(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6371.0
    phi1 = math.radians(float(lat1))
    phi2 = math.radians(float(lat2))
    dphi = math.radians(float(lat2) - float(lat1))
    dlambda = math.radians(float(lon2) - float(lon1))
    a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
    return float(r * (2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))))


def _bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1 = math.radians(float(lat1))
    phi2 = math.radians(float(lat2))
    dlambda = math.radians(float(lon2) - float(lon1))
    y = math.sin(dlambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    return float((math.degrees(math.atan2(y, x)) + 360.0) % 360.0)


def _destination_point(lat_deg: float, lon_deg: float, bearing_deg0: float, distance_km: float) -> tuple[float, float]:
    r = 6371.0
    delta = float(distance_km) / r
    theta = math.radians(float(bearing_deg0))
    phi1 = math.radians(float(lat_deg))
    lam1 = math.radians(float(lon_deg))
    sin_phi2 = math.sin(phi1) * math.cos(delta) + math.cos(phi1) * math.sin(delta) * math.cos(theta)
    phi2 = math.asin(float(np.clip(sin_phi2, -1.0, 1.0)))
    y = math.sin(theta) * math.sin(delta) * math.cos(phi1)
    x = math.cos(delta) - math.sin(phi1) * math.sin(phi2)
    lam2 = lam1 + math.atan2(y, x)
    lat2 = math.degrees(phi2)
    lon2 = (math.degrees(lam2) + 540.0) % 360.0 - 180.0
    return float(lat2), float(lon2)


def _smooth_offsets_m(n: int, *, seed: int, max_offset_km: float, smooth_window: int) -> np.ndarray:
    rng = np.random.default_rng(int(seed))
    x = rng.normal(size=int(n)).astype(float)
    x[0] = 0.0
    x[-1] = 0.0
    w = max(3, int(smooth_window))
    if w % 2 == 0:
        w += 1
    kernel = np.ones(w, dtype=float) / float(w)
    xs = np.convolve(x, kernel, mode="same")
    xs[0] = 0.0
    xs[-1] = 0.0
    m = float(np.max(np.abs(xs))) if np.any(np.isfinite(xs)) else 0.0
    if m < 1e-12:
        return np.zeros(int(n), dtype=float)
    return (xs / m) * (float(max_offset_km) * 1000.0)


def _perturb_points(base: list[tuple[float, float]], offsets_m: np.ndarray) -> list[tuple[float, float]]:
    n = len(base)
    out: list[tuple[float, float]] = []
    for i in range(n):
        if i == 0 or i == n - 1:
            out.append(base[i])
            continue
        lat0, lon0 = base[i]
        lat1, lon1 = base[i + 1]
        course = _bearing_deg(lat0, lon0, lat1, lon1)
        perp = (course + 90.0) % 360.0
        latp, lonp = _destination_point(lat0, lon0, perp, float(offsets_m[i]) / 1000.0)
        out.append((latp, lonp))
    out[0] = base[0]
    out[-1] = base[-1]
    return out


def _tailwind_component_kmh(*, wind_speed_kmh: float, wind_dir_from_deg: float, course_deg0: float) -> float:
    wind_to = (float(wind_dir_from_deg) + 180.0) % 360.0
    theta = math.radians(wind_to)
    u = float(wind_speed_kmh) * math.sin(theta)  # east
    v = float(wind_speed_kmh) * math.cos(theta)  # north
    c = math.radians(float(course_deg0))
    return float(u * math.sin(c) + v * math.cos(c))


@app.post("/co2/recommend", response_model=CO2RecommendResponse)
async def co2_recommend_endpoint(req: CO2RecommendRequest):
    if openap is None:
        raise HTTPException(status_code=503, detail="OpenAP is not installed in the backend environment")

    if req.n_perturbations < 0 or req.n_perturbations > 50:
        raise HTTPException(status_code=400, detail="n_perturbations must be between 0 and 50")
    if req.n_points < 2 or req.n_points > 100:
        raise HTTPException(status_code=400, detail="n_points must be between 2 and 100")
    if req.max_offset_km < 0.0:
        raise HTTPException(status_code=400, detail="max_offset_km must be >= 0")

    eval_time = req.eval_time_utc or datetime.now(timezone.utc)

    base = _great_circle_points(req.start_lat, req.start_lon, req.end_lat, req.end_lon, int(req.n_points))
    traj: dict[str, list[tuple[float, float]]] = {"A_geodesic": base}
    for i in range(int(req.n_perturbations)):
        seed = int(req.seed0) + i
        offsets_m = _smooth_offsets_m(len(base), seed=seed, max_offset_km=float(req.max_offset_km), smooth_window=int(req.smooth_window))
        name = f"{chr(ord('B') + i)}_seed{seed}"
        traj[name] = _perturb_points(base, offsets_m)

    wind_speed_var = f"wind_speed_{req.wind_level}"
    wind_dir_var = f"wind_direction_{req.wind_level}"

    # Fetch winds for all unique (lat,lon) samples across all trajectories
    # Keep order stable for deterministic mapping.
    all_pts: list[tuple[float, float]] = []
    seen: set[tuple[float, float]] = set()
    for pts in traj.values():
        for lat, lon in pts:
            key = (float(lat), float(lon))
            if key not in seen:
                seen.add(key)
                all_pts.append(key)

    try:
        loc_hourlies = open_meteo_client.fetch_hourly(
            latitudes=[p[0] for p in all_pts],
            longitudes=[p[1] for p in all_pts],
            hourly_vars=[wind_speed_var, wind_dir_var],
            forecast_days=2,
            timezone_name="UTC",
        )
    except RuntimeError as e:
        raise HTTPException(status_code=503, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=502, detail=f"Open-Meteo fetch failed: {e}")

    # Map each unique point to (ws_kmh, wd_from_deg) at eval_time
    wind_by_point: dict[tuple[float, float], tuple[float, float]] = {}
    for (lat, lon), lh in zip(all_pts, loc_hourlies):
        df = lh.hourly
        idx = open_meteo_client.pick_time_index(df["time"], eval_time, mode="nearest")
        try:
            ws = float(df.loc[idx, wind_speed_var])
            wd = float(df.loc[idx, wind_dir_var])
        except Exception:
            ws = 0.0
            wd = 0.0
        wind_by_point[(lat, lon)] = (ws, wd)

    # Fuel flow model (kg/s)
    try:
        fuelflow = openap.FuelFlow(str(req.aircraft))
        ff_kg_s = float(
            fuelflow.enroute(
                mass=float(req.mass_kg),
                tas=float(req.cruise_tas_kt),
                alt=float(req.cruise_alt_ft),
            )
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"OpenAP fuel flow model failed: {e}")

    KM_PER_NM = 1.852
    NM_PER_KM = 1.0 / KM_PER_NM
    KT_PER_KMH = 1.0 / KM_PER_NM

    def eval_one(name: str, pts: list[tuple[float, float]]) -> dict[str, float | str | list[tuple[float, float]]]:
        seg_dist_nm: list[float] = []
        seg_course: list[float] = []
        for i in range(len(pts) - 1):
            lat1, lon1 = pts[i]
            lat2, lon2 = pts[i + 1]
            seg_dist_nm.append(_haversine_km(lat1, lon1, lat2, lon2) * NM_PER_KM)
            seg_course.append(_bearing_deg(lat1, lon1, lat2, lon2))
        dist_nm = float(np.sum(seg_dist_nm))

        times_hr: list[float] = []
        for i, (d_nm, course) in enumerate(zip(seg_dist_nm, seg_course)):
            ws1, wd1 = wind_by_point[(float(pts[i][0]), float(pts[i][1]))]
            ws2, wd2 = wind_by_point[(float(pts[i + 1][0]), float(pts[i + 1][1]))]
            tw1 = _tailwind_component_kmh(wind_speed_kmh=ws1, wind_dir_from_deg=wd1, course_deg0=float(course))
            tw2 = _tailwind_component_kmh(wind_speed_kmh=ws2, wind_dir_from_deg=wd2, course_deg0=float(course))
            tw = float(np.mean([tw1, tw2]))
            gs_kt = max(80.0, float(req.cruise_tas_kt) + tw * KT_PER_KMH)
            times_hr.append(float(d_nm) / float(gs_kt))
        time_hr = float(np.sum(times_hr))

        fuel_kg = float(ff_kg_s * time_hr * 3600.0)
        co2_kg = float(fuel_kg * float(req.co2_kg_per_kg_fuel))
        return {
            "name": name,
            "points": pts,
            "distance_nm": dist_nm,
            "time_hr": time_hr,
            "fuel_kg": fuel_kg,
            "co2_kg": co2_kg,
        }

    evaluated = [eval_one(name, pts) for name, pts in traj.items()]
    evaluated_sorted = sorted(evaluated, key=lambda r: float(r["co2_kg"]))

    best = evaluated_sorted[0]
    geodesic = next((r for r in evaluated if r["name"] == "A_geodesic"), None)
    if geodesic is None:
        geodesic = best

    co2_best = float(best["co2_kg"])
    co2_geo = float(geodesic["co2_kg"])
    reduction = float(max(0.0, co2_geo - co2_best))

    # Keep response payload small by default; include per-trajectory details only when explicitly requested.
    out_traj = None
    if False:  # reserved for future optional debug flag
        out_traj = []

    best_points = [TrajectoryPoint(lat=float(lat), lon=float(lon)) for (lat, lon) in best["points"]]  # type: ignore[index]

    return CO2RecommendResponse(
        best_name=str(best["name"]),
        best_points=best_points,
        co2_best_kg=co2_best,
        co2_geodesic_kg=co2_geo,
        co2_reduction_kg=reduction,
        eval_time_utc=eval_time,
        wind_level=str(req.wind_level),
        trajectories=out_traj,
    )
