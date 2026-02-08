import os
import joblib
import pandas as pd
import numpy as np
from fastapi import FastAPI, HTTPException
from google.cloud import bigquery
from pydantic import BaseModel
from typing import Optional, List, Dict, Any, Literal
from contextlib import asynccontextmanager
from datetime import datetime, timezone
import math

import open_meteo_client
import weather_overlay
import weather_risk

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
