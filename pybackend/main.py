import os
import re
import json
import time
import asyncio
import joblib
import pandas as pd
import numpy as np
from fastapi import FastAPI, HTTPException
from google.cloud import bigquery
from pydantic import BaseModel
from typing import Optional, List, Dict, Any, Literal
from contextlib import asynccontextmanager
from datetime import datetime, timezone, time as dt_time, timedelta
from urllib import request as urllib_request
from urllib import error as urllib_error
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


class FuelSummaryResponse(BaseModel):
    icao: str
    flight_id: Optional[str] = None
    phase: Optional[str] = None
    fuel_rate_kg_hr: float
    fuel_used_kg: float
    co2_kg: float
    elapsed_hr: float
    rate_source: str
    elapsed_source: str = "observed"
    estimated: bool = False
    updated_at: datetime
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None

# Global variables for model and scaler
model = None
scaler = None
bq_client = None
table_columns: set[str] = set()
aircraft_db: Dict[str, Dict[str, str]] = {}
fuel_rate_cache: Dict[str, Dict[str, Any]] = {}

# Configuration
PROJECT_ID = "iitp-class-team-5-473114"
DATASET_ID = "Dataset"
TABLE_ID = "FirstRun"  # Or "FirstRun_merged_filtered_added" depending on your live data source
# Assumes BigQueryCred.json is in the BigQuery directory relative to pybackend
CREDENTIALS_PATH = os.path.join(os.path.dirname(__file__), "..", "BigQuery", "YourJsonFile.json") 

SEQUENCE_LENGTH = 30 # Time steps required by the model
FEATURE_COLUMNS = ['Altitude', 'GroundSpeed', 'VerticalRate', 'Track']

FUEL_RATE_TTL_SEC = 6 * 3600
DEFAULT_CO2_KG_PER_KG_FUEL = 3.16
GEMINI_MODEL = os.getenv("GEMINI_MODEL", "gemini-2.5-flash")

AIRCRAFT_DB_PATH = os.path.join(os.path.dirname(__file__), "..", "Win64", "AircraftDB", "aircraftDatabase.csv")


def _normalize_flight_id(value: Optional[str]) -> str:
    if not value:
        return ""
    return "".join([c for c in value.upper() if c.isalnum()])


def _normalize_icao(value: Optional[str]) -> str:
    if not value:
        return ""
    return "".join([c for c in str(value).upper() if c.isalnum()])


def _is_placeholder_icao(value: Optional[str]) -> bool:
    key = _normalize_icao(value)
    return key in {"", "NA", "NOHOOK", "NODATA", "UNKNOWN"}


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        f = float(value)
    except Exception:
        return float(default)
    if not math.isfinite(f):
        return float(default)
    return float(f)


def _load_aircraft_db_if_needed() -> None:
    global aircraft_db
    if aircraft_db:
        return
    if not os.path.exists(AIRCRAFT_DB_PATH):
        return
    try:
        df = pd.read_csv(AIRCRAFT_DB_PATH)
        if "icao24" not in df.columns:
            return
        needed_cols = ["icao24", "typecode", "model", "manufacturername", "icaoaircrafttype"]
        for _, row in df.iterrows():
            icao = str(row.get("icao24", "")).lower().strip()
            if not icao:
                continue
            aircraft_db[icao] = {
                "typecode": str(row.get("typecode", "")).strip(),
                "model": str(row.get("model", "")).strip(),
                "manufacturer": str(row.get("manufacturername", "")).strip(),
                "icaoaircrafttype": str(row.get("icaoaircrafttype", "")).strip(),
            }
    except Exception:
        # Non-fatal: the DB is optional for fuel estimation
        aircraft_db = {}


def _get_aircraft_info(icao_hex: str) -> Dict[str, str]:
    _load_aircraft_db_if_needed()
    return aircraft_db.get(str(icao_hex).lower().strip(), {})


def _refresh_table_columns() -> None:
    global table_columns
    if not bq_client:
        return
    try:
        query = f"""
            SELECT column_name
            FROM `{PROJECT_ID}.{DATASET_ID}.INFORMATION_SCHEMA.COLUMNS`
            WHERE table_name = @table_name
        """
        job_config = bigquery.QueryJobConfig(
            query_parameters=[bigquery.ScalarQueryParameter("table_name", "STRING", TABLE_ID)]
        )
        rows = bq_client.query(query, job_config=job_config).result()
        table_columns = {str(r["column_name"]) for r in rows}
    except Exception:
        table_columns = set()


def _flight_id_column() -> Optional[str]:
    for c in ["FlightNum", "FlightID", "FlightId", "Callsign", "CallSign", "Flight"]:
        if c in table_columns:
            return c
    return None


def _first_existing_column(candidates: List[str]) -> Optional[str]:
    for c in candidates:
        if c in table_columns:
            return c
    return None


def _route_column() -> Optional[str]:
    return _first_existing_column(["Route", "ROUTE", "route"])


def _lat_column() -> Optional[str]:
    return _first_existing_column(["Latitude", "LAT", "Lat", "lat", "latitude"])


def _lon_column() -> Optional[str]:
    return _first_existing_column(["Longitude", "LON", "Lon", "lon", "longitude"])


# Minimal airport coordinates for demo routes (IATA -> lat/lon)
AIRPORT_COORDS_IATA: Dict[str, tuple[float, float]] = {
    "ONT": (34.055999, -117.601997),
    "PHL": (39.871899, -75.241096),
    "CLT": (35.214001, -80.943100),
    "CAK": (40.916100, -81.442200),
    "PIT": (40.491500, -80.232903),
    "BWI": (39.175400, -76.668297),
    "DCA": (38.851200, -77.040199),
    "DTW": (42.212399, -83.353401),
}


def _haversine_nm(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r_km = 6371.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a)))
    km = r_km * c
    return km / 1.852


def _estimate_elapsed_from_route(route_text: Optional[str], cur_lat: Any, cur_lon: Any, gs_kt: Any) -> Optional[float]:
    route = str(route_text or "").strip().upper()
    # Use only simple two-airport pattern for demo reliability: AAA-BBB
    m = re.fullmatch(r"([A-Z]{3})-([A-Z]{3})", route)
    if not m:
        return None
    dep = m.group(1)
    if dep not in AIRPORT_COORDS_IATA:
        return None

    lat = _safe_float(cur_lat, default=float("nan"))
    lon = _safe_float(cur_lon, default=float("nan"))
    gs = _safe_float(gs_kt, default=0.0)
    if not math.isfinite(lat) or not math.isfinite(lon):
        return None

    dep_lat, dep_lon = AIRPORT_COORDS_IATA[dep]
    dist_nm = _haversine_nm(dep_lat, dep_lon, lat, lon)
    # Guardrails so descent/taxi speeds do not explode elapsed estimate.
    speed_for_elapsed = min(max(gs, 150.0), 520.0)
    if speed_for_elapsed <= 0.0:
        return None
    est_hr = dist_nm / speed_for_elapsed
    return max(0.0, est_hr)


def _call_gemini_for_fuel_rate(prompt: str) -> Optional[float]:
    api_key = os.getenv("GEMINI_API_KEY", "").strip()
    if not api_key:
        print("[gemini] GEMINI_API_KEY not set; skipping Gemini fuel-rate estimation.")
        return None
    url = f"https://generativelanguage.googleapis.com/v1beta/models/{GEMINI_MODEL}:generateContent?key={api_key}"
    payload = {
        "contents": [{"parts": [{"text": prompt}]}],
        "generationConfig": {
            "temperature": 0.0,
            "maxOutputTokens": 64,
            "stopSequences": ["\n"],
            "thinkingConfig": {"thinkingBudget": 0},
        },
    }
    req = urllib_request.Request(
        url,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib_request.urlopen(req, timeout=4) as resp:
            data = json.loads(resp.read().decode("utf-8"))
    except urllib_error.HTTPError as e:
        try:
            body = e.read().decode("utf-8", errors="ignore")
        except Exception:
            body = ""
        print(f"[gemini] HTTPError status={e.code}, reason={e.reason}, body={body[:300]}")
        return None
    except urllib_error.URLError as e:
        print(f"[gemini] URLError: {e}")
        return None
    except TimeoutError:
        print("[gemini] TimeoutError while calling Gemini.")
        return None
    except json.JSONDecodeError as e:
        print(f"[gemini] JSON decode error from Gemini response: {e}")
        return None
    except Exception:
        print("[gemini] Unexpected exception while calling Gemini.")
        return None
    try:
        candidate0 = data["candidates"][0]
        finish_reason = str(candidate0.get("finishReason", ""))
        if finish_reason:
            print(f"[gemini] finishReason={finish_reason}")
        parts = candidate0["content"]["parts"]
        text_chunks: List[str] = []
        for p in parts:
            t = str(p.get("text") or "").strip()
            if t:
                text_chunks.append(t)
        text = "\n".join(text_chunks).strip()
        if len(text_chunks) > 1:
            print(f"[gemini] Response split across {len(text_chunks)} text parts.")
    except Exception:
        print(f"[gemini] Unexpected response shape: {str(data)[:300]}")
        return None
    if not text:
        print("[gemini] Empty text response from Gemini.")
        return None
    raw_text = str(text).strip()
    print(f"[gemini] Raw response text: {raw_text[:240]}")
    # Preferred parse path: natural-language key/value line
    # e.g. "fuel_rate_kg_hr: 2450"
    for key in ("fuel_rate_kg_hr", "fuel rate", "fuel_rate", "rate"):
        m_key = re.search(
            rf"{re.escape(key)}\s*[:=]\s*([-+]?[0-9]+(?:\.[0-9]+)?)",
            raw_text,
            flags=re.IGNORECASE,
        )
        if m_key:
            value = float(m_key.group(1))
            print(f"[gemini] Parsed key regex {key}={value}")
            return value

    fenced = re.search(r"```(?:json)?\s*(.*?)```", raw_text, flags=re.IGNORECASE | re.DOTALL)
    candidate = fenced.group(1).strip() if fenced else raw_text
    try:
        parsed = json.loads(candidate)
        for key in ("fuel_rate_kg_hr", "fuel_rate", "rate"):
            if key in parsed:
                try:
                    value = float(parsed.get(key))
                    print(f"[gemini] Parsed {key}={value}")
                    return value
                except Exception:
                    m = re.search(r"([-+]?[0-9]+(?:\.[0-9]+)?)", str(parsed.get(key)))
                    if m:
                        value = float(m.group(1))
                        print(f"[gemini] Parsed numeric token from {key}: {value}")
                        return value
        print(f"[gemini] JSON parsed but expected key missing: {str(parsed)[:200]}")
    except Exception:
        # Try extracting a JSON object when extra prose wraps it.
        start = candidate.find("{")
        end = candidate.rfind("}")
        if start != -1 and end != -1 and end > start:
            obj_text = candidate[start : end + 1]
            try:
                parsed_obj = json.loads(obj_text)
                for key in ("fuel_rate_kg_hr", "fuel_rate", "rate"):
                    if key in parsed_obj:
                        try:
                            value = float(parsed_obj.get(key))
                            print(f"[gemini] Parsed extracted JSON {key}={value}")
                            return value
                        except Exception:
                            m = re.search(r"([-+]?[0-9]+(?:\.[0-9]+)?)", str(parsed_obj.get(key)))
                            if m:
                                value = float(m.group(1))
                                print(f"[gemini] Parsed numeric token from extracted JSON {key}: {value}")
                                return value
            except Exception:
                pass

        # Key-specific regex fallback (text that is not valid JSON).
        for key in ("fuel_rate_kg_hr", "fuel rate", "fuel_rate", "rate"):
            m_key = re.search(
                rf"{re.escape(key)}\s*[:=]\s*([-+]?[0-9]+(?:\.[0-9]+)?)",
                raw_text,
                flags=re.IGNORECASE,
            )
            if m_key:
                value = float(m_key.group(1))
                print(f"[gemini] Parsed key regex {key}={value}")
                return value

        # Final fallback: first numeric token from whole text.
        m = re.search(r"([-+]?[0-9]+(?:\.[0-9]+)?)", raw_text)
        if not m:
            print(f"[gemini] Could not parse fuel_rate_kg_hr from Gemini text: {raw_text[:200]}")
            return None
        value = float(m.group(1))
        print(f"[gemini] Parsed first numeric token={value}")
        return value


def _call_gemini_for_elapsed_hr(prompt: str) -> Optional[float]:
    api_key = os.getenv("GEMINI_API_KEY", "").strip()
    if not api_key:
        print("[gemini-elapsed] GEMINI_API_KEY not set; skipping elapsed estimation.")
        return None

    url = f"https://generativelanguage.googleapis.com/v1beta/models/{GEMINI_MODEL}:generateContent?key={api_key}"
    payload = {
        "contents": [{"parts": [{"text": prompt}]}],
        "generationConfig": {
            "temperature": 0.0,
            "maxOutputTokens": 64,
            "stopSequences": ["\n"],
            "thinkingConfig": {"thinkingBudget": 0},
        },
    }
    req = urllib_request.Request(
        url,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib_request.urlopen(req, timeout=4) as resp:
            data = json.loads(resp.read().decode("utf-8"))
    except Exception as e:
        print(f"[gemini-elapsed] request failed: {e}")
        return None

    try:
        candidate0 = data["candidates"][0]
        finish_reason = str(candidate0.get("finishReason", ""))
        if finish_reason:
            print(f"[gemini-elapsed] finishReason={finish_reason}")
        parts = candidate0["content"]["parts"]
        text = "\n".join([str(p.get("text") or "").strip() for p in parts if str(p.get("text") or "").strip()]).strip()
    except Exception:
        print(f"[gemini-elapsed] Unexpected response shape: {str(data)[:300]}")
        return None

    if not text:
        print("[gemini-elapsed] Empty text response.")
        return None

    raw_text = str(text).strip()
    print(f"[gemini-elapsed] Raw response text: {raw_text[:240]}")
    for key in ("elapsed_hr", "elapsed_hours", "flight_time_hr", "hours", "time_hr"):
        m_key = re.search(
            rf"{re.escape(key)}\s*[:=]\s*([-+]?[0-9]+(?:\.[0-9]+)?)",
            raw_text,
            flags=re.IGNORECASE,
        )
        if m_key:
            value = _safe_float(m_key.group(1), default=0.0)
            if value > 0:
                print(f"[gemini-elapsed] Parsed {key}={value}")
                return value

    m = re.search(r"([-+]?[0-9]+(?:\.[0-9]+)?)", raw_text)
    if not m:
        print(f"[gemini-elapsed] Could not parse elapsed_hr from text: {raw_text[:200]}")
        return None
    value = _safe_float(m.group(1), default=0.0)
    if value <= 0:
        return None
    print(f"[gemini-elapsed] Parsed first numeric token={value}")
    return value


def _estimate_elapsed_from_gemini(route_text: Optional[str], cur_lat: Any, cur_lon: Any, gs_kt: Any, phase: Optional[str]) -> Optional[float]:
    route = str(route_text or "").strip().upper()
    # Use first segment as departure airport hint (e.g., BOS-CMH or CLT-CAK-CLT -> CLT)
    dep = ""
    m = re.match(r"^([A-Z]{3,4})-", route)
    if m:
        dep = m.group(1)

    lat = _safe_float(cur_lat, default=float("nan"))
    lon = _safe_float(cur_lon, default=float("nan"))
    gs = _safe_float(gs_kt, default=0.0)
    has_latlon = math.isfinite(lat) and math.isfinite(lon)
    lat_text = f"{lat}" if has_latlon else "UNKNOWN"
    lon_text = f"{lon}" if has_latlon else "UNKNOWN"
    if not has_latlon:
        print("[gemini-elapsed] Missing current lat/lon in snapshot; requesting estimate with partial inputs.")
    if not route:
        print("[gemini-elapsed] Missing route in snapshot; requesting estimate with partial inputs.")

    prompt = (
        "Estimate elapsed flight time in hours.\n"
        f"dep={dep or 'UNKNOWN'} route={route or 'UNKNOWN'} "
        f"lat={lat_text} lon={lon_text} gs_kt={gs if gs > 0 else 'UNKNOWN'} phase={phase or 'UNKNOWN'}\n"
        "Output exactly one token in this format: elapsed_hr=<number>\n"
        "No words. No JSON. No markdown. Example: elapsed_hr=2.35\n"
    )
    print(
        f"[gemini-elapsed] Input dep={dep or 'UNKNOWN'} route={route or 'UNKNOWN'} "
        f"lat={lat_text} lon={lon_text} gs_kt={gs:.1f} phase={phase or 'UNKNOWN'}"
    )
    hr = _call_gemini_for_elapsed_hr(prompt)
    if hr is None:
        return None
    # Practical bounds for a single flight leg in this demo.
    hr = _safe_float(hr, default=0.0)
    if hr <= 0:
        return None
    return float(min(max(hr, 0.05), 18.0))


def _heuristic_fuel_rate(phase: str, ground_speed_kt: Optional[float]) -> float:
    gs = float(ground_speed_kt) if ground_speed_kt is not None else 0.0
    if gs <= 160:
        base = 120.0
    elif gs <= 260:
        base = 450.0
    elif gs <= 400:
        base = 1500.0
    else:
        base = 2500.0
    phase_key = (phase or "").lower()
    if "taxi" in phase_key:
        mult = 0.4
    elif "climb" in phase_key:
        mult = 1.2
    elif "descent" in phase_key:
        mult = 0.7
    else:
        mult = 1.0
    return float(max(60.0, base * mult))


def _estimate_fuel_rate(
    *,
    flight_id: str,
    phase: str,
    icao_hex: str,
    altitude_ft: Optional[float],
    ground_speed_kt: Optional[float],
) -> tuple[float, str]:
    cache_key = flight_id or icao_hex
    now = time.time()
    cached = fuel_rate_cache.get(cache_key)
    if cached and (now - cached["ts"]) <= FUEL_RATE_TTL_SEC:
        cached_rate = _safe_float(cached.get("rate"), default=0.0)
        if cached_rate > 0.0:
            return cached_rate, str(cached.get("source", "cache"))

    info = _get_aircraft_info(icao_hex)
    prompt = (
        "Estimate aircraft fuel burn rate in kg/hr.\n"
        f"flight_id={flight_id or 'UNKNOWN'} phase={phase or 'UNKNOWN'} "
        f"typecode={info.get('typecode', '') or 'UNKNOWN'} model={info.get('model', '') or 'UNKNOWN'} "
        f"alt_ft={altitude_ft if altitude_ft is not None else 'UNKNOWN'} "
        f"gs_kt={ground_speed_kt if ground_speed_kt is not None else 'UNKNOWN'}\n"
        "Output exactly one token in this format: fuel_rate_kg_hr=<number>\n"
        "No words. No JSON. No markdown. Example: fuel_rate_kg_hr=2450\n"
    )
    rate = _call_gemini_for_fuel_rate(prompt)
    if rate is not None:
        rate = _safe_float(rate, default=0.0)
    if rate is not None and rate > 0.0:
        rate = float(max(50.0, min(rate, 20000.0)))
        fuel_rate_cache[cache_key] = {"rate": rate, "source": "gemini", "ts": now}
        return rate, "gemini"

    if openap is not None and info.get("typecode"):
        try:
            fuelflow = openap.FuelFlow(str(info.get("typecode")))
            ff_kg_s = float(
                fuelflow.enroute(
                    mass=65000.0,
                    tas=float(ground_speed_kt or 450.0),
                    alt=float(altitude_ft or 35000.0),
                )
            )
            rate = _safe_float(ff_kg_s * 3600.0, default=0.0)
            if rate > 0.0:
                fuel_rate_cache[cache_key] = {"rate": rate, "source": "openap", "ts": now}
                return rate, "openap"
        except Exception:
            pass

    rate = _heuristic_fuel_rate(phase, ground_speed_kt)
    fuel_rate_cache[cache_key] = {"rate": rate, "source": "heuristic", "ts": now}
    return rate, "heuristic"

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
        _refresh_table_columns()
    except Exception as e:
        print(f"⚠️ Failed to initialize BigQuery Client: {e}")

    _load_aircraft_db_if_needed()
    
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
        WHERE HexIdent = @icao
        ORDER BY Time_MSG_Generated DESC
        LIMIT {limit}
    """
    
    try:
        job_config = bigquery.QueryJobConfig(
            query_parameters=[bigquery.ScalarQueryParameter("icao", "STRING", icao)]
        )
        query_job = bq_client.query(query, job_config=job_config)
        df = query_job.to_dataframe()
        
        if df.empty:
            return None
            
        return df
    except Exception as e:
        print(f"BigQuery Error: {e}")
        raise HTTPException(status_code=500, detail=f"BigQuery query failed: {str(e)}")

def _prepare_sequence(icao: str) -> pd.DataFrame:
    raw_df = get_latest_flight_data_sequence(icao, limit=100)
    if raw_df is None or raw_df.empty:
        raise HTTPException(status_code=404, detail=f"No data found for aircraft {icao}")
    raw_df = raw_df.sort_values('Time_MSG_Generated', ascending=True)
    df_features = raw_df[FEATURE_COLUMNS].copy()
    df_features = df_features.ffill().bfill()
    df_features = df_features.fillna(0)
    current_len = len(df_features)
    if current_len >= SEQUENCE_LENGTH:
        df_final = df_features.iloc[-SEQUENCE_LENGTH:]
    else:
        padding_len = SEQUENCE_LENGTH - current_len
        first_row = df_features.iloc[0:1]
        padding = pd.concat([first_row] * padding_len, ignore_index=True)
        df_final = pd.concat([padding, df_features], ignore_index=True)
    return df_final


def _predict_phase_name(df_final: pd.DataFrame) -> str:
    if not model or not scaler:
        raise HTTPException(status_code=503, detail="Model not loaded")
    scaled_features = scaler.transform(df_final)
    input_data = scaled_features.reshape(1, -1)
    prediction = model.predict(input_data)[0]
    phase_mapping = {0: 'Climb', 1: 'Cruise', 2: 'Descent', 3: 'Taxi'}
    prediction_int = int(prediction)
    return phase_mapping.get(prediction_int, f"Unknown ({prediction_int})")


def _get_latest_snapshot(icao: str) -> Dict[str, Any]:
    if not bq_client:
        raise HTTPException(status_code=503, detail="BigQuery client not initialized")
    flight_col = _flight_id_column()
    cols = ["Altitude", "GroundSpeed", "VerticalRate", "Track", "Time_MSG_Generated"]
    if flight_col:
        cols.append(flight_col)
    route_col = _route_column()
    lat_col = _lat_column()
    lon_col = _lon_column()
    if route_col:
        cols.append(route_col)
    if lat_col:
        cols.append(lat_col)
    if lon_col:
        cols.append(lon_col)
    query = f"""
        SELECT {", ".join(cols)}
        FROM `{PROJECT_ID}.{DATASET_ID}.{TABLE_ID}`
        WHERE HexIdent = @icao
        ORDER BY Time_MSG_Generated DESC
        LIMIT 1
    """
    job_config = bigquery.QueryJobConfig(
        query_parameters=[bigquery.ScalarQueryParameter("icao", "STRING", icao)]
    )
    df = bq_client.query(query, job_config=job_config).to_dataframe()
    if df.empty:
        return {}
    row = df.iloc[0].to_dict()
    if flight_col and flight_col in row:
        row["flight_id"] = row.get(flight_col)
    if route_col and route_col in row:
        row["route"] = row.get(route_col)
    if lat_col and lat_col in row:
        row["lat"] = row.get(lat_col)
    if lon_col and lon_col in row:
        row["lon"] = row.get(lon_col)
    return row


def _get_flight_time_range(icao: str, flight_id: str, window_hr: int = 6) -> tuple[Optional[Any], Optional[Any]]:
    if not bq_client:
        raise HTTPException(status_code=503, detail="BigQuery client not initialized")
    flight_col = _flight_id_column()
    filters = ["HexIdent = @icao"]
    params = [bigquery.ScalarQueryParameter("icao", "STRING", icao)]
    if flight_col and flight_id:
        filters.append(f"{flight_col} = @flight_id")
        params.append(bigquery.ScalarQueryParameter("flight_id", "STRING", flight_id))
    where_clause = " AND ".join(filters)
    query_first = f"""
        SELECT Time_MSG_Generated AS t
        FROM `{PROJECT_ID}.{DATASET_ID}.{TABLE_ID}`
        WHERE {where_clause}
        ORDER BY Time_MSG_Generated ASC
        LIMIT 1
    """
    query_last = f"""
        SELECT Time_MSG_Generated AS t
        FROM `{PROJECT_ID}.{DATASET_ID}.{TABLE_ID}`
        WHERE {where_clause}
        ORDER BY Time_MSG_Generated DESC
        LIMIT 1
    """
    job_config = bigquery.QueryJobConfig(query_parameters=params)
    try:
        df_first = bq_client.query(query_first, job_config=job_config).to_dataframe()
        df_last = bq_client.query(query_last, job_config=job_config).to_dataframe()
    except Exception:
        return None, None

    if df_first.empty or df_last.empty:
        return None, None
    return df_first.iloc[0].get("t"), df_last.iloc[0].get("t")


def _coerce_snapshot_time_to_datetime(value: Any, now_utc: datetime) -> Optional[datetime]:
    if value is None:
        return None
    try:
        if pd.isna(value):
            return None
    except Exception:
        pass

    if hasattr(value, "to_pydatetime"):
        value = value.to_pydatetime()

    dt: Optional[datetime] = None
    now_naive_utc = now_utc.astimezone(timezone.utc).replace(tzinfo=None)
    if isinstance(value, datetime):
        dt = value
    elif isinstance(value, dt_time):
        dt = datetime.combine(now_utc.date(), value)
        # TIME columns can belong to the previous UTC day.
        dt_cmp = dt if dt.tzinfo is None else dt.astimezone(timezone.utc).replace(tzinfo=None)
        if dt_cmp > now_naive_utc + timedelta(minutes=1):
            dt = dt - timedelta(days=1)
    elif isinstance(value, str):
        raw = value.strip()
        if not raw:
            return None
        try:
            dt = datetime.fromisoformat(raw.replace("Z", "+00:00"))
        except Exception:
            try:
                t = dt_time.fromisoformat(raw)
                dt = datetime.combine(now_utc.date(), t)
                dt_cmp = dt if dt.tzinfo is None else dt.astimezone(timezone.utc).replace(tzinfo=None)
                if dt_cmp > now_naive_utc + timedelta(minutes=1):
                    dt = dt - timedelta(days=1)
            except Exception:
                return None
    else:
        return None

    if dt is None:
        return None
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    else:
        dt = dt.astimezone(timezone.utc)
    return dt


@app.get("/predict/{icao}", response_model=PredictionResponse)
async def predict_phase(icao: str):
    """
    Predicts the flight phase for a given ICAO hex code using a sequence of data.
    """
    if not model or not scaler:
        raise HTTPException(status_code=503, detail="Model not loaded")

    # 1. Get Data Sequence
    try:
        df_final = _prepare_sequence(icao)
    except Exception:
        return PredictionResponse(icao=icao, phase="Unknown", features=[])

    # 3. Predict
    try:
        prediction = _predict_phase_name(df_final)
    except Exception:
        return PredictionResponse(icao=icao, phase="Unknown", features=[])

    # Prepare features for response (convert to list of dicts)
    features_response = df_final.to_dict(orient='records')

    return PredictionResponse(
        icao=icao,
        phase=prediction,
        features=features_response
    )


@app.get("/fuel/summary", response_model=FuelSummaryResponse)
async def fuel_summary(
    icao: str,
    flight_id: Optional[str] = None,
    phase: Optional[str] = None,
    time_window_hr: int = 6,
    route: Optional[str] = None,
    current_lat: Optional[float] = None,
    current_lon: Optional[float] = None,
    gs_kt: Optional[float] = None,
):
    if not bq_client:
        raise HTTPException(status_code=503, detail="BigQuery client not initialized")
    icao = _normalize_icao(icao)
    if _is_placeholder_icao(icao):
        raise HTTPException(status_code=400, detail="icao is required")
    flight_id_norm = _normalize_flight_id(flight_id)

    snapshot = _get_latest_snapshot(icao)
    alt = snapshot.get("Altitude")
    gs = snapshot.get("GroundSpeed")

    # Prefer realtime frontend values when provided to avoid schema/column mismatch in BQ snapshot.
    if current_lat is not None:
        snapshot["lat"] = current_lat
    if current_lon is not None:
        snapshot["lon"] = current_lon
    if route:
        route_key = str(route).strip().upper()
        if route_key and route_key not in {"N/A", "UNKNOWN", "LOADING"}:
            snapshot["route"] = route_key
    if gs_kt is not None:
        gs = gs_kt

    if not flight_id_norm and snapshot.get("flight_id"):
        flight_id_norm = _normalize_flight_id(str(snapshot.get("flight_id")))

    if not phase:
        try:
            df_final = _prepare_sequence(icao)
            phase = _predict_phase_name(df_final)
        except Exception:
            phase = "Unknown"

    # Run expensive/blocking estimations concurrently to reduce frontend timeout risk.
    fuel_task = asyncio.to_thread(
        _estimate_fuel_rate,
        flight_id=flight_id_norm,
        phase=phase or "",
        icao_hex=icao,
        altitude_ft=float(alt) if alt is not None else None,
        ground_speed_kt=float(gs) if gs is not None else None,
    )
    elapsed_gemini_task = asyncio.to_thread(
        _estimate_elapsed_from_gemini,
        route_text=snapshot.get("route"),
        cur_lat=snapshot.get("lat"),
        cur_lon=snapshot.get("lon"),
        gs_kt=gs,
        phase=phase,
    )
    time_range_task = asyncio.to_thread(_get_flight_time_range, icao, flight_id_norm, time_window_hr)

    (rate, source), gemini_elapsed_hr, (start_raw, end_raw) = await asyncio.gather(
        fuel_task, elapsed_gemini_task, time_range_task
    )

    now_utc = datetime.now(timezone.utc)
    start_time = _coerce_snapshot_time_to_datetime(start_raw, now_utc)
    end_time = _coerce_snapshot_time_to_datetime(end_raw, now_utc)

    if not end_time:
        end_time = _coerce_snapshot_time_to_datetime(snapshot.get("Time_MSG_Generated"), now_utc) or now_utc
    if not start_time:
        start_time = end_time
    if start_time > end_time:
        start_time = end_time

    safe_rate = _safe_float(rate, default=0.0)
    safe_elapsed_hr = _safe_float((end_time - start_time).total_seconds() / 3600.0, default=0.0)
    elapsed_hr = max(0.0, safe_elapsed_hr)
    elapsed_source = "observed"
    estimated = False
    route_elapsed_hr: Optional[float] = None

    # If observed duration is missing/nearly-zero, use already-fetched Gemini estimate first.
    if elapsed_hr < 0.03:  # < ~2 minutes
        if gemini_elapsed_hr is not None and gemini_elapsed_hr > elapsed_hr:
            elapsed_hr = gemini_elapsed_hr
            elapsed_source = "estimated_gemini"
            estimated = True

    # Secondary fallback: route distance estimate.
    if elapsed_hr < 0.03:
        route_elapsed_hr = _estimate_elapsed_from_route(
            route_text=snapshot.get("route"),
            cur_lat=snapshot.get("lat"),
            cur_lon=snapshot.get("lon"),
            gs_kt=gs,
        )
        if route_elapsed_hr is not None and route_elapsed_hr > elapsed_hr:
            elapsed_hr = route_elapsed_hr
            elapsed_source = "estimated_route_distance"
            estimated = True

    print(
        "[fuel_summary] "
        f"icao={icao} flight_id={flight_id_norm or 'N/A'} route={snapshot.get('route') or 'N/A'} "
        f"phase={phase or 'UNKNOWN'} rate={safe_rate:.1f}({source}) "
        f"elapsed_observed_hr={safe_elapsed_hr:.3f} "
        f"elapsed_gemini_hr={(f'{gemini_elapsed_hr:.3f}' if gemini_elapsed_hr is not None else 'None')} "
        f"elapsed_route_hr={(f'{route_elapsed_hr:.3f}' if route_elapsed_hr is not None else 'None')} "
        f"elapsed_final_hr={elapsed_hr:.3f} source={elapsed_source} estimated={estimated}"
    )

    fuel_used = _safe_float(safe_rate * elapsed_hr, default=0.0)
    co2_kg = _safe_float(fuel_used * DEFAULT_CO2_KG_PER_KG_FUEL, default=0.0)

    return FuelSummaryResponse(
        icao=icao,
        flight_id=flight_id_norm or None,
        phase=phase,
        fuel_rate_kg_hr=safe_rate,
        fuel_used_kg=fuel_used,
        co2_kg=co2_kg,
        elapsed_hr=float(elapsed_hr),
        rate_source=source,
        elapsed_source=elapsed_source,
        estimated=estimated,
        updated_at=datetime.now(timezone.utc),
        start_time=start_time,
        end_time=end_time,
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
        print(f"[weather_overlay] Open-Meteo fetch failed: {repr(e)}")
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

    wind_by_point: dict[tuple[float, float], tuple[float, float]] = {}
    try:
        loc_hourlies = open_meteo_client.fetch_hourly(
            latitudes=[p[0] for p in all_pts],
            longitudes=[p[1] for p in all_pts],
            hourly_vars=[wind_speed_var, wind_dir_var],
            forecast_days=2,
            timezone_name="UTC",
        )

        # Map each unique point to (ws_kmh, wd_from_deg) at eval_time
        for (lat, lon), lh in zip(all_pts, loc_hourlies):
            df = lh.hourly
            idx = open_meteo_client.pick_time_index(df["time"], eval_time, mode="nearest")
            try:
                ws = _safe_float(df.loc[idx, wind_speed_var], default=0.0)
                wd = _safe_float(df.loc[idx, wind_dir_var], default=0.0)
            except Exception:
                ws = 0.0
                wd = 0.0
            wind_by_point[(lat, lon)] = (ws, wd)
    except Exception:
        # Fallback for transient weather API failures: continue with no-wind assumption.
        for lat, lon in all_pts:
            wind_by_point[(lat, lon)] = (0.0, 0.0)

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
