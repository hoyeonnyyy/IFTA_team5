import os
import joblib
import pandas as pd
import numpy as np
from fastapi import FastAPI, HTTPException
from google.cloud import bigquery
from pydantic import BaseModel
from typing import Optional, List, Dict, Any
from contextlib import asynccontextmanager

class PredictionResponse(BaseModel):
    icao: str
    phase: str
    features: List[Dict[str, Any]] # Changed to return list of features used
    confidence: Optional[float] = None

# Global variables for model and scaler
model = None
scaler = None
bq_client = None

# Configuration
PROJECT_ID = "iitp-class-team-5-473114"
DATASET_ID = "Dataset"
TABLE_ID = "FirstRun"  # Or "FirstRun_merged_filtered_added" depending on your live data source
# Assumes BigQueryCred.json is in the same directory as main.py (based on previous conversations)
CREDENTIALS_PATH = os.path.join(os.path.dirname(__file__), "BigQueryCred.json") 

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
