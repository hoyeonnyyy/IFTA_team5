import pytest
from fastapi.testclient import TestClient
from unittest.mock import MagicMock, patch
import sys
import os
import pandas as pd
import joblib
import numpy as np

# Add parent directory to path to import main
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from main import app

client = TestClient(app)

# --- Fixtures for Mocking ---

@pytest.fixture
def mock_model_scaler():
    """Mocks the joblib load to return a fake model and scaler."""
    with patch("joblib.load") as mock_load:
        # Mock Model
        mock_model = MagicMock()
        mock_model.predict.return_value = ["CRUISE"]
        
        # Mock Scaler
        mock_scaler = MagicMock()
        mock_scaler.transform.return_value = np.array([[0.5, 0.5, 0.0, 0.5]])
        
        # Side effect to return scaler then model (or vice versa depending on load order in main)
        # In main: model loaded first, then scaler
        mock_load.side_effect = [mock_model, mock_scaler]
        
        yield mock_model, mock_scaler

@pytest.fixture
def mock_bigquery():
    """Mocks the BigQuery client."""
    with patch("google.cloud.bigquery.Client") as mock_client_cls:
        mock_client = MagicMock()
        mock_client_cls.return_value = mock_client
        yield mock_client

# --- Tests ---

def test_health_check(mock_model_scaler, mock_bigquery):
    """Test the health check endpoint."""
    with patch("main.model", "dummy_model"), patch("main.scaler", "dummy_scaler"), patch("main.bq_client", "dummy_client"):
        response = client.get("/health")
        assert response.status_code == 200
        assert response.json() == {"status": "ok", "model_loaded": True, "bq_connected": True}

def test_predict_mock_data(mock_bigquery):
    """Test prediction with mocked BigQuery data."""
    
    # 1. Setup Mock BigQuery Result
    mock_row = MagicMock()
    mock_row.Altitude = 35000
    mock_row.GroundSpeed = 450
    mock_row.VerticalRate = 0
    mock_row.Track = 90
    
    mock_query_job = MagicMock()
    mock_query_job.result.return_value = [mock_row] # Return list with one row
    
    # Apply the mock to the client instance inside main
    with patch("main.bq_client") as mock_bq_client:
        mock_bq_client.query.return_value = mock_query_job
        
        # 2. Setup Mock Model/Scaler (We need them to be present in main)
        mock_model = MagicMock()
        mock_model.predict.return_value = ["CRUISE"]
        mock_scaler = MagicMock()
        mock_scaler.transform.return_value = [[1, 2, 3, 4]]

        with patch("main.model", mock_model), patch("main.scaler", mock_scaler):
            
            # 3. Call API
            response = client.get("/predict/AABBCC")
            
            # 4. Assertions
            assert response.status_code == 200
            data = response.json()
            assert data["icao"] == "AABBCC"
            assert data["phase"] == "CRUISE"
            assert data["features"]["Altitude"] == 35000

def test_predict_no_data(mock_bigquery):
    """Test prediction when BigQuery returns no data."""
    
    mock_query_job = MagicMock()
    mock_query_job.result.return_value = [] # Empty list
    
    with patch("main.bq_client") as mock_bq_client:
        mock_bq_client.query.return_value = mock_query_job
        
        with patch("main.model", MagicMock()), patch("main.scaler", MagicMock()):
            response = client.get("/predict/UNKNOWN")
            assert response.status_code == 404
            assert "No data found" in response.json()["detail"]

# --- Integration Test (Optional - requires real credentials) ---
@pytest.mark.skipif(not os.path.exists(os.path.join(os.path.dirname(__file__), "..", "BigQueryCred.json")), reason="No credentials file")
def test_integration_bigquery_connection():
    """
    Real connection test. 
    WARNING: This will fail if 'BigQueryCred.json' is not valid or network is down.
    """
    from google.cloud import bigquery
    
    cred_path = os.path.join(os.path.dirname(__file__), "..", "BigQueryCred.json")
    os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = cred_path
    
    try:
        bq_client = bigquery.Client()
        query = "SELECT 1"
        job = bq_client.query(query)
        result = list(job.result())
        assert result[0][0] == 1
    except Exception as e:
        pytest.fail(f"Integration test failed: {e}")
