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
    
    # 1. Setup Mock BigQuery Result (DataFrame)
    mock_df = pd.DataFrame({
        'Altitude': [35000]*30,
        'GroundSpeed': [450]*30,
        'VerticalRate': [0]*30,
        'Track': [90]*30,
        'Time_MSG_Generated': pd.date_range(start='2024-01-01', periods=30, freq='s')
    })
    
    mock_query_job = MagicMock()
    mock_query_job.to_dataframe.return_value = mock_df # Return Mock DataFrame
    
    # Apply the mock to the client instance inside main
    with patch("main.bq_client") as mock_bq_client:
        mock_bq_client.query.return_value = mock_query_job
        
        # 2. Setup Mock Model/Scaler (We need them to be present in main)
        mock_model = MagicMock()
        mock_model.predict.return_value = [1] # Return integer (mapped to Cruise)
        mock_scaler = MagicMock()
        # Scaler expects (30, 4) input
        mock_scaler.transform.return_value = np.random.rand(30, 4)

        with patch("main.model", mock_model), patch("main.scaler", mock_scaler):
            
            # 3. Call API
            response = client.get("/predict/AABBCC")
            
            # 4. Assertions
            assert response.status_code == 200
            data = response.json()
            assert data["icao"] == "AABBCC"
            assert data["phase"] == "Cruise" # 1 maps to Cruise
            # The features list should now contain the rows from the DataFrame
            assert len(data["features"]) == 30 
            assert data["features"][0]["Altitude"] == 35000

def test_predict_no_data(mock_bigquery):
    """Test prediction when BigQuery returns no data."""
    
    mock_query_job = MagicMock()
    mock_query_job.to_dataframe.return_value = pd.DataFrame() # Empty DataFrame
    
    with patch("main.bq_client") as mock_bq_client:
        mock_bq_client.query.return_value = mock_query_job
        
        with patch("main.model", MagicMock()), patch("main.scaler", MagicMock()):
            response = client.get("/predict/UNKNOWN")
            assert response.status_code == 200
            assert response.json()["phase"] == "Unknown"


def test_speech_qa_not_question_returns_early():
    with patch(
        "main._call_gemini_for_speech_parse",
        return_value={"is_question": False, "intent": "unsupported", "icao": "", "confidence": 0.91},
    ), patch("main._get_latest_snapshot") as mock_snapshot:
        response = client.post("/speech/qa", json={"transcript": "hello team"})
        assert response.status_code == 200
        data = response.json()
        assert data["is_question"] is False
        assert data["intent"] == "not_question"
        assert data["data_source"] == "none"
        mock_snapshot.assert_not_called()


def test_speech_qa_location_from_snapshot():
    with patch(
        "main._call_gemini_for_speech_parse",
        return_value={"is_question": True, "intent": "location", "icao": "A01BBC", "confidence": 0.8},
    ), patch("main._get_latest_snapshot", return_value={}):
        response = client.post(
            "/speech/qa",
            json={
                "transcript": "where is a01bbc",
                "snapshot": {"icao": "A01BBC", "lat": 40.12345, "lon": -79.54321},
            },
        )
        assert response.status_code == 200
        data = response.json()
        assert data["intent"] == "location"
        assert data["data_source"] == "snapshot"
        assert "latitude 40.12345" in data["answer"]
        assert "longitude -79.54321" in data["answer"]


def test_speech_qa_altitude_falls_back_to_bigquery():
    with patch(
        "main._call_gemini_for_speech_parse",
        return_value={"is_question": True, "intent": "altitude", "icao": "A01BBC", "confidence": 0.7},
    ), patch("main._get_latest_snapshot", return_value={"Altitude": 12000}):
        response = client.post(
            "/speech/qa",
            json={
                "transcript": "what is altitude of a01bbc",
                "snapshot": {"icao": "A01BBC", "lat": 40.0},
            },
        )
        assert response.status_code == 200
        data = response.json()
        assert data["intent"] == "altitude"
        assert data["data_source"] == "hybrid"
        assert "12000 ft" in data["answer"]


def test_speech_qa_destination_route():
    with patch(
        "main._call_gemini_for_speech_parse",
        return_value={"is_question": True, "intent": "destination", "icao": "A01BBC", "confidence": 0.7},
    ), patch("main._get_latest_snapshot", return_value={"route": "LGD-PIT"}):
        response = client.post("/speech/qa", json={"transcript": "where is destination of a01bbc"})
        assert response.status_code == 200
        data = response.json()
        assert data["intent"] == "destination"
        assert "LGD-PIT" in data["answer"]


def test_speech_qa_speed_and_heading():
    with patch(
        "main._call_gemini_for_speech_parse",
        side_effect=[
            {"is_question": True, "intent": "speed", "icao": "A01BBC", "confidence": 0.8},
            {"is_question": True, "intent": "heading", "icao": "A01BBC", "confidence": 0.8},
        ],
    ), patch("main._get_latest_snapshot", return_value={}):
        speed_response = client.post(
            "/speech/qa",
            json={"transcript": "speed of a01bbc", "snapshot": {"icao": "A01BBC", "speed_kt": 455.0}},
        )
        heading_response = client.post(
            "/speech/qa",
            json={"transcript": "heading of a01bbc", "snapshot": {"icao": "A01BBC", "heading_deg": 271.0}},
        )
        assert speed_response.status_code == 200
        assert heading_response.status_code == 200
        assert speed_response.json()["intent"] == "speed"
        assert "455 kt" in speed_response.json()["answer"]
        assert heading_response.json()["intent"] == "heading"
        assert "271 degrees" in heading_response.json()["answer"]


def test_speech_qa_missing_icao_prompt():
    with patch(
        "main._call_gemini_for_speech_parse",
        return_value={"is_question": True, "intent": "location", "icao": "", "confidence": 0.3},
    ):
        response = client.post("/speech/qa", json={"transcript": "where is it"})
        assert response.status_code == 200
        data = response.json()
        assert data["intent"] == "missing_icao"
        assert "6-character ICAO" in data["answer"]


def test_speech_qa_unsupported_intent():
    with patch(
        "main._call_gemini_for_speech_parse",
        return_value={"is_question": True, "intent": "unsupported", "icao": "", "confidence": 0.2},
    ):
        response = client.post("/speech/qa", json={"transcript": "who is the pilot of a01bbc"})
        assert response.status_code == 200
        data = response.json()
        assert data["intent"] == "unsupported"
        assert "I can answer location" in data["answer"]


def test_speech_qa_not_found_field():
    with patch(
        "main._call_gemini_for_speech_parse",
        return_value={"is_question": True, "intent": "speed", "icao": "A01BBC", "confidence": 0.6},
    ), patch("main._get_latest_snapshot", return_value={}):
        response = client.post("/speech/qa", json={"transcript": "what speed is a01bbc"})
        assert response.status_code == 200
        data = response.json()
        assert data["intent"] == "not_found"
        assert "speed is not available" in data["answer"]


def test_speech_qa_gemini_parse_failure_uses_regex_fallback():
    with patch("main._call_gemini_for_speech_parse", return_value=None), patch(
        "main._get_latest_snapshot", return_value={"Altitude": 12000}
    ):
        response = client.post("/speech/qa", json={"transcript": "what is the altitude of A01BBC"})
        assert response.status_code == 200
        data = response.json()
        assert data["intent"] == "altitude"
        assert data["icao"] == "A01BBC"
        assert "12000 ft" in data["answer"]

# --- Integration Test (Optional - requires real credentials) ---
@pytest.mark.skipif(not os.path.exists(os.path.join(os.path.dirname(__file__), "..", "..", "BigQuery", "YourJsonFile.json")), reason="No credentials file")
def test_integration_bigquery_connection():
    """
    Real connection test. 
    WARNING: This will fail if 'YourJsonFile.json' is not valid or network is down.
    """
    from google.cloud import bigquery
    
    cred_path = os.path.join(os.path.dirname(__file__), "..", "..", "BigQuery", "YourJsonFile.json")
    os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = cred_path
    
    try:
        bq_client = bigquery.Client()
        query = "SELECT 1"
        job = bq_client.query(query)
        result = list(job.result())
        assert result[0][0] == 1
    except Exception as e:
        pytest.fail(f"Integration test failed: {e}")
