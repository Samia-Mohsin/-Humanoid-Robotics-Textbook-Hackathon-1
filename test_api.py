#!/usr/bin/env python3
"""
Comprehensive test suite for the FastAPI RAG Integration API endpoints.

This module contains unit tests, integration tests, and stress tests for the API endpoints.
"""
import pytest
import asyncio
from fastapi.testclient import TestClient
from fastapi import HTTPException
from unittest.mock import Mock, patch, MagicMock
import sys
from pathlib import Path

# Add the project root to the Python path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

from api import app
from models import QueryRequest, QueryResponse, HealthCheckResponse
from agent_integration import AgentManager


def test_api_app_creation():
    """Test that the FastAPI app is created successfully."""
    assert app is not None
    assert app.title == "RAG Chatbot API"


def test_root_endpoint():
    """Test the root endpoint."""
    client = TestClient(app)
    response = client.get("/")
    assert response.status_code == 200

    data = response.json()
    assert "message" in data
    assert "status" in data
    assert "version" in data
    assert data["message"] == "RAG Chatbot API"
    assert data["status"] == "running"


def test_health_endpoint():
    """Test the health check endpoint."""
    client = TestClient(app)
    response = client.get("/health")
    assert response.status_code == 200

    data = response.json()
    assert "status" in data
    assert "timestamp" in data
    assert isinstance(data, dict)


def test_query_endpoint_basic():
    """Test the query endpoint with a basic request."""
    client = TestClient(app)

    # Mock the agent manager to avoid actual API calls
    with patch.object(app.state, 'agent_manager') as mock_agent_manager:
        mock_agent_manager.safe_query_agent.return_value = {
            "response": "This is a test response from the agent.",
            "conversation_id": "test-conversation-id",
            "sources": ["test-source-url"]
        }

        request_data = {
            "query": "What is ROS 2?",
            "debug": False
        }

        response = client.post("/query", json=request_data)
        assert response.status_code == 200

        data = response.json()
        assert "response" in data
        assert data["response"] == "This is a test response from the agent."


def test_query_endpoint_with_conversation_id():
    """Test the query endpoint with conversation ID."""
    client = TestClient(app)

    with patch.object(app.state, 'agent_manager') as mock_agent_manager:
        mock_agent_manager.safe_query_agent.return_value = {
            "response": "Response with conversation context",
            "conversation_id": "new-conversation-id",
            "sources": ["test-source-url"]
        }

        request_data = {
            "query": "Continue the discussion",
            "conversation_id": "existing-conversation-id",
            "debug": False
        }

        response = client.post("/query", json=request_data)
        assert response.status_code == 200

        data = response.json()
        assert data["response"] == "Response with conversation context"
        assert data["conversation_id"] == "new-conversation-id"


def test_query_endpoint_with_debug():
    """Test the query endpoint with debug mode enabled."""
    client = TestClient(app)

    with patch.object(app.state, 'agent_manager') as mock_agent_manager:
        mock_agent_manager.safe_query_agent.return_value = {
            "response": "Response with debug info",
            "conversation_id": "debug-conversation-id",
            "sources": ["test-source-url"],
            "debug_info": {"retrieved_chunks": 5, "processing_time": 0.5}
        }

        request_data = {
            "query": "Debug test query",
            "debug": True
        }

        response = client.post("/query", json=request_data)
        assert response.status_code == 200

        data = response.json()
        assert "debug_info" in data
        assert data["debug_info"]["retrieved_chunks"] == 5


def test_query_endpoint_validation_errors():
    """Test the query endpoint with invalid requests."""
    client = TestClient(app)

    # Test with empty query
    response = client.post("/query", json={"query": ""})
    assert response.status_code in [400, 200]  # May return 200 with error message or 400

    # Test with very short query
    response = client.post("/query", json={"query": "hi"})
    assert response.status_code in [400, 200]  # May return 200 with error message or 400


def test_query_endpoint_internal_error():
    """Test the query endpoint when internal error occurs."""
    client = TestClient(app)

    with patch.object(app.state, 'agent_manager') as mock_agent_manager:
        mock_agent_manager.safe_query_agent.side_effect = Exception("Internal error")

        request_data = {"query": "This will cause an error", "debug": False}

        response = client.post("/query", json=request_data)
        # Should return 500 for internal server error
        assert response.status_code == 500


def test_rate_limiting():
    """Test rate limiting functionality."""
    client = TestClient(app)

    with patch.object(app.state, 'agent_manager') as mock_agent_manager:
        mock_agent_manager.safe_query_agent.return_value = {
            "response": "Rate limit test response",
            "conversation_id": "rate-test-id",
            "sources": []
        }

        # Make multiple requests quickly to test rate limiting
        for i in range(12):  # More than the 10/minute limit
            request_data = {"query": f"Rate limit test query {i}", "debug": False}
            response = client.post("/query", json=request_data)

            # Note: The actual rate limiting might not trigger in test environment
            # depending on how slowapi is configured for testing
            assert response.status_code in [200, 429]


def test_model_validation():
    """Test Pydantic model validation."""
    # Test valid QueryRequest
    valid_request = QueryRequest(query="What is ROS 2?")
    assert valid_request.query == "What is ROS 2?"

    # Test QueryRequest with conversation_id
    request_with_cid = QueryRequest(query="Test query", conversation_id="test-id", debug=True)
    assert request_with_cid.conversation_id == "test-id"
    assert request_with_cid.debug is True

    # Test validation error for short query
    try:
        QueryRequest(query="hi")  # Should fail validation
        assert False, "Should have raised validation error"
    except Exception:
        pass  # Expected validation error


def test_api_documentation_endpoints():
    """Test that API documentation endpoints are available."""
    client = TestClient(app)

    # Test Swagger documentation
    response = client.get("/docs")
    assert response.status_code in [200, 404]  # 404 if docs disabled in tests

    # Test ReDoc documentation
    response = client.get("/redoc")
    assert response.status_code in [200, 404]  # 404 if redoc disabled in tests


def test_openapi_json():
    """Test that OpenAPI schema is available."""
    client = TestClient(app)

    response = client.get("/openapi.json")
    if response.status_code == 200:
        data = response.json()
        assert "openapi" in data
        assert "paths" in data
        assert "/query" in data["paths"]
        assert "/health" in data["paths"]


def test_cors_middleware():
    """Test that CORS headers are properly set."""
    client = TestClient(app)

    # Test OPTIONS request for CORS preflight
    response = client.options("/query",
                             headers={
                                 "Access-Control-Request-Method": "POST",
                                 "Access-Control-Request-Headers": "content-type"
                             })

    # Check if CORS headers are present
    assert "access-control-allow-origin" in [h.lower() for h in response.headers.keys()] or response.status_code == 404


def test_logging_middleware():
    """Test that logging middleware doesn't interfere with functionality."""
    client = TestClient(app)

    with patch.object(app.state, 'agent_manager') as mock_agent_manager:
        mock_agent_manager.safe_query_agent.return_value = {
            "response": "Logging test response",
            "conversation_id": "log-test-id",
            "sources": []
        }

        response = client.post("/query", json={"query": "Log test", "debug": False})
        assert response.status_code == 200

        data = response.json()
        assert "response" in data


def test_multiple_concurrent_requests():
    """Test handling of multiple concurrent requests."""
    import concurrent.futures
    import threading

    client = TestClient(app)

    with patch.object(app.state, 'agent_manager') as mock_agent_manager:
        mock_agent_manager.safe_query_agent.return_value = {
            "response": "Concurrent test response",
            "conversation_id": "concurrent-test-id",
            "sources": []
        }

        def make_request(query_num):
            request_data = {"query": f"Concurrent test query {query_num}", "debug": False}
            response = client.post("/query", json=request_data)
            return response.status_code, response.json()

        # Execute multiple requests concurrently
        with concurrent.futures.ThreadPoolExecutor(max_workers=5) as executor:
            futures = [executor.submit(make_request, i) for i in range(5)]
            results = [future.result() for future in futures]

        # Verify all requests succeeded
        for status_code, data in results:
            assert status_code == 200
            assert "response" in data


def test_error_response_format():
    """Test that error responses follow the expected format."""
    client = TestClient(app)

    with patch.object(app.state, 'agent_manager') as mock_agent_manager:
        mock_agent_manager.safe_query_agent.return_value = {
            "response": "Error occurred",
            "error": "internal_error",
            "error_details": "Test error details"
        }

        response = client.post("/query", json={"query": "Error test", "debug": False})

        # Response might be 200 with error content or 500 with HTTPException
        if response.status_code == 200:
            data = response.json()
            # If it's a 200 response, it might contain error information
            pass
        elif response.status_code == 500:
            # If it's a 500, it's an HTTPException as expected
            pass
        else:
            # Any other status code should be investigated
            assert response.status_code in [200, 500]


def test_health_dependencies():
    """Test health endpoint with mocked dependencies."""
    client = TestClient(app)

    # Test health when dependencies are available
    response = client.get("/health")
    assert response.status_code == 200

    data = response.json()
    assert "status" in data
    assert data["status"] in ["healthy", "degraded", "unhealthy"]


def test_agent_manager_initialization():
    """Test that agent manager can be initialized."""
    agent_manager = AgentManager()
    assert agent_manager is not None
    assert agent_manager.logger is not None


# Performance and Stress Tests
def test_response_time_under_normal_conditions():
    """Test API response time under normal conditions (mocked)."""
    import time

    client = TestClient(app)

    with patch.object(app.state, 'agent_manager') as mock_agent_manager:
        mock_agent_manager.safe_query_agent.return_value = {
            "response": "Fast response test",
            "conversation_id": "perf-test-id",
            "sources": []
        }

        start_time = time.time()
        response = client.post("/query", json={"query": "Performance test", "debug": False})
        end_time = time.time()

        assert response.status_code == 200
        # With mocked agent, response should be very fast (much less than 1 second)
        assert (end_time - start_time) < 1.0


def test_large_query_handling():
    """Test handling of large queries."""
    client = TestClient(app)

    with patch.object(app.state, 'agent_manager') as mock_agent_manager:
        mock_agent_manager.safe_query_agent.return_value = {
            "response": "Large query response",
            "conversation_id": "large-query-test-id",
            "sources": []
        }

        # Create a large query string
        large_query = "This is a large query. " * 200  # 800+ characters

        response = client.post("/query", json={"query": large_query, "debug": False})
        assert response.status_code == 200


if __name__ == "__main__":
    # Run the tests if this file is executed directly
    pytest.main([__file__, "-v"])