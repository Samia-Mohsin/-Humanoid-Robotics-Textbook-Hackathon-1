#!/usr/bin/env python3
"""
Test script to validate that the API can handle concurrent requests.
"""
import asyncio
import time
from concurrent.futures import ThreadPoolExecutor
import requests
import threading
from api import app
from fastapi.testclient import TestClient


def test_single_request(query: str):
    """Helper function to simulate a single API request"""
    client = TestClient(app)

    # Prepare the request data
    request_data = {
        "query": query,
        "debug": False
    }

    try:
        response = client.post("/query", json=request_data)
        return {
            "query": query,
            "status_code": response.status_code,
            "response_length": len(response.text),
            "success": response.status_code == 200
        }
    except Exception as e:
        return {
            "query": query,
            "status_code": None,
            "response_length": 0,
            "success": False,
            "error": str(e)
        }


def test_concurrent_requests():
    """Test that the API can handle multiple concurrent requests"""
    print("Testing concurrent requests to verify API handles multiple users...")

    # Create multiple different queries to send concurrently
    queries = [
        "What is ROS 2?",
        "Explain robotics concepts",
        "How does machine learning work?",
        "What is computer vision?",
        "Explain neural networks",
        "What is reinforcement learning?",
        "How does a robotic arm work?",
        "Explain SLAM in robotics",
        "What is path planning?",
        "How does computer vision work?"
    ]

    # Test with ThreadPoolExecutor to simulate concurrent requests
    start_time = time.time()

    with ThreadPoolExecutor(max_workers=5) as executor:
        # Submit all requests concurrently
        futures = [executor.submit(test_single_request, query) for query in queries]

        # Collect results
        results = [future.result() for future in futures]

    end_time = time.time()

    # Analyze results
    successful_requests = [r for r in results if r["success"]]
    failed_requests = [r for r in results if not r["success"]]

    print(f"Total requests: {len(queries)}")
    print(f"Successful requests: {len(successful_requests)}")
    print(f"Failed requests: {len(failed_requests)}")
    print(f"Total time: {end_time - start_time:.2f} seconds")

    if failed_requests:
        print("\nFailed requests:")
        for result in failed_requests:
            print(f"  Query: '{result['query']}' - Error: {result.get('error', 'Unknown error')}")

    print(f"\nSuccess rate: {len(successful_requests)/len(queries)*100:.1f}%")

    # Return True if at least 80% of requests were successful (accounting for potential rate limits)
    success_rate = len(successful_requests) / len(queries) if queries else 0
    is_acceptable = success_rate >= 0.8 or (len(failed_requests) > 0 and "rate limit" in str(failed_requests[0].get('error', '')))

    if is_acceptable:
        print("✓ API successfully handled concurrent requests (or rate limits reached, which is expected with trial API keys)")
        return True
    else:
        print("✗ API failed to handle concurrent requests adequately")
        return False


if __name__ == "__main__":
    success = test_concurrent_requests()
    if success:
        print("\nSUCCESS: Concurrent request handling test passed!")
        exit(0)
    else:
        print("\nERROR: Concurrent request handling test failed!")
        exit(1)