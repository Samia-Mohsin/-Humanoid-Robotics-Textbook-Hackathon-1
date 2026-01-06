#!/usr/bin/env python3
"""
Simple test to diagnose the TestClient issue
"""
import sys
from pathlib import Path

# Add the project root to the path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

def test_basic_fastapi_app():
    """Test creating a basic FastAPI app without middleware issues."""
    from fastapi import FastAPI

    # Create a simple app without complex middleware
    simple_app = FastAPI(title="Test App")

    @simple_app.get("/")
    def root():
        return {"message": "test"}

    return simple_app

def test_with_simple_app():
    """Test TestClient with a simple app."""
    from fastapi.testclient import TestClient

    simple_app = test_basic_fastapi_app()

    try:
        client = TestClient(simple_app)
        print("SUCCESS: TestClient works with simple FastAPI app")

        response = client.get("/")
        print(f"SUCCESS: Simple app response: {response.json()}")
        return True
    except Exception as e:
        print(f"ERROR with simple app: {e}")
        return False

def test_original_app_structure():
    """Test the original app structure to identify the issue."""
    from fastapi.testclient import TestClient
    from api import app

    print("Trying to create TestClient with original app...")
    try:
        client = TestClient(app)
        print("SUCCESS: TestClient created with original app")

        response = client.get("/")
        print(f"SUCCESS: Original app response status: {response.status_code}")
        return True
    except Exception as e:
        print(f"ERROR with original app: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("Testing TestClient compatibility...\n")

    print("1. Testing with simple FastAPI app:")
    simple_success = test_with_simple_app()

    print("\n2. Testing with original app:")
    original_success = test_original_app_structure()

    print(f"\nResults:")
    print(f"- Simple app: {'PASS' if simple_success else 'FAIL'}")
    print(f"- Original app: {'PASS' if original_success else 'FAIL'}")

    if original_success:
        print("\n✅ TestClient issue resolved!")
    else:
        print("\n❌ TestClient issue still exists - need to investigate further")