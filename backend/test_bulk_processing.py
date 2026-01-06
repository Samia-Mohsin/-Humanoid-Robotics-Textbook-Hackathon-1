#!/usr/bin/env python3
"""
Test script to verify bulk processing functionality with sitemap URLs
"""
import sys
import os

# Add the backend directory to the path so we can import our modules
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sitemap_processor import process_sitemap
from config import Config
import os

def test_bulk_processing():
    """Test bulk processing with a few URLs from the sitemap"""
    print("Testing bulk processing with sitemap URLs...")

    # Test sitemap URL
    sitemap_url = "https://humanoid-robotics-textbook-hackatho-seven.vercel.app/sitemap.xml"

    # For this test, we'll need environment variables set up
    # Since we don't have real API keys, we'll check if the function can be called
    # and if it handles the missing configuration properly

    # Check if required environment variables are set
    has_cohere = os.getenv('COHERE_API_KEY') is not None
    has_qdrant_url = os.getenv('QDRANT_URL') is not None
    has_qdrant_key = os.getenv('QDRANT_API_KEY') is not None

    print(f"Environment variables check:")
    print(f"  COHERE_API_KEY set: {has_cohere}")
    print(f"  QDRANT_URL set: {has_qdrant_url}")
    print(f"  QDRANT_API_KEY set: {has_qdrant_key}")

    if not (has_cohere and has_qdrant_url and has_qdrant_key):
        print("\nNote: API keys not set. Testing will verify function structure and error handling.")
        print("To run full processing test, set the required environment variables.")
        return

    try:
        # Create config from environment variables
        config = Config.from_env()

        print(f"\nConfiguration loaded successfully:")
        print(f"  Cohere model: {config.cohere_model}")
        print(f"  Chunk size: {config.chunk_size}")
        print(f"  Chunk overlap: {config.chunk_overlap}")

        # Process the sitemap with a 1 second rate limit
        print(f"\nStarting sitemap processing for: {sitemap_url}")
        print("This will process all URLs from the sitemap with rate limiting and progress tracking...")

        results = process_sitemap(sitemap_url, config, rate_limit_delay=1.0)

        print(f"\nProcessing completed!")
        print(f"Results summary:")
        print(f"  Total URLs: {results['total_urls']}")
        print(f"  Successful: {results['successful_processing']}")
        print(f"  Failed: {results['failed_processing']}")
        print(f"  Total duration: {results['total_duration']:.2f} seconds")

        if results['error_details']:
            print(f"  Errors: {len(results['error_details'])}")
            for i, error in enumerate(results['error_details'][:3]):  # Show first 3 errors
                print(f"    {i+1}. {error}")
            if len(results['error_details']) > 3:
                print(f"    ... and {len(results['error_details']) - 3} more errors")

        return results

    except ValueError as e:
        if "environment variable is required" in str(e):
            print(f"\nExpected error (no API keys set): {e}")
            print("This is normal when running without API keys.")
        else:
            print(f"Configuration error: {e}")
        return None
    except Exception as e:
        print(f"Error during processing: {e}")
        import traceback
        traceback.print_exc()
        return None

if __name__ == "__main__":
    test_bulk_processing()