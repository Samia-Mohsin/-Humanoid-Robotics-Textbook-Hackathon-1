#!/usr/bin/env python3
"""
Test script to verify sitemap URL extraction functionality
"""
import sys
import os

# Add the backend directory to the path so we can import our modules
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import using absolute imports instead of relative
from sitemap_processor import extract_urls_from_sitemap, validate_sitemap_urls
from utils import validate_url
from config import Config

def test_sitemap_extraction():
    """Test sitemap URL extraction with the humanoid robotics textbook sitemap"""
    print("Testing sitemap URL extraction...")

    # Test sitemap URL
    sitemap_url = "https://humanoid-robotics-textbook-hackatho-seven.vercel.app/sitemap.xml"

    print(f"Fetching sitemap: {sitemap_url}")

    # Extract URLs from sitemap
    urls = extract_urls_from_sitemap(sitemap_url)

    print(f"Extracted {len(urls)} URLs from sitemap")

    if urls:
        print("\nFirst few URLs found:")
        for i, url_data in enumerate(urls[:5]):  # Show first 5 URLs
            print(f"  {i+1}. {url_data['url']}")
            if url_data['last_modified']:
                print(f"     Last modified: {url_data['last_modified']}")
            if url_data['change_frequency']:
                print(f"     Change frequency: {url_data['change_frequency']}")
            if url_data['priority']:
                print(f"     Priority: {url_data['priority']}")

        if len(urls) > 5:
            print(f"  ... and {len(urls) - 5} more URLs")

    # Validate the extracted URLs
    print(f"\nValidating {len(urls)} URLs...")
    validated_urls = validate_sitemap_urls(urls)
    print(f"Validated {len(validated_urls)} URLs")

    if len(validated_urls) != len(urls):
        print(f"Warning: {len(urls) - len(validated_urls)} URLs failed validation")

    # Print summary
    print(f"\nSummary:")
    print(f"  Total URLs extracted: {len(urls)}")
    print(f"  Valid URLs: {len(validated_urls)}")
    print(f"  Success rate: {len(validated_urls)/len(urls)*100:.1f}% " if urls else "0% (no URLs found)")

    return urls, validated_urls

if __name__ == "__main__":
    test_sitemap_extraction()