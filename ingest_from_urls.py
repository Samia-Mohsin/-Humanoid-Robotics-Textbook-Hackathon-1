#!/usr/bin/env python3
"""
Re-ingest the textbook content from online URLs to fix the source_url issue.
This script will use the actual online textbook URLs instead of local files.
"""
import os
import sys
import time
from pathlib import Path
from typing import List
import xml.etree.ElementTree as ET

# Add the backend directory to the path so we can import the modules
backend_path = Path(__file__).parent / "backend"
sys.path.insert(0, str(backend_path))

from config import Config
from main import main_pipeline
from sitemap_processor import extract_urls_from_sitemap, validate_sitemap_urls


def get_textbook_urls_from_sitemap() -> List[str]:
    """
    Extract textbook URLs from the built sitemap.
    """
    sitemap_path = Path(__file__).parent / "frontend-book" / "build" / "sitemap.xml"

    if not sitemap_path.exists():
        print(f"Sitemap not found at {sitemap_path}")
        return []

    # Read the sitemap file
    with open(sitemap_path, 'r', encoding='utf-8') as f:
        sitemap_content = f.read()

    # Parse the sitemap to extract URLs
    urls = []
    try:
        root = ET.fromstring(sitemap_content)

        # Define namespaces for XML parsing
        namespaces = {
            'sitemap': 'http://www.sitemaps.org/schemas/sitemap/0.9',
        }

        # Extract URLs from the sitemap
        for url_elem in root.findall('.//sitemap:url', namespaces):
            loc_elem = url_elem.find('sitemap:loc', namespaces)
            if loc_elem is not None:
                urls.append(loc_elem.text)

    except ET.ParseError as e:
        print(f"Error parsing sitemap XML: {e}")
        return []

    return urls


def get_textbook_urls_from_config() -> List[str]:
    """
    Alternative method: get textbook URLs from a configuration file.
    """
    # If we have a config file with textbook URLs, we could read from there
    # For now, we'll use the sitemap approach
    return []


def main():
    """Main function to re-ingest content from online URLs."""
    print("Starting re-ingestion from online textbook URLs...")

    # Get the URLs to process
    print("Extracting textbook URLs from sitemap...")
    urls = get_textbook_urls_from_sitemap()

    if not urls:
        print("No URLs found in sitemap. Checking if we can get them from config...")
        urls = get_textbook_urls_from_config()

    if not urls:
        print("No URLs found to process. Using sample textbook URLs.")
        # Fallback to a few sample URLs based on the sitemap we saw
        urls = [
            "https://samia.github.io/docs/intro",
            "https://samia.github.io/docs/ros2-basics/",
            "https://samia.github.io/docs/ros2-basics/why-ros2-for-humanoids",
            "https://samia.github.io/docs/ros2-basics/dds-concepts",
            "https://samia.github.io/docs/communication-model/",
            "https://samia.github.io/docs/communication-model/nodes-topics-services",
            "https://samia.github.io/docs/communication-model/practical-agent-controller",
            "https://samia.github.io/docs/robot-structure/",
            "https://samia.github.io/docs/robot-structure/urdf-basics",
            "https://samia.github.io/docs/robot-structure/humanoid-urdf-examples",
            "https://samia.github.io/docs/vla-integration/",
            "https://samia.github.io/docs/vla-integration/vla-system-overview/",
            "https://samia.github.io/docs/vla-integration/language-to-intent/",
            "https://samia.github.io/docs/advanced-digital-twin/",
            "https://samia.github.io/docs/isaac-ai-brain/",
            "https://samia.github.io/docs/capstone-project/"
        ]

    print(f"Found {len(urls)} URLs to process")

    # Limit to first 10 URLs for testing to avoid rate limits
    urls = urls[:10]
    print(f"Processing first {len(urls)} URLs (limited for testing)")

    # Load configuration
    try:
        config = Config.from_env()
        print("Configuration loaded successfully")
    except Exception as e:
        print(f"Error loading configuration: {e}")
        print("Make sure you have the required environment variables set")
        return 1

    # Set collection name for textbook content
    collection_name = "humanoid_robotics_docs"

    # Process URLs through the pipeline
    try:
        print(f"Starting ingestion pipeline for {len(urls)} URLs...")
        results = main_pipeline(urls, config, collection_name)

        print(f"Pipeline completed with results: {results}")

        if results.get('storage_success', False):
            print("✅ Content successfully ingested from online URLs!")
            print(f"   - {results.get('urls_processed', 0)} URLs processed")
            print(f"   - {results.get('documents_extracted', 0)} documents extracted")
            print(f"   - {results.get('chunks_created', 0)} chunks created")
            print(f"   - {results.get('embeddings_generated', 0)} embeddings generated")
            print(f"   - Execution time: {results.get('execution_time_seconds', 0):.2f} seconds")
        else:
            print("❌ Pipeline completed but storage failed")
            return 1

    except Exception as e:
        print(f"❌ Error during ingestion: {e}")
        import traceback
        traceback.print_exc()
        return 1

    return 0


if __name__ == "__main__":
    exit(main())