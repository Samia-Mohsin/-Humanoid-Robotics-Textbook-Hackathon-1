#!/usr/bin/env python3
"""
Fix the source URLs in the Qdrant collection to use proper textbook URLs instead of file:// paths.
"""
import os
import json
from pathlib import Path
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models
import re

# Load environment variables
load_dotenv()

def get_file_to_url_mapping():
    """
    Create a mapping from local file paths to proper textbook URLs.
    This is based on the Docusaurus routing where docs/some-path/index.md -> /docs/some-path/
    and docs/some-file.md -> /docs/some-file
    """
    mapping = {}

    # Base URL for the textbook (using the GitHub Pages URL from the sitemap)
    base_url = "https://samia.github.io"

    # Get all markdown files from the docs directory
    docs_dir = Path("frontend-book/docs")

    if docs_dir.exists():
        for file_path in docs_dir.rglob("*.md"):
            # Convert file path to relative path from docs_dir
            rel_path = file_path.relative_to(docs_dir)  # This gives us the path relative to docs/

            # Convert to URL format
            # Remove .md extension and convert to URL path
            url_path = str(rel_path).replace('\\', '/').replace('.md', '').replace('.mdx', '')

            # Handle index files - they become the directory path
            if url_path.endswith('/index'):
                url_path = url_path[:-6]  # Remove '/index'

            full_url = f"{base_url}/docs/{url_path}"

            # Create mapping from absolute file path to URL
            abs_file_path = str(file_path.resolve()).replace('\\', '/')
            mapping[f"file://{abs_file_path}"] = full_url

    return mapping

def fix_source_urls():
    """Fix source URLs in the Qdrant collection."""

    # Get configuration
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    collection_name = os.getenv("QDRANT_COLLECTION_NAME", "humanoid_robotics_docs")

    if not qdrant_url:
        print("QDRANT_URL environment variable not set")
        return 1

    # Create mapping from file paths to URLs
    print("Creating file to URL mapping...")
    file_to_url = get_file_to_url_mapping()

    print(f"Found {len(file_to_url)} file-to-URL mappings")

    # Connect to Qdrant
    print(f"Connecting to Qdrant: {qdrant_url}")
    client = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key,
        timeout=30
    )

    try:
        # Get collection info
        collection_info = client.get_collection(collection_name)
        print(f"Collection '{collection_name}' has {collection_info.points_count} points")

        # Get all points from the collection
        print("Fetching all points from collection...")

        # We'll use scroll to get all points
        all_points = []
        offset = None
        batch_size = 1000

        while True:
            records, next_offset = client.scroll(
                collection_name=collection_name,
                limit=batch_size,
                offset=offset,
                with_payload=True,
                with_vectors=True
            )

            all_points.extend(records)

            if next_offset is None:
                break
            offset = next_offset

            print(f"Fetched {len(all_points)} points so far...")

        print(f"Total points fetched: {len(all_points)}")

        # Process and update points with incorrect URLs
        updated_points = []
        update_count = 0

        for point in all_points:
            payload = point.payload if point.payload else {}

            # Check if the source_url is a file:// URL
            current_source_url = payload.get('source_url', '')

            if current_source_url.startswith('file://'):
                # Normalize the file path (convert to forward slashes)
                normalized_path = current_source_url.replace('\\', '/')

                # Check if we have a mapping for this file (try both original and normalized)
                new_url = None
                if current_source_url in file_to_url:
                    new_url = file_to_url[current_source_url]
                elif normalized_path in file_to_url:
                    new_url = file_to_url[normalized_path]
                else:
                    # Try to match by extracting the relative path
                    abs_path = current_source_url.replace('file://', '')
                    abs_path_normalized = abs_path.replace('\\', '/')
                    rel_path = Path(abs_path_normalized).relative_to(Path.cwd())
                    rel_path_str = str(rel_path).replace('\\', '/')

                    # Check if this relative path exists in our mapping
                    for file_path, url in file_to_url.items():
                        if rel_path_str in file_path or rel_path_str.replace('frontend-book/', '') in file_path.replace('frontend-book/', ''):
                            new_url = url
                            break

                if new_url:
                    # Update the payload with the new URL
                    updated_payload = payload.copy()
                    updated_payload['source_url'] = new_url

                    # Update metadata source_url as well if it exists
                    if 'metadata' in updated_payload and isinstance(updated_payload['metadata'], dict):
                        updated_payload['metadata']['source_url'] = new_url

                    # Create updated point
                    updated_point = models.PointStruct(
                        id=point.id,
                        vector=point.vector,
                        payload=updated_payload
                    )

                    updated_points.append(updated_point)
                    update_count += 1

                    if update_count % 100 == 0:
                        print(f"Updated {update_count} points...")

        print(f"Found {update_count} points that need URL updates")

        # Update the points in batches
        if updated_points:
            print("Updating points in Qdrant...")

            batch_size = 100
            for i in range(0, len(updated_points), batch_size):
                batch = updated_points[i:i + batch_size]

                client.upsert(
                    collection_name=collection_name,
                    points=batch
                )

                print(f"Updated batch {i//batch_size + 1}/{(len(updated_points)-1)//batch_size + 1}")

        print(f"Successfully updated {update_count} points with correct URLs")
        print("URL fixing completed successfully!")

        return 0

    except Exception as e:
        print(f"Error during URL fixing: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    exit(fix_source_urls())