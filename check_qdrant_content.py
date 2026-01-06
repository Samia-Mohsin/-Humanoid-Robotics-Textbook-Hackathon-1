#!/usr/bin/env python3
"""
Script to check if content exists in Qdrant collection without using Cohere API
"""
import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models

# Load environment variables
load_dotenv()

def check_qdrant_content():
    """Check if there are documents in the Qdrant collection"""
    print("Checking Qdrant collection for content...")

    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    collection_name = os.getenv("QDRANT_COLLECTION_NAME", "humanoid_robotics_docs")

    if not qdrant_url or not qdrant_api_key:
        print("ERROR: QDRANT_URL or QDRANT_API_KEY not found in environment")
        return False

    try:
        client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
            timeout=30
        )

        # Get collection info to see how many points exist
        collection_info = client.get_collection(collection_name)
        print(f"Collection: {collection_name}")
        print(f"Total vectors in collection: {collection_info.points_count}")

        if collection_info.points_count > 0:
            print(f"SUCCESS: Collection contains {collection_info.points_count} documents")

            # Get a few sample points to verify content
            sample_points = client.scroll(
                collection_name=collection_name,
                limit=3,
                with_payload=True,
                with_vectors=False
            )

            # The scroll method returns (points, next_page_offset)
            points = sample_points[0]

            print("\nSample documents:")
            for i, point in enumerate(points):
                point_id = point.id
                payload = point.payload
                text_preview = payload.get('text', '')[:200] + "..." if len(payload.get('text', '')) > 200 else payload.get('text', '')[:200]
                print(f"  {i+1}. ID: {point_id}")
                print(f"     Text preview: {text_preview}")
                print(f"     Metadata: {payload.get('metadata', {})}")
                print()

            return True
        else:
            print("ERROR: Collection is empty - no documents found")
            return False

    except Exception as e:
        print(f"ERROR: Error checking Qdrant collection: {str(e)}")
        return False

if __name__ == "__main__":
    success = check_qdrant_content()
    if success:
        print("SUCCESS: Qdrant collection has content")
        exit(0)
    else:
        print("ERROR: Qdrant collection is empty or inaccessible")
        exit(1)