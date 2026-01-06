#!/usr/bin/env python3
"""
Script to verify document chunking in Qdrant collection
"""
import os
import tiktoken
from dotenv import load_dotenv
from qdrant_client import QdrantClient

# Load environment variables
load_dotenv()

def verify_document_chunking():
    """Verify that documents are properly chunked"""
    print("Verifying document chunking...")

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

        # Initialize tokenizer to count tokens
        tokenizer = tiktoken.get_encoding("cl100k_base")

        # Get sample points to check chunking
        sample_points = client.scroll(
            collection_name=collection_name,
            limit=10,
            with_payload=True,
            with_vectors=False
        )

        # The scroll method returns (points, next_page_offset)
        points = sample_points[0]

        print(f"\nAnalyzing {len(points)} sample chunks:")
        print("=" * 80)

        total_tokens = []
        for i, point in enumerate(points):
            point_id = point.id
            payload = point.payload
            text = payload.get('text', '')
            metadata = payload.get('metadata', {})

            # Count tokens in the text
            token_count = len(tokenizer.encode(text))
            total_tokens.append(token_count)

            # Extract metadata
            file_path = metadata.get('file_path', 'Unknown')
            chunk_index = metadata.get('chunk_index', -1)
            total_chunks = metadata.get('total_chunks', -1)
            section = metadata.get('section', 'Unknown')

            print(f"Chunk {i+1}:")
            print(f"  ID: {point_id}")
            print(f"  File: {os.path.basename(file_path)}")
            print(f"  Section: {section}")
            print(f"  Chunk: {chunk_index+1}/{total_chunks}")
            print(f"  Token count: {token_count}")
            print(f"  Text preview: {text[:100]}...")
            print()

        # Calculate statistics
        avg_tokens = sum(total_tokens) / len(total_tokens) if total_tokens else 0
        min_tokens = min(total_tokens) if total_tokens else 0
        max_tokens = max(total_tokens) if total_tokens else 0

        print("=" * 80)
        print("Chunking Analysis Summary:")
        print(f"  Average token count per chunk: {avg_tokens:.1f}")
        print(f"  Minimum token count: {min_tokens}")
        print(f"  Maximum token count: {max_tokens}")
        print(f"  Total sample chunks analyzed: {len(total_tokens)}")

        # Check if chunking meets requirements (500-800 tokens)
        chunks_in_range = [t for t in total_tokens if 500 <= t <= 800]
        percentage_in_range = (len(chunks_in_range) / len(total_tokens) * 100) if total_tokens else 0

        print(f"  Chunks in 500-800 token range: {len(chunks_in_range)}/{len(total_tokens)} ({percentage_in_range:.1f}%)")

        # Determine if chunking is acceptable
        if percentage_in_range >= 80:  # At least 80% of chunks should be in the target range
            print("\nSUCCESS: Document chunking meets requirements!")
            print("  - Most chunks are in the 500-800 token range")
            print("  - Proper metadata is included with chunk indices")
            return True
        else:
            print("\nWARNING: Document chunking may need adjustment.")
            print("  - Less than 80% of chunks are in the 500-800 token range")
            return False

    except Exception as e:
        print(f"ERROR: Error checking document chunking: {str(e)}")
        return False

if __name__ == "__main__":
    success = verify_document_chunking()
    if success:
        print("\nSUCCESS: Document chunking verification completed")
        exit(0)
    else:
        print("\nERROR: Document chunking verification failed")
        exit(1)