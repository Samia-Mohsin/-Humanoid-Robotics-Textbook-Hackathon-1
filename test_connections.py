#!/usr/bin/env python3
"""
Test script to verify connections to Qdrant and Cohere services
"""
import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
import cohere
import tiktoken

# Load environment variables
load_dotenv()

def test_qdrant_connection():
    """Test Qdrant connection and verify collection exists"""
    print("Testing Qdrant connection...")

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

        # Test connection by getting collection info
        collection_info = client.get_collection(collection_name)
        print(f"SUCCESS: Successfully connected to Qdrant collection: {collection_name}")
        print(f"SUCCESS: Collection vector size: {collection_info.config.params.vectors.size}")
        print(f"SUCCESS: Collection distance: {collection_info.config.params.vectors.distance}")
        return True

    except Exception as e:
        print(f"ERROR: Qdrant connection failed: {str(e)}")
        return False

def test_cohere_connection():
    """Test Cohere connection"""
    print("\nTesting Cohere connection...")

    cohere_api_key = os.getenv("COHERE_API_KEY")

    if not cohere_api_key:
        print("ERROR: COHERE_API_KEY not found in environment")
        return False

    try:
        client = cohere.Client(cohere_api_key)

        # Test embedding generation
        response = client.embed(
            texts=["test"],
            model="embed-english-v3.0",
            input_type="search_query"
        )

        print("SUCCESS: Successfully connected to Cohere API")
        print(f"SUCCESS: Embedding dimension: {len(response.embeddings[0])}")
        return True

    except Exception as e:
        print(f"ERROR: Cohere connection failed: {str(e)}")
        return False

def test_tiktoken():
    """Test tiktoken functionality"""
    print("\nTesting tiktoken...")

    try:
        tokenizer = tiktoken.get_encoding("cl100k_base")
        tokens = tokenizer.encode("test")
        print(f"SUCCESS: Tiktoken working, test tokens: {tokens}")
        return True
    except Exception as e:
        print(f"ERROR: Tiktoken failed: {str(e)}")
        return False

def main():
    print("Starting connection tests...\n")

    qdrant_ok = test_qdrant_connection()
    cohere_ok = test_cohere_connection()
    tiktoken_ok = test_tiktoken()

    print(f"\nResults:")
    print(f"- Qdrant connection: {'SUCCESS' if qdrant_ok else 'ERROR'}")
    print(f"- Cohere connection: {'SUCCESS' if cohere_ok else 'ERROR'}")
    print(f"- Tiktoken: {'SUCCESS' if tiktoken_ok else 'ERROR'}")

    if qdrant_ok and cohere_ok and tiktoken_ok:
        print("\nSUCCESS: All connections successful!")
        return 0
    else:
        print("\nERROR: Some connections failed!")
        return 1

if __name__ == "__main__":
    exit(main())