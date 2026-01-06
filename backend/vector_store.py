"""
Vector storage functionality using Qdrant
"""
from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Optional, Dict, Any
from models import EmbeddingVector
import uuid


def initialize_qdrant_client(url: str, api_key: str) -> QdrantClient:
    """
    Implement Qdrant client initialization.

    Args:
        url: Qdrant Cloud URL
        api_key: Qdrant API key

    Returns:
        QdrantClient instance
    """
    client = QdrantClient(
        url=url,
        api_key=api_key,
        prefer_grpc=False  # Using HTTP for better compatibility
    )
    return client


def create_qdrant_collection(
    client: QdrantClient,
    collection_name: str,
    vector_size: int = 1024,  # Default size for Cohere embeddings
    distance_metric: str = "Cosine"
) -> bool:
    """
    Create Qdrant collection with proper schema.

    Args:
        client: Qdrant client instance
        collection_name: Name of the collection to create
        vector_size: Size of the embedding vectors
        distance_metric: Distance metric for similarity search

    Returns:
        True if collection was created successfully, False otherwise
    """
    try:
        # Check if collection already exists
        client.get_collection(collection_name)
        print(f"Collection '{collection_name}' already exists")
        return True
    except:
        # Collection doesn't exist, so create it
        client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(
                size=vector_size,
                distance=distance_metric
            )
        )
        print(f"Collection '{collection_name}' created successfully")
        return True


def store_embeddings_in_qdrant(
    client: QdrantClient,
    embeddings: List[EmbeddingVector],
    collection_name: str
) -> bool:
    """
    Implement embedding storage function in Qdrant.

    Args:
        client: Qdrant client instance
        embeddings: List of EmbeddingVector objects to store
        collection_name: Name of the collection to store embeddings in

    Returns:
        True if embeddings were stored successfully, False otherwise
    """
    if not embeddings:
        print("No embeddings to store")
        return True

    # Prepare points for insertion
    points = []
    for emb in embeddings:
        # Generate a deterministic ID based on the source URL and chunk position
        # Extract source URL from metadata if available
        source_url = None
        chunk_position = 0

        if emb.metadata:
            if isinstance(emb.metadata, dict):
                source_url = emb.metadata.get('source_url', emb.metadata.get('url', emb.chunk_id))
                chunk_position = emb.metadata.get('position', 0)

        # If we still don't have a source URL, use chunk_id as fallback
        if not source_url:
            source_url = emb.chunk_id

        # Use the deterministic ID generator function
        from utils import generate_deterministic_id
        point_id = generate_deterministic_id(str(source_url), chunk_position)

        point = models.PointStruct(
            id=point_id,  # Use deterministic ID instead of random UUID
            vector=emb.vector,
            payload={
                "embedding_id": emb.embedding_id,
                "chunk_id": emb.chunk_id,
                "model": emb.model,
                "created_at": emb.created_at.isoformat() if emb.created_at else None,
                "metadata": emb.metadata
            }
        )
        points.append(point)

    # Insert points into the collection
    try:
        client.upsert(
            collection_name=collection_name,
            points=points
        )
        print(f"Successfully stored {len(points)} embeddings in collection '{collection_name}'")
        return True
    except Exception as e:
        print(f"Error storing embeddings in Qdrant: {str(e)}")
        return False


def add_metadata_storage(
    client: QdrantClient,
    embeddings: List[EmbeddingVector],
    collection_name: str
) -> bool:
    """
    Add metadata storage with source URL and content location.

    Args:
        client: Qdrant client instance
        embeddings: List of EmbeddingVector objects with metadata
        collection_name: Name of the collection

    Returns:
        True if metadata was stored successfully, False otherwise
    """
    # This functionality is already included in the store_embeddings_in_qdrant function
    # Metadata is stored as part of the payload when storing embeddings
    return store_embeddings_in_qdrant(client, embeddings, collection_name)


def retrieve_embeddings_by_id(
    client: QdrantClient,
    embedding_ids: List[str],
    collection_name: str
) -> List[Dict[str, Any]]:
    """
    Create function to retrieve embeddings by ID.

    Args:
        client: Qdrant client instance
        embedding_ids: List of embedding IDs to retrieve
        collection_name: Name of the collection to search in

    Returns:
        List of embedding records
    """
    try:
        # Convert embedding IDs to point IDs (we need to search by the Qdrant point IDs)
        # Since we don't store the embedding_id as a separate field to search on,
        # we would need to use a different approach.
        # For now, let's retrieve points by their vector similarity to a known embedding
        # or use scroll to find specific embeddings

        # Actually, we need to use the payload to search for the embedding_id
        results = []
        for emb_id in embedding_ids:
            # Search for points that have this embedding_id in their payload
            search_result = client.scroll(
                collection_name=collection_name,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="payload.embedding_id",
                            match=models.MatchValue(value=emb_id)
                        )
                    ]
                ),
                limit=1
            )

            if search_result[0]:
                results.extend([{
                    "id": point.id,
                    "vector": point.vector,
                    "payload": point.payload
                } for point in search_result[0]])

        return results
    except Exception as e:
        print(f"Error retrieving embeddings by ID from Qdrant: {str(e)}")
        return []


def implement_error_handling_for_qdrant(func):
    """
    Implement error handling for Qdrant service unavailability.

    Args:
        func: Function that interacts with Qdrant

    Returns:
        Wrapped function with error handling
    """
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception as e:
            error_msg = f"Qdrant service error: {str(e)}"
            print(error_msg)
            # In a real implementation, you might want to implement retry logic
            # or use a fallback storage mechanism
            raise e
    return wrapper


def validate_embeddings_stored(
    client: QdrantClient,
    collection_name: str,
    expected_count: int = None
) -> bool:
    """
    Add validation function to confirm embeddings were stored successfully.

    Args:
        client: Qdrant client instance
        collection_name: Name of the collection to validate
        expected_count: Expected number of embeddings (optional)

    Returns:
        True if validation passes, False otherwise
    """
    try:
        collection_info = client.get_collection(collection_name)
        actual_count = collection_info.points_count

        if expected_count is not None:
            return actual_count >= expected_count
        else:
            # Just check that there are some embeddings in the collection
            return actual_count > 0
    except Exception as e:
        print(f"Error validating embeddings in Qdrant: {str(e)}")
        return False


def search_similar_embeddings(
    client: QdrantClient,
    query_vector: List[float],
    collection_name: str,
    limit: int = 10
) -> List[Dict[str, Any]]:
    """
    Implement vector search function.

    Args:
        client: Qdrant client instance
        query_vector: Vector to search for similar embeddings
        collection_name: Name of the collection to search in
        limit: Maximum number of results to return

    Returns:
        List of similar embedding records with similarity scores
    """
    try:
        search_results = client.search(
            collection_name=collection_name,
            query_vector=query_vector,
            limit=limit
        )

        results = []
        for result in search_results:
            results.append({
                "id": result.id,
                "vector": result.vector,
                "payload": result.payload,
                "score": result.score
            })

        return results
    except Exception as e:
        print(f"Error searching for similar embeddings in Qdrant: {str(e)}")
        return []


def add_similarity_scoring(results: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """
    Add similarity scoring to search results.

    Args:
        results: List of search results from Qdrant

    Returns:
        List of results with similarity scores (already included in Qdrant results)
    """
    # The similarity score is already included in the Qdrant search results
    # This function is more of a placeholder to satisfy the task requirement
    return results