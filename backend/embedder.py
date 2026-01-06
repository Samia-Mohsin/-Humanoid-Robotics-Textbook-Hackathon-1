"""
Embedding generation functionality using Cohere with local fallback
"""
import cohere
from typing import List, Tuple
import logging
from models import DocumentChunk, EmbeddingVector
from utils import generate_uuid
from hybrid_embeddings import generate_embedding_with_hybrid_fallback


def initialize_cohere_client(api_key: str):
    """
    Initialize Cohere client with API key, with fallback to None if unavailable.

    Args:
        api_key: Cohere API key

    Returns:
        Cohere Client instance or None if initialization fails
    """
    try:
        import cohere
        if api_key and api_key.strip():
            return cohere.Client(api_key)
        else:
            logging.warning("Cohere API key not provided, will use local embeddings")
            return None
    except ImportError:
        logging.warning("Cohere library not found, will use local embeddings")
        return None
    except Exception as e:
        logging.warning(f"Could not initialize Cohere client: {e}, will use local embeddings")
        return None


def generate_embeddings_for_chunks(
    document_chunks: List[DocumentChunk],
    cohere_client,
    model: str = "embed-english-v3.0"
) -> List[EmbeddingVector]:
    """
    Create function to generate embeddings for text chunks with hybrid fallback.

    Args:
        document_chunks: List of DocumentChunk objects to embed
        cohere_client: Initialized Cohere client (can be None for local fallback)
        model: Cohere model to use for embeddings

    Returns:
        List of EmbeddingVector objects
    """
    if not document_chunks:
        return []

    embedding_vectors = []

    # Process each document chunk individually to handle failures gracefully
    for chunk in document_chunks:
        try:
            # Use the hybrid embedding function that falls back to local embeddings
            embedding = generate_embedding_with_hybrid_fallback(
                chunk.content,
                cohere_client,
                model
            )

            embedding_vector = EmbeddingVector(
                embedding_id=generate_uuid(),
                chunk_id=chunk.chunk_id,
                vector=embedding,
                model=model,
                metadata={
                    "document_id": chunk.chunk_id,  # Add document_id as required by validation
                    "source_url": chunk.source_url,
                    "title": chunk.title,
                    "position": chunk.position,
                    "chunk_content_preview": chunk.content[:100] + "..." if len(chunk.content) > 100 else chunk.content
                }
            )
            embedding_vectors.append(embedding_vector)

        except Exception as e:
            logging.error(f"Error generating embedding for chunk {chunk.chunk_id}: {e}")
            # Skip this chunk and continue with others
            continue

    logging.info(f"Generated {len(embedding_vectors)} embeddings from {len(document_chunks)} chunks")
    return embedding_vectors


def implement_cohere_integration(api_key: str, model: str = "embed-english-v3.0"):
    """
    Implement Cohere API integration for embedding generation.

    Args:
        api_key: Cohere API key
        model: Cohere model to use

    Returns:
        Cohere client and embedding function
    """
    client = initialize_cohere_client(api_key)

    def embed_texts(texts: List[str]) -> List[List[float]]:
        """Embed a list of texts using Cohere."""
        try:
            response = client.embed(
                texts=texts,
                model=model,
                input_type="search_document"
            )
            return response.embeddings
        except Exception as e:
            raise Exception(f"Error embedding texts with Cohere: {str(e)}")

    return client, embed_texts


def add_error_handling_for_cohere(client_func):
    """
    Add error handling for Cohere API failures.

    Args:
        client_func: Function that calls Cohere API

    Returns:
        Wrapped function with error handling
    """
    def wrapper(*args, **kwargs):
        try:
            return client_func(*args, **kwargs)
        except Exception as e:
            error_msg = f"Cohere API error: {str(e)}"
            print(error_msg)
            # In a real implementation, you might want to log this differently
            raise e
    return wrapper


def validate_embedding_dimensions(embeddings: List[EmbeddingVector], expected_dim: int = None) -> bool:
    """
    Implement embedding validation to ensure consistent dimensions.

    Args:
        embeddings: List of EmbeddingVector objects to validate
        expected_dim: Expected dimension size (optional)

    Returns:
        True if all embeddings have consistent dimensions, False otherwise
    """
    if not embeddings:
        return True

    # Get the dimension of the first embedding
    first_dim = len(embeddings[0].vector) if embeddings[0].vector else 0

    # If expected dimension is provided, check against it
    if expected_dim is not None and first_dim != expected_dim:
        return False

    # Check that all embeddings have the same dimension
    for emb in embeddings:
        if len(emb.vector) != first_dim:
            return False

    return True


def generate_single_embedding(
    document_chunk: DocumentChunk,
    cohere_client: cohere.Client,
    model: str = "embed-english-v3.0"
) -> EmbeddingVector:
    """
    Generate embedding for a single document chunk.

    Args:
        document_chunk: Single DocumentChunk to embed
        cohere_client: Initialized Cohere client
        model: Cohere model to use

    Returns:
        Single EmbeddingVector object
    """
    try:
        response = cohere_client.embed(
            texts=[document_chunk.content],
            model=model,
            input_type="search_document"
        )

        embedding = response.embeddings[0]  # Get the first (and only) embedding

        embedding_vector = EmbeddingVector(
            embedding_id=generate_uuid(),
            chunk_id=document_chunk.chunk_id,
            vector=embedding,
            model=model,
            metadata={
                "document_id": document_chunk.chunk_id,  # Add document_id as required by validation
                "source_url": document_chunk.source_url,
                "title": document_chunk.title,
                "position": document_chunk.position,
                "chunk_content_preview": document_chunk.content[:100] + "..." if len(document_chunk.content) > 100 else document_chunk.content
            }
        )

        return embedding_vector

    except Exception as e:
        raise Exception(f"Error generating embedding for chunk {document_chunk.chunk_id}: {str(e)}")