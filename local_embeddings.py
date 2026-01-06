"""
Local Embedding Generator for RAG System

This module provides local embedding generation as a fallback when the Cohere API is unavailable
due to rate limits or other issues.
"""
from typing import List
import numpy as np

class LocalEmbeddingGenerator:
    """
    Provides local embedding generation using sentence-transformers as a fallback
    when the Cohere API is unavailable.
    """

    def __init__(self, model_name: str = "sentence-transformers/all-mpnet-base-v2", expected_dimension: int = 1024):
        """
        Initialize the local embedding generator.

        Args:
            model_name: Name of the sentence-transformers model to use
            expected_dimension: Expected embedding dimension to match the vector database
        """
        self.model_name = model_name
        self.expected_dimension = expected_dimension
        self._model = None

    def _load_model(self):
        """Lazy load the embedding model to avoid unnecessary loading."""
        if self._model is None:
            try:
                from sentence_transformers import SentenceTransformer
                print(f"Loading local embedding model: {self.model_name}")
                self._model = SentenceTransformer(self.model_name)
                print("Local embedding model loaded successfully")

                # Test the model with a sample text to check dimension
                sample_embedding = self._model.encode(["test"])
                actual_dimension = len(sample_embedding[0])
                print(f"Model generates {actual_dimension}-dimensional embeddings")

                if actual_dimension != self.expected_dimension:
                    print(f"Warning: Model generates {actual_dimension}D embeddings, but database expects {self.expected_dimension}D")
                    print("This may cause compatibility issues with the existing vector database")
            except ImportError:
                raise ImportError(
                    "sentence-transformers is not installed. "
                    "Please install it with: pip install sentence-transformers"
                )
            except Exception as e:
                raise Exception(f"Failed to load local embedding model: {e}")

    def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding for a single text using the local model.

        Args:
            text: Input text to embed

        Returns:
            List of float values representing the embedding
        """
        self._load_model()
        embedding = self._model.encode([text])[0]
        return embedding.tolist()

    def embed_texts(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts using the local model.

        Args:
            texts: List of input texts to embed

        Returns:
            List of lists of float values representing the embeddings
        """
        self._load_model()
        embeddings = self._model.encode(texts)
        return [embedding.tolist() for embedding in embeddings]


def generate_embedding_with_fallback(text: str, cohere_client=None, cohere_model: str = "embed-english-v3.0"):
    """
    Generate embedding for text with fallback to local embeddings when Cohere fails.

    Args:
        text: Text to generate embedding for
        cohere_client: Cohere client instance (optional)
        cohere_model: Cohere model name to use

    Returns:
        List of float values representing the embedding
    """
    # Try Cohere first if available
    if cohere_client:
        try:
            response = cohere_client.embed(
                texts=[text],
                model=cohere_model,
                input_type="search_query"
            )
            return response.embeddings[0]
        except Exception as e:
            print(f"Cohere embedding failed: {e}")
            print("Falling back to local embeddings...")

    # Fallback to local embedding
    local_generator = LocalEmbeddingGenerator()
    return local_generator.embed_text(text)


def batch_generate_embedding_with_fallback(texts: List[str], cohere_client=None, cohere_model: str = "embed-english-v3.0"):
    """
    Generate embeddings for multiple texts with fallback to local embeddings when Cohere fails.

    Args:
        texts: List of texts to generate embeddings for
        cohere_client: Cohere client instance (optional)
        cohere_model: Cohere model name to use

    Returns:
        List of lists of float values representing the embeddings
    """
    # Try Cohere first if available
    if cohere_client:
        try:
            response = cohere_client.embed(
                texts=texts,
                model=cohere_model,
                input_type="search_query"
            )
            return response.embeddings
        except Exception as e:
            print(f"Cohere batch embedding failed: {e}")
            print("Falling back to local embeddings...")

    # Fallback to local embedding
    local_generator = LocalEmbeddingGenerator()
    return local_generator.embed_texts(texts)


if __name__ == "__main__":
    # Test the local embedding generator
    print("Testing local embedding generator...")
    generator = LocalEmbeddingGenerator()

    test_text = "This is a test sentence for embedding."
    embedding = generator.embed_text(test_text)

    print(f"Generated embedding for: '{test_text}'")
    print(f"Embedding length: {len(embedding)}")
    print(f"First 5 values: {embedding[:5]}")
    print("Local embedding generator test completed successfully!")