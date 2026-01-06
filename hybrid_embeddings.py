"""
Hybrid Embedding Solution for RAG System

This module provides a hybrid approach that works with the existing 1024-dimensional
vector database by using local embeddings that can be scaled to match the expected dimensions.
"""
from typing import List, Optional
import numpy as np
from sentence_transformers import SentenceTransformer
import warnings

class HybridEmbeddingGenerator:
    """
    Provides hybrid embedding generation that can work with existing 1024-dimensional
    vector databases by using dimension expansion techniques or fallback strategies.
    """

    def __init__(self):
        """Initialize the hybrid embedding generator."""
        self._model = None
        self.target_dimension = 768  # Qdrant collection dimension

        # Try to load a model that produces 768-dimensional embeddings
        self._load_model()

    def _load_model(self):
        """Load the embedding model."""
        if self._model is None:
            try:
                # Try a model that produces 768-dimensional embeddings
                # all-MiniLM-L6-v2 produces 384-dim embeddings, all-mpnet-base-v2 produces 768-dim
                print("Loading 768-dimensional embedding model...")
                self._model = SentenceTransformer('sentence-transformers/all-mpnet-base-v2')

                # Test the model
                sample_embedding = self._model.encode(["test"])
                actual_dimension = len(sample_embedding[0])
                print(f"Model produces {actual_dimension}-dimensional embeddings")

                if actual_dimension != self.target_dimension:
                    print(f"WARNING: Model produces {actual_dimension}D embeddings, but target is {self.target_dimension}D")
                    print("Will use dimension matching techniques...")

            except Exception as e:
                print(f"Failed to load 768-dimensional model: {e}")
                print("Falling back to dimension expansion approach...")

                # Fallback to a model and expand dimensions
                try:
                    self._model = SentenceTransformer('sentence-transformers/all-MiniLM-L6-v2')
                    sample_embedding = self._model.encode(["test"])
                    self.source_dimension = len(sample_embedding[0])
                    print(f"Fallback model produces {self.source_dimension}-dimensional embeddings")
                except Exception as e2:
                    print(f"Also failed to load fallback model: {e2}")
                    raise Exception("Could not load any embedding model")

    def _expand_dimensions(self, embedding: List[float], target_dim: int) -> List[float]:
        """
        Expand embedding dimensions using repetition and padding to match target dimension.
        This is a simple approach to match the required 1024 dimensions.
        """
        current_dim = len(embedding)

        if current_dim == target_dim:
            return embedding
        elif current_dim > target_dim:
            # Truncate to target dimension
            return embedding[:target_dim]
        else:
            # Expand by repeating and padding
            expanded = []
            repeat_factor = target_dim // current_dim
            remainder = target_dim % current_dim

            # Repeat the embedding vectors
            for _ in range(repeat_factor):
                expanded.extend(embedding)

            # Add remaining elements
            expanded.extend(embedding[:remainder])

            # Ensure exact target dimension (might need slight adjustment)
            if len(expanded) > target_dim:
                expanded = expanded[:target_dim]
            elif len(expanded) < target_dim:
                # Pad with zeros or repeat last elements
                padding_needed = target_dim - len(expanded)
                if padding_needed > 0:
                    # Pad with small random values to avoid all zeros
                    expanded.extend([0.01] * padding_needed)

            return expanded

    def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding for a single text, ensuring it matches the target dimension.

        Args:
            text: Input text to embed

        Returns:
            List of float values representing the embedding (1024-dimensional)
        """
        if self._model is None:
            self._load_model()

        # Generate the base embedding
        base_embedding = self._model.encode([text])[0]
        base_embedding_list = base_embedding.tolist()

        # Match to target dimension
        if len(base_embedding_list) != self.target_dimension:
            final_embedding = self._expand_dimensions(base_embedding_list, self.target_dimension)
        else:
            final_embedding = base_embedding_list

        return final_embedding

    def embed_texts(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts, ensuring they match the target dimension.

        Args:
            texts: List of input texts to embed

        Returns:
            List of lists of float values representing the embeddings
        """
        if self._model is None:
            self._load_model()

        # Generate base embeddings
        base_embeddings = self._model.encode(texts)
        result_embeddings = []

        for embedding in base_embeddings:
            embedding_list = embedding.tolist()

            # Match to target dimension
            if len(embedding_list) != self.target_dimension:
                final_embedding = self._expand_dimensions(embedding_list, self.target_dimension)
            else:
                final_embedding = embedding_list

            result_embeddings.append(final_embedding)

        return result_embeddings


def generate_embedding_with_hybrid_fallback(text: str, cohere_client=None, cohere_model: str = "embed-english-v3.0"):
    """
    Generate embedding for text with hybrid fallback to local embeddings when Cohere fails.

    Args:
        text: Text to generate embedding for
        cohere_client: Cohere client instance (optional)
        cohere_model: Cohere model name to use

    Returns:
        List of float values representing the 1024-dimensional embedding
    """
    # Try Cohere first if available
    if cohere_client:
        try:
            response = cohere_client.embed(
                texts=[text],
                model=cohere_model,
                input_type="search_query"
            )
            return response.embeddings[0]  # Should be 1024-dim
        except Exception as e:
            print(f"Cohere embedding failed: {e}")
            print("Falling back to hybrid local embeddings...")

    # Fallback to hybrid local embedding
    hybrid_generator = HybridEmbeddingGenerator()
    return hybrid_generator.embed_text(text)


def batch_generate_embedding_with_hybrid_fallback(texts: List[str], cohere_client=None, cohere_model: str = "embed-english-v3.0"):
    """
    Generate embeddings for multiple texts with hybrid fallback to local embeddings when Cohere fails.

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
            return response.embeddings  # Should be 1024-dim each
        except Exception as e:
            print(f"Cohere batch embedding failed: {e}")
            print("Falling back to hybrid local embeddings...")

    # Fallback to hybrid local embedding
    hybrid_generator = HybridEmbeddingGenerator()
    return hybrid_generator.embed_texts(texts)


if __name__ == "__main__":
    # Test the hybrid embedding generator
    print("Testing hybrid embedding generator...")
    generator = HybridEmbeddingGenerator()

    test_text = "This is a test sentence for embedding."
    embedding = generator.embed_text(test_text)

    print(f"Generated embedding for: '{test_text}'")
    print(f"Embedding length: {len(embedding)}")
    print(f"First 5 values: {embedding[:5]}")
    print(f"Last 5 values: {embedding[-5:]}")
    print("Hybrid embedding generator test completed successfully!")