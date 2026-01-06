from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http.models import SearchRequest, VectorParams, Distance
from cohere import Client as CohereClient
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

class RAGService:
    """
    RAG (Retrieval Augmented Generation) service for content retrieval
    """

    def __init__(self):
        # Initialize Qdrant client
        self.qdrant_client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY"),
            https=True
        )
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "humanoid_robotics_docs")

        # Initialize Cohere client for embeddings
        self.cohere_client = CohereClient(
            api_key=os.getenv("COHERE_API_KEY")
        )

    def retrieve_content(self, query: str, limit: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve relevant content from the vector database based on the query

        Args:
            query: The user's query string
            limit: Maximum number of results to return

        Returns:
            List of dictionaries containing content and metadata
        """
        # Generate embedding for the query using Cohere
        response = self.cohere_client.embed(
            texts=[query],
            model=os.getenv("COHERE_MODEL", "embed-english-v3.0"),
            input_type="search_query"
        )
        query_embedding = response.embeddings[0]

        # Search in Qdrant for similar content
        search_result = self.qdrant_client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=limit
        )

        # Extract content and metadata from search results
        results = []
        for hit in search_result:
            result = {
                "content": hit.payload.get("content", ""),
                "source": hit.payload.get("source", ""),
                "title": hit.payload.get("title", ""),
                "score": hit.score
            }
            results.append(result)

        return results

    def format_context(self, retrieved_content: List[Dict[str, Any]]) -> str:
        """
        Format the retrieved content as context for the LLM

        Args:
            retrieved_content: List of retrieved content items

        Returns:
            Formatted context string
        """
        if not retrieved_content:
            return "No relevant context found in the textbook."

        context_parts = ["Relevant context from the textbook:"]
        sources = set()

        for item in retrieved_content:
            context_parts.append(f"\nSource: {item.get('source', 'Unknown')}")
            context_parts.append(f"Title: {item.get('title', 'Unknown')}")
            context_parts.append(f"Content: {item.get('content', '')}")

            if item.get('source'):
                sources.add(item['source'])

        context = "\n".join(context_parts)
        return context, list(sources)