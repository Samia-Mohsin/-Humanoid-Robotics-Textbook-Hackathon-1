"""
Patched Agent with Local Embedding Fallback

This module provides a patched version of the HumanoidRoboticsAgent that uses local embeddings
as a fallback when Cohere API is unavailable due to rate limits.
"""
import os
import json
import uuid
import time
from datetime import datetime
from typing import List, Dict, Any, Optional
from dataclasses import dataclass, field
from dotenv import load_dotenv
import openai
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.http import models
import cohere  # Using the existing cohere client approach from backend
import tiktoken
import warnings

from hybrid_embeddings import generate_embedding_with_hybrid_fallback

# Load environment variables
load_dotenv()

# Data models based on the specification
@dataclass
class ContentChunk:
    """Represents a segment of text content from the humanoid robotics textbook with associated metadata"""
    chunk_id: str
    text: str
    source_url: str
    metadata: Dict[str, Any] = field(default_factory=dict)
    relevance_score: float = 0.0


@dataclass
class Message:
    """Individual message in a conversation between user and agent"""
    message_id: str
    role: str  # "user", "assistant", "system"
    content: str
    timestamp: datetime = field(default_factory=datetime.now)
    retrieved_chunks: List[ContentChunk] = field(default_factory=list)


@dataclass
class ConversationContext:
    """State management for multi-turn interactions between user and agent"""
    conversation_id: str
    messages: List[Message] = field(default_factory=list)
    created_at: datetime = field(default_factory=datetime.now)
    last_updated: datetime = field(default_factory=datetime.now)

    def add_message(self, message: Message):
        self.messages.append(message)
        self.last_updated = datetime.now()


@dataclass
class Tool:
    """Tool component that connects to Qdrant and retrieves relevant text chunks based on user queries"""
    tool_id: str
    name: str
    description: str
    function: callable
    parameters: Dict[str, Any]


class PatchedHumanoidRoboticsAgent:
    """AI agent with local embedding fallback when Cohere API is unavailable"""

    def __init__(self,
                 model: str = "gpt-4o",
                 qdrant_url: str = None,
                 qdrant_api_key: str = None,
                 cohere_api_key: str = None,
                 collection_name: str = "humanoid_robotics_docs"):
        self.model = model

        # Get configuration from environment variables if not provided
        self.qdrant_url = qdrant_url or os.getenv("QDRANT_URL")
        self.qdrant_api_key = qdrant_api_key or os.getenv("QDRANT_API_KEY")
        self.cohere_api_key = cohere_api_key or os.getenv("COHERE_API_KEY")
        self.collection_name = collection_name or os.getenv("QDRANT_COLLECTION_NAME", "humanoid_robotics_docs")

        # Initialize Qdrant client for vector storage
        self.qdrant_client = QdrantClient(
            url=self.qdrant_url,
            api_key=self.qdrant_api_key,
            timeout=30
        )

        # Initialize Cohere client for embedding generation (reusing existing approach)
        if self.cohere_api_key:
            self.cohere_client = cohere.Client(self.cohere_api_key)
        else:
            self.cohere_client = None

        # Verify connection to Qdrant
        try:
            self.qdrant_client.get_collection(self.collection_name)
            print(f"Successfully connected to Qdrant collection: {self.collection_name}")
        except Exception as e:
            raise ConnectionError(f"Failed to connect to Qdrant collection {self.collection_name}: {str(e)}")

        # Initialize conversation contexts
        self.conversation_contexts: Dict[str, ConversationContext] = {}

    def generate_embedding(self, text: str, max_retries: int = 1) -> List[float]:
        """
        Generate embedding for the input text using Cohere with fallback to local embeddings
        """
        # Use the fallback function that tries Cohere first, then local embeddings
        return generate_embedding_with_hybrid_fallback(text, self.cohere_client)

    def retrieve_content(self, query: str, top_k: int = 6, debug: bool = False) -> List[Dict[str, Any]]:
        """
        Retrieve relevant content chunks from Qdrant based on the query
        """
        try:
            # Generate embedding for the query using fallback mechanism
            query_embedding = self.generate_embedding(query)

            # Perform similarity search using the query_points method (correct for newer Qdrant versions)
            search_results = self.qdrant_client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=top_k,
                with_payload=True
            )

            # Convert search results to our ContentChunk format
            content_chunks = []
            for hit in search_results.points:  # Access the points from QueryResponse
                # Extract payload from the result
                payload = hit.payload if hit.payload else {}
                metadata = payload.get('metadata', {}) if isinstance(payload.get('metadata'), dict) else payload
                source_url = metadata.get('source_url', 'Unknown') if isinstance(metadata, dict) else 'Unknown'

                # Add chunk_id to metadata if document_id is missing (for backward compatibility with existing documents)
                if isinstance(metadata, dict) and 'document_id' not in metadata:
                    # Try to get document_id from the top-level payload's chunk_id
                    if payload.get('chunk_id'):
                        metadata['document_id'] = payload.get('chunk_id')
                    # Or fallback to the hit.id if chunk_id is also not available
                    elif 'document_id' not in metadata:
                        metadata['document_id'] = str(hit.id)

                content_chunk = ContentChunk(
                    chunk_id=str(hit.id),
                    text=payload.get('text', '') if isinstance(payload, dict) else str(payload),
                    source_url=source_url,
                    metadata=metadata,
                    relevance_score=hit.score
                )
                content_chunks.append(content_chunk)

            # Debug mode - print retrieved chunks and similarity scores
            if debug:
                print(f"\nDEBUG: Retrieved {len(content_chunks)} chunks for query: '{query[:50]}...'")
                for i, chunk in enumerate(content_chunks):
                    print(f"  Chunk {i+1}: Score={chunk.relevance_score:.3f}, Source={chunk.source_url}")
                    print(f"    Text preview: {chunk.text[:200]}...")
                    print()

            return [chunk.__dict__ for chunk in content_chunks]

        except Exception as e:
            print(f"Error retrieving content: {str(e)}")
            return []

    def process_query_with_retrieval(self, query: str, conversation_id: str = None, debug: bool = False) -> str:
        """
        Process a query by retrieving relevant content and generating a response using OpenAI Chat Completions API
        """
        # Get or create conversation context
        if conversation_id:
            context = self.get_conversation(conversation_id)
        else:
            conversation_id = self.create_conversation()
            context = self.get_conversation(conversation_id)

        # Retrieve relevant content FIRST
        retrieved_chunks = self.retrieve_content(query, top_k=6, debug=debug)

        # Format the retrieved content for the assistant
        if retrieved_chunks:
            context_text = "\n\n".join([
                f"Source: {chunk.get('source_url', 'Unknown')}\nContent: {chunk.get('text', '')[:1000]}..."
                for chunk in retrieved_chunks
            ])
        else:
            context_text = "No relevant content found in the textbook for your query."

        # Create user message with context
        user_message = Message(
            message_id=str(uuid.uuid4()),
            role="user",
            content=query,
        )
        context.add_message(user_message)

        # Format the query with context for the AI model
        formatted_query_with_context = f"""
        CONTEXT FROM PHYSICAL AI & HUMANOID ROBOTICS TEXTBOOK:
        {context_text}

        USER QUESTION: {query}

        Please answer the user's question based ONLY on the provided context from the textbook. Do not make up information or hallucinate. If the answer is not in the provided context, clearly state that the information is not available in the textbook. Ensure all your responses are grounded in the retrieved content and cite specific information when possible.
        """

        # Build conversation history for context
        messages = [
            {
                "role": "system",
                "content": "You are a helpful assistant for humanoid robotics. Answer questions based ONLY on the provided context from the textbook. Do not make up information or hallucinate. If the answer is not in the provided context, clearly state that the information is not available in the textbook. Ensure all your responses are grounded in the retrieved content and cite specific information when possible."
            }
        ]

        # Add conversation history if available (for multi-turn conversations)
        for msg in context.messages[:-1]:  # Exclude current message
            messages.append({
                "role": msg.role,
                "content": msg.content
            })

        # Add the current user message
        messages.append({
            "role": "user",
            "content": formatted_query_with_context
        })

        # Call the OpenAI Chat Completions API
        try:
            from backend.src.client import openai_client
            response = openai_client.create_completion(
                messages=messages,
                model=self.model,
                temperature=0.7,
                max_tokens=1000
            )

            # Extract the response
            assistant_response = response.choices[0].message.content
        except Exception as e:
            print(f"Error calling OpenAI API: {str(e)}")
            assistant_response = "I encountered an error while processing your request. Please try again later."

        # Create assistant message
        processed_retrieved_chunks = []
        if retrieved_chunks:
            for chunk in retrieved_chunks:
                # Ensure all required fields are present with defaults
                processed_chunk = ContentChunk(
                    chunk_id=chunk.get('chunk_id', ''),
                    text=chunk.get('text', ''),
                    source_url=chunk.get('source_url', 'Unknown'),
                    metadata=chunk.get('metadata', {}),
                    relevance_score=chunk.get('relevance_score', 0.0)
                )
                processed_retrieved_chunks.append(processed_chunk)

        assistant_message = Message(
            message_id=str(uuid.uuid4()),
            role="assistant",
            content=assistant_response,
            retrieved_chunks=processed_retrieved_chunks
        )
        context.add_message(assistant_message)

        return assistant_response

    def create_conversation(self, conversation_id: str = None) -> str:
        """Create a new conversation context"""
        conversation_id = conversation_id or str(uuid.uuid4())
        self.conversation_contexts[conversation_id] = ConversationContext(
            conversation_id=conversation_id
        )
        return conversation_id

    def get_conversation(self, conversation_id: str) -> ConversationContext:
        """Get an existing conversation context"""
        if conversation_id not in self.conversation_contexts:
            conversation_id = self.create_conversation(conversation_id)
        return self.conversation_contexts[conversation_id]

    def get_content_status(self) -> Dict[str, Any]:
        """
        Get the status of the content in the Qdrant collection
        """
        try:
            collection_info = self.qdrant_client.get_collection(self.collection_name)
            status = {
                "collection_name": self.collection_name,
                "total_documents": collection_info.points_count,
                "vector_size": collection_info.config.params.vectors.size,
                "distance_type": collection_info.config.params.vectors.distance,
                "status": "ready"
            }
            return status
        except Exception as e:
            return {
                "collection_name": self.collection_name,
                "error": str(e),
                "status": "error"
            }

    def refresh_content(self, docs_folder: str = "frontend-book/docs") -> int:
        """
        Refresh the content in Qdrant by re-ingesting documents from the specified folder
        This is a simplified version - in a real implementation, you might want to import
        the full ingestion logic or call the ingest_docs.py script
        """
        try:
            # This would normally call the full ingestion logic from ingest_docs.py
            # For now, we'll just return the count of documents in the collection
            # to indicate the refresh operation
            import os
            import glob

            # Count the files that would be ingested
            md_files = glob.glob(f"{docs_folder}/**/*.md", recursive=True)
            mdx_files = glob.glob(f"{docs_folder}/**/*.mdx", recursive=True)
            total_files = len(md_files) + len(mdx_files)

            print(f"Would re-ingest {total_files} files from {docs_folder}/")
            print("Note: Full re-ingestion would require Cohere API quota.")
            print("To perform full re-ingestion, run: python ingest_docs.py")

            return total_files
        except Exception as e:
            print(f"Error counting files for refresh: {str(e)}")
            return 0

    def ask(self, query: str, conversation_id: str = None, debug: bool = False) -> str:
        """
        Main method to ask the agent a question
        """
        # For backward compatibility, return just the response string
        # The sources are stored in the conversation context for access if needed
        return self.process_query_with_retrieval(query, conversation_id, debug)

    def ask_with_sources(self, query: str, conversation_id: str = None, debug: bool = False) -> Dict[str, Any]:
        """
        Main method to ask the agent a question and return both answer and sources
        """
        # Get the conversation context before processing
        if conversation_id:
            context = self.get_conversation(conversation_id)
        else:
            conversation_id = self.create_conversation()
            context = self.get_conversation(conversation_id)

        # Process the query to populate the context with retrieved chunks
        response = self.process_query_with_retrieval(query, conversation_id, debug)

        # Get the conversation context to extract sources
        sources = []
        if context and context.messages:
            # Look for the last user message or assistant message that has retrieved_chunks
            for msg in reversed(context.messages):  # Go through messages in reverse order
                if hasattr(msg, 'retrieved_chunks') and msg.retrieved_chunks:
                    sources = [chunk.source_url for chunk in msg.retrieved_chunks if chunk.source_url != 'Unknown' and chunk.source_url is not None]
                    break

        return {
            "answer": response,
            "sources": sources
        }