#!/usr/bin/env python3
"""
RAG Retrieval Validation Tool

This script connects to Qdrant and loads existing vector collections,
accepts a test query and performs top-k similarity search,
and validates results using returned text, metadata, and source URLs.
"""

import argparse
import os
import time
from typing import List, Dict, Any, Optional
from dataclasses import dataclass
import requests
from dotenv import load_dotenv

from qdrant_client import QdrantClient
from qdrant_client.http import models
import cohere

# Load environment variables
load_dotenv()

@dataclass
class SearchResult:
    """Contains a single search result from the similarity search"""
    id: str
    text: str
    score: float
    metadata: Dict[str, Any]
    source_url: str

@dataclass
class RetrievalResult:
    """Aggregated results from a retrieval operation"""
    query: str
    results: List[SearchResult]
    total_results: int
    execution_time: float
    validation_passed: bool

@dataclass
class ValidationReport:
    """Report of validation checks performed on retrieval results"""
    source_url_match_count: int
    metadata_integrity_check: bool
    content_relevance_score: float
    issues_found: List[str]
    overall_status: str

class RAGRetrievalValidator:
    """Validates RAG retrieval pipeline by connecting to Qdrant and testing queries"""

    def __init__(self, qdrant_url: str, qdrant_api_key: str, cohere_api_key: str, collection_name: str):
        """
        Initialize the validator with Qdrant and Cohere credentials

        Args:
            qdrant_url: URL of the Qdrant instance
            qdrant_api_key: API key for Qdrant authentication
            cohere_api_key: API key for Cohere for embedding generation
            collection_name: Name of the Qdrant collection to search in
        """
        self.qdrant_client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
            timeout=10,
            prefer_grpc=False  # Using HTTP for better compatibility
        )
        self.cohere_client = cohere.Client(cohere_api_key)
        self.collection_name = collection_name

        # Verify connection to Qdrant
        try:
            self.qdrant_client.get_collection(collection_name)
            print(f"SUCCESS: Successfully connected to Qdrant collection: {collection_name}")
        except Exception as e:
            raise ConnectionError(f"Failed to connect to Qdrant collection {collection_name}: {str(e)}")

        # Verify Cohere connection
        try:
            self.cohere_client.embed(
                texts=["test"],
                model="embed-english-v3.0",
                input_type="search_query"
            )
            print("SUCCESS: Successfully connected to Cohere API")
        except Exception as e:
            raise ConnectionError(f"Failed to connect to Cohere API: {str(e)}")

    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for the input text using Cohere

        Args:
            text: Text to generate embedding for

        Returns:
            List of float values representing the embedding
        """
        response = self.cohere_client.embed(
            texts=[text],
            model="embed-english-v3.0",
            input_type="search_query"  # Using search_query for better retrieval performance
        )
        return response.embeddings[0]

    def search(self, query: str, top_k: int = 5) -> RetrievalResult:
        """
        Perform similarity search on the Qdrant collection

        Args:
            query: Query text to search for
            top_k: Number of top results to retrieve

        Returns:
            RetrievalResult containing the search results
        """
        start_time = time.time()

        # Generate embedding for the query
        query_embedding = self.generate_embedding(query)

        # Perform similarity search using the query_points method (correct for newer Qdrant versions)
        # For vector search, we need to use the query_points method with the vector
        search_results = self.qdrant_client.query_points(
            collection_name=self.collection_name,
            query=query_embedding,
            limit=top_k,
            with_payload=True
        )

        execution_time = time.time() - start_time

        # Convert search results to our SearchResult format
        # The query_points method returns a QueryResponse object with results
        results = []
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

            result = SearchResult(
                id=str(hit.id),
                text=payload.get('text', '') if isinstance(payload, dict) else str(payload),
                score=hit.score,
                metadata=metadata,
                source_url=source_url
            )
            results.append(result)

        return RetrievalResult(
            query=query,
            results=results,
            total_results=len(results),
            execution_time=execution_time,
            validation_passed=False  # Will be set after validation
        )

    def validate_results(self, retrieval_result: RetrievalResult) -> ValidationReport:
        """
        Validate the retrieval results based on metadata integrity and content relevance

        Args:
            retrieval_result: The results to validate

        Returns:
            ValidationReport with the validation results
        """
        issues = []
        source_url_match_count = 0
        metadata_integrity_check = True
        total_score = 0.0

        for result in retrieval_result.results:
            # Check if source URL exists and is valid
            if result.source_url and result.source_url != 'Unknown':
                source_url_match_count += 1

                # Validate URL format (basic check)
                if not result.source_url.startswith(('http://', 'https://')):
                    issues.append(f"Invalid URL format for result ID {result.id}: {result.source_url}")

            # Check metadata integrity
            required_fields = ['source_url', 'document_id']
            for field in required_fields:
                if field not in result.metadata:
                    issues.append(f"Missing required metadata field '{field}' for result ID {result.id}")
                    metadata_integrity_check = False

            # Add to total for average relevance calculation
            total_score += result.score

        # Calculate average relevance score
        avg_relevance = total_score / len(retrieval_result.results) if retrieval_result.results else 0.0

        # Determine overall status
        if issues:
            overall_status = "fail" if len([issue for issue in issues if "Missing required metadata" in issue]) > 0 else "warning"
        else:
            overall_status = "pass" if (source_url_match_count > 0 and metadata_integrity_check) else "warning"

        # Update the retrieval result with validation status
        retrieval_result.validation_passed = overall_status == "pass"

        return ValidationReport(
            source_url_match_count=source_url_match_count,
            metadata_integrity_check=metadata_integrity_check,
            content_relevance_score=avg_relevance,
            issues_found=issues,
            overall_status=overall_status
        )

    def validate_retrieval_pipeline(self, query: str, top_k: int = 5) -> tuple[RetrievalResult, ValidationReport]:
        """
        Complete pipeline: search and validate results

        Args:
            query: Query text to search for
            top_k: Number of top results to retrieve

        Returns:
            Tuple of (RetrievalResult, ValidationReport)
        """
        # Perform search
        retrieval_result = self.search(query, top_k)

        # Validate results
        validation_report = self.validate_results(retrieval_result)

        return retrieval_result, validation_report


def main():
    parser = argparse.ArgumentParser(description="RAG Retrieval Validation Tool")
    parser.add_argument("--query", type=str, required=True, help="The search query to validate")
    parser.add_argument("--top-k", type=int, default=5, help="Number of results to return (default: 5)")
    parser.add_argument("--collection", type=str, help="Qdrant collection name (default: from .env)")
    parser.add_argument("--validate", action="store_true", default=True, help="Whether to run validation checks (default: True)")

    args = parser.parse_args()

    # Validate arguments
    if not args.query.strip():
        raise ValueError("Query cannot be empty")

    if args.top_k < 1 or args.top_k > 100:
        raise ValueError("top_k must be between 1 and 100")

    # Get environment variables
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    cohere_api_key = os.getenv("COHERE_API_KEY")
    collection_name = args.collection or os.getenv("QDRANT_COLLECTION_NAME")

    if not all([qdrant_url, qdrant_api_key, cohere_api_key, collection_name]):
        missing_vars = []
        if not qdrant_url: missing_vars.append("QDRANT_URL")
        if not qdrant_api_key: missing_vars.append("QDRANT_API_KEY")
        if not cohere_api_key: missing_vars.append("COHERE_API_KEY")
        if not collection_name: missing_vars.append("QDRANT_COLLECTION_NAME")

        raise ValueError(f"Missing required environment variables: {', '.join(missing_vars)}")

    # Initialize validator
    try:
        validator = RAGRetrievalValidator(
            qdrant_url=qdrant_url,
            qdrant_api_key=qdrant_api_key,
            cohere_api_key=cohere_api_key,
            collection_name=collection_name
        )
    except ConnectionError as e:
        print(f"ERROR: Connection error: {e}")
        return 1

    print(f"\nSEARCHING: '{args.query}'")
    print(f"TOP-K: {args.top_k}")
    print(f"COLLECTION: {collection_name}")
    print("-" * 50)

    # Perform validation
    retrieval_result, validation_report = validator.validate_retrieval_pipeline(
        query=args.query,
        top_k=args.top_k
    )

    # Print results
    print(f"EXECUTION TIME: {retrieval_result.execution_time:.2f} seconds")
    print(f"TOTAL RESULTS: {retrieval_result.total_results}")
    print(f"AVERAGE RELEVANCE SCORE: {validation_report.content_relevance_score:.3f}")
    print(f"VALID SOURCE URLS: {validation_report.source_url_match_count}/{retrieval_result.total_results}")
    print(f"METADATA INTEGRITY CHECK: {'PASS' if validation_report.metadata_integrity_check else 'FAIL'}")

    print("\nRETRIEVED TEXT CHUNKS:")
    for i, result in enumerate(retrieval_result.results, 1):
        print(f"\n  {i}. Score: {result.score:.3f}")
        print(f"     Source: {result.source_url}")
        print(f"     Text: {result.text[:200]}{'...' if len(result.text) > 200 else ''}")

    if args.validate:
        print(f"\nOVERALL VALIDATION STATUS: {validation_report.overall_status.upper()}")

        if validation_report.issues_found:
            print("\nISSUES FOUND DURING VALIDATION:")
            for issue in validation_report.issues_found:
                print(f"   - {issue}")
        else:
            print("\nNO ISSUES FOUND DURING VALIDATION")

    return 0 if validation_report.overall_status != "fail" else 1


if __name__ == "__main__":
    exit(main())