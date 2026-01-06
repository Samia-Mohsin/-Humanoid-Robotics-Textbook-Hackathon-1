"""
Vector search validation functionality
"""
from typing import List, Dict, Any, Tuple
from qdrant_client import QdrantClient
from .models import EmbeddingVector
from .vector_store import search_similar_embeddings


def create_test_query_execution(
    client: QdrantClient,
    query_vector: List[float],
    collection_name: str,
    expected_content: str = None
) -> Dict[str, Any]:
    """
    Create test query execution function.

    Args:
        client: Qdrant client instance
        query_vector: Vector to search for
        collection_name: Name of the collection to search in
        expected_content: Expected content to validate relevance (optional)

    Returns:
        Dictionary with search results and validation metrics
    """
    # Perform the search
    results = search_similar_embeddings(
        client=client,
        query_vector=query_vector,
        collection_name=collection_name,
        limit=5  # Return top 5 results
    )

    # Prepare validation results
    validation_results = {
        "query_executed": True,
        "total_results": len(results),
        "results": results,
        "expected_content_found": False,
        "relevance_score": 0.0
    }

    # If expected content is provided, validate relevance
    if expected_content:
        for result in results:
            payload_content = result.get("payload", {}).get("metadata", {}).get("chunk_content_preview", "")
            if expected_content.lower() in payload_content.lower():
                validation_results["expected_content_found"] = True
                break

    return validation_results


def implement_relevance_validation(
    search_results: List[Dict[str, Any]],
    query: str,
    threshold: float = 0.7
) -> Tuple[bool, float, List[Dict[str, Any]]]:
    """
    Implement relevance validation logic.

    Args:
        search_results: List of search results from Qdrant
        query: Original query string
        threshold: Relevance threshold (0.0 to 1.0)

    Returns:
        Tuple of (is_relevant, average_score, relevant_results)
    """
    if not search_results:
        return False, 0.0, []

    # Calculate average similarity score
    total_score = sum(result.get("score", 0.0) for result in search_results)
    avg_score = total_score / len(search_results)

    # Filter results above threshold
    relevant_results = [
        result for result in search_results
        if result.get("score", 0.0) >= threshold
    ]

    is_relevant = len(relevant_results) > 0

    return is_relevant, avg_score, relevant_results


def run_multiple_test_queries(
    client: QdrantClient,
    queries: List[str],
    collection_name: str,
    embedder_func = None
) -> Dict[str, Any]:
    """
    Create function to run multiple test queries and validate results.

    Args:
        client: Qdrant client instance
        queries: List of test queries to run
        collection_name: Name of the collection to search in
        embedder_func: Function to convert text queries to embeddings

    Returns:
        Dictionary with overall validation results
    """
    all_results = []
    total_queries = len(queries)
    successful_queries = 0
    total_relevant_results = 0

    for query in queries:
        try:
            # If we have an embedder function, convert the text query to a vector
            if embedder_func:
                query_embedding = embedder_func([query])
                query_vector = query_embedding[0] if query_embedding else None
            else:
                # For testing purposes, create a dummy vector
                query_vector = [0.1] * 1024  # Default Cohere embedding size

            if query_vector:
                # Execute the search
                search_results = search_similar_embeddings(
                    client=client,
                    query_vector=query_vector,
                    collection_name=collection_name,
                    limit=5
                )

                # Validate relevance
                is_relevant, avg_score, relevant_results = implement_relevance_validation(
                    search_results, query
                )

                # Add to results
                query_result = {
                    "query": query,
                    "is_relevant": is_relevant,
                    "avg_score": avg_score,
                    "total_results": len(search_results),
                    "relevant_results": len(relevant_results),
                    "results": search_results
                }

                all_results.append(query_result)

                if is_relevant:
                    total_relevant_results += len(relevant_results)
                    successful_queries += 1

        except Exception as e:
            print(f"Error processing query '{query}': {str(e)}")
            query_result = {
                "query": query,
                "is_relevant": False,
                "error": str(e)
            }
            all_results.append(query_result)

    # Calculate overall metrics
    overall_results = {
        "total_queries": total_queries,
        "successful_queries": successful_queries,
        "successful_rate": successful_queries / total_queries if total_queries > 0 else 0,
        "total_relevant_results": total_relevant_results,
        "query_results": all_results,
        "validation_passed": successful_queries == total_queries  # All queries should succeed
    }

    return overall_results


def validate_search_precision_at_k(
    client: QdrantClient,
    test_queries: List[Tuple[str, List[str]]],  # (query, expected_result_ids)
    collection_name: str,
    k: int = 5,
    embedder_func = None
) -> float:
    """
    Validate search precision at k positions.

    Args:
        client: Qdrant client instance
        test_queries: List of tuples (query, list of expected result IDs)
        collection_name: Name of the collection to search in
        k: Number of top results to consider
        embedder_func: Function to convert text queries to embeddings

    Returns:
        Precision at k score
    """
    total_precision = 0.0
    num_queries = len(test_queries)

    for query, expected_ids in test_queries:
        try:
            # Convert query to embedding if needed
            if embedder_func:
                query_embedding = embedder_func([query])
                query_vector = query_embedding[0] if query_embedding else None
            else:
                query_vector = [0.1] * 1024

            if query_vector:
                # Get top k results
                search_results = search_similar_embeddings(
                    client=client,
                    query_vector=query_vector,
                    collection_name=collection_name,
                    limit=k
                )

                # Calculate precision
                relevant_found = 0
                for result in search_results[:k]:
                    result_id = result.get("payload", {}).get("chunk_id", "")
                    if result_id in expected_ids:
                        relevant_found += 1

                precision_at_k = relevant_found / min(k, len(search_results)) if search_results else 0
                total_precision += precision_at_k

        except Exception as e:
            print(f"Error validating precision for query '{query}': {str(e)}")

    return total_precision / num_queries if num_queries > 0 else 0.0