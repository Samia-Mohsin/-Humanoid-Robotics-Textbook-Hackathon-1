"""
Sitemap processing functionality for the URL Ingestion & Embedding Pipeline
"""
import xml.etree.ElementTree as ET
import requests
from typing import List, Dict, Any, Optional
import time
import logging
from urllib.parse import urljoin, urlparse
import re
import psutil
import os

from models import DocumentChunk
from utils import validate_url
from config import Config
from vector_store import initialize_qdrant_client


def parse_sitemap_xml(sitemap_content: str) -> List[Dict[str, Any]]:
    """
    Parse sitemap XML content and extract URLs with metadata.

    Args:
        sitemap_content: Raw XML content of the sitemap

    Returns:
        List of dictionaries containing URL and metadata
    """
    try:
        root = ET.fromstring(sitemap_content)

        # Handle both regular sitemap and sitemap index
        urls = []

        # Define namespaces for XML parsing
        namespaces = {
            'sitemap': 'http://www.sitemaps.org/schemas/sitemap/0.9',
            'xhtml': 'http://www.w3.org/1999/xhtml'
        }

        # Look for both 'url' and 'sitemap' elements
        for url_elem in root.findall('.//sitemap:url', namespaces):
            loc_elem = url_elem.find('sitemap:loc', namespaces)
            if loc_elem is not None:
                url_data = {
                    'url': loc_elem.text,
                    'last_modified': None,
                    'change_frequency': None,
                    'priority': None
                }

                # Extract additional metadata if available
                lastmod_elem = url_elem.find('sitemap:lastmod', namespaces)
                if lastmod_elem is not None:
                    url_data['last_modified'] = lastmod_elem.text

                changefreq_elem = url_elem.find('sitemap:changefreq', namespaces)
                if changefreq_elem is not None:
                    url_data['change_frequency'] = changefreq_elem.text

                priority_elem = url_elem.find('sitemap:priority', namespaces)
                if priority_elem is not None:
                    try:
                        url_data['priority'] = float(priority_elem.text)
                    except (ValueError, TypeError):
                        url_data['priority'] = None

                urls.append(url_data)

        return urls

    except ET.ParseError as e:
        logging.error(f"Error parsing sitemap XML: {e}")
        return []


def extract_urls_from_sitemap(sitemap_url: str) -> List[Dict[str, Any]]:
    """
    Fetch and parse sitemap from URL, extracting all URLs.

    Args:
        sitemap_url: URL of the sitemap.xml file

    Returns:
        List of dictionaries containing URL and metadata
    """
    try:
        headers = {
            'User-Agent': 'Mozilla/5.0 (compatible; SitemapProcessor/1.0)'
        }
        response = requests.get(sitemap_url, headers=headers, timeout=30)

        if response.status_code == 200:
            return parse_sitemap_xml(response.text)
        else:
            logging.error(f"Failed to fetch sitemap {sitemap_url}: HTTP {response.status_code}")
            return []

    except requests.RequestException as e:
        logging.error(f"Error fetching sitemap {sitemap_url}: {e}")
        return []


def validate_sitemap_urls(urls: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """
    Validate extracted URLs from sitemap.

    Args:
        urls: List of URL dictionaries from sitemap parsing

    Returns:
        List of validated URL dictionaries
    """
    validated_urls = []

    for url_data in urls:
        if validate_url(url_data['url']):
            validated_urls.append(url_data)
        else:
            logging.warning(f"Invalid URL found in sitemap: {url_data['url']}")

    return validated_urls


def validate_embeddings_count(original_count: int, stored_count: int) -> bool:
    """
    Validate that the number of stored embeddings matches the expected count.

    Args:
        original_count: Number of embeddings that should have been stored
        stored_count: Number of embeddings actually stored

    Returns:
        Boolean indicating if the counts match
    """
    return original_count == stored_count


def retrieve_and_verify_stored_content(collection_name: str, qdrant_client) -> Dict[str, Any]:
    """
    Retrieve and verify stored content from the vector database.

    Args:
        collection_name: Name of the Qdrant collection to check
        qdrant_client: Initialized Qdrant client

    Returns:
        Dictionary with verification results
    """
    try:
        # Get the count of points in the collection
        collection_info = qdrant_client.get_collection(collection_name)
        point_count = collection_info.points_count

        return {
            "success": True,
            "point_count": point_count,
            "message": f"Successfully retrieved collection info. Points: {point_count}"
        }
    except Exception as e:
        logging.error(f"Error retrieving collection {collection_name}: {e}")
        return {
            "success": False,
            "point_count": 0,
            "message": f"Error retrieving collection: {str(e)}"
        }


def validate_content_matches_original(url: str, stored_content: Any) -> bool:
    """
    Validate that stored content matches the original sitemap URL content.

    Args:
        url: Original URL from sitemap
        stored_content: Content retrieved from vector database

    Returns:
        Boolean indicating if content matches
    """
    # This is a simplified check - in a real implementation, you would compare
    # the original content with what was stored in the database
    # For now, we just check that we have stored content for the URL
    return stored_content is not None


def comprehensive_content_validation(url: str, original_content: Optional[str] = None,
                                  stored_chunks: Optional[List] = None,
                                  stored_embeddings: Optional[List] = None) -> Dict[str, Any]:
    """
    Comprehensive validation of content from sitemap processing.

    Args:
        url: Original URL from sitemap
        original_content: Original content fetched from the URL (optional)
        stored_chunks: List of document chunks stored (optional)
        stored_embeddings: List of embeddings stored (optional)

    Returns:
        Dictionary with validation results
    """
    validation_results = {
        "url": url,
        "content_fetched": original_content is not None and len(original_content) > 0,
        "content_length": len(original_content) if original_content else 0,
        "chunks_stored": stored_chunks is not None and len(stored_chunks) > 0,
        "chunks_count": len(stored_chunks) if stored_chunks else 0,
        "embeddings_stored": stored_embeddings is not None and len(stored_embeddings) > 0,
        "embeddings_count": len(stored_embeddings) if stored_embeddings else 0,
        "content_chunks_match": False,  # Would need more complex logic to compare
        "chunks_embeddings_match": False,  # Would need more complex logic to compare
        "validation_passed": False
    }

    # Basic validation - if we have content, chunks, and embeddings, consider it passed
    if (validation_results["content_fetched"] and
        validation_results["chunks_stored"] and
        validation_results["embeddings_stored"]):
        validation_results["validation_passed"] = True

    return validation_results


def get_memory_usage() -> Dict[str, Any]:
    """
    Get current memory usage statistics.

    Returns:
        Dictionary with memory usage information
    """
    process = psutil.Process(os.getpid())
    memory_info = process.memory_info()

    return {
        "rss": memory_info.rss,  # Resident Set Size: actual physical memory currently used by the process
        "vms": memory_info.vms,  # Virtual Memory Size: total amount of virtual memory used by the process
        "percent": process.memory_percent(),  # Percentage of total system memory used by the process
        "timestamp": time.time()
    }


def validate_all_content_stored_in_vector_db(collection_name: str, expected_count: int,
                                           qdrant_client) -> Dict[str, Any]:
    """
    Validate that all successful content is stored in the vector database.

    Args:
        collection_name: Name of the Qdrant collection to check
        expected_count: Number of items that should be stored
        qdrant_client: Initialized Qdrant client

    Returns:
        Dictionary with validation results
    """
    try:
        collection_info = qdrant_client.get_collection(collection_name)
        actual_count = collection_info.points_count

        return {
            "success": True,
            "expected_count": expected_count,
            "actual_count": actual_count,
            "validation_passed": actual_count >= expected_count,
            "message": f"Collection has {actual_count} points, expected at least {expected_count}"
        }
    except Exception as e:
        return {
            "success": False,
            "expected_count": expected_count,
            "actual_count": 0,
            "validation_passed": False,
            "message": f"Error checking collection: {str(e)}"
        }


def process_sitemap(
    sitemap_url: str,
    config: Config,
    rate_limit_delay: float = 1.0,
    collection_name: str = "document_embeddings"
) -> Dict[str, Any]:
    """
    Process all URLs from a sitemap through the ingestion pipeline.

    Args:
        sitemap_url: URL of the sitemap.xml file to process
        config: Configuration object with API keys and settings
        rate_limit_delay: Delay in seconds between requests
        collection_name: Name of the Qdrant collection to store embeddings

    Returns:
        Dictionary with processing results and statistics
    """
    logging.info(f"Starting sitemap processing: {sitemap_url}")

    # Extract URLs from sitemap
    sitemap_urls = extract_urls_from_sitemap(sitemap_url)
    logging.info(f"Extracted {len(sitemap_urls)} URLs from sitemap")

    # Validate URLs
    validated_urls = validate_sitemap_urls(sitemap_urls)
    logging.info(f"Validated {len(validated_urls)} URLs from sitemap")

    if not validated_urls:
        logging.error("No valid URLs found in sitemap")
        return {
            "total_urls": len(sitemap_urls),
            "successful_processing": 0,
            "failed_processing": len(sitemap_urls),
            "processed_urls": [],
            "start_time": time.time(),
            "end_time": time.time(),
            "total_duration": 0,
            "error_details": ["No valid URLs found in sitemap"]
        }

    # Process each URL through the pipeline
    processed_results = []
    successful_count = 0
    failed_count = 0
    error_details = []

    start_time = time.time()

    for i, url_data in enumerate(validated_urls):
        print(f"Processing sitemap: {sitemap_url}")
        print(f"Progress: {((i+1)/len(validated_urls)*100):.1f}% ({i+1}/{len(validated_urls)})")
        time_elapsed = time.time() - start_time
        if i > 0:
            avg_time_per_url = time_elapsed / i
            estimated_remaining = avg_time_per_url * (len(validated_urls) - i)
            print(f"Time elapsed: {time_elapsed:.1f} seconds")
            print(f"Estimated time remaining: {estimated_remaining:.1f} seconds")

        try:
            # Process single URL through the main pipeline
            # Import here to avoid circular import
            from main import main_pipeline
            result = main_pipeline([url_data['url']], config, collection_name)

            if result['status'] == 'completed':
                # Validate that embeddings were generated and stored correctly
                expected_embeddings = result.get('embeddings_generated', 0)

                # For now, we assume if the pipeline completed successfully,
                # the embeddings were stored (the pipeline handles storage internally)
                if result.get('storage_success', False):
                    successful_count += 1
                    processed_results.append({
                        **url_data,
                        'status': 'completed',
                        'result': result
                    })
                else:
                    failed_count += 1
                    processed_results.append({
                        **url_data,
                        'status': 'failed',
                        'result': result,
                        'error': 'Embeddings not stored successfully'
                    })
                    error_details.append(f"Embeddings not stored for {url_data['url']}: Storage failed")
            else:
                failed_count += 1
                processed_results.append({
                    **url_data,
                    'status': 'failed',
                    'result': result
                })
                error_details.append(f"Failed to process {url_data['url']}: {result}")

        except Exception as e:
            failed_count += 1
            logging.error(f"Error processing URL {url_data['url']}: {e}")
            error_details.append(f"Failed to process URL: {url_data['url']}, Error: {str(e)}")
            processed_results.append({
                **url_data,
                'status': 'failed',
                'error': str(e)
            })

        # Rate limiting - delay between requests
        if i < len(validated_urls) - 1:  # Don't delay after the last request
            time.sleep(rate_limit_delay)

    end_time = time.time()

    # Perform memory monitoring
    try:
        final_memory_usage = get_memory_usage()
        logging.info(f"Final memory usage: {final_memory_usage}")
    except Exception as e:
        logging.error(f"Error getting memory usage: {e}")
        final_memory_usage = None

    # Perform final validation of stored content
    try:
        # Initialize Qdrant client for validation
        qdrant_client = initialize_qdrant_client(config.qdrant_url, config.qdrant_api_key)

        # Retrieve and verify stored content
        verification_result = retrieve_and_verify_stored_content(collection_name, qdrant_client)

        if verification_result['success']:
            logging.info(f"Content verification successful: {verification_result['message']}")
        else:
            logging.error(f"Content verification failed: {verification_result['message']}")
            error_details.append(f"Content verification failed: {verification_result['message']}")

        # Validate that all successful content is stored in the vector database
        expected_count = successful_count  # Each successful URL should result in content stored
        storage_validation_result = validate_all_content_stored_in_vector_db(
            collection_name, expected_count, qdrant_client
        )

        if storage_validation_result['validation_passed']:
            logging.info(f"Storage validation passed: {storage_validation_result['message']}")
        else:
            logging.warning(f"Storage validation warning: {storage_validation_result['message']}")
            # Don't add to error_details as it might be a warning rather than a failure
    except Exception as e:
        logging.error(f"Error during content verification: {e}")
        error_details.append(f"Content verification error: {str(e)}")
        storage_validation_result = None

    # Prepare final result
    result = {
        "total_urls": len(validated_urls),
        "successful_processing": successful_count,
        "failed_processing": failed_count,
        "processed_urls": processed_results,
        "start_time": start_time,
        "end_time": end_time,
        "total_duration": end_time - start_time,
        "error_details": error_details,
        "content_verification": verification_result if 'verification_result' in locals() else None,
        "memory_usage": final_memory_usage,
        "storage_validation": storage_validation_result
    }

    logging.info(f"Sitemap processing completed. Success: {successful_count}, Failed: {failed_count}")
    return result


if __name__ == "__main__":
    # Example usage
    sitemap_url = "https://example.com/sitemap.xml"
    print(f"Processing sitemap: {sitemap_url}")

    # This would need proper configuration setup
    # result = process_sitemap(sitemap_url, config)
    # print(f"Processing complete: {result}")
    pass