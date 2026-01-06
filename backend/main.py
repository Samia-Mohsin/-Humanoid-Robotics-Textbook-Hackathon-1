"""
URL Ingestion & Embedding Pipeline for Docusaurus documentation
"""
import os
import sys
import argparse
import time
from typing import List
from dotenv import load_dotenv

from config import Config, validate_config
from scraper import process_multiple_urls
from chunker import chunk_document
from embedder import initialize_cohere_client, generate_embeddings_for_chunks
from vector_store import (
    initialize_qdrant_client,
    create_qdrant_collection,
    store_embeddings_in_qdrant
)
from utils import setup_logging
from sitemap_processor import process_sitemap

# Load environment variables
load_dotenv()

def main_pipeline(
    urls: List[str],
    config: Config,
    collection_name: str = "document_embeddings"
):
    """
    Main ingestion pipeline function orchestrating all components.

    Args:
        urls: List of URLs to process
        config: Configuration object with API keys and settings
        collection_name: Name of the Qdrant collection to store embeddings

    Returns:
        Dictionary with pipeline execution results and metrics
    """
    # Validate configuration
    config_errors = validate_config(config)
    if config_errors:
        raise ValueError(f"Configuration validation failed: {', '.join(config_errors)}")

    # Setup logging
    logger = setup_logging()

    # Initialize components
    logger.info("Initializing components...")
    cohere_client = initialize_cohere_client(config.cohere_api_key)
    qdrant_client = initialize_qdrant_client(config.qdrant_url, config.qdrant_api_key)

    # Create Qdrant collection
    logger.info(f"Creating Qdrant collection: {collection_name}")
    create_qdrant_collection(
        qdrant_client,
        collection_name,
        vector_size=1024,  # Default size for Cohere embeddings
        distance_metric="Cosine"
    )

    # Execute pipeline: fetch → clean → chunk → embed → store
    start_time = time.time()

    # 1. Fetch and clean content
    logger.info(f"Fetching content from {len(urls)} URLs...")
    document_chunks = process_multiple_urls(urls, delay=0.5)  # 0.5 second delay between requests
    logger.info(f"Extracted {len(document_chunks)} document chunks")

    # 2. Chunk documents
    logger.info("Chunking documents...")
    all_chunked_docs = []
    for doc_chunk in document_chunks:
        chunked_docs = chunk_document(
            doc_chunk,
            chunk_size=config.chunk_size,
            overlap=config.chunk_overlap
        )
        all_chunked_docs.extend(chunked_docs)
    logger.info(f"Created {len(all_chunked_docs)} text chunks after chunking")

    # 3. Generate embeddings
    logger.info("Generating embeddings...")
    embeddings = generate_embeddings_for_chunks(
        all_chunked_docs,
        cohere_client,
        config.cohere_model
    )
    logger.info(f"Generated {len(embeddings)} embeddings")

    # 4. Store embeddings in Qdrant
    logger.info(f"Storing embeddings in Qdrant collection: {collection_name}")
    store_success = store_embeddings_in_qdrant(
        qdrant_client,
        embeddings,
        collection_name
    )

    end_time = time.time()
    execution_time = end_time - start_time

    # Prepare results
    results = {
        "urls_processed": len(urls),
        "documents_extracted": len(document_chunks),
        "chunks_created": len(all_chunked_docs),
        "embeddings_generated": len(embeddings),
        "storage_success": store_success,
        "execution_time_seconds": execution_time,
        "status": "completed" if store_success else "failed"
    }

    logger.info(f"Pipeline completed in {execution_time:.2f} seconds")
    logger.info(f"Processed {len(all_chunked_docs)} chunks and stored {len(embeddings)} embeddings")

    return results


def main():
    """Main function to orchestrate the full ingestion pipeline."""
    print("URL Ingestion & Embedding Pipeline")
    print("Starting the ingestion process...")

    # 1. Parse command line arguments
    parser = argparse.ArgumentParser(description='URL Ingestion & Embedding Pipeline')
    # Add mutually exclusive group for sitemap vs individual URLs
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--urls', nargs='+', help='List of URLs to process')
    group.add_argument('--sitemap', type=str, help='Sitemap URL to process all URLs from')
    parser.add_argument('--collection', type=str, default='document_embeddings',
                        help='Qdrant collection name (default: document_embeddings)')
    parser.add_argument('--chunk-size', type=int, default=512,
                        help='Chunk size in tokens (default: 512)')
    parser.add_argument('--chunk-overlap', type=int, default=50,
                        help='Chunk overlap in tokens (default: 50)')
    parser.add_argument('--model', type=str, default='embed-english-v3.0',
                        help='Cohere model to use (default: embed-english-v3.0)')
    parser.add_argument('--rate-limit', type=float, default=1.0,
                        help='Delay in seconds between requests for sitemap processing (default: 1.0)')

    args = parser.parse_args()

    # 2. Load configuration
    try:
        config = Config.from_env()
        # Override config values with command line arguments if provided
        if args.chunk_size:
            os.environ['CHUNK_SIZE'] = str(args.chunk_size)
        if args.chunk_overlap:
            os.environ['CHUNK_OVERLAP'] = str(args.chunk_overlap)
        if args.model:
            os.environ['COHERE_MODEL'] = args.model

        # Reload config with potential overrides
        config = Config.from_env()
    except ValueError as e:
        print(f"Configuration error: {e}")
        sys.exit(1)

    # 3. Execute the pipeline based on input type
    try:
        if args.sitemap:
            # Process sitemap
            results = process_sitemap(args.sitemap, config, args.rate_limit, args.collection)
            print(f"Sitemap processing completed successfully!")
            print(f"Results: {results}")
        else:
            # Process individual URLs
            results = main_pipeline(args.urls, config, args.collection)
            print(f"Pipeline execution completed successfully!")
            print(f"Results: {results}")
    except Exception as e:
        print(f"Pipeline execution failed: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()