# URL Ingestion & Embedding Pipeline

This project implements a complete pipeline for ingesting Docusaurus documentation URLs, generating embeddings using Cohere, and storing them in Qdrant for vector search capabilities.

## Features

- **URL Crawling**: Automatically crawl and extract clean text content from Docusaurus sites
- **Sitemap Processing**: Extract and process all URLs from sitemap.xml files
- **Text Chunking**: Split content into manageable chunks with configurable size and overlap
- **Embedding Generation**: Generate semantic embeddings using Cohere's models
- **Vector Storage**: Store embeddings in Qdrant vector database with metadata
- **Search Validation**: Validate vector search returns relevant results
- **Progress Tracking**: Monitor processing progress with percentage completion and time estimates
- **Rate Limiting**: Respectful processing with configurable delays between requests
- **Content Validation**: Verify that all content is properly stored in the vector database
- **Memory Monitoring**: Track memory usage during processing of large sitemaps

## Prerequisites

- Python 3.9+
- `uv` package manager (or pip)
- Cohere API key
- Qdrant Cloud account and API key

## Installation

1. Clone the repository
2. Navigate to the backend directory
3. Install dependencies:

```bash
uv sync  # or pip install -r requirements.txt
```

## Configuration

Create a `.env` file in the backend directory with the following variables:

```env
# Cohere API Configuration
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Cloud Configuration
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here

# Processing Configuration
CHUNK_SIZE=512
CHUNK_OVERLAP=50
COHERE_MODEL=embed-english-v3.0
```

## Usage

Run the ingestion pipeline for individual URLs:

```bash
uv run python main.py --urls https://example-docs.com/page1 https://example-docs.com/page2 --collection my_embeddings
```

Or process all URLs from a sitemap:

```bash
uv run python main.py --sitemap https://example.com/sitemap.xml --collection my_embeddings
```

Additional options:
- `--collection`: Qdrant collection name (default: document_embeddings)
- `--chunk-size`: Chunk size in tokens (default: 512)
- `--chunk-overlap`: Chunk overlap in tokens (default: 50)
- `--model`: Cohere model to use (default: embed-english-v3.0)
- `--rate-limit`: Delay in seconds between requests for sitemap processing (default: 1.0)

## Architecture

The pipeline consists of several components:

1. **Scraper**: Handles URL fetching and content extraction
2. **Chunker**: Splits content into manageable text chunks
3. **Embedder**: Generates embeddings using Cohere API
4. **Vector Store**: Stores embeddings in Qdrant with metadata
5. **Search Validator**: Validates search relevance

## Data Flow

1. URLs are crawled and clean text content is extracted
2. Content is chunked with configurable size and overlap
3. Embeddings are generated using Cohere models
4. Embeddings are stored in Qdrant with metadata
5. Search functionality is validated with test queries

## Error Handling

The system includes comprehensive error handling for:
- Network issues during URL fetching
- Invalid URLs or inaccessible content
- Cohere API failures
- Qdrant service unavailability
- Invalid configuration parameters

## Command Line Usage Examples

### Process Individual URLs
```bash
# Process a single URL
uv run python main.py --urls https://example.com/docs/intro

# Process multiple URLs
uv run python main.py --urls https://example.com/docs/page1 https://example.com/docs/page2 https://example.com/docs/page3

# Process URLs with custom collection name
uv run python main.py --urls https://example.com/docs --collection my_custom_collection

# Process URLs with custom chunk size and overlap
uv run python main.py --urls https://example.com/docs --chunk-size 1024 --chunk-overlap 100
```

### Process Sitemap URLs
```bash
# Process all URLs from a sitemap
uv run python main.py --sitemap https://example.com/sitemap.xml

# Process sitemap with custom collection and rate limiting
uv run python main.py --sitemap https://example.com/sitemap.xml --collection sitemap_embeddings --rate-limit 2.0

# Process sitemap with custom chunk settings
uv run python main.py --sitemap https://example.com/sitemap.xml --chunk-size 768 --chunk-overlap 75
```

### Example with Humanoid Robotics Textbook
```bash
# Process the humanoid robotics textbook sitemap
uv run python main.py --sitemap https://humanoid-robotics-textbook-hackatho-seven.vercel.app/sitemap.xml --collection humanoid_robotics_docs --rate-limit 1.0
```

## Performance Considerations

- Respectful crawling with configurable delays
- Batch processing of embeddings for efficiency
- Proper indexing in Qdrant for fast retrieval
- Memory-efficient processing of large documents

## Security

- API keys are loaded from environment variables
- No sensitive information is logged
- Input validation for all parameters
- Proper error handling to prevent information leakage