# Quickstart: Sitemap-Based Multi-Point Ingestion Fix

## Overview
This feature fixes the single-point ingestion issue in the sitemap-based pipeline, ensuring that each URL produces multiple embedded chunks instead of a single embedding. The system now properly chunks documents, generates unique deterministic IDs, and stores multiple vector points per source URL.

## Prerequisites
- Python 3.11+
- Backend pipeline dependencies installed
- Valid Cohere API key
- Valid Qdrant Cloud credentials
- Access to the target sitemap.xml file

## Setup
1. Ensure you have the backend pipeline environment set up:
   ```bash
   cd backend
   pip install -r requirements.txt  # if requirements file exists
   ```

2. Configure your environment variables in `.env`:
   ```bash
   COHERE_API_KEY="your_cohere_api_key"
   QDRANT_URL="your_qdrant_url"
   QDRANT_API_KEY="your_qdrant_api_key"
   ```

## Usage
The fixed sitemap ingestion can be performed using the command line interface:

```bash
# Process a sitemap with multi-chunk ingestion
python main.py --sitemap "https://humanoid-robotics-textbook-hackatho-seven.vercel.app/sitemap.xml" --collection multi_chunk_docs
```

## Expected Output
When processing a sitemap with the fix, you'll see:
- Progress percentage as URLs are processed
- Number of chunks created per URL (should be multiple for substantial documents)
- Total vectors stored (should be greater than number of URLs)
- Validation that multiple points are stored per source URL
- Final summary with URL count, chunk count, and vector storage verification

## Processing Details
- Documents are properly chunked into multiple segments based on content size
- Each chunk gets a unique, deterministic ID based on source URL and chunk index
- Multiple vectors are stored per source URL in Qdrant
- Progress tracking shows detailed multi-chunk metrics
- Failed chunks are logged but don't stop the entire process
- Content is stored in the Qdrant vector database with proper metadata linking to source and position

## Verification
To verify the fix is working:
1. Check that multiple vectors are stored per URL in Qdrant
2. Confirm that chunk IDs follow the deterministic pattern
3. Verify that substantial documents produce multiple chunks
4. Validate that re-running the ingestion is idempotent without creating duplicates