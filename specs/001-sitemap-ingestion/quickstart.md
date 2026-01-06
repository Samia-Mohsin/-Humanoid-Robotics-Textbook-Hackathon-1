# Quickstart: Sitemap-Based Content Ingestion Pipeline

## Overview
This feature enables bulk processing of content from sitemap.xml files, allowing you to ingest entire websites or documentation sites like the humanoid robotics textbook in a single operation.

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
The sitemap ingestion can be performed using the command line interface:

```bash
# Process a sitemap with a single command
python main.py --sitemap "https://humanoid-robotics-textbook-hackatho-seven.vercel.app/sitemap.xml"
```

Alternatively, you can use the new sitemap_processor module directly:

```bash
python -m sitemap_processor "https://humanoid-robotics-textbook-hackatho-seven.vercel.app/sitemap.xml"
```

## Expected Output
When processing a sitemap, you'll see:
- Progress percentage as URLs are processed
- Number of URLs successfully processed
- Number of URLs that failed (if any)
- Total processing time
- Final summary with success/failure counts

## Processing Details
- URLs are processed sequentially with rate limiting (1 second delay between requests)
- Each URL is processed through the full pipeline (scraping, chunking, embedding, storage)
- Progress is reported continuously during processing
- Failed URLs are logged but don't stop the entire process
- Content is stored in the Qdrant vector database with proper embeddings