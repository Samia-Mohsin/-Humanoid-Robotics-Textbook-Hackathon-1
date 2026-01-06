# Quickstart Guide: URL Ingestion & Embedding Pipeline

## Prerequisites

- Python 3.9 or higher
- uv package manager
- Cohere API key
- Qdrant Cloud account and API key

## Setup

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Navigate to backend directory**
   ```bash
   cd backend/
   ```

3. **Install dependencies with uv**
   ```bash
   uv sync
   ```

4. **Set up environment variables**
   Create a `.env` file in the backend directory with the following:
   ```env
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_cloud_url_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   ```

## Running the Ingestion Pipeline

1. **Run the main ingestion pipeline**
   ```bash
   uv run python main.py
   ```

2. **Or run with specific parameters**
   ```bash
   uv run python main.py --urls "https://example-docs.com" --chunk-size 512
   ```

## Configuration Options

- `--urls`: List of URLs to process (default: reads from config)
- `--chunk-size`: Size of text chunks (default: 512)
- `--chunk-overlap`: Overlap between chunks (default: 50)
- `--model`: Cohere model to use (default: embed-english-v3.0)

## Verification

After running the pipeline, verify that:
1. Text content has been extracted from the URLs
2. Embeddings have been generated successfully
3. Data has been stored in Qdrant collection
4. Test queries return relevant results