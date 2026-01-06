# How to Re-Ingest Textbook Content

This document explains the process for re-ingesting textbook content into the Qdrant vector database when the source documents are updated.

## Prerequisites

Before re-ingesting content, ensure you have:

1. **Environment configured**: Make sure your `.env` file contains the correct Qdrant and Cohere API credentials
2. **Sufficient Cohere API quota**: Note that embedding generation consumes API calls from your monthly quota
3. **Updated source documents**: Ensure the `/frontend-book/docs` folder contains the latest textbook content

## Re-Ingestion Process

### Step 1: Verify Environment

First, ensure your environment is properly configured:

```bash
# Check that environment variables are set
python -c "import os; from dotenv import load_dotenv; load_dotenv(); print('Qdrant URL:', os.getenv('QDRANT_URL')); print('Cohere API Key present:', bool(os.getenv('COHERE_API_KEY')))"
```

### Step 2: Run the Ingestion Script

Execute the ingestion script to process all documents in the `/frontend-book/docs` folder:

```bash
python ingest_docs.py
```

The script will:
- Connect to your Qdrant collection
- Process all `.md` and `.mdx` files in the `/frontend-book/docs` folder
- Clean and chunk the content appropriately (500-800 tokens with 150-token overlap)
- Generate embeddings using the Cohere API
- Store the content with rich metadata in the Qdrant collection

### Step 3: Verify Ingestion Success

After ingestion completes, verify that the content was properly ingested:

```bash
# Check the number of vectors in your collection
python -c "
from dotenv import load_dotenv
import os
from qdrant_client import QdrantClient
load_dotenv()
client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))
collection_info = client.get_collection(os.getenv('QDRANT_COLLECTION_NAME', 'humanoid_robotics_docs'))
print(f'Total vectors in collection: {collection_info.points_count}')
"
```

## Important Notes

- **API Usage**: Each document chunk consumes one Cohere API call. Monitor your API usage to avoid hitting rate limits.
- **Rate Limits**: If you hit Cohere rate limits during ingestion, the script will automatically retry after a delay.
- **Overwrite Behavior**: The ingestion process will overwrite existing content in Qdrant. Document IDs are generated deterministically, so re-ingesting the same content will update existing vectors.
- **Chunking Strategy**: Documents are automatically chunked by headings with 500-800 token limits and 150-token overlap to maintain context while staying within model limits.

## Troubleshooting

### Common Issues

1. **Cohere API Rate Limits**: If you encounter rate limit errors, wait for the retry period (typically 60 seconds) or upgrade your API plan.

2. **Qdrant Connection Issues**: Verify your Qdrant URL and API key are correct in your `.env` file.

3. **Large Document Processing**: Very large documents are automatically chunked into smaller pieces to maintain optimal retrieval performance.

## Verification

After re-ingestion, test the system with sample queries to ensure the new content is properly retrievable:

```bash
python -c "
from agent import HumanoidRoboticsAgent
agent = HumanoidRoboticsAgent()
response = agent.ask('What is ROS 2?', debug=True)
print(response)
"
```