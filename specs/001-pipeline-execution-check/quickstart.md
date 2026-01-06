# Quickstart: Pipeline Execution Summary and Result Analysis

## Overview
This feature enhances the pipeline execution summary to provide comprehensive metrics and validation as specified in the feature requirements. The implementation improves the existing execution summary to include detailed counts, execution time reporting, storage success status, and validation that stored embeddings match generated embeddings.

## Prerequisites
- Python 3.11+
- Backend pipeline dependencies installed
- Valid Cohere API key
- Valid Qdrant Cloud credentials

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
Run the pipeline with URL(s) as usual - the enhanced execution summary will automatically display comprehensive metrics:

```bash
python main.py --urls "https://example.com" "https://docs.example.com"
```

## Expected Output
After running the pipeline, you'll see an enhanced execution summary including:
- Number of URLs processed
- Number of documents extracted
- Number of chunks created
- Number of embeddings generated
- Storage success status
- Total execution time
- Status (completed/failed/partial)

## Validation
The enhanced pipeline will also validate that:
- Stored embeddings match generated embeddings count
- Error messages are clear and actionable
- Performance metrics are captured for each phase