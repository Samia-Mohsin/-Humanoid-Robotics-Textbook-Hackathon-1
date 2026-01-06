# Quickstart: RAG Retrieval Validation

## Setup

1. **Install Dependencies**
   ```bash
   pip install qdrant-client cohere python-dotenv
   ```

2. **Configure Environment**
   Create a `.env` file in the root directory with:
   ```
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_api_key
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_COLLECTION_NAME=your_collection_name
   ```

## Usage

### Run the Retrieval Validation Tool

```bash
python retrieve.py --query "What is humanoid robotics?" --top-k 5
```

### Options

- `--query`: The search query to validate (required)
- `--top-k`: Number of results to return (default: 5)
- `--collection`: Qdrant collection name (default: from .env)
- `--validate`: Whether to run validation checks (default: True)

### Example

```bash
# Basic retrieval
python retrieve.py --query "ROS for humanoid robots"

# Retrieval with specific number of results
python retrieve.py --query "advanced physics simulation" --top-k 10

# Detailed validation
python retrieve.py --query "sensor fusion techniques" --validate
```

## Expected Output

The tool will output:
- Connection status to Qdrant
- Query execution time
- Retrieved text chunks with source URLs
- Validation report showing metadata integrity
- Overall validation status (pass/fail)

## Troubleshooting

- If you get connection errors, verify your Qdrant URL and API key in `.env`
- If no results are returned, check that the collection contains data
- If validation fails, ensure the Cohere API key is valid and has sufficient quota