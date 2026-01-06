# Quickstart: FastAPI RAG Integration

**Feature**: FastAPI RAG Integration
**Created**: 2025-12-29

## Overview
This guide provides quick instructions for setting up and running the FastAPI server that integrates with the RAG agent for the Humanoid Robotics textbook chatbot.

## Prerequisites
- Python 3.11+
- pip package manager
- Access to Qdrant Cloud with humanoid_robotics_docs collection
- Cohere API key
- OpenAI API key

## Setup

### 1. Clone the repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Create virtual environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install dependencies
```bash
pip install fastapi uvicorn python-dotenv pydantic
pip install openai qdrant-client cohere
```

### 4. Set up environment variables
Create a `.env` file with the following:
```
QDRANT_URL="your_qdrant_url"
QDRANT_API_KEY="your_qdrant_api_key"
QDRANT_COLLECTION_NAME="humanoid_robotics_docs"
COHERE_API_KEY="your_cohere_api_key"
OPENAI_API_KEY="your_openai_api_key"
```

## Running the Server

### 1. Start the FastAPI server
```bash
uvicorn api:app --reload --port 8000
```

### 2. Verify the server is running
Open your browser to `http://localhost:8000/health` or use curl:
```bash
curl http://localhost:8000/health
```

### 3. Access API documentation
- Swagger UI: `http://localhost:8000/docs`
- ReDoc: `http://localhost:8000/redoc`

## Using the API

### Query the RAG agent
```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS 2?",
    "debug": false
  }'
```

### Example response
```json
{
  "response": "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software...",
  "conversation_id": "123e4567-e89b-12d3-a456-426614174000",
  "sources": ["file://docs/intro.md", "file://docs/ros2-basics/index.md"]
}
```

## Development

### File Structure
```
api.py              # Main FastAPI application
models.py           # Pydantic models for request/response validation
agent_integration.py # RAG agent integration layer
endpoints.py        # API endpoint definitions
config.py           # Configuration settings
test_api.py         # Tests for the API
```

### Running tests
```bash
python -m pytest test_api.py
```

## Deployment

### Using Docker
```dockerfile
FROM python:3.11-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install -r requirements.txt

COPY . .

CMD ["uvicorn", "api:app", "--host", "0.0.0.0", "--port", "8000"]
```

### Environment Variables for Production
Ensure the following environment variables are set in your production environment:
- `QDRANT_URL`
- `QDRANT_API_KEY`
- `QDRANT_COLLECTION_NAME`
- `COHERE_API_KEY`
- `OPENAI_API_KEY`
- `ENVIRONMENT` (development/production)

## Troubleshooting

### Common Issues

1. **Agent initialization failure**: Check that Qdrant and Cohere credentials are correct
2. **Rate limit errors**: The Cohere API has rate limits; implement proper retry logic
3. **Slow responses**: Large documents or complex queries may take time to process

### Debug Mode
Enable debug mode in requests to get additional information about the retrieval process:
```json
{
  "query": "Your question here",
  "debug": true
}
```