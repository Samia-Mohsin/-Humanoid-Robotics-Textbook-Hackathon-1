# RAG Chatbot API for Humanoid Robotics Textbook

This project implements a Retrieval-Augmented Generation (RAG) chatbot API that integrates with the Physical AI & Humanoid Robotics textbook content. The API allows users to ask questions about humanoid robotics and receive responses based on the textbook content.

## Features

- **FastAPI-based API**: Modern, high-performance API with automatic documentation
- **RAG Integration**: Questions are answered based on textbook content using retrieval-augmented generation
- **Qdrant Vector Database**: Efficient similarity search for relevant content retrieval
- **Cohere Embeddings**: High-quality text embeddings for semantic search
- **Conversation Support**: Maintains conversation context across multiple queries
- **Rate Limiting**: Prevents API abuse with configurable rate limits
- **Comprehensive Logging**: Detailed request/response logging for monitoring
- **Production Ready**: Includes Docker configuration and deployment setup

## Architecture

The system consists of:

1. **FastAPI Server**: Hosts the API endpoints
2. **RAG Agent**: Processes queries using textbook content
3. **Qdrant Database**: Stores document embeddings for similarity search
4. **Cohere API**: Generates text embeddings
5. **OpenAI API**: Powers the response generation

## Prerequisites

- Python 3.11+
- Qdrant Cloud account
- Cohere API key
- OpenAI API key

## Setup Instructions

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Create Virtual Environment

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

### 4. Configure Environment Variables

Create a `.env` file in the project root with the following variables:

```env
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=humanoid_robotics_docs
COHERE_API_KEY=your_cohere_api_key
OPENAI_API_KEY=your_openai_api_key
DEBUG=false
LOG_LEVEL=INFO
```

### 5. Run the Application

```bash
# Development mode with hot reload
python api.py

# Or using uvicorn directly
uvicorn api:app --reload --host 0.0.0.0 --port 8000
```

## API Endpoints

### Health Check
- `GET /health` - Check API status
- `GET /` - Root endpoint with basic information

### Query Endpoint
- `POST /query` - Send a query and receive a response from the RAG agent

Request body:
```json
{
  "query": "Your question here",
  "conversation_id": "optional conversation ID for continuity",
  "debug": false
}
```

Response:
```json
{
  "response": "The agent's response",
  "conversation_id": "The conversation ID",
  "sources": ["List of source citations"],
  "debug_info": "Debug information (when debug=true)"
}
```

## API Documentation

Interactive API documentation is available at:
- Swagger UI: `http://localhost:8000/docs`
- ReDoc: `http://localhost:8000/redoc`

## Docker Deployment

### Build and Run with Docker

```bash
# Build the image
docker build -t rag-chatbot-api .

# Run the container
docker run -p 8000:8000 --env-file .env rag-chatbot-api
```

### Run with Docker Compose

```bash
docker-compose up -d
```

## Configuration

### Environment Variables

- `QDRANT_URL`: URL for your Qdrant instance
- `QDRANT_API_KEY`: API key for Qdrant access
- `QDRANT_COLLECTION_NAME`: Name of the collection containing textbook embeddings
- `COHERE_API_KEY`: Cohere API key for embeddings
- `OPENAI_API_KEY`: OpenAI API key for response generation
- `DEBUG`: Enable debug mode (true/false)
- `LOG_LEVEL`: Logging level (DEBUG, INFO, WARNING, ERROR)
- `HOST`: Host to bind to (default: 0.0.0.0)
- `PORT`: Port to listen on (default: 8000)

## Usage Examples

### Query the API

```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS 2?",
    "debug": false
  }'
```

### Start a conversation

```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain nodes and topics in ROS 2",
    "conversation_id": "my-conversation-123",
    "debug": false
  }'
```

## Development

### Running Tests

```bash
python -m pytest tests/
```

### Code Formatting

```bash
# Format code with black
black .

# Check for linting issues
flake8 .
```

## Troubleshooting

### Common Issues

1. **API Rate Limits**: If you encounter rate limit errors, this is due to Cohere/OpenAI API limits. Consider upgrading to a production key with higher limits.

2. **Qdrant Connection Issues**: Verify your Qdrant URL and API key are correct in your environment variables.

3. **Empty Responses**: If queries return "information not available", verify that the textbook content has been properly ingested into Qdrant.

### Debugging

Enable debug mode by setting `"debug": true` in your query requests to see retrieved chunks and similarity scores.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

[Specify license here]

## Contact

For questions or support, please contact [your contact information].