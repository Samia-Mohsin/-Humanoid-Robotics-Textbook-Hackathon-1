# Quickstart: RAG Chatbot Textbook Integration Fix

## Setup

### Prerequisites
- Python 3.11+
- OpenAI API key
- Cohere API key
- Qdrant Cloud account with collection "humanoid_robotics_docs"

### Installation
1. Install dependencies:
   ```bash
   pip install openai qdrant-client cohere python-dotenv langchain
   ```

2. Set up environment variables:
   ```bash
   # Create .env file with:
   OPENAI_API_KEY=your_openai_api_key
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_URL=your_qdrant_cloud_url
   QDRANT_API_KEY=your_qdrant_api_key
   ```

### Document Ingestion
1. Run the ingestion script to load textbook content:
   ```bash
   python ingest_docs.py
   ```

2. Verify documents are loaded:
   - Check console output for successful ingestion messages
   - Verify chunks created and uploaded to Qdrant

## Usage

### Run the Agent
1. Start the RAG agent:
   ```bash
   python agent.py
   ```

2. Ask questions about humanoid robotics:
   - "What is ROS 2?"
   - "Explain nodes and topics in ROS 2"
   - "What is URDF?"

### Debug Mode
Run with debug output to see retrieved chunks:
```bash
DEBUG=1 python agent.py
```

## Expected Output
When you ask "What is ROS 2?", the agent should return detailed information from Module 1 of the textbook rather than saying "information not available".

## Troubleshooting

### Common Issues
- **"Information not available"**: Run ingestion script to ensure documents are loaded
- **Connection errors**: Verify API keys and service availability
- **Poor responses**: Check that document ingestion completed successfully with proper metadata

### Validation Queries
Run these queries to verify the fix is working:
- "What is ROS 2?"
- "Explain nodes and topics in ROS 2"
- "What is URDF?"
- "How does Gazebo simulate physics?"
- "What is NVIDIA Isaac Sim used for?"

All queries should return content directly from the textbook with proper citations.