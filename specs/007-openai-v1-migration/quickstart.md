# Quickstart: OpenAI Assistants v1 to v2 Migration

## Overview
This guide explains how to use the migrated API that now uses OpenAI Chat Completions API instead of the deprecated Assistants v1 API.

## API Endpoints

### Query Endpoint
- **Method**: `POST`
- **Path**: `/query`
- **Content-Type**: `application/json`

#### Request Body
```json
{
  "query": "Your question here",
  "conversation_id": "optional conversation identifier",
  "debug": false
}
```

#### Response Format
```json
{
  "response": "AI-generated answer",
  "conversation_id": "conversation identifier",
  "sources": ["array", "of", "source", "urls"],
  "debug_info": null
}
```

### Health Check Endpoint
- **Method**: `GET`
- **Path**: `/health`
- **Response**: Health status of the API service

## API Version Headers and Validation

The API now enforces modern OpenAI API patterns:
- Uses OpenAI SDK's Chat Completions API
- Validates request format with Pydantic models
- Enforces proper error handling
- Maintains response contract validation

## Testing the API

### Basic Query
```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is humanoid robotics?"}'
```

### With Debug Mode
```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{"query": "Explain reinforcement learning", "debug": true}'
```

## Integration Notes

### For Frontend Developers
- No changes required to frontend integration
- Response format remains identical to previous version
- All existing functionality preserved

### For Backend Developers
- Agent now uses `openai_client.chat.completions.create()` instead of deprecated APIs
- RAG functionality maintained using system messages with context
- Error handling follows same patterns as before