# Data Model: OpenAI Assistants v1 to v2 Migration

## Entities

### QueryRequest
**Description**: Request model for the query endpoint
**Fields**:
- `query`: string (required, min_length=3, max_length=5000) - The user's question or query text
- `conversation_id`: string (optional) - Unique identifier for conversation continuity
- `debug`: boolean (optional, default=False) - Flag to enable debug mode

**Validation Rules**:
- Query must be at least 3 characters long
- Query cannot exceed 5000 characters
- Query cannot be empty or whitespace only

### QueryResponse
**Description**: Response model for the query endpoint
**Fields**:
- `response`: string (required) - The RAG agent's response to the query
- `conversation_id`: string (optional) - Conversation identifier for continuity
- `sources`: array of strings (optional) - List of source citations for the response
- `debug_info`: object (optional) - Debug information when debug mode is enabled

**Validation Rules**:
- Response field must be present and non-empty
- Sources array items must be valid URLs or identifiers
- Debug info must follow expected structure when present

### Agent Response Structure
**Description**: Internal structure returned by the agent's new API
**Fields**:
- `answer`: string - The AI-generated response text
- `sources`: array of strings - Array of source URLs from retrieved content

**State Transitions**:
- Request received → Content retrieved from Qdrant → Context provided to OpenAI → Response generated → Response formatted → Response returned

## API Contract

### Request Contract
- Endpoint: `POST /query`
- Content-Type: `application/json`
- Required fields: `query`
- Authentication: API key in environment variables (not in request)

### Response Contract
- Success status: `200 OK`
- Content-Type: `application/json`
- Required fields: `response`
- Optional fields: `conversation_id`, `sources`, `debug_info`

### Error Contract
- Error status: `400 Bad Request` for validation errors, `500 Internal Server Error` for internal errors
- Error format: Follows FastAPI default error response format
- Error messages: User-friendly without exposing internal details