# Research: OpenAI Assistants v1 to v2 Migration

## Phase 0: Research Findings

### Task 0.1: Identify Deprecated API Usage

**Decision**: Identified deprecated OpenAI Assistants v1 API endpoints in `agent.py`
**Rationale**: The agent was using the beta threads/runs API which is now deprecated
**Specific deprecated endpoints found**:
- `openai_client.beta.threads.create()` - Create thread
- `openai_client.beta.threads.messages.create()` - Add message to thread
- `openai_client.beta.assistants.create()` - Create assistant
- `openai_client.beta.threads.runs.create()` - Create run
- `openai_client.beta.threads.runs.retrieve()` - Retrieve run status
- `openai_client.beta.threads.messages.list()` - List thread messages

**Alternatives considered**:
1. OpenAI Chat Completions API (selected) - Direct API calls, simpler implementation
2. OpenAI Assistants v2 API - More feature-rich but complex migration
3. OpenAI Messages API - Part of newer assistants API

### Task 0.2: Research Modern API Options

**Decision**: Selected OpenAI Chat Completions API as replacement
**Rationale**: Simpler implementation, maintains RAG functionality, direct replacement
**Comparison**:
- **Chat Completions API**: Direct call, system message for context, maintains RAG functionality
- **Assistants v2 API**: More complex (assistant/thread/run pattern), requires more setup
- **Feature parity**: Both support RAG pattern, Chat Completions is more straightforward

**API Version Headers**: Using `OpenAI-Beta: assistants=v2` header is not needed for Chat Completions API, but for Assistants v2 it would be required.

### Task 0.3: Analyze Frontend Contract

**Decision**: Frontend expects QueryResponse format with specific fields
**Rationale**: Existing frontend integration expects specific response structure
**Response contract**:
```json
{
  "response": "answer text",
  "conversation_id": "conversation identifier",
  "sources": ["array", "of", "source", "urls"],
  "debug_info": "optional debug information"
}
```

**Field mappings**:
- `response` → maps to AI-generated answer text
- `conversation_id` → maintains conversation context
- `sources` → array of retrieved document sources
- `debug_info` → optional debugging information

## Implementation Strategy

### Migration Approach
**Decision**: Replace Assistants v1 with Chat Completions API
**Rationale**: Maintains all functionality while using supported API, simpler migration path

### Technical Implementation
1. **Replace API calls**: `openai_client.chat.completions.create()` instead of threads/runs
2. **Context provision**: Provide retrieved content in system message instead of separate context
3. **Conversation history**: Maintain multi-turn conversations with message history
4. **Response format**: Map new API response to existing QueryResponse structure

### API Version and Contract Validation

**Decision**: Enforce modern API patterns with proper error handling
**Rationale**: Ensure production readiness and maintainability

**Version enforcement**:
- Use latest OpenAI SDK version
- Implement proper error handling for API changes
- Validate response formats at API boundaries

**Contract validation**:
- Input validation using Pydantic models
- Output validation through response models
- Error response consistency