# Data Model: Fixed FastAPI TestClient and SlowAPI Integration

## Updated Request Models

### QueryRequest
- **query**: string (required)
  - **min_length**: 1 character
  - **max_length**: 2000 characters (increased from previous limit)
  - **validation**: Required field with proper length constraints
  - **purpose**: User's query text for RAG processing

- **conversation_id**: string (optional)
  - **pattern**: UUID format if provided
  - **default**: None
  - **purpose**: Maintains conversation context across queries

- **debug**: boolean (optional)
  - **default**: False
  - **purpose**: Enables debug mode showing retrieved chunks

### QueryResponse
- **response**: string (required)
  - **min_length**: 1 character
  - **purpose**: The RAG agent's response to the query

- **conversation_id**: string (optional)
  - **purpose**: Conversation identifier for continuity

- **sources**: array of strings (optional)
  - **item_type**: URL or document reference
  - **purpose**: Source citations for the response

- **debug_info**: object (optional)
  - **conditional**: Only present when debug mode enabled
  - **purpose**: Contains retrieval details for debugging

## TestClient Request Pattern

### Correct TestClient Usage
- **instantiation**: `TestClient(app)` (positional argument)
- **not**: `TestClient(app=app)` (keyword argument - deprecated)
- **purpose**: Avoids TypeError with newer Starlette/httpx versions

### Test Request Payload
- **method**: POST
- **endpoint**: `/query`
- **content_type**: `application/json`
- **payload**: QueryRequest model instance
- **validation**: Must conform to QueryRequest schema

## Rate Limiting Configuration

### Rate Limit Model
- **limit_pattern**: "requests_per_time_period"
- **default**: "10/minute" (10 requests per minute per IP)
- **key_func**: `get_remote_address` from SlowAPI
- **exceeded_status**: 429 (Too Many Requests)

### Request Object Requirements
- **type**: `starlette.requests.Request` (not Pydantic model)
- **dependency**: Must be injected as function parameter
- **position**: Typically first parameter in rate-limited endpoints
- **usage**: Required for SlowAPI to extract client identity

## Response Validation Models

### Success Response
- **status_code**: 200
- **content_type**: `application/json`
- **body**: QueryResponse model
- **headers**: Standard FastAPI response headers

### Rate Limit Exceeded Response
- **status_code**: 429
- **content_type**: `application/json`
- **body**: Error message with rate limit details
- **headers**: Retry-After header with reset time

### Validation Error Response
- **status_code**: 429 or 400 (depending on error type)
- **content_type**: `application/json`
- **body**: Pydantic validation error details
- **headers**: Standard error response headers

## Logging Configuration Model

### Test Environment Logging
- **level**: INFO for tests
- **handlers**: Console only (no file handlers during tests)
- **format**: Test-safe format without file conflicts
- **cleanup**: Automatic resource release after tests

### Production Logging
- **level**: INFO or configurable
- **handlers**: Console and file (rotating)
- **format**: Structured JSON format
- **performance**: Async-safe to avoid blocking

## Error Handling Models

### Rate Limit Exceeded Error
- **exception_type**: `RateLimitExceeded` from SlowAPI
- **response_code**: 429
- **retry_after**: Calculated reset time
- **message**: Human-readable rate limit exceeded message

### Validation Error
- **exception_type**: `RequestValidationError` from FastAPI
- **response_code**: 422
- **details**: Field-specific validation errors
- **message**: Structured validation error response

### Internal Server Error
- **exception_type**: `HTTPException` or generic Exception
- **response_code**: 500
- **details**: Error logging (not exposed to client)
- **message**: Generic error message for security