# Quickstart: FastAPI TestClient and SlowAPI Integration Fixes

## Overview
This guide helps developers quickly understand and implement the fixes for FastAPI TestClient and SlowAPI integration issues.

## Common Issues Addressed

### 1. TestClient Initialization Error
**Problem**: `TypeError: Client.__init__() got an unexpected keyword argument 'app'`
**Solution**: Use positional argument instead of keyword argument

```python
# ❌ Old (causes error)
client = TestClient(app=app)

# ✅ New (correct)
client = TestClient(app)
```

### 2. SlowAPI Request Object Issues
**Problem**: SlowAPI decorators not receiving proper Request objects
**Solution**: Ensure proper dependency injection patterns

```python
# ✅ Correct pattern
@limiter.limit("10/minute")
async def query_endpoint(request: Request, query_data: QueryRequest):
    # Both request and query_data are properly injected
    pass
```

### 3. Large Query Validation Errors
**Problem**: 422 errors for queries exceeding validation limits
**Solution**: Adjust validation models to support larger inputs

```python
# ✅ Increased validation limits
class QueryRequest(BaseModel):
    query: str = Field(..., min_length=1, max_length=2000)  # Was 1000
```

## Quick Fix Steps

### 1. Update TestClient Usage
1. Find all `TestClient(app=app)` patterns in test files
2. Replace with `TestClient(app)`
3. Run tests to confirm no initialization errors

### 2. Fix SlowAPI Decorators
1. Verify all rate-limited endpoints receive proper Request objects
2. Ensure decorator placement is correct
3. Test rate limiting functionality

### 3. Adjust Validation Models
1. Increase max_length limits in Pydantic models
2. Align with TestClient payload patterns
3. Test with large query inputs

## Testing the Fixes

### Before Applying Fixes
```bash
# This will show the TestClient errors
pytest
```

### After Applying Fixes
```bash
# This should run without TestClient initialization errors
pytest

# Verify large queries work
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{"query": "Very long query with more than 1000 characters..."}'
```

## Key Files to Update

1. `test_api.py` - Fix TestClient instantiations
2. `api.py` - Update SlowAPI decorator patterns
3. `models.py` - Adjust validation limits
4. `endpoints.py` - Ensure proper request handling
5. `config.py` - Verify logging configuration for tests

## Verification Steps

1. Run `pytest` - All tests should pass without TestClient errors
2. Test rate limiting with multiple requests
3. Submit large queries (>1000 chars) without 422 errors
4. Confirm no WinError 6 occurs during testing
5. Verify all existing functionality remains intact

## Troubleshooting

### If TestClient errors persist:
- Double-check all test files for `TestClient(app=app)` patterns
- Ensure using correct positional argument: `TestClient(app)`

### If SlowAPI still has issues:
- Verify Request parameter is properly typed in endpoints
- Check that limiter is correctly configured and attached

### If large queries still fail:
- Confirm validation models have increased limits
- Verify TestClient payloads match expected format