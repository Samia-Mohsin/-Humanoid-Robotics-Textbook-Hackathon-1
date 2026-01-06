# /sp.specify fix-testclient-keyword-error

## Overview
- Issue: Starlette TestClient API change – 'app' keyword argument removed in newer versions
- Affected: All test files using TestClient(app=app) or similar
- Target Success: All tests pass with zero TypeError
- Compatibility: Starlette ≥ 0.30+, FastAPI ≥ 0.100+, Python 3.11+

## Correct TestClient Instantiation
- Valid syntax (positional argument):
  client = TestClient(app)
- Valid with additional arguments:
  client = TestClient(app, base_url="http://testserver", raise_server_exceptions=False)
- Invalid (causes error):
  client = TestClient(app=app)  # ← This is broken in current versions

## Migration Rules
- Replace every occurrence of:
  TestClient(app=app, ...)
  TestClient(app=app)
- With:
  TestClient(app, ...)
  TestClient(app)

## Example Fixes
Before:
  client = TestClient(app=app)
After:
  client = TestClient(app)

Before:
  client = TestClient(app=app, raise_server_exceptions=False)
After:
  client = TestClient(app, raise_server_exceptions=False)

## Affected Files
- test_api.py (all test functions instantiating TestClient)

## Success Criteria
- All 17+ failing tests now pass
- No TypeError related to TestClient initialization
- Test behavior and assertions unchanged

## Constraints
- Minimal code changes (only TestClient calls)
- No version downgrades
- Maintains full test coverage

## Deliverables
- Updated test_api.py with corrected TestClient usage
- Comment in tests: "# TestClient uses positional 'app' argument (Starlette ≥0.30 compatibility)"