# Feature Specification: Fix FastAPI Test Failures

**Feature Branch**: `006-fix-fastapi-test-failures`
**Created**: 2025-12-30
**Status**: Draft
**Input**: User description: "Debug and fix FastAPI test failures caused by incorrect request parameter injection and middleware interactions. Ensure SlowAPI decorators receive a proper starlette.requests.Request (not a Pydantic body model), refactor /query endpoint signature to use a single Request plus body schema, and update tests accordingly. Fix 422 errors for large queries by aligning request models, validation, and TestClient payloads. Stabilize logging to avoid WinError 6 during pytest and ensure all tests pass on Python 3.13 with pinned FastAPI/Starlette/httpx versions."

## User Scenarios & Testing *(mandatory)*

<!--
  IMPORTANT: User stories should be PRIORITIZED as user journeys ordered by importance.
  Each user story/journey must be INDEPENDENTLY TESTABLE - meaning if you implement just ONE of them,
  you should still have a viable MVP (Minimum Viable Product) that delivers value.

  Assign priorities (P1, P2, P3, etc.) to each story, where P1 is the most critical.
  Think of each story as a standalone slice of functionality that can be:
  - Developed independently
  - Tested independently
  - Deployed independently
  - Demonstrated to users independently
-->

### User Story 1 - Fix TestClient Parameter Injection (Priority: P1)

As a developer, I need the TestClient to work correctly with the API endpoints so that I can run tests without encountering "unexpected keyword argument 'app'" errors.

**Why this priority**: This is the most critical issue preventing all tests from running successfully.

**Independent Test**: Can run pytest without TestClient initialization errors, delivering the ability to validate all functionality through automated tests.

**Acceptance Scenarios**:
1. **Given** I run `pytest` on the test suite, **When** tests execute, **Then** TestClient instantiates without "unexpected keyword argument" errors
2. **Given** I call the /query endpoint in tests, **When** I pass query parameters correctly, **Then** endpoint processes requests without parameter injection errors

---

### User Story 2 - Fix SlowAPI Decorator Issues (Priority: P2)

As a developer, I need the SlowAPI rate limiting decorators to work properly with correct Request objects so that rate limiting functions without errors.

**Why this priority**: Rate limiting is essential for production functionality and preventing API abuse.

**Independent Test**: Can apply rate limiting decorators to endpoints and they receive proper Request objects, delivering proper rate limiting functionality.

**Acceptance Scenarios**:
1. **Given** SlowAPI decorator is applied to an endpoint, **When** a request comes in, **Then** decorator receives proper starlette.requests.Request object
2. **Given** Rate limit is exceeded, **When** subsequent requests arrive, **Then** proper 429 responses are returned

---

### User Story 3 - Fix Large Query Handling (Priority: P3)

As a user, I need the API to handle large queries without 422 validation errors so that I can submit detailed questions to the RAG system.

**Why this priority**: Ensures the system works with realistic user inputs of varying lengths.

**Independent Test**: Can submit queries of various lengths (including large ones) without encountering 422 errors, delivering support for comprehensive questions.

**Acceptance Scenarios**:
1. **Given** I submit a large query (500+ characters), **When** request is processed, **Then** 200 response is returned without validation errors
2. **Given** I submit a query at the maximum supported length, **When** request is processed, **Then** response is generated successfully

---

### User Story 4 - Stabilize Logging for Tests (Priority: P4)

As a developer, I need stable logging during tests so that I don't encounter WinError 6 issues that interrupt test execution.

**Why this priority**: Clean test execution is necessary for CI/CD and development workflow.

**Independent Test**: Can run pytest without encountering WinError 6 logging issues, delivering reliable test execution.

**Acceptance Scenarios**:
1. **Given** I run pytest, **When** tests execute with logging, **Then** no WinError 6 occurs during test execution
2. **Given** API endpoints are called during tests, **When** logging occurs, **Then** logs are written without file handle errors

---

### Edge Cases
- What happens when the TestClient is used with FastAPI dependency overrides?
- How does the system handle queries that exceed the maximum allowed length after the fix?
- What occurs when multiple rate limiting decorators are applied to the same endpoint?
- How does the system behave when running tests in parallel with shared resources?

## Requirements *(mandatory)*

<!--
  ACTION REQUIRED: The content in this section represents placeholders.
  Fill them out with the right functional requirements.
-->

### Functional Requirements

- **FR-001**: System MUST instantiate TestClient with positional app argument (not keyword argument)
- **FR-002**: System MUST pass proper starlette.requests.Request objects to SlowAPI decorators
- **FR-003**: System MUST accept query requests up to 2000 characters without 422 errors
- **FR-004**: System MUST handle large query validation without parameter binding errors
- **FR-005**: System MUST apply rate limiting consistently across all endpoints
- **FR-006**: System MUST log messages without causing WinError 6 during tests
- **FR-007**: System MUST maintain backward compatibility with existing API contracts
- **FR-008**: System MUST support concurrent test execution without resource conflicts

### Key Entities *(include if feature involves data)*

- **TestClient**: FastAPI test client for endpoint testing
- **Request Object**: Starlette request object passed to middleware and decorators
- **Rate Limit Configuration**: Settings for SlowAPI rate limiting parameters
- **Query Model**: Pydantic model for query validation and processing

## Success Criteria *(mandatory)*

<!--
  ACTION REQUIRED: Define measurable success criteria.
  These must be technology-agnostic and measurable.
-->

### Measurable Outcomes

- **SC-001**: All pytest tests pass without TestClient initialization errors
- **SC-002**: Rate limiting works correctly with proper Request object handling
- **SC-003**: Queries up to 2000 characters are processed without 422 errors
- **SC-004**: No WinError 6 occurs during test execution
- **SC-005**: Test execution completes with 100% success rate
- **SC-006**: API maintains response time under 10 seconds for all query sizes
- **SC-007**: Concurrent test execution runs without conflicts
- **SC-008**: All existing functionality remains intact after fixes