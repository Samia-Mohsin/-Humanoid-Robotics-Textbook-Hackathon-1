# Feature Specification: OpenAI Assistants v1 to v2 Migration

**Feature Branch**: `007-openai-v1-migration`
**Created**: 2025-12-31
**Status**: Draft
**Input**: User description: "You are a Spec-Kit Plus compliant debugging agent.

Context:
A Docusaurus + FastAPI AI assistant fails with error:
\"The v1 Assistants API has been deprecated\".

Root Causes:
- Backend calls deprecated OpenAI Assistants v1 endpoints
- Missing OpenAI-Beta header or modern Responses API usage
- No version pinning or runtime validation

Requirements:
1. Remove all Assistants v1 API usage.
2. Migrate backend to OpenAI Responses API.
3. Enforce explicit model and API version selection.
4. Backend must expose a single `/query` endpoint returning:
   { \"answer\": string, \"sources\": optional array }
5. Frontend must remain unchanged except endpoint URL.
6. Add startup validation that fails fast if deprecated APIs are referenced.

Deliverables:
- Updated FastAPI `query` handler using Responses API
- Dependency-safe OpenAI client initialization
- Minimal logging for failures (no secrets)
- Backward-compatible JSON response

Constraints:
- No Assistants v1 or beta APIs
- No UI changes
- Production-safe implementation

Output:
Provide only code diffs and final corrected files."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Query AI Assistant (Priority: P1)

As a user of the Physical AI & Humanoid Robotics textbook website, I want to be able to ask questions about the content and receive accurate responses so that I can better understand the material.

**Why this priority**: This is the core functionality of the AI assistant - without it working, the entire feature fails to deliver value.

**Independent Test**: Can be fully tested by sending a query to the `/query` endpoint and receiving a response with an answer and optional sources, delivering the core AI assistance capability.

**Acceptance Scenarios**:

1. **Given** the AI assistant is properly configured with OpenAI credentials, **When** a user submits a query to `/query`, **Then** the system returns a response with an "answer" field containing the AI response and optional "sources" array.
2. **Given** the AI assistant is configured with OpenAI credentials, **When** a user submits a query that references documentation content, **Then** the system returns a response with both "answer" and "sources" fields populated.

---

### User Story 2 - System Startup Validation (Priority: P2)

As a system administrator, I want the application to fail fast during startup if deprecated OpenAI APIs are referenced, so that I can catch configuration issues early.

**Why this priority**: Prevents runtime failures and ensures the system is properly configured before going live.

**Independent Test**: Can be tested by starting the application with deprecated API references and verifying it fails to start with a clear error message.

**Acceptance Scenarios**:

1. **Given** the application has deprecated OpenAI API references, **When** the system starts up, **Then** it fails with a clear error message identifying the deprecated API usage.

---

### User Story 3 - Error Handling (Priority: P3)

As a user, I want the system to handle API errors gracefully without exposing sensitive information, so that the service remains available and secure.

**Why this priority**: Ensures production stability and security while providing appropriate user feedback.

**Independent Test**: Can be tested by triggering various error conditions and verifying appropriate error responses without exposing internal details.

**Acceptance Scenarios**:

1. **Given** an OpenAI API error occurs, **When** a user submits a query, **Then** the system returns a user-friendly error message without exposing internal API details.

---

## Edge Cases

- What happens when OpenAI API keys are invalid or missing?
- How does system handle rate limiting from OpenAI API?
- What happens when the query is too long for the API?
- How does the system handle network timeouts to OpenAI?
- What happens when the OpenAI service is temporarily unavailable?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST use OpenAI Responses API (v2) instead of deprecated Assistants v1 API
- **FR-002**: System MUST expose a `/query` endpoint that accepts JSON requests with a "query" field
- **FR-003**: System MUST return JSON responses with "answer" (string) and optional "sources" (array) fields
- **FR-004**: System MUST validate at startup that no deprecated OpenAI Assistants v1 APIs are referenced
- **FR-005**: System MUST initialize OpenAI client with explicit model and API version selection
- **FR-006**: System MUST log errors minimally without exposing secrets or internal details
- **FR-007**: System MUST maintain backward compatibility with existing frontend JSON response format

### Key Entities *(include if feature involves data)*

- **Query Request**: User input containing the question or request to be processed by the AI
- **Query Response**: AI-generated response containing answer and optional source references
- **OpenAI Client**: Interface to communicate with OpenAI's Responses API with proper authentication

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can submit queries to the `/query` endpoint and receive responses with "answer" field populated within 10 seconds
- **SC-002**: System startup fails fast with clear error message when deprecated OpenAI Assistants v1 APIs are detected
- **SC-003**: 95% of valid queries return successful responses without internal errors
- **SC-004**: System handles OpenAI API errors gracefully without exposing sensitive information