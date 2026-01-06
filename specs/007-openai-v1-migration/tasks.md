---
description: "Task list for OpenAI Assistants v1 to v2 Migration feature"
---

# Tasks: OpenAI Assistants v1 to v2 Migration

**Input**: Design documents from `/specs/[007-openai-v1-migration]/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/`
- Paths shown below assume single project - adjust based on plan.md structure

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Update requirements.txt to ensure compatible OpenAI SDK version
- [x] T002 [P] Configure environment variables for OpenAI API access
- [x] T003 [P] Verify existing project structure and dependencies

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Create OpenAI client initialization with Chat Completions API in backend/src/client.py
- [x] T005 [P] Update models.py to match QueryRequest and QueryResponse schemas from data-model.md
- [x] T006 [P] Create RAG service interface for content retrieval
- [x] T007 Configure error handling and logging infrastructure for API calls
- [x] T008 Setup validation to detect deprecated API usage at startup

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Query AI Assistant (Priority: P1) 🎯 MVP

**Goal**: Enable users to submit queries to the AI assistant and receive responses with answer and optional sources

**Independent Test**: Can be fully tested by sending a query to the `/query` endpoint and receiving a response with an answer and optional sources, delivering the core AI assistance capability.

### Tests for User Story 1 (OPTIONAL - only if tests requested) ⚠️

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T009 [P] [US1] Contract test for /query endpoint in tests/contract/test_query_endpoint.py
- [ ] T010 [P] [US1] Integration test for query processing in tests/integration/test_query_flow.py

### Implementation for User Story 1

- [x] T011 [P] [US1] Create RAG service implementation in backend/src/rag_service.py
- [x] T012 [US1] Migrate agent.py from deprecated Assistants v1 to Chat Completions API (depends on T004)
- [x] T013 [US1] Update agent_integration.py to work with new API (depends on T012)
- [x] T014 [US1] Implement /query endpoint in backend/src/api.py with Chat Completions API
- [x] T015 [US1] Map response from new API to expected QueryResponse format { "response": string, "conversation_id": string, "sources": array, "debug_info": object }
- [x] T016 [US1] Add validation and error handling for query endpoint
- [x] T017 [US1] Add logging for query operations

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - System Startup Validation (Priority: P2)

**Goal**: Ensure the application fails fast during startup if deprecated OpenAI APIs are referenced

**Independent Test**: Can be tested by starting the application with deprecated API references and verifying it fails to start with a clear error message.

### Tests for User Story 2 (OPTIONAL - only if tests requested) ⚠️

- [ ] T018 [P] [US2] Test startup validation with deprecated API references in tests/unit/test_startup_validation.py
- [ ] T019 [P] [US2] Test successful startup with valid API configuration in tests/unit/test_startup_success.py

### Implementation for User Story 2

- [x] T020 [P] [US2] Create startup validation module in backend/src/startup_validator.py
- [x] T021 [US2] Implement deprecated API detection logic
- [x] T022 [US2] Integrate startup validation with main application startup in backend/main.py
- [x] T023 [US2] Add clear error messaging for deprecated API usage

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Error Handling (Priority: P3)

**Goal**: Handle API errors gracefully without exposing sensitive information

**Independent Test**: Can be tested by triggering various error conditions and verifying appropriate error responses without exposing internal details.

### Tests for User Story 3 (OPTIONAL - only if tests requested) ⚠️

- [ ] T024 [P] [US3] Test OpenAI API error handling in tests/unit/test_error_handling.py
- [ ] T025 [P] [US3] Test rate limiting error scenarios in tests/unit/test_rate_limiting.py

### Implementation for User Story 3

- [x] T026 [P] [US3] Create error handling utilities in backend/src/error_handler.py
- [x] T027 [US3] Implement secure error responses that don't expose internal details
- [x] T028 [US3] Add proper error logging without exposing secrets
- [x] T029 [US3] Update /query endpoint with enhanced error handling

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T030 [P] Update documentation in docs/ for new API integration
- [x] T031 Code cleanup and refactoring of migrated components
- [x] T032 Performance optimization for API calls and response handling
- [x] T033 [P] Additional unit tests in tests/unit/ for all migrated functionality
- [x] T034 Security hardening of API calls and error responses
- [x] T035 Run quickstart.md validation to ensure all functionality works

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together (if tests requested):
Task: "Contract test for /query endpoint in tests/contract/test_query_endpoint.py"
Task: "Integration test for query processing in tests/integration/test_query_flow.py"

# Launch all models for User Story 1 together:
Task: "Create RAG service implementation in backend/src/rag_service.py"
Task: "Create OpenAI client initialization with Chat Completions API in backend/src/client.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence