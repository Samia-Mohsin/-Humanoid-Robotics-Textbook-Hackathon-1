# Implementation Plan: OpenAI Assistants v1 to v2 Migration

**Feature**: OpenAI Assistants v1 to v2 Migration | **Branch**: `007-openai-v1-migration` | **Date**: 2025-12-31

## Overview

This plan addresses the deprecation of OpenAI Assistants v1 API by migrating to the modern OpenAI Responses API. The system currently fails with "The v1 Assistants API has been deprecated" error and needs to be updated to use the new API while maintaining the same frontend interface.

## Technical Context

The system uses a FastAPI backend with an AI agent that communicates with OpenAI's API. The current implementation uses deprecated Assistants v1 endpoints which need to be replaced with the modern Chat Completions API. The frontend expects a specific JSON response format that must be preserved.

Based on research in `research.md`:
- Deprecated endpoints: `openai_client.beta.threads.create`, `openai_client.beta.threads.messages.create`, `openai_client.beta.assistants.create`, `openai_client.beta.threads.runs.create`, etc.
- Expected response contract: `{ "response": string, "conversation_id": string, "sources": array, "debug_info": object }`
- Authentication: Using OpenAI API key from environment variables, same as before

## Constitution Check

Based on the project constitution and best practices:
- The solution must maintain backward compatibility with existing frontend
- Security: API keys must be properly handled and not exposed
- Performance: Response times should be maintained or improved
- Reliability: Error handling must be robust
- Maintainability: Code should follow existing patterns and be well-documented

## Gates

- ✅ Feature specification exists and is complete
- ✅ No security vulnerabilities introduced (proper API key handling)
- ✅ Backward compatibility maintained with frontend
- ✅ Performance requirements met (no significant degradation)
- ✅ Error handling and logging implemented properly

## Phase 0: Research

### Task 0.1: Identify Deprecated API Usage
- Locate all current OpenAI Assistants v1 API calls
- Document specific endpoints and methods being used
- Map current functionality to new API equivalents

### Task 0.2: Research Modern API Options
- Investigate OpenAI Chat Completions API as replacement
- Evaluate OpenAI Assistants v2 API as alternative
- Compare feature parity and migration complexity

### Task 0.3: Analyze Frontend Contract
- Document exact response format expected by frontend
- Identify any field mappings needed for compatibility
- Verify endpoint URLs and request formats

## Phase 1: Design & Contracts

### Task 1.1: Data Model Design
- ✅ Define entity relationships for new API integration
- ✅ Document validation rules for API requests/responses
- ✅ Map deprecated fields to new API equivalents

### Task 1.2: API Contract Definition
- ✅ Create OpenAPI specification for updated endpoints
- ✅ Define request/response schemas
- ✅ Document authentication and versioning headers

### Task 1.3: Implementation Design
- ✅ Plan agent layer migration strategy
- ✅ Design integration layer updates
- ✅ Plan error handling and fallback mechanisms

## Phase 2: Implementation Planning

### Task 2.1: Agent Layer Migration
- ✅ Replace deprecated OpenAI client usage
- ✅ Update agent methods to use new API
- ✅ Maintain RAG (Retrieval Augmented Generation) functionality

### Task 2.2: Integration Layer Updates
- ✅ Update agent integration to work with new API
- ✅ Ensure response format compatibility
- ✅ Update error handling patterns

### Task 2.3: API Layer Validation
- ✅ Verify endpoint responses match contract
- ✅ Test authentication and versioning headers
- ✅ Validate error scenarios and responses

## Phase 3: Post-Design Updates

### Task 3.1: Agent Context Update
- ✅ Run update-agent-context.ps1 to update agent-specific context files
- ✅ Add new technology stack information to agent context
- ✅ Preserve existing manual additions

## Success Criteria

- Application starts without deprecation warnings
- `/query` endpoint returns responses in expected format: `{ "answer": string, "sources": optional array }`
- OpenAI API calls use modern Responses API instead of deprecated Assistants v1
- Frontend continues to work without changes
- Error handling maintains security without exposing secrets
- API version headers properly enforced
- Contract validation working across backend boundaries

## Risk Mitigation

- Maintain backward compatibility with response format
- Implement proper error handling to prevent exposure of sensitive information
- Add comprehensive validation to catch any remaining deprecated API usage
- Thoroughly test with various query types and error conditions