---
id: plan-20251227-001
title: sitemap-based-multi-point-ingestion-fix-plan
stage: plan
date: 2025-12-27
surface: command
model: Claude Sonnet 4.5
feature: sitemap-ingestion-fix
branch: 002-sitemap-ingestion-fix
user: samia
command: /sp.plan
labels: [sitemap, ingestion, fix, multi-point, qdrant, plan]
links:
  spec: ../specs/002-sitemap-ingestion-fix/spec.md
  plan: ../specs/002-sitemap-ingestion-fix/plan.md
  ticket:
  adr:
  pr:
files:
- path: specs/002-sitemap-ingestion-fix/plan.md
  action: created
  summary: Implementation plan for fixing single-point ingestion in sitemap-based pipeline
- path: specs/002-sitemap-ingestion-fix/research.md
  action: created
  summary: Research document analyzing multi-chunk processing approach
- path: specs/002-sitemap-ingestion-fix/data-model.md
  action: created
  summary: Data models for multi-point ingestion entities
- path: specs/002-sitemap-ingestion-fix/quickstart.md
  action: created
  summary: Quickstart guide for the multi-point ingestion fix
tests:
- name: plan-completeness-check
  result: passed
  summary: Plan covers all required implementation phases
- name: architecture-review
  result: passed
  summary: Architecture aligns with existing pipeline infrastructure
- name: compliance-check
  result: passed
  summary: Plan complies with project constitution
---

## Prompt

/sp.plan
1. Parse sitemap.xml, extract and validate all /docs URLs, then iterate through each page to extract, chunk, and embed content.
2. Upsert every chunk into Qdrant using unique deterministic IDs and verify ingestion via vector counts and test searches.

## Response snapshot

Successfully created comprehensive implementation plan for fixing single-point ingestion in the sitemap-based Qdrant pipeline. The plan includes:

## Plan Structure
- **Plan Document**: Complete implementation strategy with phases and steps
- **Research Document**: Technical analysis of multi-chunk processing approach
- **Data Model**: Entities for DocumentChunk, QdrantPoint, and validation results
- **Quickstart Guide**: Usage instructions for the fixed pipeline

## Key Implementation Areas
1. **Multi-Chunk Processing Architecture**: Enhance sitemap_processor.py with proper chunking logic
2. **Unique Deterministic ID System**: Create IDs based on source URL and chunk index
3. **Verification and Validation**: Confirm multi-point storage in Qdrant
4. **Integration Points**: Update chunker and vector_store components

## Implementation Approach
- **Sitemap URL Processing**: Enhanced to ensure multi-chunk output per document
- **Document Chunking**: Updated to generate multiple chunks with proper metadata
- **Qdrant ID Generation**: Unique deterministic IDs for each chunk
- **Verification Components**: Validation of multi-point storage and retrieval

## Success Metrics
- **Quantitative**: Each substantial document produces multiple chunks, 100% unique IDs
- **Qualitative**: Clear multi-chunk metrics, error recovery, integration quality

## Risk Mitigation
- **Technical**: Address single chunk output, ID collisions, performance impact
- **Implementation**: Reuse existing components, clear defaults, comprehensive testing

The plan comprehensively addresses both user requirements:
1. ✓ Parse sitemap.xml, extract and validate all /docs URLs, then iterate through each page to extract, chunk, and embed content
2. ✓ Upsert every chunk into Qdrant using unique deterministic IDs and verify ingestion via vector counts and test searches

## Outcome

- ✅ Impact: Complete plan created addressing single-point ingestion fix
- 🧪 Tests: Plan completeness, architecture review, and compliance tests passed
- 📁 Files: Created plan.md, research.md, data-model.md, and quickstart.md
- 🔁 Next prompts: Ready for task generation (/sp.tasks) or implementation
- 🧠 Reflection: Plan maintains integration with existing pipeline while fixing multi-point ingestion

## Evaluation notes (flywheel)

- Failure modes observed: None - all tests passed successfully
- Graders run and results (PASS/FAIL): All tests PASSED
- Prompt variant (if applicable): Standard /sp.plan command
- Next experiment (smallest change to try): Generate implementation tasks from plan