# Feature Specification: Sitemap-Based Multi-Point Ingestion Fix

**Feature Branch**: `002-sitemap-ingestion-fix`
**Created**: 2025-12-27
**Status**: Draft
**Input**: User description: "Resolve single-point ingestion in sitemap-based Qdrant pipeline for https://humanoid-robotics-textbook-hackatho-seven.vercel.app/sitemap.xml"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Multi-URL Processing (Priority: P1)

As a developer debugging RAG ingestion, I want the system to process all documentation URLs from the sitemap.xml so that I can ensure comprehensive content ingestion instead of single-URL processing.

**Why this priority**: This is the foundational fix needed to resolve the single-point ingestion issue and ensure all documentation pages are processed.

**Independent Test**: Can be fully tested by providing the sitemap URL and verifying that multiple /docs URLs are extracted and processed, not just a single URL.

**Acceptance Scenarios**:

1. **Given** a valid sitemap.xml URL, **When** I initiate the sitemap ingestion process, **Then** the system extracts and processes multiple URLs from the sitemap
2. **Given** the sitemap contains 50+ /docs URLs for the humanoid robotics textbook, **When** I run the sitemap ingestion, **Then** all URLs are processed individually with proper chunking

---

### User Story 2 - Multi-Chunk Embedding per Document (Priority: P2)

As a developer debugging RAG ingestion, I want each URL to produce multiple embedded chunks instead of a single embedding so that I can ensure proper document chunking and comprehensive vector storage.

**Why this priority**: This ensures that each document is properly chunked and multiple embeddings are created per document, resolving the single-point storage issue.

**Independent Test**: Can be fully tested by running the system on a single URL and verifying that multiple chunks are generated and embedded, not just one.

**Acceptance Scenarios**:

1. **Given** a single documentation URL from the sitemap, **When** I process it through the pipeline, **Then** multiple text chunks are created and each generates its own embedding
2. **Given** a processed URL, **When** I check the vector database, **Then** multiple vector points exist for that single source URL with different chunk indices

---

### User Story 3 - Unique Deterministic Qdrant Point IDs (Priority: P3)

As a developer debugging RAG ingestion, I want each chunk to have a unique, deterministic Qdrant point ID so that I can ensure proper storage of multiple vector points without conflicts and enable idempotent re-runs.

**Why this priority**: This ensures that each chunk gets a unique ID that can be consistently reproduced, preventing overwrites and enabling re-runnable ingestion.

**Independent Test**: Can be fully tested by verifying that each chunk gets a unique ID that follows a deterministic pattern based on source URL and chunk index.

**Acceptance Scenarios**:

1. **Given** multiple chunks from the same document, **When** they are stored in Qdrant, **Then** each has a unique point ID
2. **Given** a re-run of the same ingestion, **When** chunks are processed again, **Then** they generate the same deterministic IDs allowing for idempotent operations

---

### Edge Cases

- What happens when a document is too small to be chunked and results in only one chunk?
- How does the system handle documents with different content lengths requiring different chunking strategies?
- What if the same URL appears multiple times in the sitemap with different metadata?
- How does the system handle very large documents that require many chunks?
- What happens when Qdrant storage limits are approached during bulk processing?
- How does the system handle duplicate content across different URLs?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST extract multiple URLs from the provided sitemap.xml file (not just single URL)
- **FR-002**: System MUST process each extracted URL individually through the ingestion pipeline
- **FR-003**: System MUST chunk each document into multiple text segments based on configurable size
- **FR-004**: System MUST generate separate embeddings for each text chunk
- **FR-005**: System MUST create unique, deterministic Qdrant point IDs for each chunk
- **FR-006**: System MUST store multiple vector points per source document in Qdrant
- **FR-007**: System MUST support idempotent, re-runnable ingestion without creating duplicate points
- **FR-008**: System MUST validate that multiple chunks are created per document
- **FR-009**: System MUST handle documents of varying sizes appropriately
- **FR-010**: System MUST provide logging to verify multi-point ingestion is working

### Key Entities

- **SitemapURL**: A URL extracted from the sitemap.xml file that needs to be processed
- **DocumentChunk**: A text segment extracted from a document that will have its own embedding
- **EmbeddingVector**: A vector representation of a document chunk with associated metadata
- **QdrantPoint**: A stored vector point in Qdrant with unique ID and source metadata

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 100% of URLs in the sitemap.xml are extracted and processed for multi-chunk ingestion
- **SC-002**: Each processed URL generates multiple embedded chunks (minimum 2 chunks per document > 1000 characters)
- **SC-003**: Qdrant stores multiple vector points per source URL (not just one point per URL)
- **SC-004**: Each vector point has a unique, deterministic ID based on source URL and chunk index
- **SC-005**: Re-running the same ingestion is idempotent and doesn't create duplicate points
- **SC-006**: Stored points span multiple source URLs and chunk indices as verified in Qdrant
- **SC-007**: At least 95% of valid sitemap URLs result in multiple vector points being stored
- **SC-008**: System can process all URLs from the humanoid robotics textbook sitemap with multi-chunk output

## Assumptions

- The existing sitemap ingestion infrastructure can be enhanced to support multi-chunk processing
- The chunking algorithm can be configured to create multiple segments per document
- Qdrant supports storing multiple points with unique IDs per source document
- The Cohere embedding model can handle multiple chunks per document efficiently
- Document content is substantial enough to be meaningfully chunked into multiple segments