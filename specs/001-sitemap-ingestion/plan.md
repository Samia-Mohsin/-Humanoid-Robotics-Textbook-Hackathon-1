# Implementation Plan: [FEATURE]

**Branch**: `[###-feature-name]` | **Date**: [DATE] | **Spec**: [link]
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement a sitemap-based content ingestion pipeline that can extract URLs from sitemap.xml files and process them through the existing pipeline infrastructure. The implementation will include a new sitemap_processor.py module that handles sitemap parsing, URL extraction, bulk processing with progress tracking, rate limiting, and error handling. The solution will integrate with existing pipeline components (scraper, chunker, embedder, vector_store) to process all URLs from the humanoid robotics textbook sitemap while respecting rate limits and providing comprehensive progress reporting.

## Technical Context

**Language/Version**: Python 3.11+ (existing backend pipeline infrastructure)
**Primary Dependencies**: requests, BeautifulSoup4, xml.etree.ElementTree, Cohere API client, Qdrant client, dataclasses, typing, argparse
**Storage**: Qdrant Cloud vector database (for embeddings storage and retrieval)
**Testing**: pytest (for backend pipeline testing)
**Target Platform**: Linux/Windows/MacOS server environment
**Project Type**: backend processing pipeline (single project with backend focus)
**Performance Goals**: Process all sitemap URLs within 30 minutes, maintain rate limiting of 1 request per second, memory usage under 1GB
**Constraints**: Must integrate with existing pipeline infrastructure, maintain backward compatibility, respect source website rate limits
**Scale/Scope**: Handle sitemaps with 50+ URLs, process humanoid robotics textbook content, provide progress tracking

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Pre-Design Compliance Verification

**I. Spec-first workflow using Spec-Kit Plus**: ✅ COMPLIANT - Feature specification already completed using Spec-Kit Plus templates with detailed requirements, acceptance criteria, and test scenarios in `specs/001-sitemap-ingestion/spec.md`

**II. Technical accuracy from official sources**: ✅ COMPLIANT - Implementation will be based on existing pipeline infrastructure and official documentation for dependencies (Cohere API, Qdrant client, XML parsing libraries)

**III. Clear, developer-focused writing**: ✅ COMPLIANT - Implementation will provide clear progress tracking and error reporting for developers and system administrators

**IV. Reproducible setup and deployment**: ✅ COMPLIANT - Implementation will integrate with existing pipeline infrastructure maintaining reproducible setup

**V. RAG Chatbot Grounded in Book Content**: N/A - This feature focuses on content ingestion, not RAG chatbot functionality

**VI. GitHub-based source control and collaboration**: ✅ COMPLIANT - All changes will be managed through GitHub with proper branching strategies

### Technology Stack Standards Compliance

✅ All technologies (Python, requests, BeautifulSoup4, xml.etree.ElementTree, Cohere, Qdrant) are well-documented and actively maintained as required by constitution

### Development Workflow Compliance

✅ Following Spec-Kit Plus workflow with specifications created before implementation as required

### Post-Design Compliance Re-check

All constitutional requirements continue to be met after Phase 1 design. The sitemap-based ingestion design maintains compliance with all constitution principles, particularly:
- The spec-first workflow remains intact with complete specifications before implementation
- Technical accuracy is maintained by building on existing, documented infrastructure
- Developer-focused approach is enhanced with comprehensive progress tracking and error reporting
- Integration with existing pipeline infrastructure ensures reproducible setup is maintained
- GitHub-based collaboration is preserved with proper branching strategies

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── main.py                 # Main pipeline execution entry point
├── config.py              # Configuration management
├── scraper.py             # Web scraping functionality
├── chunker.py             # Text chunking functionality
├── embedder.py            # Embedding generation functionality
├── vector_store.py        # Vector storage functionality
├── models.py              # Data models (DocumentChunk, EmbeddingVector)
├── utils.py               # Utility functions
├── crawler.py             # Crawler functionality
├── sitemap_processor.py   # New: Sitemap processing functionality
├── test_pipeline.py       # Pipeline tests
├── README.md              # Documentation
├── example_config.yaml    # Example configuration
└── .env                   # Environment variables
```

**Structure Decision**: The feature will enhance the existing backend pipeline infrastructure by adding a new sitemap_processor.py module to handle sitemap URL extraction and bulk processing. The implementation will integrate with the existing pipeline components while maintaining consistency with the current architecture.

## Phase 1: Design (Parallel Execution)

### 1.1 Sitemap Processing Architecture
- **Component**: `sitemap_processor.py` module
- **Purpose**: Parse sitemap.xml, extract URLs, coordinate bulk processing
- **Integration**: Reuse existing `main_pipeline` function for individual URL processing
- **Design Pattern**: Single responsibility principle - focus solely on sitemap processing

### 1.2 URL Management System
- **Extraction**: Use `xml.etree.ElementTree` to parse sitemap.xml structure
- **Validation**: Validate URLs before processing to avoid unnecessary requests
- **Filtering**: Apply filters to ensure only `/docs` URLs are processed
- **Deduplication**: Remove duplicate URLs to avoid redundant processing

### 1.3 Bulk Processing Pipeline
- **Sequential Processing**: Process URLs one-by-one with rate limiting
- **Progress Tracking**: Implement percentage completion and time estimation
- **Error Resilience**: Continue processing even if individual URLs fail
- **Memory Management**: Process in a memory-efficient manner for large sitemaps

### 1.4 Integration Points
- **CLI Enhancement**: Add `--sitemap` argument to existing CLI
- **Pipeline Reuse**: Leverage existing scraper, chunker, embedder, vector_store
- **Configuration**: Extend config to support sitemap-specific settings

## Phase 2: Implementation Approach

### 2.1 Sitemap Fetch and Parse Component
- **Function**: `extract_urls_from_sitemap(sitemap_url: str) -> List[Dict[str, Any]]`
- **Implementation**: Use requests to fetch sitemap, xml.etree.ElementTree to parse
- **Output**: List of URL objects with metadata (loc, lastmod, changefreq, priority)
- **Error Handling**: Graceful handling of malformed XML or network issues

### 2.2 URL Validation and Filtering Component
- **Function**: `validate_and_filter_urls(urls: List[Dict]) -> List[Dict]`
- **Filtering**: Only allow URLs matching `/docs` pattern
- **Deduplication**: Remove duplicate URLs using URL as key
- **Validation**: Verify URLs are accessible and return 200 status

### 2.3 Bulk Processing Orchestrator
- **Function**: `process_sitemap(sitemap_url: str, config: Config, rate_limit_delay: float)`
- **Implementation**: Process each URL through existing pipeline with rate limiting
- **Progress Tracking**: Report percentage completion and estimated time remaining
- **Resumability**: Allow processing to continue from last successful URL if interrupted

### 2.4 Quality Assurance Components
- **Content Verification**: Verify extracted content is meaningful (not empty/error pages)
- **Embedding Validation**: Confirm embeddings are generated and stored properly
- **Vector Count Verification**: Compare expected vs. actual stored vectors

## Phase 3: Implementation Steps

### 3.1 Infrastructure Setup
1. Create `sitemap_processor.py` with basic structure
2. Add sitemap processing functions to handle XML parsing
3. Implement URL extraction and validation logic
4. Add progress tracking utilities

### 3.2 Integration Development
1. Modify `main.py` to accept `--sitemap` argument
2. Integrate sitemap processing with existing pipeline
3. Add configuration options for sitemap processing
4. Implement rate limiting functionality

### 3.3 Quality Assurance Implementation
1. Add comprehensive error handling and logging
2. Implement progress tracking with percentage and time estimates
3. Add content and embedding validation checks
4. Create verification functions to confirm successful ingestion

### 3.4 Testing and Validation
1. Test with humanoid robotics textbook sitemap
2. Verify all 70+ URLs are processed successfully
3. Confirm embeddings are stored correctly in Qdrant
4. Validate that progress tracking works as expected

## Implementation Constraints & Considerations

### Performance Requirements
- **Rate Limiting**: Implement 1 request per second delay to respect source website
- **Memory Usage**: Keep memory consumption under 1GB even for large sitemaps
- **Processing Time**: Complete processing within 30 minutes for typical sitemaps
- **Progress Reporting**: Update progress every 10 URLs or every 30 seconds

### Error Handling Requirements
- **Graceful Degradation**: Continue processing if individual URLs fail
- **Comprehensive Logging**: Log all errors with sufficient context for debugging
- **Partial Success**: Allow successful URLs to be processed even if some fail
- **Retry Logic**: Implement basic retry for network-related failures

### Data Integrity Requirements
- **Deterministic Chunking**: Use consistent chunking algorithm with stable IDs
- **Idempotent Operations**: Ensure re-running doesn't create duplicate vectors
- **Metadata Preservation**: Maintain source URL and content metadata with embeddings
- **Verification Checks**: Confirm all expected vectors are stored in Qdrant

## Success Metrics

### Quantitative Metrics
- **Processing Rate**: 95%+ of valid URLs successfully processed
- **Time Performance**: Complete within 30 minutes for sitemaps up to 100 URLs
- **Memory Usage**: Stay under 1GB RAM during processing
- **Vector Accuracy**: 100% of processed content results in stored embeddings

### Qualitative Metrics
- **User Experience**: Clear progress reporting with percentage and time estimates
- **Error Recovery**: System continues operation despite individual URL failures
- **Integration Quality**: Seamless integration with existing pipeline components
- **Maintainability**: Clean, well-documented code following existing patterns

## Risk Mitigation

### Technical Risks
- **Large Sitemaps**: Implement streaming/batch processing to handle memory constraints
- **Network Failures**: Add retry logic and timeout handling for robustness
- **Rate Limiting**: Respectful processing to avoid being blocked by source websites
- **Data Corruption**: Validate content before processing and after storage

### Implementation Risks
- **Integration Complexity**: Reuse existing pipeline components to minimize new code
- **Configuration Issues**: Provide clear defaults and comprehensive error messages
- **Performance Degradation**: Monitor and optimize for processing speed and memory usage
- **Testing Gaps**: Create comprehensive test suite covering edge cases

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
