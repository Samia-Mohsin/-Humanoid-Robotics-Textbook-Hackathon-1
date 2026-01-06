# Research: Sitemap-Based Multi-Point Ingestion Fix

## Decision: Multi-Chunk Processing Implementation Approach
**Rationale**: The existing pipeline infrastructure may be producing single embeddings per document instead of multiple chunks. This research identifies the root cause and determines the best approach to ensure proper multi-chunk processing.

## Technical Implementation Approach
1. **Document Analysis**: Examine existing sitemap_processor.py to identify single-point ingestion issues
2. **Chunking Review**: Review the chunker.py implementation to ensure it produces multiple chunks per substantial document
3. **ID Generation**: Implement unique, deterministic Qdrant point IDs based on source URL and chunk index
4. **Verification System**: Create validation functions to confirm multi-point storage in Qdrant

## Key Implementation Areas
1. **SitemapProcessor Enhancement**: Modify sitemap_processor.py to ensure documents are properly chunked into multiple segments
   - Update process_sitemap function to handle multi-chunk output
   - Ensure each URL generates multiple chunks when content is substantial
   - Implement proper metadata tracking for source and chunk position

2. **Chunking Logic**: Verify chunker.py produces multiple chunks per document
   - Adjust chunk size and overlap parameters if needed
   - Ensure documents large enough to be chunked actually produce multiple chunks
   - Add validation that minimum chunk count is met for substantial content

3. **Unique ID System**: Implement deterministic ID generation for Qdrant storage
   - Create unique IDs based on source URL and chunk index
   - Ensure IDs are consistent across re-runs for idempotent processing
   - Add metadata to track source document and chunk position

4. **Verification Functions**: Create validation to confirm multi-point ingestion
   - Count verification to ensure multiple vectors per URL
   - Search validation to confirm proper retrieval
   - Progress tracking with detailed multi-chunk metrics

## Alternatives Considered
1. **Single pipeline vs. multi-chunk**: Process each document as single entity vs. multiple chunks - chose multi-chunk to enable proper RAG functionality
2. **Random vs. deterministic IDs**: Use random IDs vs. deterministic IDs based on source and position - chose deterministic for idempotent re-runs
3. **Simple concatenation vs. proper chunking**: Combine all content vs. proper document chunking - chose proper chunking to maintain semantic meaning
4. **Batch vs. individual chunk storage**: Store all chunks together vs. individual storage with unique IDs - chose individual with unique IDs for better retrieval

## Root Cause Analysis
- **Potential Issue 1**: Chunk size parameters may be too large, causing documents to fit in a single chunk
- **Potential Issue 2**: Document processing logic may not be iterating through all chunks properly
- **Potential Issue 3**: Qdrant storage may be overwriting points instead of creating unique ones
- **Potential Issue 4**: Metadata tracking may not distinguish between different chunks from same document

## Multi-Chunk Processing Strategy
- **Chunk Size**: Set appropriate size (e.g., 512 tokens) with overlap (e.g., 50 tokens) to ensure multiple chunks for substantial documents
- **Minimum Threshold**: Ensure documents with more than X characters are chunked into multiple segments
- **Metadata Tracking**: Include source URL and chunk index in each embedding's metadata
- **ID Generation**: Create deterministic IDs using source URL hash and chunk index for consistency

## Verification Approach
- **Count Verification**: Compare expected chunk count vs. actual stored vectors per URL
- **Metadata Validation**: Confirm each stored vector has proper source and position metadata
- **Search Testing**: Run test searches to verify retrieval of specific chunks
- **Progress Tracking**: Monitor multi-chunk processing with detailed metrics