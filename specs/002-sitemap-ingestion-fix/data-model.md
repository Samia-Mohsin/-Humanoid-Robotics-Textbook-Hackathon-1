# Data Model: Sitemap-Based Multi-Point Ingestion Fix

## Entity: DocumentChunk
**Description**: A text segment extracted from a document that will have its own embedding and unique Qdrant ID

**Fields**:
- `source_url`: String - The original URL where the content was extracted from
- `content`: String - The text content of the chunk
- `chunk_index`: Integer - The position of this chunk within the original document
- `total_chunks`: Integer - The total number of chunks for this document
- `chunk_size`: Integer - The size of this chunk in tokens/characters
- `metadata`: Dict - Additional metadata about the chunk (source, position, etc.)

**Validation Rules**:
- source_url must be a valid URL format
- content must not be empty
- chunk_index must be non-negative
- chunk_index must be less than total_chunks
- chunk_size must be positive

## Entity: QdrantPoint
**Description**: A stored vector point in Qdrant with unique ID and source metadata

**Fields**:
- `id`: String - Unique, deterministic ID based on source URL and chunk index
- `vector`: List[float] - The embedding vector
- `payload`: Dict - Metadata including source_url, chunk_index, and content metadata
- `source_url`: String - The original URL of the document
- `chunk_index`: Integer - The position of this chunk within the document
- `created_at`: DateTime - When this point was created

**Validation Rules**:
- id must be unique across all points
- id must follow deterministic format based on source and chunk_index
- vector must have correct dimensions for the embedding model
- payload must contain required metadata fields

## Entity: MultiChunkIngestionResult
**Description**: The outcome of processing a sitemap with multi-chunk ingestion, including success/failure counts and validation results

**Fields**:
- `total_urls`: Integer - Total number of URLs extracted from the sitemap
- `processed_urls`: Integer - Number of URLs successfully processed
- `failed_urls`: Integer - Number of URLs that failed processing
- `total_chunks_created`: Integer - Total number of chunks created across all documents
- `total_vectors_stored`: Integer - Total number of vectors stored in Qdrant
- `url_results`: Array - Results for each processed URL with chunk counts
- `start_time`: DateTime - When the ingestion started
- `end_time`: DateTime - When the ingestion completed
- `total_duration`: Float - Total duration of processing in seconds
- `validation_results`: Dict - Results of multi-point ingestion validation

**Validation Rules**:
- total_urls must equal processed_urls + failed_urls
- total_vectors_stored should be greater than or equal to processed_urls (since each URL should produce multiple chunks)
- start_time must be before end_time
- All count fields must be non-negative integers

## Entity: ChunkValidationResult
**Description**: Validation result for a single chunk to confirm proper multi-point ingestion

**Fields**:
- `source_url`: String - The URL of the source document
- `chunk_index`: Integer - The index of the chunk within the document
- `qdrant_id`: String - The ID used in Qdrant for this chunk
- `is_valid`: Boolean - Whether the chunk was properly stored with unique ID
- `vector_size`: Integer - Size of the stored vector
- `metadata_valid`: Boolean - Whether metadata is properly attached
- `error_message`: String - Error message if validation failed

**Validation Rules**:
- chunk_index must be non-negative
- qdrant_id must follow deterministic format
- is_valid must be boolean
- vector_size must be positive

## Relationships
- Each MultiChunkIngestionResult contains multiple ChunkValidationResult objects in the validation_results
- Each QdrantPoint is associated with one DocumentChunk through source_url and chunk_index
- Each URL in the sitemap can produce multiple DocumentChunk objects
- Multiple QdrantPoint objects can be associated with the same source_url but different chunk_index values