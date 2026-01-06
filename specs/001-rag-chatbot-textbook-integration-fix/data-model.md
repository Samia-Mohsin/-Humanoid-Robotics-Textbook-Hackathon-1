# Data Model: RAG Chatbot Textbook Integration Fix

## Core Entities

### DocumentChunk
- **Description**: Represents a segment of text content extracted from the humanoid robotics textbook with associated metadata
- **Fields**:
  - `chunk_id`: string - Unique identifier for the content chunk
  - `text`: string - The actual text content of the chunk (500-800 tokens)
  - `source_url`: string - URL or file path of the original document
  - `metadata`: dict - Additional metadata including module, section, and file information
  - `relevance_score`: float - Similarity score for the chunk in relation to a query

### IngestionPipeline
- **Description**: Process that loads, processes, chunks, and stores textbook content in Qdrant
- **Fields**:
  - `pipeline_id`: string - Unique identifier for the ingestion pipeline
  - `source_path`: string - Path to the source documents (/docs folder)
  - `chunk_size`: int - Size of chunks in tokens (500-800)
  - `overlap_size`: int - Overlap between chunks in tokens (100-200)
  - `embedding_model`: string - Model used for generating embeddings (Cohere embed-english-v3.0)
  - `target_collection`: string - Qdrant collection name for storage ("humanoid_robotics_docs")

### RetrievalResult
- **Description**: Result of a retrieval operation containing relevant content chunks
- **Fields**:
  - `query`: string - Original query text
  - `chunks`: list of DocumentChunk - Retrieved content chunks ranked by relevance
  - `retrieval_time`: float - Time taken for the retrieval operation in seconds
  - `similarity_threshold`: float - Minimum similarity score for inclusion (optional)

### ResponseContext
- **Description**: Assembled context from retrieved chunks used for answer generation
- **Fields**:
  - `assembled_chunks`: list of DocumentChunk - Chunks included in the context
  - `source_modules`: list of string - Modules referenced in the context
  - `total_tokens`: int - Total token count of the assembled context
  - `metadata`: dict - Additional context information

## Relationships
- An `IngestionPipeline` creates multiple `DocumentChunk` instances
- A `RetrievalResult` contains multiple `DocumentChunk` items
- A `ResponseContext` aggregates multiple `DocumentChunk` items for generation

## Validation Rules
- `DocumentChunk.chunk_id` must be unique within the collection
- `DocumentChunk.text` must be between 500-800 tokens
- `DocumentChunk.metadata` must contain required fields (module, section, file_path)
- `IngestionPipeline.chunk_size` must be between 500-800 tokens
- `IngestionPipeline.overlap_size` must be between 100-200 tokens
- `RetrievalResult.chunks` must not exceed 8 items
- `ResponseContext.total_tokens` must not exceed model context limit