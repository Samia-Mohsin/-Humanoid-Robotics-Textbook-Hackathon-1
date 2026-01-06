# Data Model: URL Ingestion & Embedding Pipeline

## Document Chunk Entity

- **chunk_id**: String (UUID) - Unique identifier for the text chunk
- **source_url**: String - Original URL where the content was found
- **content**: Text - The actual text content of the chunk
- **title**: String - Title of the page where the chunk was found
- **section**: String - Section heading where the chunk was found
- **position**: Integer - Position of the chunk within the document
- **created_at**: DateTime - Timestamp when the chunk was created
- **metadata**: JSON - Additional metadata about the chunk

## Embedding Vector Entity

- **embedding_id**: String (UUID) - Unique identifier for the embedding
- **chunk_id**: String - Reference to the source document chunk
- **vector**: Array of Floats - The embedding vector values
- **model**: String - The model used to generate the embedding
- **created_at**: DateTime - Timestamp when the embedding was created
- **metadata**: JSON - Additional metadata including source URL and content info

## Configuration Parameters

- **cohere_api_key**: String - API key for Cohere services
- **qdrant_url**: String - URL endpoint for Qdrant Cloud
- **qdrant_api_key**: String - API key for Qdrant Cloud
- **chunk_size**: Integer - Target size for text chunks (default: 512)
- **chunk_overlap**: Integer - Overlap between consecutive chunks (default: 50)
- **cohere_model**: String - Model name for embedding generation (default: embed-english-v3.0)

## Validation Rules

1. **Document Chunk Validation**:
   - content must not be empty
   - source_url must be a valid URL format
   - position must be non-negative

2. **Embedding Vector Validation**:
   - vector must have consistent dimensions
   - chunk_id must reference an existing document chunk
   - model must be a supported Cohere model

3. **Configuration Validation**:
   - API keys must be provided
   - URLs must be properly formatted
   - chunk_size must be positive
   - chunk_overlap must be less than chunk_size