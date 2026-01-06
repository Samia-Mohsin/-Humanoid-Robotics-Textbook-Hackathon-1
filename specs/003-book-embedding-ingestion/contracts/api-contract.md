# API Contract: URL Ingestion & Embedding Pipeline

## Main Pipeline Function

### Function: `ingest_urls(urls: List[str], config: Config) -> IngestionResult`

**Description**: Main function to run the full ingestion pipeline end-to-end

**Parameters**:
- `urls`: List of URLs to process
- `config`: Configuration object with API keys and settings

**Returns**: IngestionResult object with statistics and status

**Example Usage**:
```python
urls = ["https://example-docs.com/page1", "https://example-docs.com/page2"]
config = Config(
    cohere_api_key="...",
    qdrant_url="...",
    qdrant_api_key="...",
    chunk_size=512,
    chunk_overlap=50
)
result = ingest_urls(urls, config)
```

## Sub-functions

### Function: `fetch_content(url: str) -> str`

**Description**: Fetch and extract clean text content from a URL

**Parameters**:
- `url`: URL to fetch content from

**Returns**: Clean text content

### Function: `chunk_text(text: str, chunk_size: int, overlap: int) -> List[TextChunk]`

**Description**: Split text into overlapping chunks

**Parameters**:
- `text`: Text to chunk
- `chunk_size`: Target size of each chunk
- `overlap`: Overlap between chunks

**Returns**: List of text chunks

### Function: `generate_embeddings(chunks: List[TextChunk], model: str) -> List[EmbeddingVector]`

**Description**: Generate embeddings for text chunks using Cohere

**Parameters**:
- `chunks`: List of text chunks to embed
- `model`: Cohere model to use

**Returns**: List of embedding vectors

### Function: `store_embeddings(embeddings: List[EmbeddingVector], collection_name: str)`

**Description**: Store embeddings in Qdrant collection

**Parameters**:
- `embeddings`: List of embeddings to store
- `collection_name`: Name of Qdrant collection

## Data Types

### Config
- `cohere_api_key`: str
- `qdrant_url`: str
- `qdrant_api_key`: str
- `chunk_size`: int
- `chunk_overlap`: int
- `cohere_model`: str

### TextChunk
- `id`: str
- `content`: str
- `source_url`: str
- `title`: str
- `position`: int

### EmbeddingVector
- `chunk_id`: str
- `vector`: List[float]
- `metadata`: dict